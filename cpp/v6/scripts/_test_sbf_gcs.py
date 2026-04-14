#!/usr/bin/env python3
"""Quick test: SBF boxes -> GCS path optimization.

Run ONE query via subprocess to isolate C++ memory issues.
Usage: python _test_sbf_gcs.py [AS_TS|LB_RB]
"""
import sys, os, gc, subprocess, json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build', 'python'))

import numpy as np

CONFIGS = {
    'AS': [6.42e-05, 0.4719533, -0.0001493, -0.6716735,
           0.0001854, 0.4261696, 1.5706922],
    'TS': [-1.55e-04, 0.3972726, 0.0002196, -1.3674756,
           0.0002472, -0.1929518, 1.5704688],
    'LB': [1.3326656, 0.7865932, 0.3623384, -1.4916529,
           -0.3192509, 0.9217325, 1.7911904],
    'RB': [-1.3324624, 0.7866478, -0.3626562, -1.4916528,
           0.3195340, 0.9217833, 1.3502090],
}

PAIRS = {"AS_TS": ("AS", "TS"), "LB_RB": ("LB", "RB")}


def run_single_query(s_name, g_name):
    """Run SBF + GCS for one query pair. Returns results dict."""
    import _sbf5_cpp as sbf5
    import time

    from pydrake.geometry.optimization import (
        GraphOfConvexSets, GraphOfConvexSetsOptions,
        HPolyhedron, Point as DrakePoint)
    from pydrake.solvers import MosekSolver, L2NormCost, Binding, Cost

    q_s = np.array(CONFIGS[s_name])
    q_g = np.array(CONFIGS[g_name])
    label = f"{s_name}->{g_name}"

    robot = sbf5.Robot.from_json(
        os.path.join(os.path.dirname(__file__), '..', 'data', 'iiwa14.json'))

    # Obstacles
    def make_shelves():
        ox, oy, oz = 0.85, 0.0, 0.4
        obs = []
        def add(lx,ly,lz,fx,fy,fz):
            obs.append(sbf5.Obstacle(
                ox+lx-fx/2, oy+ly-fy/2, oz+lz-fz/2,
                ox+lx+fx/2, oy+ly+fy/2, oz+lz+fz/2))
        add(0, 0.292, 0, 0.3, 0.016, 0.783)
        add(0,-0.292, 0, 0.3, 0.016, 0.783)
        add(0, 0, 0.3995, 0.3, 0.6, 0.016)
        add(0, 0,-0.13115, 0.3, 0.6, 0.016)
        add(0, 0, 0.13115, 0.3, 0.6, 0.016)
        return obs

    def make_bins():
        obs = []
        def add_bin(bx,by,bz):
            def add(lx,ly,lz,fx,fy,fz):
                obs.append(sbf5.Obstacle(
                    bx-ly-fy/2, by+lx-fx/2, bz+lz-fz/2,
                    bx-ly+fy/2, by+lx+fx/2, bz+lz+fz/2))
            add(0.22,0,0.105, 0.05,0.63,0.21)
            add(-0.22,0,0.105, 0.05,0.63,0.21)
            add(0,0.29,0.105, 0.49,0.05,0.21)
            add(0,-0.29,0.105, 0.49,0.05,0.21)
            add(0,0,0.0075, 0.49,0.63,0.015)
        add_bin(0,-0.6,0)
        add_bin(0,0.6,0)
        return obs

    def make_table():
        return [sbf5.Obstacle(0.4-2.5/2, -2.5/2, -0.25-0.2/2,
                               0.4+2.5/2, 2.5/2, -0.25+0.2/2)]

    obstacles = make_shelves() + make_bins() + make_table()

    # --- SBF planning ---
    config = sbf5.SBFPlannerConfig()
    config.grower.timeout_ms = 10000
    planner = sbf5.SBFPlanner(robot, config)
    result = planner.plan(q_s, q_g, obstacles, 15000)

    if not result.success:
        return {"label": label, "success": False}

    # Extract box data BEFORE deleting planner
    boxes = planner.boxes()
    box_data = {}
    for b in boxes:
        lb = [iv.lo for iv in b.joint_intervals]
        ub = [iv.hi for iv in b.joint_intervals]
        box_data[b.id] = (lb, ub)

    seq = list(result.box_sequence)
    sbf_path_len = result.path_length
    n_boxes = result.n_boxes

    # Free C++ memory
    del result, planner, robot, obstacles, boxes
    gc.collect()

    # --- Compute full box adjacency graph ---
    # Match C++ boxes_adjacent() exactly (adjacency.cpp, tol=1e-6)
    TOL = 1e-6
    all_ids = list(box_data.keys())

    def boxes_adjacent(id_a, id_b):
        la, ua = box_data[id_a]
        lb2, ub2 = box_data[id_b]
        ndof = len(la)
        shared_dims = 0
        overlap_dims = 0
        for d in range(ndof):
            overlap_lo = max(la[d], lb2[d])
            overlap_hi = min(ua[d], ub2[d])
            if overlap_hi < overlap_lo - TOL:
                return False  # separated
            if overlap_hi - overlap_lo < TOL:
                shared_dims += 1  # face contact
            else:
                overlap_dims += 1
        return (shared_dims >= 1) or (overlap_dims == ndof)

    # Build adjacency
    adj = {bid: [] for bid in all_ids}
    for i in range(len(all_ids)):
        for j in range(i + 1, len(all_ids)):
            a, b = all_ids[i], all_ids[j]
            if boxes_adjacent(a, b):
                adj[a].append(b)
                adj[b].append(a)
    n_adj_edges = sum(len(v) for v in adj.values()) // 2

    # Dijkstra for backbone
    import heapq
    def dijkstra_backbone(adj_graph, start_id, goal_id):
        dist = {start_id: 0.0}
        prev = {}
        pq = [(0.0, start_id)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, float('inf')):
                continue
            if u == goal_id:
                break
            for v in adj_graph.get(u, []):
                la, ua = box_data[u]
                lb2, ub2 = box_data[v]
                cu = np.array([(la[d]+ua[d])/2 for d in range(len(la))])
                cv = np.array([(lb2[d]+ub2[d])/2 for d in range(len(la))])
                w = float(np.linalg.norm(cu - cv))
                nd = d + w
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if goal_id not in dist:
            return []
        path = []
        cur = goal_id
        while cur != start_id:
            path.append(cur)
            cur = prev[cur]
        path.append(start_id)
        path.reverse()
        return path

    # The first/last boxes in the SBF sequence contain start/goal
    # Deduplicate sequence first
    deduped = [seq[0]]
    for bid in seq[1:]:
        if bid != deduped[-1]:
            deduped.append(bid)
    start_box = deduped[0]
    goal_box = deduped[-1]

    if not start_box or not goal_box:
        return {"label": label, "success": True, "sbf_path_len": sbf_path_len,
                "n_boxes": n_boxes, "gcs_success": False, "error": "no_containing_box"}

    backbone = dijkstra_backbone(adj, start_box, goal_box)
    if not backbone:
        return {"label": label, "success": True, "sbf_path_len": sbf_path_len,
                "n_boxes": n_boxes, "gcs_success": False, "error": "no_path"}

    # Expand corridor by 2 hops
    corridor = set(backbone)
    frontier = set(backbone)
    for _ in range(2):
        nxt = set()
        for bid in frontier:
            for nbr in adj.get(bid, []):
                if nbr not in corridor:
                    corridor.add(nbr)
                    nxt.add(nbr)
        frontier = nxt

    # --- GCS optimization on expanded corridor ---
    ndof = len(q_s)
    gcs = GraphOfConvexSets()
    verts = {}
    for bid in corridor:
        lb, ub = box_data[bid]
        verts[bid] = gcs.AddVertex(
            HPolyhedron.MakeBox(np.array(lb), np.array(ub)), f"box_{bid}")

    source = gcs.AddVertex(DrakePoint(q_s), "source")
    target = gcs.AddVertex(DrakePoint(q_g), "target")

    # Source -> start box, goal box -> target
    gcs.AddEdge(source, verts[backbone[0]])
    gcs.AddEdge(verts[backbone[-1]], target)

    # All adjacency edges within corridor (bidirectional)
    edge_set = set()
    for u_bid in corridor:
        for v_bid in adj.get(u_bid, []):
            if v_bid in corridor:
                pair = (min(u_bid, v_bid), max(u_bid, v_bid))
                if pair not in edge_set:
                    edge_set.add(pair)
                    gcs.AddEdge(verts[u_bid], verts[v_bid])
                    gcs.AddEdge(verts[v_bid], verts[u_bid])

    # L2 cost
    A_l2 = np.hstack((-np.eye(ndof), np.eye(ndof)))
    b_l2 = np.zeros(ndof)
    l2_cost = L2NormCost(A_l2, b_l2)
    for edge in gcs.Edges():
        edge.AddCost(Binding[Cost](l2_cost,
                                   np.concatenate([edge.xu(), edge.xv()])))

    n_verts = len(gcs.Vertices())
    n_edges = len(gcs.Edges())

    t0 = time.perf_counter()
    opts = GraphOfConvexSetsOptions()
    opts.convex_relaxation = True
    opts.preprocessing = True
    opts.max_rounded_paths = 10
    opts.max_rounding_trials = 100
    opts.solver = MosekSolver()
    res = gcs.SolveShortestPath(source, target, opts)
    dt = time.perf_counter() - t0

    if not res.is_success():
        return {"label": label, "success": True, "sbf_path_len": sbf_path_len,
                "n_boxes": n_boxes, "gcs_success": False}

    # Extract path by following active edges from source to target
    ordered = [q_s.tolist()]
    current = source
    visited = {source.id()}
    for _ in range(n_verts + 2):
        found = False
        best_edge = None
        best_phi = -1
        for edge in gcs.Edges():
            if edge.u() == current and edge.v().id() not in visited:
                phi = res.GetSolution(edge.phi())
                if phi > best_phi:
                    best_phi = phi
                    best_edge = edge
        if best_edge is not None and best_phi > 0.01:
            wp = res.GetSolution(best_edge.xv())
            if best_edge.v() != target:
                ordered.append(wp.tolist())
            current = best_edge.v()
            visited.add(current.id())
            found = True
        if not found or current == target:
            break
    ordered.append(q_g.tolist())

    # Remove near-duplicate consecutive waypoints
    filtered = [ordered[0]]
    for wp in ordered[1:]:
        if np.linalg.norm(np.array(wp) - np.array(filtered[-1])) > 1e-8:
            filtered.append(wp)
    ordered = filtered

    gcs_len = sum(np.linalg.norm(np.array(ordered[i]) - np.array(ordered[i-1]))
                  for i in range(1, len(ordered)))

    # Cleanup Drake objects before exit
    del res, gcs, source, target, verts, opts
    gc.collect()

    return {
        "label": label, "success": True,
        "sbf_path_len": round(sbf_path_len, 4),
        "n_boxes": n_boxes,
        "n_adj_edges": n_adj_edges,
        "backbone_len": len(backbone),
        "corridor_len": len(corridor),
        "gcs_verts": n_verts,
        "gcs_edges": n_edges,
        "gcs_success": True,
        "gcs_path_len": round(gcs_len, 4),
        "gcs_time": round(dt, 3),
        "gcs_waypoints": len(ordered),
        "improvement_pct": round((1 - gcs_len / sbf_path_len) * 100, 1),
    }


if __name__ == "__main__":
    query = sys.argv[1] if len(sys.argv) > 1 else None

    if query and query in PAIRS:
        # Run single query (called as subprocess)
        s, g = PAIRS[query]
        r = run_single_query(s, g)
        # Print JSON to stdout (flush immediately)
        sys.stdout.write("RESULT:" + json.dumps(r) + "\n")
        sys.stdout.flush()
        os._exit(0)  # Skip Python cleanup to avoid C++ destructor crashes
    else:
        # Run all queries via subprocesses
        script = os.path.abspath(__file__)
        for key, (s, g) in PAIRS.items():
            label = f"{s}->{g}"
            print(f"\n{'='*50}")
            print(f"Running {label}...")
            proc = subprocess.run(
                [sys.executable, script, key],
                capture_output=True, text=True, timeout=120)
            # Find result line in stdout
            result = None
            for line in proc.stdout.splitlines():
                if line.startswith("RESULT:"):
                    result = json.loads(line[7:])
                    break
            if result is None:
                print(f"  {label}: CRASHED (exit={proc.returncode})")
                if proc.stderr:
                    # Show last few lines of stderr
                    err_lines = proc.stderr.strip().splitlines()
                    for el in err_lines[-3:]:
                        print(f"  stderr: {el}")
                continue

            if not result["success"]:
                print(f"  {label}: SBF FAILED")
                continue

            print(f"  SBF: path_len={result['sbf_path_len']}, "
                  f"boxes={result['n_boxes']}, "
                  f"adj_edges={result.get('n_adj_edges','?')}, "
                  f"backbone={result.get('backbone_len','?')}, "
                  f"corridor={result.get('corridor_len','?')}")

            if not result.get("gcs_success"):
                print(f"  GCS: FAILED ({result.get('error','')})")
                continue

            print(f"  GCS: {result.get('gcs_verts','?')}V/{result.get('gcs_edges','?')}E, "
                  f"path_len={result['gcs_path_len']} "
                  f"({result['gcs_time']}s), "
                  f"{result['gcs_waypoints']} waypoints")
            print(f"  Improvement: {result['sbf_path_len']} -> "
                  f"{result['gcs_path_len']} "
                  f"({result['improvement_pct']}%)")
