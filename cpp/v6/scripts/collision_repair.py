#!/usr/bin/env python3
"""
collision_repair.py — Post-process GCS paths to eliminate collisions.

Uses Drake collision checker (same model as visualization) to verify each
segment. Colliding segments are replaced with corresponding C++ path
sub-segments (which are verified collision-free).

Usage:
    conda activate sbf
    python scripts/collision_repair.py result/gcs_results_v2.json result/paths.json
"""

import argparse
import json
import logging
import os
import sys
import time
from pathlib import Path

import numpy as np

from pydrake.multibody.parsing import Parser, LoadModelDirectives, ProcessModelDirectives
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder

logging.basicConfig(level=logging.INFO, format="[REPAIR] %(message)s")
log = logging.getLogger(__name__)

# ──────────────────────── Drake collision checker ───────────────────────────

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_V5_ROOT = os.path.dirname(_THIS_DIR)
_PROJ_ROOT = os.path.dirname(os.path.dirname(_V5_ROOT))
_GCS_ROOT = os.path.join(_PROJ_ROOT, "gcs-science-robotics")
_YAML_FILE = os.path.join(_GCS_ROOT, "models", "iiwa14_welded_gripper.yaml")


class DrakeCollisionChecker:
    """Thin wrapper around Drake collision checking."""

    def __init__(self):
        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder, time_step=0.0)
        parser = Parser(self.plant)
        parser.package_map().Add("gcs", _GCS_ROOT)
        directives = LoadModelDirectives(_YAML_FILE)
        ProcessModelDirectives(directives, self.plant, parser)
        self.plant.Finalize()
        self.diagram = builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyMutableContextFromRoot(self.context)
        self.sg_context = self.scene_graph.GetMyMutableContextFromRoot(self.context)
        self.iiwa = self.plant.GetModelInstanceByName("iiwa")
        log.info("Drake collision checker initialized")

    def check_collision(self, q):
        """Return True if configuration q is in collision."""
        self.plant.SetPositions(self.plant_context, self.iiwa, q)
        qo = self.scene_graph.get_query_output_port().Eval(self.sg_context)
        return qo.HasCollisions()

    def check_segment(self, q1, q2, resolution=0.02):
        """Check segment from q1 to q2. Returns True if ANY point collides."""
        dist = float(np.linalg.norm(q2 - q1))
        n_checks = max(2, int(np.ceil(dist / resolution)))
        for k in range(n_checks + 1):
            t = k / n_checks
            qt = q1 + t * (q2 - q1)
            if self.check_collision(qt):
                return True
        return False


# ──────────────────── Path utilities ────────────────────────────

def densify_path(waypoints, step_rad=0.05):
    """Densify path so no segment exceeds step_rad."""
    if len(waypoints) < 2:
        return list(waypoints)
    dense = [waypoints[0].copy()]
    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i + 1]
        dist = float(np.linalg.norm(p2 - p1))
        if dist <= step_rad:
            dense.append(p2.copy())
        else:
            n_sub = int(np.ceil(dist / step_rad))
            for k in range(1, n_sub + 1):
                t = k / n_sub
                dense.append(p1 + t * (p2 - p1))
    return dense


def path_length(wps):
    """Total Euclidean path length."""
    return sum(float(np.linalg.norm(wps[i + 1] - wps[i]))
               for i in range(len(wps) - 1))


def find_nearest_idx(target, path):
    """Find index of closest point on path to target."""
    dists = [float(np.linalg.norm(p - target)) for p in path]
    return int(np.argmin(dists))


# ──────────── Collision repair: replace colliding sections ──────────

def repair_path(gcs_wps, cpp_wps, checker, label=""):
    """
    Repair a GCS path by replacing colliding sections with C++ path segments.

    Strategy:
    1. Densify the C++ path (collision-free reference)
    2. Walk through GCS path, checking each segment
    3. For contiguous colliding regions, splice in C++ sub-path between
       the nearest collision-free GCS waypoints

    Returns: repaired path (list of np.ndarray), n_segments_repaired.
    """
    n = len(gcs_wps)
    if n < 2:
        return gcs_wps, 0

    # Densify C++ reference path
    cpp_dense = densify_path(cpp_wps, step_rad=0.02)

    # Check each GCS segment for collision
    seg_collision = []
    for i in range(n - 1):
        has_col = checker.check_segment(gcs_wps[i], gcs_wps[i + 1], resolution=0.02)
        seg_collision.append(has_col)

    n_colliding = sum(seg_collision)
    if n_colliding == 0:
        log.info(f"  {label}: all {n-1} segments CLEAN")
        return gcs_wps, 0

    log.info(f"  {label}: {n_colliding}/{n-1} segments in collision, repairing...")

    # Find contiguous collision regions
    regions = []  # list of (start_idx, end_idx) — inclusive segment indices
    i = 0
    while i < len(seg_collision):
        if seg_collision[i]:
            j = i
            while j < len(seg_collision) and seg_collision[j]:
                j += 1
            regions.append((i, j - 1))
            i = j
        else:
            i += 1

    log.info(f"  {label}: {len(regions)} contiguous collision regions")

    # Build repaired path by replacing collision regions with C++ sub-paths
    repaired = []
    prev_end = 0

    for reg_start, reg_end in regions:
        # Add clean GCS segments before this region
        for k in range(prev_end, reg_start + 1):
            repaired.append(gcs_wps[k].copy())

        # Find C++ path sub-section that covers from gcs_wps[reg_start] to gcs_wps[reg_end+1]
        entry_pt = gcs_wps[reg_start]
        exit_pt = gcs_wps[reg_end + 1]

        cpp_entry_idx = find_nearest_idx(entry_pt, cpp_dense)
        cpp_exit_idx = find_nearest_idx(exit_pt, cpp_dense)

        # Ensure correct direction
        if cpp_entry_idx > cpp_exit_idx:
            cpp_entry_idx, cpp_exit_idx = cpp_exit_idx, cpp_entry_idx

        # Extract C++ sub-path
        cpp_sub = cpp_dense[cpp_entry_idx:cpp_exit_idx + 1]

        if len(cpp_sub) >= 2:
            # Replace first and last points with the GCS boundary points
            # so the path remains continuous
            # (actually, use C++ points which are collision-free)
            for pt in cpp_sub[1:-1]:  # skip first/last to avoid duplicates
                repaired.append(pt.copy())
            log.info(f"    Region [{reg_start}-{reg_end}]: replaced with "
                     f"{len(cpp_sub)-2} C++ wps (cpp_idx {cpp_entry_idx}..{cpp_exit_idx})")
        else:
            log.warning(f"    Region [{reg_start}-{reg_end}]: no C++ sub-path found")

        prev_end = reg_end + 1

    # Add remaining clean GCS segments
    for k in range(prev_end, n):
        repaired.append(gcs_wps[k].copy())

    log.info(f"  {label}: repaired {n} -> {len(repaired)} waypoints")
    return repaired, n_colliding


def full_path_collision_free(wps, checker, resolution=0.02):
    """Full collision check of a path. Returns (clean, n_colliding_segs)."""
    n_col = 0
    for i in range(len(wps) - 1):
        if checker.check_segment(wps[i], wps[i + 1], resolution):
            n_col += 1
    return n_col == 0, n_col


# ──────────────────── Main ────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Collision repair for GCS paths using Drake checker")
    parser.add_argument("gcs_json", help="gcs_results_v2.json")
    parser.add_argument("paths_json", help="paths.json from C++ exp2")
    parser.add_argument("--output", type=str, default=None,
                        help="Output JSON path (default: overwrite gcs_json)")
    parser.add_argument("--resolution", type=float, default=0.02,
                        help="Collision check resolution in rad")
    args = parser.parse_args()

    # Load data
    with open(args.gcs_json) as f:
        gcs_data = json.load(f)
    with open(args.paths_json) as f:
        cpp_data = json.load(f)

    cpp_paths = {p["pair_idx"]: p for p in cpp_data["paths"]}

    # Initialize Drake collision checker
    checker = DrakeCollisionChecker()

    # First, verify that C++ paths are indeed collision-free
    log.info("\n=== Verifying C++ reference paths ===")
    for qi in sorted(cpp_paths.keys()):
        p = cpp_paths[qi]
        if not p["success"]:
            continue
        label = cpp_data["queries"][qi]["label"]
        wps = [np.array(w) for w in p["waypoints"]]
        # Densify first (C++ segments can be huge)
        dense = densify_path(wps, step_rad=0.02)
        clean, n_col = full_path_collision_free(dense, checker, args.resolution)
        status = "CLEAN" if clean else f"COLLISION ({n_col} segs)"
        log.info(f"  C++ {label}: {status} ({len(wps)} wps -> {len(dense)} dense)")

    # Repair GCS paths
    log.info("\n=== Repairing GCS paths ===")
    for result in gcs_data["results"]:
        if not result.get("success"):
            continue
        qi = result["pair_idx"]
        label = result["label"]

        gcs_wps = [np.array(w) for w in result["waypoints"]]
        cpp_p = cpp_paths.get(qi)
        if not cpp_p or not cpp_p.get("success"):
            log.warning(f"  {label}: no C++ reference path, skipping")
            continue
        cpp_wps = [np.array(w) for w in cpp_p["waypoints"]]

        log.info(f"\n{'='*60}")
        log.info(f"  {label}: {len(gcs_wps)} GCS wps")

        # Repair
        repaired, n_rep = repair_path(gcs_wps, cpp_wps, checker, label)

        # Verify repaired path
        clean, n_col = full_path_collision_free(repaired, checker, args.resolution)
        rep_len = path_length(repaired)

        if not clean:
            log.warning(f"  {label}: still {n_col} colliding segments after repair!")
            # Fallback: use entire C++ path
            log.info(f"  {label}: falling back to densified C++ path")
            repaired = densify_path(cpp_wps, step_rad=0.05)
            rep_len = path_length(repaired)
            clean2, n_col2 = full_path_collision_free(repaired, checker, args.resolution)
            if not clean2:
                log.warning(f"  {label}: C++ path also has {n_col2} collisions at res={args.resolution}!")

        # Update result
        result["waypoints"] = [p.tolist() for p in repaired]
        result["total_path_length"] = round(rep_len, 6)
        result["collision_repaired"] = n_rep > 0
        result["n_segments_repaired"] = n_rep

        dijk_len = result.get("dijkstra_path_length", 1.0)
        ratio = rep_len / dijk_len if dijk_len > 0 else 0
        result["improvement_ratio"] = round(ratio, 6)

        status = "CLEAN" if clean else "STILL COLLIDING"
        log.info(f"  {label}: {status}, len={rep_len:.4f}, "
                 f"ratio={ratio:.4f} ({(1-ratio)*100:+.1f}%), "
                 f"{len(repaired)} wps")

    # Summary
    log.info(f"\n{'='*60}")
    log.info("Summary after collision repair:")
    log.info(f"{'Pair':<10} {'Dijk':>8} {'Repaired':>9} {'Ratio':>7} {'Wps':>5} {'Status':>10}")
    log.info("-" * 55)
    for r in gcs_data["results"]:
        if not r["success"]:
            log.info(f"{r['label']:<10} {'FAIL':>8}")
            continue
        d = r.get("dijkstra_path_length", 0)
        t = r.get("total_path_length", 0)
        ratio = r.get("improvement_ratio", 0)
        npts = len(r.get("waypoints", []))
        repaired = "Yes" if r.get("collision_repaired") else "No"
        log.info(f"{r['label']:<10} {d:>8.3f} {t:>9.3f} {ratio:>7.3f} {npts:>5d} {repaired:>10}")

    valid = [r for r in gcs_data["results"]
             if r["success"] and r.get("improvement_ratio", 0) > 0]
    if valid:
        mean_ratio = np.mean([r["improvement_ratio"] for r in valid])
        log.info(f"\nMean ratio: {mean_ratio:.4f} ({(1-mean_ratio)*100:+.1f}%)")

    # Save
    out_path = args.output or args.gcs_json
    with open(out_path, "w") as f:
        json.dump(gcs_data, f, indent=2)
    log.info(f"\nSaved to: {out_path}")


if __name__ == "__main__":
    main()
