"""
planner/pipeline.py - Panda 7-DOF 多方法规划管线

从 v2/examples/panda_planner.py 提取的核心逻辑：
- PandaGCSConfig: 管线配置
- grow_forest(): forest 生长（串行/并行）
- 邻接+连通分量构建
- Solver A: GCS SOCP (凸松弛 + rounding + refine)
- Solver B: Dijkstra on box graph + SOCP refine
- Solver C: Visibility Graph (seed configs + collision check)
- grow_and_prepare(): 共享前半段管线
- run_method_with_bridge(): 带桥接的求解调度
"""

import heapq
import math
import os
import logging
import threading
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

try:
    import cvxpy as cp
except ImportError:
    cp = None  # GCS solver 不可用

from aabb.robot import load_robot
from forest.scene import Scene
from forest.collision import CollisionChecker
from forest.models import BoxNode
from forest.connectivity import find_islands, bridge_islands, UnionFind
from forest.hier_aabb_tree import HierAABBTree, build_kd_partitions
from forest.safe_box_forest import SafeBoxForest
from planner.sbf_planner import SBFPlanner
from planner.models import SBFConfig, gmean_edge_length
from forest.coarsen import coarsen_forest

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════════════
# Geodesic helpers (periodic / torus topology)
# ═══════════════════════════════════════════════════════════════════════════

def _geo_diff(a: np.ndarray, b: np.ndarray,
              period: Optional[float]) -> np.ndarray:
    """Signed shortest difference b - a on torus (or plain diff)."""
    if period is None:
        return b - a
    half = period / 2.0
    return ((b - a) + half) % period - half


def _geo_dist(a: np.ndarray, b: np.ndarray,
              period: Optional[float]) -> float:
    """Geodesic distance between two configs."""
    return float(np.linalg.norm(_geo_diff(a, b, period)))


def _geo_path_cost(waypoints, period: Optional[float]) -> float:
    """Sum of geodesic segment lengths along a waypoint list."""
    return sum(_geo_dist(waypoints[i], waypoints[i + 1], period)
               for i in range(len(waypoints) - 1))


# ═══════════════════════════════════════════════════════════════════════════
# Config
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class PandaGCSConfig:
    seed: int = 0  # 0 → 运行时随机

    # Panda start/goal — 远距离构型
    q_start: List[float] = field(
        default_factory=lambda: [0.5, -1.2, 0.5, -2.5, 0.5, 0.8, 1.5])
    q_goal: List[float] = field(
        default_factory=lambda: [-2.0, 1.2, -1.8, -0.5, -2.0, 3.0, -1.8])

    # scene
    n_obstacles: int = 6
    workspace_radius: float = 0.85
    workspace_z_range: Tuple[float, float] = (0.0, 1.0)
    obs_size_range: Tuple[float, float] = (0.08, 0.25)

    # forest growth
    max_consecutive_miss: int = 20
    max_boxes: int = 500
    ffb_min_edge: float = 0.04
    guided_sample_ratio: float = 0.8
    boundary_expand_epsilon: float = 0.01
    parallel_grow: bool = False
    n_partitions_depth: int = 3
    parallel_workers: int = 4

    # island / bridge
    min_island_size: float = 0.0

    # GCS solver
    corridor_hops: int = 2

    # coarsen
    coarsen_max_rounds: int = 20

    # cache
    wait_cache_thread: bool = False

    # viz
    dpi: int = 140


# ═══════════════════════════════════════════════════════════════════════════
# Scene
# ═══════════════════════════════════════════════════════════════════════════

def build_panda_scene(
    rng: np.random.Generator,
    cfg: PandaGCSConfig,
    robot=None,
    q_start=None,
    q_goal=None,
    max_trials: int = 100,
) -> Scene:
    """生成 3D 随机障碍物场景, 确保起终点无碰撞."""
    for _ in range(max_trials):
        scene = Scene()
        for i in range(cfg.n_obstacles):
            r = rng.uniform(0.25 * cfg.workspace_radius,
                            0.85 * cfg.workspace_radius)
            theta = rng.uniform(-math.pi, math.pi)
            cx = r * math.cos(theta)
            cy = r * math.sin(theta)
            cz = rng.uniform(cfg.workspace_z_range[0] + 0.1,
                             cfg.workspace_z_range[1] - 0.1)
            hx = rng.uniform(*cfg.obs_size_range)
            hy = rng.uniform(*cfg.obs_size_range)
            hz = rng.uniform(*cfg.obs_size_range)
            scene.add_obstacle(
                min_point=[cx - hx, cy - hy, cz - hz],
                max_point=[cx + hx, cy + hy, cz + hz],
                name=f"obs_{i}",
            )
        if robot is not None:
            checker = CollisionChecker(robot=robot, scene=scene)
            if (checker.check_config_collision(q_start)
                    or checker.check_config_collision(q_goal)):
                continue
        return scene
    raise RuntimeError("Failed to build collision-free scene for Panda")


def make_planner_config(cfg: PandaGCSConfig) -> SBFConfig:
    """从 PandaGCSConfig 构建 SBFConfig."""
    return SBFConfig(
        max_iterations=999999,
        max_box_nodes=999999,
        guided_sample_ratio=cfg.guided_sample_ratio,
        segment_collision_resolution=0.05,
        connection_radius=1.5,
        verbose=False,
        forest_path=None,
        boundary_expand_epsilon=cfg.boundary_expand_epsilon,
    )


# ═══════════════════════════════════════════════════════════════════════════
# Parallel grow worker (top-level for pickle)
# ═══════════════════════════════════════════════════════════════════════════

def _grow_partition_worker(payload: Dict) -> Dict:
    """ProcessPool worker: 在单个 KD 分区内独立生长 boxes."""
    robot = payload["robot"]
    obstacles = payload["obstacles"]
    q_start = np.asarray(payload["q_start"], dtype=np.float64)
    q_goal = np.asarray(payload["q_goal"], dtype=np.float64)
    partition_intervals = payload["intervals"]
    max_boxes = int(payload["max_boxes"])
    max_miss = int(payload["max_miss"])
    seed_val = payload.get("seed")

    ndim = len(partition_intervals)
    rng = np.random.default_rng(seed_val)

    local_tree = HierAABBTree(robot, joint_limits=partition_intervals)
    obs_packed = local_tree._prepack_obstacles_c(obstacles)
    local_tree.warmup_fk_cache(max_depth=6)

    local_scene = Scene()
    local_scene._obstacles = list(obstacles)
    local_checker = CollisionChecker(robot, local_scene)

    lows = np.array([lo for lo, _ in partition_intervals], dtype=np.float64)
    highs = np.array([hi for _, hi in partition_intervals], dtype=np.float64)
    batch_size = 32

    local_boxes: Dict[int, Dict] = {}
    next_id = 0
    consec = 0

    def _sample_batch_local():
        configs = rng.uniform(lows, highs, size=(batch_size, ndim))
        collisions = local_checker.check_config_collision_batch(configs)
        return [configs[i] for i in range(batch_size) if not collisions[i]]

    seed_buffer: List[np.ndarray] = []

    for qs in [q_start, q_goal]:
        in_part = all(
            lo <= qs[d] <= hi
            for d, (lo, hi) in enumerate(partition_intervals))
        if in_part and not local_tree.is_occupied(qs):
            ffb = local_tree.find_free_box(
                qs, obstacles, mark_occupied=True, forest_box_id=next_id,
                obs_packed=obs_packed)
            if ffb is not None:
                vol = 1.0
                for lo, hi in ffb.intervals:
                    vol *= max(hi - lo, 0)
                local_boxes[next_id] = {
                    "joint_intervals": ffb.intervals,
                    "seed_config": qs.copy(),
                    "volume": vol,
                }
                next_id += 1

    while consec < max_miss:
        if len(local_boxes) >= max_boxes:
            break

        if not seed_buffer:
            seed_buffer = _sample_batch_local()
        if not seed_buffer:
            consec += 1
            continue
        q = seed_buffer.pop()

        if local_tree.is_occupied(q):
            consec += 1
            continue

        if not local_tree.can_expand(q, obs_packed=obs_packed):
            consec += 1
            continue

        ffb = local_tree.find_free_box(
            q, obstacles, mark_occupied=True, forest_box_id=next_id,
            obs_packed=obs_packed)
        if ffb is None:
            consec += 1
            continue

        vol = 1.0
        for lo, hi in ffb.intervals:
            vol *= max(hi - lo, 0)

        if ffb.absorbed_box_ids:
            for aid in ffb.absorbed_box_ids:
                local_boxes.pop(int(aid), None)

        local_boxes[next_id] = {
            "joint_intervals": ffb.intervals,
            "seed_config": q.copy(),
            "volume": vol,
        }
        next_id += 1
        consec = 0

    return {
        "partition_id": int(payload.get("partition_id", 0)),
        "boxes": [local_boxes[k] for k in sorted(local_boxes.keys())],
    }


# ═══════════════════════════════════════════════════════════════════════════
# Forest grow
# ═══════════════════════════════════════════════════════════════════════════

def grow_forest(
    planner: SBFPlanner,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    seed: int,
    max_miss: int = 20,
    ndim: int = 7,
    max_boxes: int = 1500,
    parallel_grow: bool = False,
    n_partitions_depth: int = 3,
    parallel_workers: int = 4,
):
    """生长 forest.

    终止条件:
      - 主条件: 连续 max_miss 个 seed 未命中
      - 安全上限: max_boxes 防止 7D 空间无限增长

    Returns:
        (boxes, forest, timing_detail)
    """
    rng = np.random.default_rng(seed)
    forest = planner._load_or_create_forest()
    forest.hier_tree = planner.hier_tree

    # FK 缓存预热
    t_warmup_start = time.perf_counter()
    n_warmed = planner.hier_tree.warmup_fk_cache(max_depth=6)
    t_warmup = (time.perf_counter() - t_warmup_start) * 1000
    print(f"    [warmup] FK cache: {n_warmed} nodes ({t_warmup:.0f} ms)")

    t_obs_pack_start = time.perf_counter()
    obs_packed = planner.hier_tree._prepack_obstacles_c(planner.obstacles)
    t_obs_pack = (time.perf_counter() - t_obs_pack_start) * 1000

    # ── 并行分区生长 ──
    if parallel_grow:
        t_par0 = time.perf_counter()
        partitions = build_kd_partitions(
            planner.joint_limits, n_partitions_depth)
        n_parts = len(partitions)
        per_part_boxes = max(10, max_boxes // n_parts)
        if parallel_workers > 0:
            workers = parallel_workers
        else:
            workers = max(1, (os.cpu_count() or 4))
        workers = min(workers, n_parts)
        print(f"    [parallel grow] {n_parts} partitions, "
              f"{workers} workers, {per_part_boxes} boxes/part")

        payloads = []
        for pid, part_ivs in enumerate(partitions):
            payloads.append({
                "robot": planner.robot,
                "obstacles": planner.obstacles,
                "q_start": q_start,
                "q_goal": q_goal,
                "intervals": part_ivs,
                "partition_id": pid,
                "max_boxes": per_part_boxes,
                "max_miss": max_miss,
                "seed": int(rng.integers(0, 2**31 - 1)),
            })

        local_results: List[Dict] = []
        try:
            with ProcessPoolExecutor(max_workers=workers) as ex:
                futs = [ex.submit(_grow_partition_worker, p) for p in payloads]
                for fut in as_completed(futs):
                    local_results.append(fut.result())
        except Exception as e:
            print(f"    [parallel grow] ProcessPool failed ({e}), "
                  f"fallback to serial")
            for p in payloads:
                local_results.append(_grow_partition_worker(p))

        partition_box_ids = forest.merge_partition_forests(local_results)
        t_par_ms = (time.perf_counter() - t_par0) * 1000

        n_total = sum(len(ids) for ids in partition_box_ids.values())
        print(f"    [parallel grow] merged {n_total} boxes, "
              f"{t_par_ms:.0f} ms total")

        timing = dict(
            warmup_ms=t_warmup,
            parallel_grow_ms=t_par_ms,
            n_partitions=n_parts,
            n_workers=workers,
        )
        boxes = {}
        for bid, b in forest.boxes.items():
            boxes[bid] = BoxNode(
                node_id=b.node_id,
                joint_intervals=[tuple(iv) for iv in b.joint_intervals],
                seed_config=b.seed_config.copy(), volume=b.volume)
        return boxes, forest, timing

    # ── 串行生长 ──
    t_serial_scope_start = time.perf_counter()
    t_sample = 0.0
    t_is_occ = 0.0
    t_probe = 0.0
    t_ffb = 0.0
    t_add = 0.0
    t_vol = 0.0
    n_sample_calls = 0
    n_is_occ_calls = 0
    n_probe_calls = 0
    n_probe_reject = 0
    n_ffb_calls = 0
    n_ffb_none = 0
    n_absorbed = 0

    intervals = planner.joint_limits
    lows = np.array([lo for lo, _ in intervals], dtype=np.float64)
    highs = np.array([hi for _, hi in intervals], dtype=np.float64)
    guided_ratio = getattr(planner.config, 'guided_sample_ratio', 0.8)
    has_hier_tree = hasattr(planner, 'hier_tree') and planner.hier_tree is not None
    batch_size = 32

    def _sample_batch():
        rolls = rng.uniform(size=batch_size)
        configs = np.empty((batch_size, ndim), dtype=np.float64)
        if has_hier_tree:
            guided_mask = rolls < guided_ratio
        else:
            guided_mask = np.zeros(batch_size, dtype=bool)
        uniform_mask = ~guided_mask

        n_uni = int(uniform_mask.sum())
        if n_uni > 0:
            configs[uniform_mask] = rng.uniform(lows, highs, size=(n_uni, ndim))

        n_guided = int(guided_mask.sum())
        if n_guided > 0:
            guided_samples = planner.hier_tree.sample_unoccupied_seed_batch(
                n_guided, rng)
            n_valid = len(guided_samples)
            guided_idxs = np.flatnonzero(guided_mask)
            if n_valid > 0:
                configs[guided_idxs[:n_valid]] = guided_samples
            # fill remaining guided slots with uniform samples
            n_fill = n_guided - n_valid
            if n_fill > 0:
                configs[guided_idxs[n_valid:]] = rng.uniform(
                    lows, highs, size=(n_fill, ndim))

        collisions = planner.collision_checker.check_config_collision_batch(
            configs)
        return [configs[i] for i in range(batch_size) if not collisions[i]]

    seed_buffer: List[np.ndarray] = []

    # boundary expand state (BFS with unidirectional excluded_faces)
    from collections import deque
    bfs_queue = deque()  # (box, excluded_faces_frozenset)
    n_boundary_attempts = 0
    n_boundary_ok = 0
    t_boundary = 0.0

    # ── 清除缓存树中残留的旧 forest 占用标记 ──
    # 当 HierAABBTree 从磁盘缓存加载时, 其 occupied / forest_id / subtree_occ
    # 仍保留上一次 forest 实例的状态。但当前 forest 是全新创建, 这些旧标记
    # 会导致: (1) is_occupied(q_start/q_goal) 误判为已占用, 跳过 anchor 创建;
    #          (2) guided sampling 规避了旧占用区, 降低覆盖率。
    _store = planner.hier_tree._store
    _stale_fids = _store.forest_ids_array()
    _stale_idxs = np.flatnonzero(_stale_fids >= 0)
    if len(_stale_idxs) > 0:
        for _si in _stale_idxs:
            _store.unmark_occupied(int(_si))
        logger.debug("[stale clear] cleared %d stale occupied nodes",
                     len(_stale_idxs))

    t_anchor_start = time.perf_counter()
    for qs in [q_start, q_goal]:
        if not planner.hier_tree.is_occupied(qs):
            nid = forest.allocate_id()
            # Anchors are critical — use smaller min_edge to resolve
            # conservative interval AABB near obstacles.
            _anchor_min_edge = getattr(
                planner.config, 'ffb_min_edge', 0.05) * 0.1
            ffb = planner.hier_tree.find_free_box(
                qs, planner.obstacles, mark_occupied=True, forest_box_id=nid,
                obs_packed=obs_packed, min_edge_length=_anchor_min_edge)
            if not ffb:
                # Fallback: 当min_edge仍不足时,
                # 创建一个以 qs 为中心的小 box (采样验证碰撞安全).
                _anchor_halfwidth = 0.02
                _anchor_ivs = []
                for d in range(len(qs)):
                    lo_d, hi_d = planner.joint_limits[d]
                    _lo = max(qs[d] - _anchor_halfwidth, lo_d)
                    _hi = min(qs[d] + _anchor_halfwidth, hi_d)
                    _anchor_ivs.append((_lo, _hi))
                if not planner.collision_checker.check_box_collision(
                        _anchor_ivs):
                    ffb_intervals = _anchor_ivs
                elif not planner.collision_checker.check_box_collision_sampling(
                        _anchor_ivs, n_samples=50):
                    # interval AABB conservative → sampling says safe
                    ffb_intervals = _anchor_ivs
                else:
                    ffb_intervals = None
                if ffb_intervals is not None:
                    class _FfbLike:
                        def __init__(self, ivs):
                            self.intervals = ivs
                    ffb = _FfbLike(ffb_intervals)
                    logger.debug("[anchor] fallback small box at %s",
                                 np.array2string(qs, precision=2))
            if ffb:
                vol = 1.0
                for lo, hi in ffb.intervals:
                    vol *= max(hi - lo, 0)
                anchor_box = BoxNode(
                    node_id=nid, joint_intervals=ffb.intervals,
                    seed_config=qs.copy(), volume=vol)
                forest.add_box_direct(anchor_box)
                _in = anchor_box.contains(qs)
                logger.debug("[anchor] created box %d, contains=%s, vol=%.2e",
                             nid, _in, vol)
                # 把 anchor box 也加入 BFS, 使始末点参与边界拓展
                bfs_queue.append((anchor_box, frozenset()))
            else:
                logger.warning("[anchor] FAILED for %s, reason=%s",
                               np.array2string(qs, precision=2),
                               planner.hier_tree._last_ffb_none_reason)
    t_anchor = (time.perf_counter() - t_anchor_start) * 1000

    t_serial_prep = (time.perf_counter() - t_serial_scope_start) * 1000

    consec = 0
    t0 = time.perf_counter()
    terminated_by = "miss"
    while consec < max_miss:
        if forest.n_boxes >= max_boxes:
            terminated_by = "max_boxes"
            break

        # ── 优先处理 BFS 队列中的边界扩张 ──
        if bfs_queue:
            _ts = time.perf_counter()
            bfs_box, excluded = bfs_queue.popleft()
            seeds = planner._generate_boundary_seeds(bfs_box, rng, excluded)
            t_boundary += time.perf_counter() - _ts
            n_boundary_attempts += len(seeds)
            for dim, side, q_seed in seeds:
                if forest.n_boxes >= max_boxes:
                    break
                _ts_occ = time.perf_counter()
                occ_bfs = planner.hier_tree.is_occupied(q_seed)
                t_is_occ += time.perf_counter() - _ts_occ
                n_is_occ_calls += 1
                if occ_bfs:
                    continue
                nid = forest.allocate_id()
                _ts2 = time.perf_counter()
                ffb = planner.hier_tree.find_free_box(
                    q_seed, planner.obstacles, mark_occupied=True,
                    forest_box_id=nid, obs_packed=obs_packed)
                t_ffb += time.perf_counter() - _ts2
                n_ffb_calls += 1
                if ffb is None:
                    n_ffb_none += 1
                    continue
                _ts3 = time.perf_counter()
                vol = 1.0
                for lo, hi in ffb.intervals:
                    vol *= max(hi - lo, 0)
                new_box = BoxNode(node_id=nid,
                                  joint_intervals=ffb.intervals,
                                  seed_config=q_seed.copy(), volume=vol)
                if ffb.absorbed_box_ids:
                    n_absorbed += len(ffb.absorbed_box_ids)
                    forest.remove_boxes_no_adjacency(ffb.absorbed_box_ids)
                forest.add_box_no_adjacency(new_box)
                t_add += time.perf_counter() - _ts3
                n_boundary_ok += 1
                # 单向性：排除反向面
                child_excluded = excluded | frozenset({(dim, 1 - side)})
                bfs_queue.append((new_box, child_excluded))
            continue  # BFS 轮次不计入 consec

        # ── 正常随机采样 ──
        _ts = time.perf_counter()
        if not seed_buffer:
            seed_buffer = _sample_batch()
            n_sample_calls += 1
        if not seed_buffer:
            t_sample += time.perf_counter() - _ts
            consec += 1
            continue
        q = seed_buffer.pop()
        t_sample += time.perf_counter() - _ts

        _ts = time.perf_counter()
        occ = planner.hier_tree.is_occupied(q)
        t_is_occ += time.perf_counter() - _ts
        n_is_occ_calls += 1

        if occ:
            consec += 1
            continue

        _ts = time.perf_counter()
        can = planner.hier_tree.can_expand(q, obs_packed=obs_packed)
        t_probe += time.perf_counter() - _ts
        n_probe_calls += 1

        if not can:
            n_probe_reject += 1
            consec += 1
            continue

        nid = forest.allocate_id()
        _ts = time.perf_counter()
        ffb = planner.hier_tree.find_free_box(
            q, planner.obstacles, mark_occupied=True, forest_box_id=nid,
            obs_packed=obs_packed)
        t_ffb += time.perf_counter() - _ts
        n_ffb_calls += 1

        if ffb is None:
            n_ffb_none += 1
            consec += 1
            continue

        _ts = time.perf_counter()
        vol = 1.0
        for lo, hi in ffb.intervals:
            vol *= max(hi - lo, 0)
        t_vol += time.perf_counter() - _ts

        _ts = time.perf_counter()
        box = BoxNode(node_id=nid, joint_intervals=ffb.intervals,
                      seed_config=q.copy(), volume=vol)
        if ffb.absorbed_box_ids:
            n_absorbed += len(ffb.absorbed_box_ids)
            forest.remove_boxes_no_adjacency(ffb.absorbed_box_ids)
        forest.add_box_no_adjacency(box)
        t_add += time.perf_counter() - _ts

        # 新 box 进入 BFS 队列
        bfs_queue.append((box, frozenset()))

        consec = 0
        if forest.n_boxes % 100 == 0:
            elapsed = time.perf_counter() - t0
            print(f"    [grow] {forest.n_boxes} boxes, {elapsed:.1f}s")

    elapsed = time.perf_counter() - t0
    print(f"    [grow] terminated by {terminated_by}: "
          f"{forest.n_boxes} boxes, {elapsed:.1f}s")

    # 后处理: 双向 stepping stone 铺路 (已跳过)
    _period = getattr(planner, '_period', None)
    if False:
        _ts_connect = time.perf_counter()
        n_gap_fill = 0
        _gap_rounds = 3

        def _box_center(bx):
            return np.array([(lo + hi) / 2
                             for lo, hi in bx.joint_intervals])

        def _find_st_bids(snap):
            sb = tb = None
            for bid, bx in snap.items():
                if sb is None and all(
                        bx.joint_intervals[d][0] - 1e-12 <= q_start[d]
                        <= bx.joint_intervals[d][1] + 1e-12
                        for d in range(ndim)):
                    sb = bid
                if tb is None and all(
                        bx.joint_intervals[d][0] - 1e-12 <= q_goal[d]
                        <= bx.joint_intervals[d][1] + 1e-12
                        for d in range(ndim)):
                    tb = bid
                if sb is not None and tb is not None:
                    return sb, tb
            return sb, tb

        def _grow_one_box(q_cfg):
            if planner.collision_checker.check_config_collision(q_cfg):
                return None
            nid2 = forest.allocate_id()
            ffb2 = planner.hier_tree.find_free_box(
                q_cfg, planner.obstacles, mark_occupied=True,
                forest_box_id=nid2, obs_packed=obs_packed)
            if ffb2 is None:
                return None
            vol2 = 1.0
            for lo2, hi2 in ffb2.intervals:
                vol2 *= max(hi2 - lo2, 0)
            box2 = BoxNode(node_id=nid2, joint_intervals=ffb2.intervals,
                           seed_config=q_cfg.copy(), volume=vol2)
            if ffb2.absorbed_box_ids:
                forest.remove_boxes(ffb2.absorbed_box_ids)
            forest.add_box_direct(box2)
            return box2

        def _pave_toward(start_c, end_c, max_steps=300):
            if _period is not None:
                half = _period / 2.0
                direction = ((end_c - start_c) + half) % _period - half
            else:
                direction = end_c - start_c
            dist = np.linalg.norm(direction)
            if dist < 1e-6:
                return 0
            direction /= dist
            pos = start_c.copy()
            n_placed = 0
            step_idx = 0
            while step_idx < max_steps:
                if _period is not None:
                    diff_r = ((end_c - pos) + half) % _period - half
                    d_remain = np.linalg.norm(diff_r)
                else:
                    d_remain = np.linalg.norm(end_c - pos)
                if d_remain < 0.05:
                    break
                placed = False
                for attempt in range(15):
                    if attempt == 0:
                        q_try = pos.copy()
                    else:
                        q_try = pos + rng.normal(0, 0.05, size=ndim)
                    for d in range(ndim):
                        q_try[d] = np.clip(q_try[d],
                                           planner.joint_limits[d][0],
                                           planner.joint_limits[d][1])
                    box_new = _grow_one_box(q_try)
                    if box_new is not None:
                        n_placed += 1
                        min_edge = min(hi - lo
                                       for lo, hi in box_new.joint_intervals)
                        step_size = max(0.01, min(min_edge * 0.4, 0.1))
                        pos = pos + direction * step_size
                        placed = True
                        break
                if not placed:
                    pos = pos + direction * 0.03
                step_idx += 1
            return n_placed

        _gr = 0  # track rounds for reporting
        for _gr in range(_gap_rounds):
            _snap = {}
            for bid, b in forest.boxes.items():
                _snap[bid] = BoxNode(
                    node_id=b.node_id,
                    joint_intervals=[tuple(iv) for iv in b.joint_intervals],
                    seed_config=b.seed_config.copy(), volume=b.volume)
            _, _uf, _isls = _build_adjacency_and_islands(_snap, period=_period)

            _src_bid, _tgt_bid = _find_st_bids(_snap)
            if _src_bid is None or _tgt_bid is None:
                break
            if _uf.same(_src_bid, _tgt_bid):
                break

            for _st_bid, _qs in [(_src_bid, q_start), (_tgt_bid, q_goal)]:
                _st_isl = set()
                for isl in _isls:
                    if _st_bid in isl:
                        _st_isl = isl
                        break
                _st_c = _box_center(_snap[_st_bid])

                _best_d = float('inf')
                _tgt_c = None
                for bid, bx in _snap.items():
                    if bid in _st_isl:
                        continue
                    mc = _box_center(bx)
                    if _period is not None:
                        half = _period / 2.0
                        _diff = ((mc - _st_c) + half) % _period - half
                        dd = np.linalg.norm(_diff)
                    else:
                        dd = np.linalg.norm(mc - _st_c)
                    if dd < _best_d:
                        _best_d = dd
                        _tgt_c = mc
                if _tgt_c is None:
                    continue

                n1 = _pave_toward(_st_c, _tgt_c, max_steps=200)
                n2 = _pave_toward(_tgt_c, _st_c, max_steps=200)
                n_gap_fill += n1 + n2

        gap_ms = (time.perf_counter() - _ts_connect) * 1000
        t_boundary += (time.perf_counter() - _ts_connect)
        if n_gap_fill > 0:
            print(f"    [gap fill] +{n_gap_fill} boxes in {_gr + 1} rounds, "
                  f"total={forest.n_boxes} ({gap_ms:.0f}ms)")
        elif _gr > 0:
            print(f"    [gap fill] s/t disconnected, 0 gap boxes "
                  f"({gap_ms:.0f}ms)")

    t_export_start = time.perf_counter()
    boxes = {}
    for bid, b in forest.boxes.items():
        boxes[bid] = BoxNode(
            node_id=b.node_id,
            joint_intervals=[tuple(iv) for iv in b.joint_intervals],
            seed_config=b.seed_config.copy(), volume=b.volume)
    t_export = (time.perf_counter() - t_export_start) * 1000

    # 详细计时报告
    timing = dict(
        warmup_ms=t_warmup,
        obs_pack_ms=t_obs_pack,
        seed_anchor_ms=t_anchor,
        serial_prep_ms=t_serial_prep,
        sample_ms=t_sample * 1000,
        boundary_ms=t_boundary * 1000,
        is_occupied_ms=t_is_occ * 1000,
        probe_ms=t_probe * 1000,
        find_free_box_ms=t_ffb * 1000,
        volume_check_ms=t_vol * 1000,
        add_box_ms=t_add * 1000,
        export_boxes_ms=t_export,
        overhead_ms=(elapsed - t_sample - t_boundary - t_is_occ - t_probe
                     - t_ffb - t_vol - t_add) * 1000,
        n_sample_calls=n_sample_calls,
        n_is_occ_calls=n_is_occ_calls,
        n_probe_calls=n_probe_calls,
        n_probe_reject=n_probe_reject,
        n_ffb_calls=n_ffb_calls,
        n_ffb_none=n_ffb_none,
        n_absorbed=n_absorbed,
    )
    print(f"    [grow detail]")
    print(f"      warmup_fk       : {timing['warmup_ms']:8.1f} ms  "
          f"({n_warmed} nodes)")
    print(f"      obs_pack        : {timing['obs_pack_ms']:8.1f} ms")
    print(f"      seed_anchor     : {timing['seed_anchor_ms']:8.1f} ms")
    print(f"      serial_prep     : {timing['serial_prep_ms']:8.1f} ms")
    print(f"      sample_batch    : {timing['sample_ms']:8.1f} ms  "
          f"({n_sample_calls} batches)")
    print(f"      boundary_expand : {timing['boundary_ms']:8.1f} ms  "
          f"({n_boundary_ok}/{n_boundary_attempts} ok)")
    print(f"      is_occupied     : {timing['is_occupied_ms']:8.1f} ms  "
          f"({n_is_occ_calls} calls)")
    print(f"      can_expand      : {timing['probe_ms']:8.1f} ms  "
          f"({n_probe_calls} calls, {n_probe_reject} rejected)")
    print(f"      find_free_box   : {timing['find_free_box_ms']:8.1f} ms  "
          f"({n_ffb_calls} calls, {n_ffb_none} none)")
    print(f"      volume_check    : {timing['volume_check_ms']:8.1f} ms")
    print(f"      add_box         : {timing['add_box_ms']:8.1f} ms  "
          f"({n_absorbed} absorbed)")
    print(f"      export_boxes    : {timing['export_boxes_ms']:8.1f} ms")
    print(f"      overhead/other  : {timing['overhead_ms']:8.1f} ms")
    return boxes, forest, timing


# ═══════════════════════════════════════════════════════════════════════════
# Shared infrastructure: adjacency + islands
# ═══════════════════════════════════════════════════════════════════════════

def _build_adjacency_and_islands(boxes, period=None):
    """O(N²) overlap → adjacency dict + UnionFind + islands.

    当 period 不为 None 时, 使用周期边界检测 overlap (π ↔ -π 视为相邻).
    """
    ids = list(boxes.keys())
    n = len(ids)
    adj: Dict[int, Set[int]] = {bid: set() for bid in ids}
    uf = UnionFind(ids)

    if n < 2:
        return adj, uf, uf.components()

    eps = 1e-9

    def _overlap_1d(lo1, hi1, lo2, hi2):
        if hi1 >= lo2 - eps and hi2 >= lo1 - eps:
            return True
        if period is None:
            return False
        # 周期维度: 每一维独立考虑 box2 的平移，避免“所有维度同时平移”误判
        if hi1 >= (lo2 + period) - eps and (hi2 + period) >= lo1 - eps:
            return True
        if hi1 >= (lo2 - period) - eps and (hi2 - period) >= lo1 - eps:
            return True
        return False

    for i in range(n):
        bi = ids[i]
        iv_i = boxes[bi].joint_intervals
        for j in range(i + 1, n):
            bj = ids[j]
            iv_j = boxes[bj].joint_intervals

            ok = True
            for d in range(len(iv_i)):
                lo1, hi1 = iv_i[d]
                lo2, hi2 = iv_j[d]
                if not _overlap_1d(lo1, hi1, lo2, hi2):
                    ok = False
                    break

            if ok:
                adj[bi].add(bj)
                adj[bj].add(bi)
                uf.union(bi, bj)

    islands = uf.components()
    return adj, uf, islands


def _add_bridge_to_adj(adj, bridge_edges, uf=None):
    """把 bridge edges 加入已有邻接图."""
    for e in bridge_edges:
        s, t = e.source_box_id, e.target_box_id
        if s in adj and t in adj:
            adj[s].add(t)
            adj[t].add(s)
            if uf is not None:
                uf.union(s, t)


# ═══════════════════════════════════════════════════════════════════════════
# GCS helpers (from gcs_planner_2dof.py)
# ═══════════════════════════════════════════════════════════════════════════

def find_box_containing(
    q: np.ndarray,
    boxes: Dict[int, BoxNode],
) -> Optional[int]:
    """返回包含 q 的 box id, 若无则返回距离最近的。"""
    best_id, best_d = None, float("inf")
    for bid, b in boxes.items():
        if b.contains(q):
            return bid
        d = b.distance_to_config(q)
        if d < best_d:
            best_d = d
            best_id = bid
    return best_id


def extract_connected_subgraph(
    adj: Dict[int, Set[int]],
    source_id: int,
    target_id: int,
) -> Optional[Set[int]]:
    """BFS 找 source 所在连通分量, 若 target 不在其中返回 None."""
    visited = set()
    queue = [source_id]
    visited.add(source_id)
    while queue:
        u = queue.pop(0)
        for v in adj.get(u, set()):
            if v not in visited:
                visited.add(v)
                queue.append(v)
    if target_id not in visited:
        return None
    return visited


def corridor_prune(
    adj: Dict[int, Set[int]],
    source_id: int,
    target_id: int,
    hops: int = 2,
) -> Optional[Set[int]]:
    """保留最短路径 ± hops 跳范围内的顶点 (corridor 子图)."""
    from collections import deque

    parent: Dict[int, Optional[int]] = {source_id: None}
    queue = deque([source_id])
    while queue:
        u = queue.popleft()
        if u == target_id:
            break
        for v in adj.get(u, set()):
            if v not in parent:
                parent[v] = u
                queue.append(v)
    if target_id not in parent:
        return None

    path_verts: Set[int] = set()
    cur = target_id
    while cur is not None:
        path_verts.add(cur)
        cur = parent[cur]

    corridor: Set[int] = set(path_verts)
    for pv in path_verts:
        frontier = {pv}
        for _ in range(hops):
            next_frontier: Set[int] = set()
            for u in frontier:
                for v in adj.get(u, set()):
                    if v not in corridor:
                        next_frontier.add(v)
            corridor |= next_frontier
            frontier = next_frontier
            if not frontier:
                break
    return corridor


def _deperspective_point(xv_val, phi_val, ei, boxes, v, ndim):
    """从 perspective 变量恢复实际坐标并 clip 到 box."""
    ph = max(phi_val[ei], 1e-10)
    pt = xv_val[ei] / ph
    box_v = boxes[v]
    for d in range(ndim):
        lo, hi = box_v.joint_intervals[d]
        pt[d] = np.clip(pt[d], lo, hi)
    return pt


def _round_greedy(
    phi_val, xu_val, xv_val,
    edges, out_edges, id2idx,
    boxes, source_id, target_id, q_start, q_goal, ndim, nv,
) -> Optional[Tuple[bool, float, List[np.ndarray], List[int]]]:
    """Greedy flow tracing rounding."""
    waypoints = [q_start.copy()]
    box_seq = [source_id]
    current = source_id
    visited_edges = set()
    max_steps = nv + 10

    for _ in range(max_steps):
        if current == target_id:
            break
        ci = id2idx[current]
        best_ei, best_phi = -1, -1e-9
        for ei in out_edges[ci]:
            if ei not in visited_edges and phi_val[ei] > best_phi:
                best_phi = phi_val[ei]
                best_ei = ei
        if best_ei < 0 or best_phi < 1e-8:
            break
        visited_edges.add(best_ei)
        u, v = edges[best_ei]
        pt_v = _deperspective_point(xv_val, phi_val, best_ei, boxes, v, ndim)
        waypoints.append(pt_v.copy())
        box_seq.append(v)
        current = v

    if current != target_id:
        return None

    waypoints[-1] = q_goal.copy()
    total_cost = sum(
        float(np.linalg.norm(waypoints[i + 1] - waypoints[i]))
        for i in range(len(waypoints) - 1))
    return True, total_cost, waypoints, box_seq


def _round_dfs(
    phi_val, xu_val, xv_val,
    edges, out_edges, id2idx,
    boxes, source_id, target_id, q_start, q_goal, ndim, nv,
    phi_threshold: float = 1e-4,
) -> Optional[Tuple[bool, float, List[np.ndarray], List[int]]]:
    """DFS with backtracking rounding."""
    sorted_out: List[List[int]] = [[] for _ in range(nv)]
    for vi in range(nv):
        eis = [ei for ei in out_edges[vi] if phi_val[ei] > phi_threshold]
        eis.sort(key=lambda ei: -phi_val[ei])
        sorted_out[vi] = eis

    src_idx = id2idx[source_id]
    stack = [(source_id, 0, {source_id})]
    path_edges: List[int] = []

    while stack:
        cur, ei_pos, visited = stack[-1]
        ci = id2idx[cur]
        if cur == target_id:
            break
        found_next = False
        while ei_pos < len(sorted_out[ci]):
            ei = sorted_out[ci][ei_pos]
            ei_pos += 1
            stack[-1] = (cur, ei_pos, visited)
            u, v = edges[ei]
            if v not in visited:
                path_edges.append(ei)
                new_visited = visited | {v}
                stack.append((v, 0, new_visited))
                found_next = True
                break
        if not found_next:
            stack.pop()
            if path_edges:
                path_edges.pop()

    if not stack or stack[-1][0] != target_id:
        return None

    waypoints = [q_start.copy()]
    box_seq = [source_id]
    for ei in path_edges:
        u, v = edges[ei]
        pt_v = _deperspective_point(xv_val, phi_val, ei, boxes, v, ndim)
        waypoints.append(pt_v.copy())
        box_seq.append(v)
    waypoints[-1] = q_goal.copy()
    total_cost = sum(
        float(np.linalg.norm(waypoints[i + 1] - waypoints[i]))
        for i in range(len(waypoints) - 1))
    return True, total_cost, waypoints, box_seq


def _round_random_once(
    phi_val, xu_val, xv_val,
    edges, out_edges, id2idx,
    boxes, source_id, target_id, q_start, q_goal, ndim, nv,
    rng: np.random.Generator,
    phi_threshold: float = 1e-4,
    noise_scale: float = 0.3,
) -> Optional[Tuple[bool, float, List[np.ndarray], List[int]]]:
    """Randomized DFS rounding."""
    sorted_out: List[List[int]] = [[] for _ in range(nv)]
    for vi in range(nv):
        eis = [ei for ei in out_edges[vi] if phi_val[ei] > phi_threshold]
        if not eis:
            sorted_out[vi] = []
            continue
        phi_arr = np.array([phi_val[ei] for ei in eis])
        noise = rng.uniform(0, noise_scale * phi_arr.max(), size=len(eis))
        order = np.argsort(-(phi_arr + noise))
        sorted_out[vi] = [eis[i] for i in order]

    stack = [(source_id, 0, {source_id})]
    path_edges: List[int] = []

    while stack:
        cur, ei_pos, visited = stack[-1]
        ci = id2idx[cur]
        if cur == target_id:
            break
        found_next = False
        while ei_pos < len(sorted_out[ci]):
            ei = sorted_out[ci][ei_pos]
            ei_pos += 1
            stack[-1] = (cur, ei_pos, visited)
            u, v = edges[ei]
            if v not in visited:
                path_edges.append(ei)
                stack.append((v, 0, visited | {v}))
                found_next = True
                break
        if not found_next:
            stack.pop()
            if path_edges:
                path_edges.pop()

    if not stack or stack[-1][0] != target_id:
        return None

    waypoints = [q_start.copy()]
    box_seq = [source_id]
    for ei in path_edges:
        u, v = edges[ei]
        pt_v = _deperspective_point(xv_val, phi_val, ei, boxes, v, ndim)
        waypoints.append(pt_v.copy())
        box_seq.append(v)
    waypoints[-1] = q_goal.copy()
    total_cost = sum(
        float(np.linalg.norm(waypoints[i + 1] - waypoints[i]))
        for i in range(len(waypoints) - 1))
    return True, total_cost, waypoints, box_seq


# ═══════════════════════════════════════════════════════════════════════════
# Solver A: GCS SOCP (凸松弛 + rounding + refine)
# ═══════════════════════════════════════════════════════════════════════════

def solve_gcs(
    boxes: Dict[int, BoxNode],
    adj: Dict[int, Set[int]],
    source_id: int,
    target_id: int,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    ndim: int = 2,
    corridor_hops: int = 2,
) -> Tuple[bool, float, List[np.ndarray], List[int]]:
    """GCS 凸松弛求最短路径 (Marcucci et al. 2023)."""
    if cp is None:
        raise ImportError("cvxpy is required for GCS solver")

    reachable = corridor_prune(adj, source_id, target_id, hops=corridor_hops)
    if reachable is None:
        print(f"    [GCS] source {source_id} and target {target_id} "
              f"are disconnected")
        return False, float("inf"), [], []

    sub_adj: Dict[int, Set[int]] = {
        u: adj[u] & reachable for u in reachable}

    edges = []
    edge_set = set()
    for u, nbrs in sub_adj.items():
        for v in nbrs:
            if (u, v) not in edge_set:
                edges.append((u, v))
                edge_set.add((u, v))
            if (v, u) not in edge_set:
                edges.append((v, u))
                edge_set.add((v, u))

    ne = len(edges)
    if ne == 0:
        return False, float("inf"), [], []

    box_ids = sorted(reachable)
    nv = len(box_ids)
    print(f"    [GCS] subgraph: {nv} vertices, {ne} directed edges")
    id2idx = {bid: i for i, bid in enumerate(box_ids)}
    src_idx = id2idx[source_id]
    tgt_idx = id2idx[target_id]

    phi = cp.Variable(ne, nonneg=True)
    xu = cp.Variable((ne, ndim))
    xv = cp.Variable((ne, ndim))

    constraints = [phi <= 1.0]

    out_edges_list = [[] for _ in range(nv)]
    in_edges_list = [[] for _ in range(nv)]
    for ei, (u, v) in enumerate(edges):
        out_edges_list[id2idx[u]].append(ei)
        in_edges_list[id2idx[v]].append(ei)

    for vi in range(nv):
        flow_out = (cp.sum(phi[out_edges_list[vi]])
                    if out_edges_list[vi] else 0.0)
        flow_in = (cp.sum(phi[in_edges_list[vi]])
                   if in_edges_list[vi] else 0.0)
        if vi == src_idx:
            constraints.append(flow_out - flow_in == 1.0)
        elif vi == tgt_idx:
            constraints.append(flow_out - flow_in == -1.0)
        else:
            constraints.append(flow_out - flow_in == 0.0)

    for ei, (u, v) in enumerate(edges):
        box_u = boxes[u]
        box_v = boxes[v]
        for d in range(ndim):
            lo_u, hi_u = box_u.joint_intervals[d]
            lo_v, hi_v = box_v.joint_intervals[d]
            constraints.append(xu[ei, d] >= lo_u * phi[ei])
            constraints.append(xu[ei, d] <= hi_u * phi[ei])
            constraints.append(xv[ei, d] >= lo_v * phi[ei])
            constraints.append(xv[ei, d] <= hi_v * phi[ei])

    if out_edges_list[src_idx]:
        constraints.append(
            cp.sum(xu[out_edges_list[src_idx], :], axis=0) == q_start)

    if in_edges_list[tgt_idx]:
        constraints.append(
            cp.sum(xv[in_edges_list[tgt_idx], :], axis=0) == q_goal)

    for vi in range(nv):
        if vi == src_idx or vi == tgt_idx:
            continue
        if in_edges_list[vi] and out_edges_list[vi]:
            constraints.append(
                cp.sum(xv[in_edges_list[vi], :], axis=0)
                == cp.sum(xu[out_edges_list[vi], :], axis=0))

    cost_terms = [cp.norm(xu[ei, :] - xv[ei, :], 2) for ei in range(ne)]
    objective = cp.Minimize(cp.sum(cost_terms))

    prob = cp.Problem(objective, constraints)

    for solver_name, solver_kwargs in [
        ("CLARABEL", dict(solver=cp.CLARABEL, verbose=False)),
        ("SCS", dict(solver=cp.SCS, verbose=False, max_iters=20000,
                     eps=1e-6)),
    ]:
        prob.solve(**solver_kwargs)
        if prob.status not in ("optimal", "optimal_inaccurate"):
            print(f"    [GCS] {solver_name}: {prob.status}")
            continue
        print(f"    [GCS] {solver_name}: {prob.status}, "
              f"obj={prob.value:.4f}")

        rounding_args = (
            phi.value, xu.value, xv.value,
            edges, out_edges_list, id2idx,
            boxes, source_id, target_id, q_start, q_goal, ndim, nv,
        )
        candidates = []

        for round_fn, round_name in [
            (_round_greedy, "greedy"),
            (_round_dfs, "DFS"),
        ]:
            result = round_fn(*rounding_args)
            if result is not None:
                candidates.append((round_name, result))

        for trial in range(20):
            rng_trial = np.random.default_rng(trial * 7 + 13)
            result = _round_random_once(*rounding_args, rng=rng_trial)
            if result is not None:
                candidates.append((f"rand{trial}", result))

        if not candidates:
            print(f"    [GCS] {solver_name}: all rounding failed")
            continue

        candidates.sort(key=lambda x: x[1][1])
        best_result = None
        best_cost = float("inf")
        best_name = ""

        for name, (ok, raw_cost, wps, bseq) in candidates[:5]:
            # build per-waypoint bounds from box sequence
            _wp_bounds = []
            for idx_wp, bid in enumerate(bseq):
                if bid in boxes:
                    box = boxes[bid]
                    lo = np.array([lo_d for lo_d, _ in box.joint_intervals])
                    hi = np.array([hi_d for _, hi_d in box.joint_intervals])
                    _wp_bounds.append((lo, hi))
                else:
                    # fallback: no constraint
                    _wp_bounds.append(
                        (np.full(ndim, -1e6), np.full(ndim, 1e6)))
            # pad if wps longer than bseq
            while len(_wp_bounds) < len(wps):
                _wp_bounds.append(_wp_bounds[-1] if _wp_bounds
                                  else (np.full(ndim, -1e6), np.full(ndim, 1e6)))
            wps_r, refined_cost = _refine_path_in_boxes(
                wps, _wp_bounds, q_start, q_goal, ndim)
            if refined_cost < best_cost:
                best_cost = refined_cost
                best_result = (ok, refined_cost, wps_r, bseq)
                best_name = name

        if best_result is not None:
            print(f"    [GCS] best rounding: {best_name}, "
                  f"refined={best_cost:.4f} "
                  f"(from {len(candidates)} candidates)")
            return best_result
        print(f"    [GCS] {solver_name}: refinement failed")

    return False, float("inf"), [], []


def _overlap_bounds_1d(lo1, hi1, lo2, hi2, period=None):
    """计算两个 1-D 区间的交集 (可选周期 wrap).

    Returns (ov_lo, ov_hi).  若只 touch 不重叠则 ov_lo == ov_hi.
    无任何交集时返回两区间最近边界的中点.
    """
    eps = 1e-9
    best_lo, best_hi, best_w = (lo1 + lo2) / 2, (lo1 + lo2) / 2, -1.0
    offsets = [0.0]
    if period is not None:
        offsets += [period, -period]
    for off in offsets:
        ol = max(lo1, lo2 + off)
        oh = min(hi1, hi2 + off)
        if oh >= ol - eps:
            w = max(0.0, oh - ol)
            if w > best_w:
                best_lo = ol
                best_hi = max(ol, oh)
                best_w = w
    return best_lo, best_hi


def _build_transition_waypoints(
    box_seq: List[int],
    boxes: Dict[int, BoxNode],
    q_start: np.ndarray,
    q_goal: np.ndarray,
    period: Optional[float] = None,
    bridge_edge_map: Optional[Dict] = None,
) -> Tuple[List[np.ndarray], List[Tuple[np.ndarray, np.ndarray]]]:
    """构建 transition waypoints 和 per-waypoint bounds.

    相邻 box 之间的转折点放在两 box 交集 (overlap) 中心,
    保证: 每段线段的两端都在同一个凸 box 内 → 段落自动无碰撞.

    Returns:
        waypoints: List[ndarray]  路径点
        wp_bounds: List[(lo, hi)]  每个点的可行域
    """
    if bridge_edge_map is None:
        bridge_edge_map = {}

    ndim = len(q_start)
    half = period / 2.0 if period is not None else None

    def _box_bounds(bid):
        box = boxes[bid]
        lo = np.array([lo_d for lo_d, _ in box.joint_intervals])
        hi = np.array([hi_d for _, hi_d in box.joint_intervals])
        return lo, hi

    waypoints = [q_start.copy()]
    wp_bounds = [_box_bounds(box_seq[0])]

    for k in range(len(box_seq) - 1):
        bid_cur = box_seq[k]
        bid_nxt = box_seq[k + 1]
        be = bridge_edge_map.get((bid_cur, bid_nxt))
        if be is not None:
            # 桥接边: 使用已验证碰撞的端点
            s_bid = find_box_containing(be.source_config, boxes)
            if s_bid == bid_cur:
                waypoints.append(be.source_config.copy())
                wp_bounds.append(_box_bounds(bid_cur))
                waypoints.append(be.target_config.copy())
                wp_bounds.append(_box_bounds(bid_nxt))
            else:
                waypoints.append(be.target_config.copy())
                wp_bounds.append(_box_bounds(bid_nxt))
                waypoints.append(be.source_config.copy())
                wp_bounds.append(_box_bounds(bid_cur))
        else:
            # 计算两 box 交集中心作为 transition point
            box_a, box_b = boxes[bid_cur], boxes[bid_nxt]
            ov_lo = np.empty(ndim)
            ov_hi = np.empty(ndim)
            for d, ((lo1, hi1), (lo2, hi2)) in enumerate(
                    zip(box_a.joint_intervals, box_b.joint_intervals)):
                ov_lo[d], ov_hi[d] = _overlap_bounds_1d(
                    lo1, hi1, lo2, hi2, period)
            transition = (ov_lo + ov_hi) / 2.0
            if half is not None:
                transition = ((transition + half) % period) - half
            waypoints.append(transition)
            wp_bounds.append((ov_lo, ov_hi))

    waypoints.append(q_goal.copy())
    wp_bounds.append(_box_bounds(box_seq[-1]))
    return waypoints, wp_bounds


def _pull_tight_in_bounds(
    waypoints: List[np.ndarray],
    wp_bounds: List[Tuple[np.ndarray, np.ndarray]],
    period: Optional[float] = None,
    n_iters: int = 30,
) -> List[np.ndarray]:
    """迭代拉紧: 将每个内部 waypoint 拉向邻居连线上, 约束在 bounds 内."""
    if len(waypoints) <= 2:
        return waypoints[:]
    wps = [w.copy() for w in waypoints]
    if period is not None:
        half = period / 2.0
    for _it in range(n_iters):
        max_move = 0.0
        for i in range(1, len(wps) - 1):
            prev, nxt = wps[i - 1], wps[i + 1]
            if period is not None:
                diff = ((nxt - prev) + half) % period - half
                d_cur = ((wps[i] - prev) + half) % period - half
            else:
                diff = nxt - prev
                d_cur = wps[i] - prev
            seg_len_sq = float(np.dot(diff, diff))
            if seg_len_sq < 1e-20:
                target = prev.copy()
            else:
                t = float(np.dot(d_cur, diff)) / seg_len_sq
                t = np.clip(t, 0.0, 1.0)
                target = prev + t * diff
                if period is not None:
                    target = ((target + half) % period) - half
            lo_b, hi_b = wp_bounds[i]
            for d in range(len(target)):
                target[d] = np.clip(target[d], lo_b[d], hi_b[d])
            move = float(np.linalg.norm(target - wps[i]))
            if move > max_move:
                max_move = move
            wps[i] = target
        if max_move < 1e-8:
            break
    return wps


def _refine_path_in_boxes(
    waypoints: List[np.ndarray],
    wp_bounds: List[Tuple[np.ndarray, np.ndarray]],
    q_start: np.ndarray,
    q_goal: np.ndarray,
    ndim: int,
    period: Optional[float] = None,
) -> Tuple[List[np.ndarray], float]:
    """SOCP 精炼: 在 per-waypoint bounds 约束内优化 waypoint 位置.

    注意: SOCP 本身使用 Euclidean L2 范数 (凸松弛),
    对环面空间只是近似; 但代价统计使用 geodesic.
    """
    # 周期空间下，Euclidean SOCP 会把点往坐标边界/角点拉，
    # 与 geodesic 目标不一致；直接保留 geodesic pull-tight 结果。
    if period is not None:
        cost = _geo_path_cost(waypoints, period)
        return waypoints, cost

    if cp is None:
        cost = _geo_path_cost(waypoints, period)
        return waypoints, cost

    m = len(waypoints)
    if m <= 2:
        cost = _geo_dist(q_start, q_goal, period)
        return waypoints, cost

    n_free = m - 2
    w = cp.Variable((n_free, ndim))

    constraints = []
    for i in range(n_free):
        lo_b, hi_b = wp_bounds[i + 1]
        for d in range(ndim):
            constraints.append(w[i, d] >= lo_b[d])
            constraints.append(w[i, d] <= hi_b[d])

    segs = [cp.norm(w[0, :] - q_start, 2)]
    for i in range(n_free - 1):
        segs.append(cp.norm(w[i + 1, :] - w[i, :], 2))
    segs.append(cp.norm(q_goal - w[n_free - 1, :], 2))

    prob = cp.Problem(cp.Minimize(cp.sum(segs)), constraints)
    try:
        prob.solve(solver=cp.CLARABEL, verbose=False)
    except Exception:
        cost = _geo_path_cost(waypoints, period)
        return waypoints, cost

    if (prob.status in ("optimal", "optimal_inaccurate")
            and w.value is not None):
        refined = [q_start.copy()]
        for i in range(n_free):
            pt = w.value[i].copy()
            lo_b, hi_b = wp_bounds[i + 1]
            for d in range(ndim):
                pt[d] = np.clip(pt[d], lo_b[d], hi_b[d])
            refined.append(pt)
        refined.append(q_goal.copy())
        cost = _geo_path_cost(refined, period)
        return refined, cost

    cost = _geo_path_cost(waypoints, period)
    return waypoints, cost


# ═══════════════════════════════════════════════════════════════════════════
# Solver B: Dijkstra on box graph + SOCP refine
# ═══════════════════════════════════════════════════════════════════════════

def _dijkstra_box_graph(boxes, adj, src, tgt, period=None,
                        q_goal=None):
    """A* search with boundary-aware edge weights and goal heuristic.

    改进 (相比旧版 center-to-center Dijkstra):
      1. 边权使用 box 边界最小 L2 距离; 相邻/重叠 box 取 5% 中心距离
         → 避免大 box 中心距离过长导致路径绕行
      2. 当提供 q_goal 时, 启用 A* 启发式 (box 内最近点到 goal 的距离)
         → 搜索方向偏好地理上更靠近终点的 box
    """
    # ── 预计算 box lo / hi / center ──
    box_lo, box_hi, centers, diags = {}, {}, {}, {}
    for bid, box in boxes.items():
        lo = np.array([lo_d for lo_d, hi_d in box.joint_intervals])
        hi = np.array([hi_d for lo_d, hi_d in box.joint_intervals])
        box_lo[bid] = lo
        box_hi[bid] = hi
        centers[bid] = (lo + hi) * 0.5
        diags[bid] = float(np.linalg.norm(hi - lo))

    # ── A* 启发式: box 内最近点到 goal 的距离 (period 感知) ──
    _use_h = (q_goal is not None)

    def _dist_to_interval_periodic(x, lo, hi):
        if lo <= x <= hi:
            return 0.0
        if period is None:
            return min(abs(x - lo), abs(x - hi))
        best = float('inf')
        for k in (-1, 0, 1):
            xk = x + k * period
            if xk < lo:
                d = lo - xk
            elif xk > hi:
                d = xk - hi
            else:
                d = 0.0
            if d < best:
                best = d
        return best

    def _h(bid):
        if not _use_h:
            return 0.0
        d2 = 0.0
        for d in range(len(q_goal)):
            dist_d = _dist_to_interval_periodic(
                float(q_goal[d]), float(box_lo[bid][d]), float(box_hi[bid][d]))
            d2 += dist_d * dist_d
        return float(np.sqrt(d2))

    # ── 边权: 边界距离 + 轻微 box 尺寸惩罚（抑制“大 box 偏置”） ──
    def _w(u, v):
        if period is None:
            gap = np.maximum(0.0, np.maximum(box_lo[v] - box_hi[u],
                                              box_lo[u] - box_hi[v]))
            surface_dist = float(np.linalg.norm(gap))
            size_penalty = 0.02 * 0.5 * (diags[u] + diags[v])
            if surface_dist > 1e-10:
                return surface_dist + size_penalty
            return max(0.15 * float(np.linalg.norm(centers[u] - centers[v]))
                       + size_penalty, 1e-12)

        half = period / 2.0
        gap_sq = 0.0
        for d in range(len(box_lo[u])):
            lo_u, hi_u = float(box_lo[u][d]), float(box_hi[u][d])
            lo_v, hi_v = float(box_lo[v][d]), float(box_hi[v][d])

            gap_d = float('inf')
            for shift in (-period, 0.0, period):
                lo_vs = lo_v + shift
                hi_vs = hi_v + shift
                g = max(lo_vs - hi_u, lo_u - hi_vs, 0.0)
                if g < gap_d:
                    gap_d = g

            if gap_d > half:
                gap_d = period - gap_d
            gap_sq += gap_d * gap_d

        surface_dist = float(np.sqrt(gap_sq))
        size_penalty = 0.02 * 0.5 * (diags[u] + diags[v])
        if surface_dist > 1e-10:
            return surface_dist + size_penalty

        diff = ((centers[u] - centers[v]) + half) % period - half
        return max(0.15 * float(np.linalg.norm(diff)) + size_penalty, 1e-12)

    # ── A* 主循环 ──
    g_map: Dict[int, float] = {src: 0.0}
    prev_map: Dict[int, Optional[int]] = {}
    cnt = 0
    heap = [(_h(src), cnt, src)]
    closed: set = set()

    while heap:
        _, _, u = heapq.heappop(heap)
        if u in closed:
            continue
        closed.add(u)
        if u == tgt:
            break
        for v in adj.get(u, set()):
            if v in closed:
                continue
            tentative = g_map[u] + _w(u, v)
            if tentative < g_map.get(v, float('inf')):
                g_map[v] = tentative
                prev_map[v] = u
                cnt += 1
                heapq.heappush(heap, (tentative + _h(v), cnt, v))

    if tgt not in closed:
        return None, float('inf')

    seq: list = []
    cur: Optional[int] = tgt
    while cur is not None:
        seq.append(cur)
        cur = prev_map.get(cur)
    seq.reverse()
    return seq, g_map.get(tgt, float('inf'))


def _shortcut_box_sequence(box_seq, adj):
    """贪心跳跃: 在 box 序列中跳过可直接相邻到达的中间 box.

    从序列起点开始, 每一步尽可能跳到最远的、与当前 box 直接相邻/
    相同的 box, 从而缩短 box 序列, 减少不必要的绕行.
    """
    if len(box_seq) <= 2:
        return box_seq
    n = len(box_seq)
    result = [box_seq[0]]
    i = 0
    while i < n - 1:
        farthest = i + 1
        nbrs = adj.get(box_seq[i], set())
        for j in range(n - 1, i + 1, -1):
            if box_seq[j] in nbrs:
                farthest = j
                break
        result.append(box_seq[farthest])
        i = farthest
    return result


def _solve_method_gcs(boxes, adj, src, tgt, q_start, q_goal, ndim,
                      corridor_hops=2, label="GCS", **kwargs):
    """GCS SOCP 求解."""
    t0 = time.perf_counter()
    success, cost, waypoints, box_seq = solve_gcs(
        boxes, adj, src, tgt, q_start, q_goal, ndim,
        corridor_hops=corridor_hops)
    ms = (time.perf_counter() - t0) * 1000
    return dict(method=label, success=success, cost=cost,
                waypoints=waypoints, box_seq=box_seq, plan_ms=ms,
                plan_graph_ms=0.0,
                plan_waypoints_ms=0.0,
                plan_refine_ms=0.0,
                plan_shortcut_ms=0.0)


def _segment_in_box(p_a: np.ndarray, p_b: np.ndarray, box: BoxNode,
                    n_samples: int = 6) -> bool:
    """Check if the line segment from p_a to p_b stays within box."""
    for t in np.linspace(0.0, 1.0, n_samples):
        pt = p_a + t * (p_b - p_a)
        for d, (lo, hi) in enumerate(box.joint_intervals):
            if pt[d] < lo - 1e-9 or pt[d] > hi + 1e-9:
                return False
    return True


def _geometric_shortcut(
    waypoints: List[np.ndarray],
    boxes: Dict[int, BoxNode],
    box_seq: List[int],
    period: Optional[float] = None,
) -> Tuple[List[np.ndarray], float]:
    """贪心几何缩短: 在 SOCP 精炼后, 跳过 box 内部的冗余中间路径点.

    对于每个路径点 i, 尝试直接连接到尽可能远的路径点 j,
    使得线段 p_i → p_j 完全在某个 box 内 (即 box_seq 中任一 box
    能包含这条线段). 成功则跳过中间所有路径点.
    """
    if len(waypoints) <= 2:
        cost = _geo_path_cost(waypoints, period)
        return waypoints, cost

    # Collect all box bounds for containment checks
    unique_boxes = [boxes[bid] for bid in box_seq if bid in boxes]

    result = [waypoints[0]]
    i = 0
    n = len(waypoints)
    while i < n - 1:
        farthest = i + 1
        for j in range(n - 1, i + 1, -1):
            # Check: does segment waypoints[i]→waypoints[j] fit in any box?
            for box in unique_boxes:
                if _segment_in_box(waypoints[i], waypoints[j], box):
                    farthest = j
                    break
            if farthest == j:
                break
        result.append(waypoints[farthest])
        i = farthest

    cost = _geo_path_cost(result, period)
    return result, cost


def _solve_method_dijkstra(boxes, adj, src, tgt, q_start, q_goal, ndim,
                           label="Dijkstra", period=None,
                           bridge_edge_map=None):
    """Dijkstra on box graph → shortcut → transition waypoints → refine."""
    t0 = time.perf_counter()

    t_graph = time.perf_counter()
    box_seq, raw_dist = _dijkstra_box_graph(boxes, adj, src, tgt,
                                            period=period, q_goal=q_goal)
    graph_ms = (time.perf_counter() - t_graph) * 1000
    if box_seq is None:
        ms = (time.perf_counter() - t0) * 1000
        print(f"    [{label}] Dijkstra: no path found")
        return dict(method=label, success=False, cost=float('inf'),
                    waypoints=[], box_seq=[], plan_ms=ms,
                    plan_graph_ms=graph_ms,
                    plan_waypoints_ms=0.0,
                    plan_refine_ms=0.0,
                    plan_shortcut_ms=0.0)

    # ── 贪心跳跃: 跳过可直接相邻到达的中间 box ──
    t_short_seq = time.perf_counter()
    short_seq = _shortcut_box_sequence(box_seq, adj)
    short_seq_ms = (time.perf_counter() - t_short_seq) * 1000

    # ── 构建 transition waypoints (在 box 交集中心) ──
    t_waypoints = time.perf_counter()
    waypoints, wp_bounds = _build_transition_waypoints(
        short_seq, boxes, q_start, q_goal,
        period=period, bridge_edge_map=bridge_edge_map)

    # ── Pull-tight: 在 overlap bounds 内拉紧 ──
    waypoints = _pull_tight_in_bounds(
        waypoints, wp_bounds, period=period, n_iters=30)
    waypoints_ms = (time.perf_counter() - t_waypoints) * 1000

    # ── SOCP 精炼 (如果可用) ──
    t_refine = time.perf_counter()
    refined_wps, refined_cost = _refine_path_in_boxes(
        waypoints, wp_bounds, q_start, q_goal, ndim, period=period)
    refine_ms = (time.perf_counter() - t_refine) * 1000

    # ── 几何路径缩短: 跳过 box 内冗余的中间路径点 ──
    t_geo_shortcut = time.perf_counter()
    refined_wps, refined_cost = _geometric_shortcut(
        refined_wps, boxes, short_seq, period=period)
    geo_shortcut_ms = (time.perf_counter() - t_geo_shortcut) * 1000

    ms = (time.perf_counter() - t0) * 1000
    n_skip = len(box_seq) - len(short_seq)
    skip_info = (f", shortcut {len(box_seq)}->{len(short_seq)}"
                 if n_skip > 0 else "")
    print(f"    [{label}] {len(box_seq)} boxes{skip_info}, "
          f"raw_dist={raw_dist:.4f}, refined={refined_cost:.4f}, "
          f"{len(refined_wps)} wp ({ms:.0f}ms)")
    return dict(method=label, success=True, cost=refined_cost,
                waypoints=refined_wps, box_seq=short_seq, plan_ms=ms,
                plan_graph_ms=graph_ms,
                plan_waypoints_ms=waypoints_ms,
                plan_refine_ms=refine_ms,
                plan_shortcut_ms=short_seq_ms + geo_shortcut_ms)


# ═══════════════════════════════════════════════════════════════════════════
# Solver C: Visibility Graph
# ═══════════════════════════════════════════════════════════════════════════

def _solve_method_visgraph(boxes, q_start, q_goal, collision_checker,
                           segment_resolution=0.05, label="VisGraph",
                           period=None):
    """可视图法: box seed configs 为节点, collision check 为边."""
    t0 = time.perf_counter()

    node_ids = ['start', 'goal']
    node_configs = [q_start.copy(), q_goal.copy()]
    for bid, box in boxes.items():
        if box.seed_config is not None:
            node_ids.append(bid)
            node_configs.append(np.asarray(box.seed_config, dtype=np.float64))

    n_nodes = len(node_ids)
    print(f"    [{label}] {n_nodes} nodes, checking edges ...")

    configs_arr = np.array(node_configs)
    k_nearest = min(50, n_nodes - 1)

    vis_adj: Dict[int, List[Tuple[int, float]]] = {
        i: [] for i in range(n_nodes)}
    n_checks = 0
    n_edges = 0

    for i in range(n_nodes):
        qi = configs_arr[i]
        # 计算 geodesic 距离
        diffs = np.array([_geo_diff(qi, configs_arr[j], period)
                          for j in range(n_nodes)])
        dists = np.linalg.norm(diffs, axis=1)
        dists[i] = float('inf')

        nearest_idxs = np.argpartition(
            dists, min(k_nearest, n_nodes - 2))[:k_nearest]
        nearest_idxs = nearest_idxs[np.argsort(dists[nearest_idxs])]

        for j_idx in nearest_idxs:
            j = int(j_idx)
            d = float(dists[j])
            if d == float('inf'):
                continue
            if any(nbr == i for nbr, _ in vis_adj[j]):
                vis_adj[i].append((j, d))
                n_edges += 1
                continue
            n_checks += 1
            if not collision_checker.check_segment_collision(
                    configs_arr[i], configs_arr[j], segment_resolution,
                    period=period):
                vis_adj[i].append((j, d))
                vis_adj[j].append((i, d))
                n_edges += 2

    t_graph = (time.perf_counter() - t0) * 1000
    print(f"    [{label}] graph: {n_edges // 2} edges, "
          f"{n_checks} collision checks ({t_graph:.0f}ms)")

    dist_map = [float('inf')] * n_nodes
    prev_map = [-1] * n_nodes
    dist_map[0] = 0.0
    pq = [(0.0, 0)]

    while pq:
        d, u = heapq.heappop(pq)
        if d > dist_map[u]:
            continue
        if u == 1:
            break
        for v, w in vis_adj[u]:
            nd = d + w
            if nd < dist_map[v]:
                dist_map[v] = nd
                prev_map[v] = u
                heapq.heappush(pq, (nd, v))

    if dist_map[1] == float('inf'):
        ms = (time.perf_counter() - t0) * 1000
        print(f"    [{label}] no path found ({ms:.0f}ms)")
        return dict(method=label, success=False, cost=float('inf'),
                    waypoints=[], box_seq=[], plan_ms=ms)

    path_idxs = []
    cur = 1
    while cur != -1:
        path_idxs.append(cur)
        cur = prev_map[cur]
    path_idxs.reverse()

    raw_waypoints = [node_configs[i].copy() for i in path_idxs]
    raw_cost = dist_map[1]

    shortcut_wps = _greedy_shortcut(
        raw_waypoints, collision_checker, segment_resolution, period=period)

    final_cost = _geo_path_cost(shortcut_wps, period)

    ms = (time.perf_counter() - t0) * 1000
    print(f"    [{label}] raw={raw_cost:.4f} ({len(raw_waypoints)} wp) "
          f"→ shortcut={final_cost:.4f} ({len(shortcut_wps)} wp) ({ms:.0f}ms)")
    return dict(method=label, success=True, cost=final_cost,
                waypoints=shortcut_wps, box_seq=[], plan_ms=ms,
                plan_graph_ms=t_graph,
                plan_waypoints_ms=0.0,
                plan_refine_ms=0.0,
                plan_shortcut_ms=max(ms - t_graph, 0.0))


def _greedy_shortcut(waypoints, collision_checker, resolution, period=None):
    """贪心路径缩短: 依次尝试跳过中间点."""
    if len(waypoints) <= 2:
        return waypoints
    result = [waypoints[0]]
    i = 0
    while i < len(waypoints) - 1:
        farthest = i + 1
        for j in range(len(waypoints) - 1, i + 1, -1):
            if not collision_checker.check_segment_collision(
                    waypoints[i], waypoints[j], resolution,
                    period=period):
                farthest = j
                break
        result.append(waypoints[farthest])
        i = farthest
    return result


def _ensure_geodesic_safe(
    waypoints: List[np.ndarray],
    collision_checker,
    resolution: float,
    period: float,
    max_depth: int = 6,
) -> List[np.ndarray]:
    """Subdivide colliding segments until geodesic interpolation is safe.

    For each segment that collides under geodesic interpolation, recursively
    insert geodesic midpoints.  This turns wrapping segments into multiple
    short non-wrapping segments that stay in free space.

    This runs *inside the pipeline* so that callers don't need a heavy
    post-hoc repair phase.
    """
    if period is None or len(waypoints) < 2:
        return waypoints

    half = period / 2.0

    def _geo_mid(q1, q2):
        diff = ((q2 - q1) + half) % period - half
        m = q1 + 0.5 * diff
        return ((m + half) % period) - half

    def _subdivide(q1, q2, depth):
        if not collision_checker.check_segment_collision(
                q1, q2, resolution, period=period):
            return [q2]
        if depth >= max_depth:
            return [q2]  # give up — let downstream repair handle it
        mid = _geo_mid(q1, q2)
        left = _subdivide(q1, mid, depth + 1)
        right = _subdivide(mid, q2, depth + 1)
        return left + right

    result = [waypoints[0]]
    for i in range(len(waypoints) - 1):
        result.extend(_subdivide(waypoints[i], waypoints[i + 1], 0))
    return result


def _ensure_segment_safe(
    waypoints: List[np.ndarray],
    collision_checker,
    resolution: float,
    max_depth: int = 6,
) -> List[np.ndarray]:
    """Subdivide colliding segments (Euclidean, non-periodic spaces).

    For each segment that collides under linear interpolation, recursively
    insert midpoints until all sub-segments are collision-free or max_depth
    is reached.
    """
    if len(waypoints) < 2:
        return waypoints

    def _subdivide(q1, q2, depth):
        if not collision_checker.check_segment_collision(
                q1, q2, resolution):
            return [q2]
        if depth >= max_depth:
            return [q2]
        mid = 0.5 * (q1 + q2)
        left = _subdivide(q1, mid, depth + 1)
        right = _subdivide(mid, q2, depth + 1)
        return left + right

    result = [waypoints[0]]
    for i in range(len(waypoints) - 1):
        result.extend(_subdivide(waypoints[i], waypoints[i + 1], 0))
    return result


# ═══════════════════════════════════════════════════════════════════════════
# Pipeline: shared forest growth + per-method planning
# ═══════════════════════════════════════════════════════════════════════════

def grow_and_prepare(robot, scene, cfg, q_start, q_goal, ndim,
                     no_cache=False):
    """共享前半段: grow → cache (异步) → coarsen.

    返回 dict 包含 planner, boxes, forest_obj, timing 等.
    """
    t_prepare_all = time.perf_counter()
    planner_cfg = make_planner_config(cfg)
    planner_cfg.ffb_min_edge = getattr(cfg, 'ffb_min_edge', 0.04)
    planner = SBFPlanner(robot=robot, scene=scene, config=planner_cfg,
                         no_cache=no_cache)

    t0 = time.perf_counter()
    boxes, forest_obj, grow_detail = grow_forest(
        planner, q_start, q_goal, cfg.seed,
        cfg.max_consecutive_miss, ndim,
        max_boxes=cfg.max_boxes,
        parallel_grow=cfg.parallel_grow,
        n_partitions_depth=cfg.n_partitions_depth,
        parallel_workers=cfg.parallel_workers)
    grow_ms = (time.perf_counter() - t0) * 1000
    n_grown = len(forest_obj.boxes)
    n_nodes = planner.hier_tree.n_nodes

    # AABB cache: 后台线程保存
    cache_result = {}
    cache_thread = None

    t_cache_start = time.perf_counter()
    if not no_cache:
        def _save_cache():
            _t0 = time.perf_counter()
            _path = planner.hier_tree.auto_save()
            _ms = (time.perf_counter() - _t0) * 1000
            cache_result['path'] = _path
            cache_result['ms'] = _ms

        cache_thread = threading.Thread(target=_save_cache, daemon=True)
        cache_thread.start()
        print(f"    [cache] saving {n_nodes} nodes in background thread ...")
    else:
        print(f"    [cache] skipped (no_cache mode), {n_nodes} nodes")
    cache_start_ms = (time.perf_counter() - t_cache_start) * 1000

    # coarsen (与 cache save 并行)
    n_before_coarsen = len(forest_obj.boxes)
    coarsen_stats = coarsen_forest(
        tree=planner.hier_tree, forest=forest_obj,
        obstacles=planner.obstacles, safety_margin=0.0,
        max_rounds=cfg.coarsen_max_rounds,
    )
    n_after_coarsen = len(forest_obj.boxes)
    coarsen_ms = coarsen_stats.time_ms
    print(f"    [coarsen] {n_before_coarsen} -> {n_after_coarsen} boxes "
          f"({coarsen_stats.n_merges} merges in "
          f"{coarsen_stats.n_rounds} rounds, {coarsen_ms:.0f}ms)")

    t_box_export = time.perf_counter()
    boxes = forest_obj.boxes
    box_export_ms = (time.perf_counter() - t_box_export) * 1000

    growprep_total_ms = (time.perf_counter() - t_prepare_all) * 1000
    prepare_misc_ms = max(
        0.0,
        growprep_total_ms - (grow_ms + coarsen_ms + cache_start_ms + box_export_ms)
    )

    return dict(
        planner=planner, boxes=boxes, forest_obj=forest_obj,
        grow_ms=grow_ms, cache_ms=0.0, coarsen_ms=coarsen_ms,
        cache_start_ms=cache_start_ms,
        box_export_ms=box_export_ms,
        prepare_misc_ms=prepare_misc_ms,
        growprep_total_ms=growprep_total_ms,
        coarsen_stats=coarsen_stats, n_grown=n_grown,
        n_cache_nodes=n_nodes,
        grow_detail=grow_detail,
        _cache_thread=cache_thread,
        _cache_result=cache_result,
    )


def run_method_with_bridge(method_fn, method_name, prep, cfg, q_start,
                           q_goal, ndim, seed=0, **method_kwargs):
    """运行需要 adjacency 的方法, 含 lazy bridge."""
    boxes = prep['boxes']
    planner = prep['planner']
    forest_obj = prep['forest_obj']

    t0 = time.perf_counter()
    # 优先使用 planner._period（由 SBFPlanner.__init__ 统一计算）
    # forest_obj.period 仅作为兜底（兼容旧缓存）
    period = getattr(planner, '_period', None)
    if period is None:
        fp = getattr(forest_obj, 'period', None)
        # 验证 forest period 与 joint_limits 一致性
        if fp is not None:
            jl = getattr(planner, 'joint_limits', None) or forest_obj.joint_limits
            spans = [hi - lo for lo, hi in jl]
            all_same = all(abs(s - spans[0]) < 1e-6 for s in spans)
            if all_same and abs(spans[0] - fp) < 0.1:
                period = fp  # 一致，使用 forest period
            # else: 不一致，保持 period=None
    adj, uf, islands = _build_adjacency_and_islands(boxes, period=period)
    adj_ms = (time.perf_counter() - t0) * 1000
    n_edges = sum(len(v) for v in adj.values()) // 2
    print(f"    [adj] {len(adj)} vertices, {n_edges} edges ({adj_ms:.0f}ms)")

    src = find_box_containing(q_start, boxes)
    tgt = find_box_containing(q_goal, boxes)
    if src is None or tgt is None:
        print(f"    [{method_name}] ERROR: start or goal not in any box")
        return None

    n_before_islands = len(islands)
    bridge_ms = 0.0
    bridge_edges = []
    bridge_boxes_list = []

    if not uf.same(src, tgt):
        print(f"    [{method_name}] s-t disconnected "
              f"({n_before_islands} islands), bridging ...")
        t0 = time.perf_counter()
        _ffb_me = getattr(planner.config, 'ffb_min_edge', 0.05)
        _min_island = _ffb_me * 5.0
        bridge_result = bridge_islands(
            boxes=boxes,
            collision_checker=planner.collision_checker,
            segment_resolution=0.03,
            max_pairs_per_island_pair=10,
            max_rounds=5,
            period=period,
            hier_tree=planner.hier_tree,
            obstacles=planner.obstacles,
            forest=forest_obj,
            n_bridge_seeds=7,
            min_island_size=_min_island,
            precomputed_uf=uf,
            precomputed_islands=islands,
            target_pair=(src, tgt),
            rrt_seed=seed,
        )
        bridge_edges_res, final_islands, _, bridge_boxes_res, discarded, \
            rrt_fallback_path = bridge_result
        bridge_ms = (time.perf_counter() - t0) * 1000
        bridge_edges = bridge_edges_res
        bridge_boxes_list = bridge_boxes_res
        boxes = forest_obj.boxes

        # If RRT-Connect provided a direct collision-free path, use it
        # instead of going through the box graph (which may not faithfully
        # reproduce the RRT path).
        if rrt_fallback_path is not None:
            rrt_wps = [np.asarray(w, dtype=np.float64)
                       for w in rrt_fallback_path]
            # Replace first/last with exact q_start/q_goal
            rrt_wps[0] = q_start.copy()
            rrt_wps[-1] = q_goal.copy()
            # Shortcut redundant waypoints
            _cc_rrt = getattr(planner, 'collision_checker', None)
            if _cc_rrt is not None:
                rrt_wps = _greedy_shortcut(
                    rrt_wps, _cc_rrt, 0.05, period=period)
            rrt_cost = _geo_path_cost(rrt_wps, period)
            logger.info("[RRT-bridge] direct path: %d wp, cost=%.4f (%.0f ms)",
                       len(rrt_wps), rrt_cost, bridge_ms)
            plan_result = dict(
                method=method_name, success=True, cost=rrt_cost,
                waypoints=rrt_wps, box_seq=[], plan_ms=0.0,
                plan_graph_ms=0.0, plan_waypoints_ms=0.0,
                plan_refine_ms=0.0, plan_shortcut_ms=0.0,
                post_safe_ms=0.0,
                boxes=dict(boxes), adj={},
                n_before_islands=n_before_islands,
                n_after_islands=len(final_islands),
                bridge_edges=bridge_edges,
                bridge_boxes=bridge_boxes_list,
                adj_ms=adj_ms, bridge_ms=bridge_ms,
                grow_ms=prep['grow_ms'], cache_ms=prep['cache_ms'],
                coarsen_ms=prep['coarsen_ms'],
                coarsen_stats=prep['coarsen_stats'],
                n_grown=prep['n_grown'],
                n_cache_nodes=prep['n_cache_nodes'],
                rrt_bridge_used=True,
            )
            return plan_result

        for bb in bridge_boxes_list:
            if bb.node_id not in adj:
                neighbor_ids = forest_obj._adjacent_existing_ids_from_cache(
                    bb, tol=forest_obj.config.adjacency_tolerance)
                adj[bb.node_id] = set(neighbor_ids)
                for nb in neighbor_ids:
                    adj.setdefault(nb, set()).add(bb.node_id)
        _add_bridge_to_adj(adj, bridge_edges, uf)
        n_after_islands = len(uf.components())
        logger.info("[bridge] islands: %d -> %d  (%d edges, %d bridge-boxes, %.0f ms)",
                   n_before_islands, n_after_islands, len(bridge_edges),
                   len(bridge_boxes_list), bridge_ms)

        if src not in boxes:
            src = find_box_containing(q_start, boxes)
        if tgt not in boxes:
            tgt = find_box_containing(q_goal, boxes)
    else:
        logger.debug("[%s] s-t already connected, skipping bridge", method_name)
        n_after_islands = n_before_islands

    if src is None or tgt is None:
        print(f"    [{method_name}] ERROR: start or goal not in any box "
              f"after bridge")
        return None

    # ── 直连检测: 碰撞检测 straight-line, 跳过图搜索 ──
    _cc = getattr(planner, 'collision_checker', None)
    if _cc is not None:
        _res = getattr(cfg, 'segment_collision_resolution', 0.05)
        if not _cc.check_segment_collision(q_start, q_goal, resolution=_res,
                                            period=period):
            if period is not None:
                _half = np.asarray(period) / 2.0
                _diff = ((q_goal - q_start) + _half) % period - _half
                direct_cost = float(np.linalg.norm(_diff))
            else:
                direct_cost = float(np.linalg.norm(q_goal - q_start))
            ms_total = (time.perf_counter() - t0) * 1000 if 't0' in dir() else 0
            print(f"    [{method_name}] direct-connect OK! "
                  f"cost={direct_cost:.4f}")
            plan_result = dict(
                method=method_name, success=True, cost=direct_cost,
                waypoints=[q_start.copy(), q_goal.copy()],
                box_seq=[src, tgt] if src != tgt else [src],
                plan_ms=0.0)
            plan_result.update(
                boxes=dict(boxes), adj=adj,
                n_before_islands=n_before_islands,
                n_after_islands=n_before_islands,
                bridge_edges=bridge_edges,
                bridge_boxes=bridge_boxes_list,
                adj_ms=adj_ms, bridge_ms=bridge_ms,
                grow_ms=prep['grow_ms'], cache_ms=prep['cache_ms'],
                coarsen_ms=prep['coarsen_ms'],
                coarsen_stats=prep['coarsen_stats'],
                n_grown=prep['n_grown'],
                n_cache_nodes=prep['n_cache_nodes'],
            )
            return plan_result

    # ── 构建 bridge_edge_map ──
    bridge_edge_map = {}
    for e in bridge_edges:
        s_bid = find_box_containing(e.source_config, boxes)
        t_bid = find_box_containing(e.target_config, boxes)
        if s_bid is not None and t_bid is not None:
            bridge_edge_map[(s_bid, t_bid)] = e
            bridge_edge_map[(t_bid, s_bid)] = e

    plan_result = method_fn(
        boxes=boxes, adj=adj, src=src, tgt=tgt,
        q_start=q_start, q_goal=q_goal, ndim=ndim,
        label=method_name, period=period,
        bridge_edge_map=bridge_edge_map,
        **method_kwargs)

    # ── Segment safety: subdivide colliding segments ──
    # For periodic spaces: uses geodesic midpoints to fix wrapping segments.
    # For non-periodic spaces: uses Euclidean midpoints to fix segments
    #   that leave safe boxes (e.g. Panda's 7-DOF path).
    post_safe_ms = 0.0
    if (_cc is not None and plan_result.get('success')):
        _ts_safe = time.perf_counter()
        wps = plan_result.get('waypoints', [])
        if len(wps) >= 2:
            _res = getattr(cfg, 'segment_collision_resolution', 0.05)
            if period is not None:
                safe_wps = _ensure_geodesic_safe(
                    wps, _cc, _res, period, max_depth=6)
            else:
                safe_wps = _ensure_segment_safe(
                    wps, _cc, _res, max_depth=6)
            safe_wps = _greedy_shortcut(
                safe_wps, _cc, _res, period=period)
            plan_result['waypoints'] = safe_wps
            plan_result['cost'] = _geo_path_cost(safe_wps, period)
        post_safe_ms = (time.perf_counter() - _ts_safe) * 1000
    plan_result['post_safe_ms'] = post_safe_ms

    plan_result.update(
        boxes=dict(boxes),
        adj=adj,
        n_before_islands=n_before_islands,
        n_after_islands=n_after_islands,
        bridge_edges=bridge_edges,
        bridge_boxes=bridge_boxes_list,
        adj_ms=adj_ms,
        bridge_ms=bridge_ms,
        grow_ms=prep['grow_ms'],
        cache_ms=prep['cache_ms'],
        coarsen_ms=prep['coarsen_ms'],
        coarsen_stats=prep['coarsen_stats'],
        n_grown=prep['n_grown'],
        n_cache_nodes=prep['n_cache_nodes'],
    )
    return plan_result


def incremental_regrow(
    seeds: List[np.ndarray],
    planner: SBFPlanner,
    forest_obj: SafeBoxForest,
    boxes: Dict[int, BoxNode],
    adj: Dict[int, Set[int]],
    uf: UnionFind,
    obs_packed,
    budget: int = 60,
    rng: Optional[np.random.Generator] = None,
    ndim: int = 7,
) -> int:
    """在释放的 hier_tree 区域中增量补种 box.

    Seeds 来源:
      1. 被移除 box 的 seed_config (最高优先)
      2. sample_unoccupied_seed (自然偏向被释放区域)

    Returns:
        新增 box 数量
    """
    n_added = 0
    attempted = set()

    def _try_expand_one(q):
        nonlocal n_added
        # 已在某 box 内部则跳过
        existing = find_box_containing(q, boxes)
        if existing is not None:
            return
        if planner.hier_tree.is_occupied(q):
            return
        nid = forest_obj.allocate_id()
        ffb = planner.hier_tree.find_free_box(
            q, planner.obstacles, mark_occupied=True,
            forest_box_id=nid, obs_packed=obs_packed)
        if ffb is None:
            return
        vol = 1.0
        for lo, hi in ffb.intervals:
            vol *= max(hi - lo, 0)
        box = BoxNode(node_id=nid, joint_intervals=ffb.intervals,
                      seed_config=q.copy(), volume=vol)
        if ffb.absorbed_box_ids:
            for aid in ffb.absorbed_box_ids:
                boxes.pop(aid, None)
                forest_obj.boxes.pop(aid, None)
                if aid in adj:
                    nbrs = adj.pop(aid, set())
                    for nb in nbrs:
                        if nb in adj:
                            adj[nb].discard(aid)
        forest_obj.add_box_direct(box)
        boxes[nid] = box
        # 增量邻接
        _incremental_adj_uf(box, boxes, adj, uf)
        n_added += 1

    # Phase 1: 用被移除 box 的 seed 补种
    for seed in seeds:
        if n_added >= budget:
            break
        key = tuple(np.round(seed, 8))
        if key in attempted:
            continue
        attempted.add(key)
        _try_expand_one(seed)

    # Phase 2: sample_unoccupied_seed (偏向被释放空间)
    if rng is not None:
        miss = 0
        max_miss = 15
        while n_added < budget and miss < max_miss:
            try:
                q = planner.hier_tree.sample_unoccupied_seed(rng)
            except ValueError:
                break
            if q is None:
                miss += 1
                continue
            if planner.collision_checker.check_config_collision(q):
                miss += 1
                continue
            _try_expand_one(q)
            if n_added > 0:
                miss = 0
            else:
                miss += 1

    return n_added


def _incremental_adj_uf(
    box: BoxNode,
    boxes: Dict[int, BoxNode],
    adj: Dict[int, Set[int]],
    uf: UnionFind,
    tol: float = 1e-9,
) -> None:
    """增量更新邻接和 UnionFind — 仅检查新 box 与已有 box 的重叠."""
    nid = box.node_id
    adj.setdefault(nid, set())
    if nid not in uf._parent:
        uf._parent[nid] = nid
        uf._rank[nid] = 0
    ndim = len(box.joint_intervals)
    for bid, other in boxes.items():
        if bid == nid:
            continue
        overlap = True
        for d in range(ndim):
            if (box.joint_intervals[d][1] < other.joint_intervals[d][0] - tol or
                    other.joint_intervals[d][1] < box.joint_intervals[d][0] - tol):
                overlap = False
                break
        if overlap:
            adj[nid].add(bid)
            adj.setdefault(bid, set()).add(nid)
            uf.union(nid, bid)


def incremental_obstacle_update(
    prep: dict,
    scene: Scene,
    added_obstacles: list,
    removed_obstacle_names: list,
    regrow_budget: int = 60,
    rng: Optional[np.random.Generator] = None,
) -> Dict:
    """增量更新 forest 以响应障碍物变化 (核心 API).

    流程:
      1. 对新增障碍物 → invalidate + remove colliding boxes
      2. 更新 scene (添加/移除)
      3. 重建 CollisionChecker
      4. 在释放空间中 regrow

    Args:
        prep: grow_and_prepare 返回的 dict (含 planner, forest_obj, boxes 等)
        scene: 当前场景 (会被就地修改)
        added_obstacles: 新增障碍物列表 [{min_point, max_point, name}]
        removed_obstacle_names: 要移除的障碍物名称列表
        regrow_budget: 补种预算
        rng: 随机数生成器

    Returns:
        dict 包含时间分解和统计信息
    """
    planner = prep['planner']
    forest_obj = prep['forest_obj']
    boxes = prep['boxes']
    ndim = planner.robot.n_joints

    # 重建 adj + uf (如果 prep 中没有)
    if 'adj' not in prep or 'uf' not in prep:
        _period_inc = getattr(planner, '_period', None)
        adj, uf, _ = _build_adjacency_and_islands(boxes, period=_period_inc)
        prep['adj'] = adj
        prep['uf'] = uf
    adj = prep['adj']
    uf = prep['uf']

    n_before = len(forest_obj.boxes)
    all_removed_seeds: List[np.ndarray] = []
    total_invalidated = 0

    # ── Step 1: invalidate against new obstacles ──
    t0 = time.perf_counter()
    for obs_info in added_obstacles:
        from forest.models import Obstacle
        obs = Obstacle(
            min_point=np.asarray(obs_info['min_point']),
            max_point=np.asarray(obs_info['max_point']),
            name=obs_info.get('name', 'new_obs'),
        )
        colliding = forest_obj.invalidate_against_obstacle(
            obs, planner.robot, safety_margin=0.0)
        total_invalidated += len(colliding)
        removed = forest_obj.remove_invalidated(colliding)
        all_removed_seeds.extend(
            b.seed_config for b in removed if b.seed_config is not None)
        # 从 adj 中移除
        for bid in colliding:
            boxes.pop(bid, None)
            if bid in adj:
                nbrs = adj.pop(bid, set())
                for nb in nbrs:
                    if nb in adj:
                        adj[nb].discard(bid)
    invalidate_ms = (time.perf_counter() - t0) * 1000

    # ── Step 2: update scene ──
    t0 = time.perf_counter()
    for name in removed_obstacle_names:
        scene.remove_obstacle(name)
    for obs_info in added_obstacles:
        scene.add_obstacle(
            obs_info['min_point'], obs_info['max_point'],
            name=obs_info.get('name', 'new_obs'))
    # 重建 collision checker
    planner.collision_checker = CollisionChecker(
        robot=planner.robot, scene=scene)
    planner.obstacles = scene.get_obstacles()
    obs_packed = planner.hier_tree._prepack_obstacles_c(planner.obstacles)
    scene_ms = (time.perf_counter() - t0) * 1000

    # ── Step 3: rebuild UF ──
    t0 = time.perf_counter()
    uf = UnionFind(list(adj.keys()))
    for bid, nbrs in adj.items():
        for nb in nbrs:
            uf.union(bid, nb)
    prep['uf'] = uf
    uf_ms = (time.perf_counter() - t0) * 1000

    # ── Step 4: regrow ──
    t0 = time.perf_counter()
    n_regrown = incremental_regrow(
        seeds=all_removed_seeds,
        planner=planner, forest_obj=forest_obj,
        boxes=boxes, adj=adj, uf=uf,
        obs_packed=obs_packed,
        budget=regrow_budget, rng=rng, ndim=ndim,
    )
    regrow_ms = (time.perf_counter() - t0) * 1000
    total_ms = invalidate_ms + scene_ms + uf_ms + regrow_ms
    return dict(
        n_before=n_before,
        n_after=len(forest_obj.boxes),
        n_invalidated=total_invalidated,
        n_removed_seeds=len(all_removed_seeds),
        n_regrown=n_regrown,
        invalidate_ms=invalidate_ms,
        scene_ms=scene_ms,
        uf_rebuild_ms=uf_ms,
        regrow_ms=regrow_ms,
        total_ms=total_ms,
    )


def run_method_visgraph(prep, cfg, q_start, q_goal, collision_checker, ndim):
    """运行 Visibility Graph 方法."""
    boxes = prep['boxes']
    planner = prep['planner']
    period = getattr(planner, '_period', None)

    # ── 直连检测 ──
    _res = getattr(cfg, 'segment_collision_resolution', 0.05)
    if not collision_checker.check_segment_collision(
            q_start, q_goal, resolution=_res, period=period):
        direct_cost = _geo_dist(q_start, q_goal, period)
        print(f"    [VisGraph] direct-connect OK! cost={direct_cost:.4f}")
        plan_result = dict(
            method="VisGraph", success=True, cost=direct_cost,
            waypoints=[q_start.copy(), q_goal.copy()],
            box_seq=[], plan_ms=0.0)
    else:
        plan_result = _solve_method_visgraph(
            boxes, q_start, q_goal, collision_checker,
            segment_resolution=0.05, label="VisGraph",
            period=period)

    plan_result.update(
        boxes=dict(boxes),
        adj={},
        n_before_islands=0,
        n_after_islands=0,
        bridge_edges=[],
        bridge_boxes=[],
        adj_ms=0.0,
        bridge_ms=0.0,
        grow_ms=prep['grow_ms'],
        cache_ms=prep['cache_ms'],
        coarsen_ms=prep['coarsen_ms'],
        coarsen_stats=prep['coarsen_stats'],
        n_grown=prep['n_grown'],
        n_cache_nodes=prep['n_cache_nodes'],
    )
    return plan_result
