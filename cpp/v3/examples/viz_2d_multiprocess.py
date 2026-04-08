"""
examples/viz_2d_multiprocess.py — 多进程 ForestGrower 2D 可视化

架构 (对齐 forest_grower_plan.md §2/§5):
  主进程:
    ├── 场景创建 + 碰撞底图
    ├── Root 选取 (FPS / start-goal) + Subtree 分区
    ├── 全局 Box 注册 + Sweep-and-Prune 增量邻接图
    ├── 快照 + 渲染 + GIF 合成
    └── Worker 生命周期管理 (启动/停止/收集)
  子进程 (×N, N = n_roots):
    ├── 独立 FFBEngine (限于 subtree 区域, 无锁)
    ├── Wavefront BFS / RRT 扩展
    └── 通过 Queue 回报新 Box 数据

通信:
  result_queue : worker → main  (box 数据批次)
  stop_event   : main → workers (全局停止信号)

用法:
    cd safeboxforest/v3
    python examples/viz_2d_multiprocess.py [--seed 42] [--n-roots 3] [--max-boxes 300]
"""
from __future__ import annotations

import argparse
import bisect
import json
import multiprocessing as mp
import queue
import sys
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

# ── 导入核心类 (复用主可视化模块) ────────────────────────────────────
_parent = Path(__file__).resolve().parent
if str(_parent) not in sys.path:
    sys.path.insert(0, str(_parent))

from viz_2d_forest_grower import (
    GrowVizConfig, Obstacle2D, CSpace2D,
    FFBEngine, BoxInfo,
    build_random_scene, ROOT_COLORS,
    plot_snapshot, compose_gif,
)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── 中文字体 ─────────────────────────────────────────────────────────
for _fname in ["Microsoft YaHei", "SimHei", "SimSun", "STSong"]:
    try:
        matplotlib.font_manager.FontProperties(family=_fname).get_name()
        plt.rcParams["font.sans-serif"] = [_fname, "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        break
    except Exception:
        continue


# ═══════════════════════════════════════════════════════════════════════════
#  Sweep-and-Prune 增量邻接图 (主进程管理)
# ═══════════════════════════════════════════════════════════════════════════

class SweepAdjacencyGraph:
    """基于 Sweep-and-Prune 的增量邻接图.

    对齐 C++ AdjacencyGraph (plan §4.3 / §5.5):
    - sweep_dim: fill_ratio 最小的维度 → 最分散 → 最强剪枝
    - 增量 add_box: 二分定位 + 左右局部扫描  O(log N + K)
    - 批量 rebuild: 全量双指针 sweep          O(N·K)

    面邻接条件:
    所有维度 overlap >= -tol, 且至少一个维度 overlap <= tol.
    """

    def __init__(self, n_dims: int = 2, tol: float = 1e-9):
        self.n_dims = n_dims
        self.tol = tol
        self.sweep_dim = 0

        self.boxes: Dict[int, BoxInfo] = {}
        self.adj: Dict[int, Set[int]] = {}

        # 按 sweep_dim 排序的 lo 值和 box_id
        self._sorted_lo: List[float] = []
        self._sorted_ids: List[int] = []

        # 统计
        self.stat_sweep_checks = 0
        self.stat_full_checks = 0
        self.stat_pairs = 0

    # ── 增量添加 (plan §5.5 增量添加) ─────────────────────────────────
    def add_box(self, box: BoxInfo) -> List[int]:
        """增量添加 box, 返回新发现的邻居 ID 列表.

        1. 二分查找 sweep 维度排序位置
        2. 向右扫描: 直到 lo_j > new_hi[sweep] + tol → BREAK
        3. 向左扫描: 直到 hi_j < new_lo[sweep] - tol → BREAK
        4. 通过 sweep 剪枝的对 → 全维度邻接检查
        """
        bid = box.box_id
        self.boxes[bid] = box
        self.adj[bid] = set()

        lo_s = float(box.lo[self.sweep_dim])
        hi_s = float(box.hi[self.sweep_dim])

        # 二分插入
        pos = bisect.bisect_left(self._sorted_lo, lo_s)
        self._sorted_lo.insert(pos, lo_s)
        self._sorted_ids.insert(pos, bid)

        new_neighbors: List[int] = []

        # 向右扫描: lo_j > hi_s + tol → 不可能重叠 → break
        j = pos + 1
        while j < len(self._sorted_lo):
            if self._sorted_lo[j] > hi_s + self.tol:
                break
            self.stat_sweep_checks += 1
            oid = self._sorted_ids[j]
            self.stat_full_checks += 1
            if self._check_adjacent(box, self.boxes[oid]):
                self.adj[bid].add(oid)
                self.adj[oid].add(bid)
                new_neighbors.append(oid)
                self.stat_pairs += 1
            j += 1

        # 向左扫描: hi_j < lo_s - tol → 不可能重叠 → break (启发式)
        j = pos - 1
        while j >= 0:
            oid = self._sorted_ids[j]
            other = self.boxes[oid]
            self.stat_sweep_checks += 1
            if other.hi[self.sweep_dim] < lo_s - self.tol:
                break
            self.stat_full_checks += 1
            if self._check_adjacent(box, other):
                self.adj[bid].add(oid)
                self.adj[oid].add(bid)
                new_neighbors.append(oid)
                self.stat_pairs += 1
            j -= 1

        return new_neighbors

    # ── 批量重建 (plan §5.5 批量检测) ─────────────────────────────────
    def rebuild(self):
        """全量 Sweep-and-Prune O(N·K)."""
        self.adj = {bid: set() for bid in self.boxes}
        items = sorted(self.boxes.items(),
                       key=lambda x: float(x[1].lo[self.sweep_dim]))
        self._sorted_ids = [bid for bid, _ in items]
        self._sorted_lo  = [float(b.lo[self.sweep_dim]) for _, b in items]

        n = len(items)
        brute_pairs = n * (n - 1) // 2
        sweep_cand = 0
        self.stat_pairs = 0

        for i in range(n):
            id_i = self._sorted_ids[i]
            box_i = self.boxes[id_i]
            hi_i = float(box_i.hi[self.sweep_dim])

            for j in range(i + 1, n):
                if self._sorted_lo[j] > hi_i + self.tol:
                    break
                sweep_cand += 1
                id_j = self._sorted_ids[j]
                if self._check_adjacent(box_i, self.boxes[id_j]):
                    self.adj[id_i].add(id_j)
                    self.adj[id_j].add(id_i)
                    self.stat_pairs += 1

        prune = (1 - sweep_cand / max(brute_pairs, 1)) * 100
        print(f"  [adj-rebuild] N={n} brute={brute_pairs} "
              f"sweep_cand={sweep_cand} pairs={self.stat_pairs} "
              f"prune={prune:.1f}%")

    # ── 查询 ──────────────────────────────────────────────────────────
    def neighbors(self, box_id: int) -> Set[int]:
        return self.adj.get(box_id, set())

    def to_dict(self) -> Dict[int, Set[int]]:
        return {k: set(v) for k, v in self.adj.items()}

    def n_edges(self) -> int:
        return sum(len(v) for v in self.adj.values()) // 2

    def select_sweep_dim(self, q_lo: np.ndarray, q_hi: np.ndarray):
        """选择 fill_ratio 最小的维度 (plan §5.5 维度排名)."""
        if not self.boxes:
            return
        ranges = q_hi - q_lo
        avg_w = np.zeros(self.n_dims)
        for box in self.boxes.values():
            avg_w += (box.hi - box.lo)
        avg_w /= len(self.boxes)
        fill_ratios = avg_w / (ranges + 1e-12)
        self.sweep_dim = int(np.argmin(fill_ratios))

    # ── 面邻接检查 ────────────────────────────────────────────────────
    def _check_adjacent(self, a: BoxInfo, b: BoxInfo) -> bool:
        has_touch = False
        for d in range(self.n_dims):
            overlap = min(a.hi[d], b.hi[d]) - max(a.lo[d], b.lo[d])
            if overlap < -self.tol:
                return False
            if overlap <= self.tol:
                has_touch = True
        return has_touch


# ═══════════════════════════════════════════════════════════════════════════
#  Box 序列化 (跨进程传输, picklable)
# ═══════════════════════════════════════════════════════════════════════════

def _box_to_dict(box: BoxInfo) -> dict:
    return {
        "box_id": box.box_id,
        "lo": box.lo.tolist(),
        "hi": box.hi.tolist(),
        "seed": box.seed.tolist(),
        "parent_box_id": box.parent_box_id,
        "expand_face_dim": box.expand_face_dim,
        "expand_face_side": box.expand_face_side,
        "root_id": box.root_id,
    }


def _dict_to_box(d: dict) -> BoxInfo:
    return BoxInfo(
        box_id=d["box_id"],
        lo=np.array(d["lo"]),
        hi=np.array(d["hi"]),
        seed=np.array(d["seed"]),
        parent_box_id=d["parent_box_id"],
        expand_face_dim=d["expand_face_dim"],
        expand_face_side=d["expand_face_side"],
        root_id=d["root_id"],
    )


# ═══════════════════════════════════════════════════════════════════════════
#  子进程: 树扩展 Worker (plan §5.3 / §5.4)
# ═══════════════════════════════════════════════════════════════════════════

def _worker_expand(
    worker_id: int,
    subtree_lo: list,
    subtree_hi: list,
    root_seed: list,
    obstacles_data: list,
    config: dict,
    result_queue,          # mp.Queue: worker → main
    stop_event,            # mp.Event: main → workers
    id_base: int,
    goal_config: Optional[list],
):
    """子进程入口: 在 subtree 区域内独立执行树扩展.

    每个 Worker 拥有:
    - 独立 CSpace2D (限于 subtree 区域)
    - 独立 FFBEngine (无共享状态, 无锁)
    - 独立 RNG

    通过 result_queue 向主进程回报新 box 数据批次.
    主进程通过 stop_event 发送全局停止信号.
    """
    try:
        # ── 还原对象 ──────────────────────────────────────────────────
        obstacles = [Obstacle2D(np.array(o["lo"]), np.array(o["hi"]))
                     for o in obstacles_data]
        q_lo = np.array(subtree_lo)
        q_hi = np.array(subtree_hi)
        cspace = CSpace2D(q_lo, q_hi, obstacles)

        ffb = FFBEngine(cspace,
                        min_edge=config["min_edge"],
                        max_depth=config["max_depth"])
        rng = np.random.default_rng(config["seed"] + worker_id * 1337)

        eps           = config["boundary_epsilon"]
        n_bnd_samples = config["n_boundary_samples"]
        gf_bias       = config["goal_face_bias"]
        max_miss      = config["max_consecutive_miss"]
        max_local     = config["max_local_boxes"]
        mode          = config["mode"]
        rrt_step_r    = config.get("rrt_step_ratio", 0.15)
        rrt_goal_b    = config.get("rrt_goal_bias", 0.1)

        has_goal  = goal_config is not None
        goal_pt   = np.array(goal_config) if has_goal else None

        # ── 本地状态 ──────────────────────────────────────────────────
        boxes: Dict[int, BoxInfo] = {}
        next_id = id_base
        batch: List[dict] = []
        BATCH_SIZE = 3

        def flush():
            nonlocal batch
            if batch:
                result_queue.put(("batch", worker_id, list(batch)))
                batch.clear()

        def try_create(seed, parent_id=-1, face_dim=-1, face_side=-1):
            nonlocal next_id
            result = ffb.find_free_box(seed)
            if result is None:
                return -1
            lo, hi = result
            bid = next_id
            next_id += 1
            ffb.mark_box_id(lo, hi, bid)
            box = BoxInfo(
                box_id=bid, lo=lo, hi=hi, seed=seed.copy(),
                parent_box_id=parent_id,
                expand_face_dim=face_dim,
                expand_face_side=face_side,
                root_id=worker_id)
            boxes[bid] = box
            batch.append(_box_to_dict(box))
            if len(batch) >= BATCH_SIZE:
                flush()
            return bid

        def sample_boundary(box):
            """边界采样 (对齐 plan §5.3 / C++ sample_boundary)."""
            faces = []
            for d in range(2):
                if box.lo[d] - eps >= q_lo[d]:
                    faces.append((d, 0))
                if box.hi[d] + eps <= q_hi[d]:
                    faces.append((d, 1))
            if box.expand_face_dim >= 0:
                faces = [(d, s) for d, s in faces
                         if not (d == box.expand_face_dim
                                 and s == box.expand_face_side)]
            if not faces:
                return []

            if has_goal:
                center = box.center()
                to_goal = goal_pt - center
                faces.sort(key=lambda f: to_goal[f[0]] if f[1] == 1
                           else -to_goal[f[0]], reverse=True)
            else:
                rng.shuffle(faces)

            seeds = []
            for s_idx in range(min(n_bnd_samples, len(faces))):
                if has_goal and rng.uniform() < gf_bias and faces:
                    fi = 0
                else:
                    fi = s_idx % len(faces)
                dim, side = faces[fi]
                seed = np.zeros(2)
                for d in range(2):
                    if d == dim:
                        seed[d] = (box.lo[d] - eps) if side == 0 \
                                  else (box.hi[d] + eps)
                    else:
                        seed[d] = rng.uniform(box.lo[d], box.hi[d])
                seed = np.clip(seed, q_lo, q_hi)
                seeds.append((dim, side, seed))
            return seeds

        def sample_near_boundary():
            """从现有 box 边界随机采样 (随机回退)."""
            if not boxes:
                return None
            bid = list(boxes.keys())[rng.integers(0, len(boxes))]
            box = boxes[bid]
            faces = []
            for d in range(2):
                if box.lo[d] - eps >= q_lo[d]:
                    faces.append((d, 0))
                if box.hi[d] + eps <= q_hi[d]:
                    faces.append((d, 1))
            if not faces:
                return None
            dim, side = faces[rng.integers(0, len(faces))]
            seed = np.zeros(2)
            for d in range(2):
                if d == dim:
                    seed[d] = (box.lo[d] - eps) if side == 0 \
                              else (box.hi[d] + eps)
                else:
                    seed[d] = rng.uniform(box.lo[d], box.hi[d])
            return np.clip(seed, q_lo, q_hi)

        def find_nearest(q):
            best, best_d = None, float("inf")
            for box in boxes.values():
                d = np.linalg.norm(q - box.center())
                if d < best_d:
                    best_d, best = d, box
            return best

        # ── 创建 root box ─────────────────────────────────────────────
        root_seed_np = np.clip(np.array(root_seed), q_lo, q_hi)
        root_bid = try_create(root_seed_np)
        if root_bid < 0:
            flush()
            result_queue.put(("done", worker_id, {"n_boxes": 0, "elapsed": 0}))
            return

        t0 = time.time()

        # ── Wavefront BFS 扩展 (plan §5.3) ───────────────────────────
        if mode == "wavefront":
            bfs_q: deque = deque()
            bfs_q.append(root_bid)
            miss = 0

            while (len(boxes) < max_local
                   and miss < max_miss
                   and not stop_event.is_set()):

                if bfs_q:
                    bid = bfs_q.popleft()
                    if bid not in boxes:
                        continue
                    box = boxes[bid]
                    for dim, side, q_seed in sample_boundary(box):
                        if len(boxes) >= max_local or stop_event.is_set():
                            break
                        new_id = try_create(q_seed, parent_id=box.box_id,
                                            face_dim=dim, face_side=side)
                        if new_id >= 0:
                            bfs_q.append(new_id)
                            miss = 0
                        else:
                            miss += 1
                else:
                    # 随机回退: 从现有边界采样
                    seed = sample_near_boundary()
                    if seed is None:
                        seed = rng.uniform(q_lo, q_hi)
                    new_id = try_create(seed)
                    if new_id >= 0:
                        bfs_q.append(new_id)
                        miss = 0
                    else:
                        miss += 1

        # ── RRT 扩展 (plan §5.4) ─────────────────────────────────────
        else:
            q_range = q_hi - q_lo
            step = rrt_step_r * float(np.max(q_range))
            miss = 0

            while (len(boxes) < max_local
                   and miss < max_miss
                   and not stop_event.is_set()):

                if has_goal and rng.uniform() < rrt_goal_b:
                    q_rand = goal_pt.copy()
                else:
                    q_rand = rng.uniform(q_lo, q_hi)

                nearest = find_nearest(q_rand)
                if nearest is None:
                    miss += 1
                    continue
                nc = nearest.center()
                direction = q_rand - nc
                d_norm = np.linalg.norm(direction)
                if d_norm > 1e-12:
                    if d_norm > step:
                        direction = direction / d_norm * step
                else:
                    direction = rng.uniform(-1, 1, size=2) * step

                seed = np.clip(nc + direction, q_lo, q_hi)
                new_id = try_create(seed, parent_id=nearest.box_id)
                if new_id >= 0:
                    miss = 0
                else:
                    miss += 1

        # ── 发送剩余 + 完成信号 ───────────────────────────────────────
        flush()
        elapsed = time.time() - t0
        result_queue.put(("done", worker_id, {
            "n_boxes": len(boxes),
            "elapsed": round(elapsed, 3),
        }))

    except Exception as e:
        import traceback
        result_queue.put(("error", worker_id, traceback.format_exc()))


# ═══════════════════════════════════════════════════════════════════════════
#  MultiProcessForestGrower (主进程协调器)
# ═══════════════════════════════════════════════════════════════════════════

class MultiProcessForestGrower:
    """主进程: 协调多个子进程扩展 + 管理增量邻接图.

    对齐 plan §5 算法流程:
    1. Root 选取 (FPS / start-goal)
    2. Subtree 分区 (KD-tree 二分)
    3. 启动 N 个 Worker 子进程
    4. 收集 box 批次 → 增量邻接图
    5. 最终 Sweep-and-Prune 重建
    """

    def __init__(self, cspace: CSpace2D, cfg: GrowVizConfig):
        self.cspace = cspace
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)

        self.boxes: Dict[int, BoxInfo] = {}
        self.adj_graph = SweepAdjacencyGraph(n_dims=2)

        self.has_endpoints = (cfg.q_start is not None and cfg.q_goal is not None)
        self.start_config = np.array(cfg.q_start) if cfg.q_start else None
        self.goal_config  = np.array(cfg.q_goal)  if cfg.q_goal  else None

        # subtree 分区信息
        self.subtrees: List[dict] = []
        # root seeds (FPS 选取的点, 不一定是 box center)
        self.root_seeds: List[np.ndarray] = []

        self.snapshots: List[dict] = []

    # ── Root Seed 选取 (plan §5.1) ────────────────────────────────────
    def _select_root_seeds(self):
        """选取 n_roots 个 root seed (碰撞安全的点, 非 box)."""
        seeds = []
        cspace = self.cspace

        if self.has_endpoints:
            # root 0: start, root 1: goal
            seeds.append(self.start_config.copy())
            seeds.append(self.goal_config.copy())

            # 余下: start-goal 线上 + 高斯扰动 + FPS
            if self.cfg.n_roots > 2:
                sg = self.goal_config - self.start_config
                sg_norm = np.linalg.norm(sg)
                K = 30
                for r in range(2, self.cfg.n_roots):
                    best_cand, best_score = None, -1.0
                    for _ in range(K):
                        t = self.rng.uniform(0.1, 0.9)
                        cand = self.start_config + t * sg
                        cand += self.rng.normal(0, 0.3 * sg_norm, size=2)
                        cand = np.clip(cand, cspace.q_lo, cspace.q_hi)
                        if cspace.is_collision(cand):
                            continue
                        min_d = min((np.linalg.norm(cand - s) for s in seeds),
                                    default=0.0)
                        if min_d > best_score:
                            best_score = min_d
                            best_cand = cand
                    if best_cand is not None:
                        seeds.append(best_cand)
                    else:
                        # fallback: 随机
                        for _ in range(100):
                            cand = self.rng.uniform(cspace.q_lo, cspace.q_hi)
                            if not cspace.is_collision(cand):
                                seeds.append(cand)
                                break
        else:
            # 无端点: FPS
            for _ in range(100):
                s = self.rng.uniform(cspace.q_lo, cspace.q_hi)
                if not cspace.is_collision(s):
                    seeds.append(s)
                    break
            K = 50
            for r in range(1, self.cfg.n_roots):
                best_cand, best_score = None, -1.0
                for _ in range(K):
                    cand = self.rng.uniform(cspace.q_lo, cspace.q_hi)
                    if cspace.is_collision(cand):
                        continue
                    min_d = min((np.linalg.norm(cand - s) for s in seeds),
                                default=0.0)
                    if min_d > best_score:
                        best_score = min_d
                        best_cand = cand
                if best_cand is not None:
                    seeds.append(best_cand)

        self.root_seeds = seeds
        print(f"  [main] root seeds: {len(seeds)}")

    # ── Subtree 分区 (plan §5.2) ─────────────────────────────────────
    def _partition_subtrees(self):
        """按 root seed 位置递归二分 C-space."""
        self.subtrees.clear()
        seeds = self.root_seeds
        if not seeds:
            return

        initial = (self.cspace.q_lo.copy(), self.cspace.q_hi.copy(),
                   list(range(len(seeds))))
        work = [initial]
        done = []
        max_depth = 6
        split_count = 0

        while work and split_count < max_depth:
            next_work = []
            for (clo, chi, indices) in work:
                if len(indices) <= 1:
                    done.append((clo, chi, indices))
                    continue
                dim = split_count % 2
                mid = (clo[dim] + chi[dim]) * 0.5
                left  = [i for i in indices if seeds[i][dim] <= mid]
                right = [i for i in indices if seeds[i][dim] >  mid]
                if not left or not right:
                    done.append((clo, chi, indices))
                else:
                    lhi = chi.copy(); lhi[dim] = mid
                    rlo = clo.copy(); rlo[dim] = mid
                    next_work.append((clo.copy(), lhi, left))
                    next_work.append((rlo, chi.copy(), right))
            work = next_work
            split_count += 1
        done.extend(work)

        for clo, chi, indices in done:
            for i in indices:
                self.subtrees.append({
                    "root_id": i,
                    "lo": clo.copy(),
                    "hi": chi.copy(),
                    "seed": seeds[i].copy(),
                })

        print(f"  [main] partition: {len(seeds)} roots -> "
              f"{len(done)} cells, {split_count} splits")

    # ── 快照 ──────────────────────────────────────────────────────────
    def _take_snapshot(self, new_box_id: int):
        boxes_copy = {}
        for bid, b in self.boxes.items():
            boxes_copy[bid] = BoxInfo(
                box_id=b.box_id, lo=b.lo.copy(), hi=b.hi.copy(),
                seed=b.seed.copy(), parent_box_id=b.parent_box_id,
                expand_face_dim=b.expand_face_dim,
                expand_face_side=b.expand_face_side,
                root_id=b.root_id)
        self.snapshots.append({
            "n_boxes": len(self.boxes),
            "boxes": boxes_copy,
            "adjacency": self.adj_graph.to_dict(),
            "new_box_id": new_box_id,
        })

    # ── 主入口 ────────────────────────────────────────────────────────
    def grow(self, snapshot_every: int = 3):
        """启动多进程生长 + 增量邻接图管理."""
        # Phase 1: Root 选取 + Subtree 分区
        self._select_root_seeds()
        self._partition_subtrees()

        n_workers = len(self.subtrees)
        if n_workers == 0:
            print("  [main] 无可用 subtree, 退出")
            return self.snapshots

        # Phase 2: 序列化数据 (传给子进程)
        obstacles_data = [{"lo": o.lo.tolist(), "hi": o.hi.tolist()}
                          for o in self.cspace.obstacles]
        goal_list = self.goal_config.tolist() if self.goal_config is not None \
                    else None

        max_per_worker = max(self.cfg.max_boxes * 2 // n_workers, 50)
        id_stride = max_per_worker + 100  # 预留空间

        config = {
            "seed": self.cfg.seed,
            "mode": self.cfg.mode,
            "min_edge": self.cfg.min_edge,
            "max_depth": self.cfg.max_depth,
            "boundary_epsilon": self.cfg.boundary_epsilon,
            "n_boundary_samples": self.cfg.n_boundary_samples,
            "goal_face_bias": self.cfg.goal_face_bias,
            "max_consecutive_miss": self.cfg.max_consecutive_miss,
            "max_local_boxes": max_per_worker,
            "rrt_step_ratio": self.cfg.rrt_step_ratio,
            "rrt_goal_bias": self.cfg.rrt_goal_bias,
        }

        # Phase 3: 创建共享对象 + 启动 Workers
        result_queue = mp.Queue()
        stop_event = mp.Event()

        workers = []
        for wi, st in enumerate(self.subtrees):
            p = mp.Process(
                target=_worker_expand,
                args=(
                    wi,
                    st["lo"].tolist(),
                    st["hi"].tolist(),
                    st["seed"].tolist(),
                    obstacles_data,
                    config,
                    result_queue,
                    stop_event,
                    wi * id_stride,  # id_base
                    goal_list,
                ),
                name=f"worker-{wi}",
            )
            workers.append(p)

        print(f"  [main] 启动 {n_workers} 个 worker ...")
        t0 = time.time()
        for p in workers:
            p.start()

        # Phase 4: 收集循环 — 增量邻接图管理
        added_since_snap = 0
        workers_done = 0
        last_new_id = -1

        while workers_done < n_workers:
            try:
                msg = result_queue.get(timeout=0.2)
            except queue.Empty:
                # 检查是否所有 worker 都已退出
                if all(not p.is_alive() for p in workers):
                    break
                continue

            msg_type = msg[0]

            if msg_type == "batch":
                _, wid, batch = msg
                for box_dict in batch:
                    if len(self.boxes) >= self.cfg.max_boxes:
                        stop_event.set()
                        break
                    box = _dict_to_box(box_dict)
                    self.boxes[box.box_id] = box

                    # ── 增量邻接图更新 (主进程核心职责) ────
                    neighbors = self.adj_graph.add_box(box)
                    last_new_id = box.box_id

                    added_since_snap += 1
                    if added_since_snap >= snapshot_every:
                        self._take_snapshot(last_new_id)
                        added_since_snap = 0

            elif msg_type == "done":
                _, wid, stats = msg
                workers_done += 1
                print(f"  [main] worker-{wid} 完成: "
                      f"{stats['n_boxes']} boxes, "
                      f"{stats.get('elapsed', '?')}s")

            elif msg_type == "error":
                _, wid, tb = msg
                workers_done += 1
                print(f"  [main] worker-{wid} 错误:\n{tb}")

        # 清理
        for p in workers:
            p.join(timeout=5)
            if p.is_alive():
                p.terminate()

        elapsed = time.time() - t0

        # Phase 5: 最终 Sweep-and-Prune 重建 (修正增量期间可能的遗漏)
        print(f"  [main] 最终邻接图重建 ...")
        self.adj_graph.select_sweep_dim(self.cspace.q_lo, self.cspace.q_hi)
        self.adj_graph.rebuild()

        # 末尾快照
        if self.boxes:
            if not self.snapshots or \
                    self.snapshots[-1]["n_boxes"] != len(self.boxes):
                # 用重建后的邻接
                self._take_snapshot(last_new_id)

        print(f"  [main] 总计: {len(self.boxes)} boxes, "
              f"{self.adj_graph.n_edges()} edges, {elapsed:.2f}s")

        return self.snapshots


# ═══════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="多进程 ForestGrower 2D 可视化")
    parser.add_argument("--mode", default="wavefront",
                        choices=["wavefront", "rrt"])
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--n-roots", type=int, default=3)
    parser.add_argument("--max-boxes", type=int, default=300)
    parser.add_argument("--snap-every", type=int, default=3)
    parser.add_argument("--obstacles", type=int, default=8)
    parser.add_argument("--no-endpoints", action="store_true")
    parser.add_argument("--compare", action="store_true",
                        help="同时运行单进程版本做对比")
    args = parser.parse_args()

    cfg = GrowVizConfig(
        seed=args.seed,
        mode=args.mode,
        n_roots=args.n_roots,
        max_boxes=args.max_boxes,
        snapshot_every=args.snap_every,
        n_obstacles=args.obstacles,
    )
    if args.no_endpoints:
        cfg.q_start = None
        cfg.q_goal = None

    rng = np.random.default_rng(cfg.seed)

    # ── 输出目录 ──────────────────────────────────────────────────────
    ts = time.strftime("%Y%m%d_%H%M%S")
    out_dir = Path(__file__).resolve().parent.parent / "results" / \
              f"viz_mp_{ts}"
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    # ── 场景 ──────────────────────────────────────────────────────────
    print(f"[mp-viz] 场景生成 (seed={cfg.seed}, obs={cfg.n_obstacles}) ...")
    cspace = build_random_scene(cfg, rng)

    scene_info = {
        "q_lo": list(cfg.q_lo), "q_hi": list(cfg.q_hi),
        "n_obstacles": len(cspace.obstacles),
        "obstacles": [{"lo": o.lo.tolist(), "hi": o.hi.tolist()}
                      for o in cspace.obstacles],
    }
    (out_dir / "scene.json").write_text(
        json.dumps(scene_info, indent=2), encoding="utf-8")

    # ── 碰撞底图 ──────────────────────────────────────────────────────
    print("[mp-viz] 扫描碰撞底图 ...")
    cmap_data, extent = cspace.scan_collision_map(cfg.collision_resolution)

    # ── 多进程 ForestGrower ──────────────────────────────────────────
    print(f"[mp-viz] 多进程生长 ({cfg.mode}, roots={cfg.n_roots}, "
          f"max_boxes={cfg.max_boxes}) ...")
    t0 = time.time()
    grower = MultiProcessForestGrower(cspace, cfg)
    snapshots = grower.grow(snapshot_every=cfg.snapshot_every)
    mp_elapsed = time.time() - t0
    mp_n_boxes = len(grower.boxes)
    mp_n_edges = grower.adj_graph.n_edges()
    print(f"  多进程: {mp_n_boxes} boxes, {len(snapshots)} snapshots, "
          f"{mp_elapsed:.2f}s")

    # ── 单进程对比 (可选) ─────────────────────────────────────────────
    sp_elapsed = None
    if args.compare:
        from viz_2d_forest_grower import ForestGrower2D
        print("[mp-viz] 单进程对比 ...")
        t0 = time.time()
        sp_grower = ForestGrower2D(cspace, cfg)
        sp_grower.grow(snapshot_every=cfg.snapshot_every)
        sp_elapsed = time.time() - t0
        print(f"  单进程: {len(sp_grower.boxes)} boxes, "
              f"{len(sp_grower.snapshots)} snapshots, {sp_elapsed:.2f}s")

    # ── 渲染帧 ────────────────────────────────────────────────────────
    print(f"[mp-viz] 渲染 {len(snapshots)} 帧 ...")
    for idx, snap in enumerate(snapshots):
        fig = plot_snapshot(snap, cmap_data, extent, cfg, idx,
                            grower.subtrees)
        fig.savefig(frames_dir / f"frame_{idx:04d}.png",
                    dpi=cfg.dpi, bbox_inches="tight")
        plt.close(fig)

    # ── GIF ────────────────────────────────────────────────────────────
    gif_path = out_dir / "forest_grower_mp.gif"
    print("[mp-viz] 合成 GIF ...")
    ok = compose_gif(frames_dir, gif_path, cfg.gif_frame_ms)

    # ── 碰撞底图 ──────────────────────────────────────────────────────
    fig_c, ax_c = plt.subplots(1, 1, figsize=(8, 6))
    ax_c.imshow(cmap_data, origin="lower", extent=extent,
                cmap="Reds", alpha=0.85, aspect="auto")
    ax_c.set_xlabel("q0"); ax_c.set_ylabel("q1")
    ax_c.set_title("C-space Collision Map"); ax_c.grid(True, alpha=0.2)
    fig_c.savefig(out_dir / "collision_map.png",
                  dpi=cfg.dpi, bbox_inches="tight")
    plt.close(fig_c)

    # ── 最终总览图 ────────────────────────────────────────────────────
    if snapshots:
        fig_final = plot_snapshot(
            snapshots[-1], cmap_data, extent, cfg,
            len(snapshots) - 1, grower.subtrees)
        fig_final.savefig(out_dir / "final_forest.png",
                          dpi=cfg.dpi, bbox_inches="tight")
        plt.close(fig_final)

    # ── Summary ───────────────────────────────────────────────────────
    summary = {
        "architecture": "multi-process",
        "n_workers": len(grower.subtrees),
        "config": {
            "seed": cfg.seed, "mode": cfg.mode,
            "n_roots": cfg.n_roots, "max_boxes": cfg.max_boxes,
            "n_obstacles": cfg.n_obstacles,
            "snapshot_every": cfg.snapshot_every,
        },
        "result": {
            "n_boxes": mp_n_boxes,
            "n_adj_edges": mp_n_edges,
            "n_snapshots": len(snapshots),
            "elapsed_s": round(mp_elapsed, 3),
        },
        "adjacency_stats": {
            "sweep_dim": grower.adj_graph.sweep_dim,
            "incremental_sweep_checks": grower.adj_graph.stat_sweep_checks,
            "incremental_full_checks": grower.adj_graph.stat_full_checks,
        },
    }
    if sp_elapsed is not None:
        summary["single_process_elapsed_s"] = round(sp_elapsed, 3)
        summary["speedup"] = round(sp_elapsed / max(mp_elapsed, 0.001), 2)

    (out_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    # ── README ────────────────────────────────────────────────────────
    md = [
        "# 多进程 ForestGrower 2D 可视化",
        "",
        "## 架构",
        "```",
        "主进程: 全局 Box 注册 + Sweep-and-Prune 增量邻接图 + 快照/渲染",
        f"子进程: {len(grower.subtrees)} 个 Worker, "
        "各自独立 FFBEngine + Wavefront/RRT",
        "通信: result_queue (worker→main) + stop_event (main→workers)",
        "```",
        "",
        "## 参数",
        f"- mode: {cfg.mode}",
        f"- seed: {cfg.seed}",
        f"- n_roots (workers): {cfg.n_roots}",
        f"- max_boxes: {cfg.max_boxes}",
        f"- n_obstacles: {cfg.n_obstacles}",
        "",
        "## 结果",
        f"- boxes: {mp_n_boxes}",
        f"- adj edges: {mp_n_edges}",
        f"- snapshots: {len(snapshots)}",
        f"- elapsed: {mp_elapsed:.2f}s",
    ]
    if sp_elapsed is not None:
        md.extend([
            "",
            "## 与单进程对比",
            f"- 单进程: {sp_elapsed:.2f}s",
            f"- 多进程: {mp_elapsed:.2f}s",
            f"- speedup: {sp_elapsed / max(mp_elapsed, 0.001):.2f}x",
        ])
    md.extend([
        "",
        "## 文件",
        "- `forest_grower_mp.gif` — 多进程生长动画",
        "- `final_forest.png` — 最终 forest",
        "- `collision_map.png` — 碰撞底图",
        "- `scene.json` — 场景定义",
        "- `summary.json` — 运行摘要 + 邻接统计",
        "- `frames/` — 逐帧 PNG",
        "",
        f"![forest_grower_mp](forest_grower_mp.gif)",
    ])
    (out_dir / "README.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    # ── 输出 ──────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"输出目录: {out_dir}")
    print(f"  workers:    {len(grower.subtrees)}")
    print(f"  boxes:      {mp_n_boxes}")
    print(f"  adj edges:  {mp_n_edges}")
    print(f"  snapshots:  {len(snapshots)}")
    print(f"  elapsed:    {mp_elapsed:.2f}s")
    if sp_elapsed is not None:
        print(f"  single-proc: {sp_elapsed:.2f}s")
        print(f"  speedup:     {sp_elapsed / max(mp_elapsed, 0.001):.2f}x")
    if ok:
        print(f"  gif:        {gif_path}")
    print(f"{'='*60}")


if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    main()
