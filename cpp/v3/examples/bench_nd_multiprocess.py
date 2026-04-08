"""
examples/bench_nd_multiprocess.py — 高维场景多进程 vs 单进程性能对比

将 ForestGrower 核心算法泛化到 N 维 C-space, 对比:
  - 单进程串行 wavefront
  - 多进程并行 wavefront (每个 root 一个子进程)

测试维度: 2D, 4D, 6D, 8D
测试规模: 500-2000 boxes

架构与 viz_2d_multiprocess.py 一致:
  主进程: Box 注册 + Sweep-and-Prune 增量邻接图
  子进程: 独立 FFBEngine + Wavefront BFS 扩展

用法:
    cd safeboxforest/v3
    python examples/bench_nd_multiprocess.py
    python examples/bench_nd_multiprocess.py --dims 2 4 6 --max-boxes 1000
"""
from __future__ import annotations

import argparse
import bisect
import multiprocessing as mp
import queue
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import numpy as np


# ═══════════════════════════════════════════════════════════════════════════
#  N 维 C-space 碰撞模型
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class ObstacleND:
    lo: np.ndarray
    hi: np.ndarray

    def overlaps(self, box_lo: np.ndarray, box_hi: np.ndarray) -> bool:
        return bool(np.all(box_lo < self.hi) and np.all(self.lo < box_hi))


class CSpaceND:
    """N 维 C-space (向量化碰撞检查)."""

    def __init__(self, q_lo: np.ndarray, q_hi: np.ndarray,
                 obstacles: List[ObstacleND]):
        self.q_lo = np.asarray(q_lo, dtype=np.float64)
        self.q_hi = np.asarray(q_hi, dtype=np.float64)
        self.n_dims = len(q_lo)
        self.obstacles = obstacles
        # 向量化: 堆叠所有障碍物为 (N_obs, n_dims) 数组
        if obstacles:
            self._obs_lo = np.array([o.lo for o in obstacles])  # (N, D)
            self._obs_hi = np.array([o.hi for o in obstacles])  # (N, D)
        else:
            self._obs_lo = np.empty((0, self.n_dims))
            self._obs_hi = np.empty((0, self.n_dims))

    def is_collision(self, q: np.ndarray) -> bool:
        if len(self._obs_lo) == 0:
            return False
        # (N, D): obs_lo <= q and q <= obs_hi  -> all dims -> any obstacle
        inside = np.all(self._obs_lo <= q, axis=1) & np.all(q <= self._obs_hi, axis=1)
        return bool(np.any(inside))

    def box_collides(self, lo: np.ndarray, hi: np.ndarray) -> bool:
        if len(self._obs_lo) == 0:
            return False
        # AABB overlap: lo < obs_hi AND obs_lo < hi in ALL dims
        overlap = np.all(lo < self._obs_hi, axis=1) & np.all(self._obs_lo < hi, axis=1)
        return bool(np.any(overlap))


# ═══════════════════════════════════════════════════════════════════════════
#  N 维 BoxInfo
# ═══════════════════════════════════════════════════════════════════════════

class BoxInfoND:
    __slots__ = ("box_id", "lo", "hi", "seed", "parent_box_id",
                 "expand_face_dim", "expand_face_side", "root_id")

    def __init__(self, box_id, lo, hi, seed,
                 parent_box_id=-1, expand_face_dim=-1,
                 expand_face_side=-1, root_id=-1):
        self.box_id = box_id
        self.lo = lo
        self.hi = hi
        self.seed = seed
        self.parent_box_id = parent_box_id
        self.expand_face_dim = expand_face_dim
        self.expand_face_side = expand_face_side
        self.root_id = root_id

    @property
    def volume(self) -> float:
        return float(np.prod(self.hi - self.lo))

    def center(self) -> np.ndarray:
        return (self.lo + self.hi) * 0.5

    def is_adjacent(self, other: "BoxInfoND", tol: float = 1e-9) -> bool:
        n = len(self.lo)
        has_touch = False
        for d in range(n):
            overlap = min(self.hi[d], other.hi[d]) - max(self.lo[d], other.lo[d])
            if overlap < -tol:
                return False
            if overlap <= tol:
                has_touch = True
        return has_touch


# ═══════════════════════════════════════════════════════════════════════════
#  N 维 持久化 KD-tree FFB
# ═══════════════════════════════════════════════════════════════════════════

class _KDNodeND:
    __slots__ = ("lo", "hi", "depth", "left", "right", "box_id", "n_dims")

    def __init__(self, lo, hi, depth=0, n_dims=2):
        self.lo = lo
        self.hi = hi
        self.depth = depth
        self.n_dims = n_dims
        self.left = None
        self.right = None
        self.box_id = -1

    @property
    def is_leaf(self):
        return self.left is None

    @property
    def is_occupied(self):
        return self.box_id >= 0


class FFBEngineND:
    """N 维持久化 KD-tree FFB."""

    def __init__(self, cspace: CSpaceND, min_edge: float = 0.03,
                 max_depth: int = 30):
        self.cspace = cspace
        self.n_dims = cspace.n_dims
        self.min_edge = min_edge
        self.max_depth = max_depth
        self.root = _KDNodeND(cspace.q_lo.copy(), cspace.q_hi.copy(),
                              depth=0, n_dims=self.n_dims)

    def is_occupied(self, q: np.ndarray) -> bool:
        node = self._find_leaf(q)
        return node is not None and node.is_occupied

    def find_free_box(self, seed: np.ndarray, box_id: int = 0) \
            -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if not (np.all(seed >= self.root.lo - 1e-10) and
                np.all(seed <= self.root.hi + 1e-10)):
            return None
        if self.cspace.is_collision(seed):
            return None
        node = self._descend_and_split(self.root, seed)
        if node is None:
            return None
        node.box_id = box_id
        return (node.lo.copy(), node.hi.copy())

    def _find_leaf(self, q):
        node = self.root
        while not node.is_leaf:
            dim = node.depth % self.n_dims
            mid = (node.lo[dim] + node.hi[dim]) * 0.5
            node = node.left if q[dim] <= mid else node.right
        return node

    def _descend_and_split(self, node, seed):
        # 迭代下降到叶子
        while not node.is_leaf:
            dim = node.depth % self.n_dims
            mid = (node.lo[dim] + node.hi[dim]) * 0.5
            node = node.left if seed[dim] <= mid else node.right

        # 迭代 split 直到找到无碰撞叶子
        while True:
            if node.is_occupied:
                return None

            if not self.cspace.box_collides(node.lo, node.hi):
                return node

            widths = node.hi - node.lo
            dim = node.depth % self.n_dims
            if widths[dim] < self.min_edge * 2:
                found = False
                for alt_dim in np.argsort(-widths):
                    if widths[alt_dim] >= self.min_edge * 2:
                        dim = int(alt_dim)
                        found = True
                        break
                if not found:
                    return None

            if node.depth >= self.max_depth:
                return None

            mid = (node.lo[dim] + node.hi[dim]) * 0.5
            left_hi = node.hi.copy(); left_hi[dim] = mid
            right_lo = node.lo.copy(); right_lo[dim] = mid

            node.left = _KDNodeND(node.lo.copy(), left_hi,
                                   node.depth + 1, self.n_dims)
            node.right = _KDNodeND(right_lo, node.hi.copy(),
                                    node.depth + 1, self.n_dims)

            node = node.left if seed[dim] <= mid else node.right


# ═══════════════════════════════════════════════════════════════════════════
#  Sweep-and-Prune 增量邻接图
# ═══════════════════════════════════════════════════════════════════════════

class SweepAdjacencyGraphND:
    """N 维 Sweep-and-Prune 增量邻接图."""

    def __init__(self, n_dims: int, tol: float = 1e-9):
        self.n_dims = n_dims
        self.tol = tol
        self.sweep_dim = 0
        self.boxes: Dict[int, BoxInfoND] = {}
        self.adj: Dict[int, Set[int]] = {}
        self._sorted_lo: List[float] = []
        self._sorted_ids: List[int] = []

    def add_box(self, box: BoxInfoND) -> List[int]:
        bid = box.box_id
        self.boxes[bid] = box
        self.adj[bid] = set()

        lo_s = float(box.lo[self.sweep_dim])
        hi_s = float(box.hi[self.sweep_dim])

        pos = bisect.bisect_left(self._sorted_lo, lo_s)
        self._sorted_lo.insert(pos, lo_s)
        self._sorted_ids.insert(pos, bid)

        new_neighbors = []

        # 向右
        j = pos + 1
        while j < len(self._sorted_lo):
            if self._sorted_lo[j] > hi_s + self.tol:
                break
            oid = self._sorted_ids[j]
            if self._check_adjacent(box, self.boxes[oid]):
                self.adj[bid].add(oid)
                self.adj[oid].add(bid)
                new_neighbors.append(oid)
            j += 1

        # 向左
        j = pos - 1
        while j >= 0:
            oid = self._sorted_ids[j]
            other = self.boxes[oid]
            if other.hi[self.sweep_dim] < lo_s - self.tol:
                break
            if self._check_adjacent(box, other):
                self.adj[bid].add(oid)
                self.adj[oid].add(bid)
                new_neighbors.append(oid)
            j -= 1

        return new_neighbors

    def rebuild(self):
        self.adj = {bid: set() for bid in self.boxes}
        items = sorted(self.boxes.items(),
                       key=lambda x: float(x[1].lo[self.sweep_dim]))
        self._sorted_ids = [bid for bid, _ in items]
        self._sorted_lo = [float(b.lo[self.sweep_dim]) for _, b in items]
        n = len(items)
        pairs = 0
        for i in range(n):
            id_i = self._sorted_ids[i]
            box_i = self.boxes[id_i]
            hi_i = float(box_i.hi[self.sweep_dim])
            for j in range(i + 1, n):
                if self._sorted_lo[j] > hi_i + self.tol:
                    break
                id_j = self._sorted_ids[j]
                if self._check_adjacent(box_i, self.boxes[id_j]):
                    self.adj[id_i].add(id_j)
                    self.adj[id_j].add(id_i)
                    pairs += 1
        return pairs

    def select_sweep_dim(self, q_lo, q_hi):
        if not self.boxes:
            return
        ranges = q_hi - q_lo
        avg_w = np.zeros(self.n_dims)
        for box in self.boxes.values():
            avg_w += (box.hi - box.lo)
        avg_w /= len(self.boxes)
        fill_ratios = avg_w / (ranges + 1e-12)
        self.sweep_dim = int(np.argmin(fill_ratios))

    def n_edges(self):
        return sum(len(v) for v in self.adj.values()) // 2

    def _check_adjacent(self, a, b):
        has_touch = False
        for d in range(self.n_dims):
            overlap = min(a.hi[d], b.hi[d]) - max(a.lo[d], b.lo[d])
            if overlap < -self.tol:
                return False
            if overlap <= self.tol:
                has_touch = True
        return has_touch


# ═══════════════════════════════════════════════════════════════════════════
#  场景生成
# ═══════════════════════════════════════════════════════════════════════════

def build_nd_scene(n_dims: int, n_obstacles: int, rng,
                   q_range: float = 3.14159) -> CSpaceND:
    """生成 N 维随机 AABB 障碍物场景."""
    q_lo = np.full(n_dims, -q_range)
    q_hi = np.full(n_dims, q_range)

    obstacles = []
    for _ in range(n_obstacles):
        cx = rng.uniform(-0.7 * q_range, 0.7 * q_range, size=n_dims)
        half_w = rng.uniform(0.3, 0.8, size=n_dims)
        lo = np.clip(cx - half_w, q_lo, q_hi)
        hi = np.clip(cx + half_w, q_lo, q_hi)
        obstacles.append(ObstacleND(lo, hi))

    return CSpaceND(q_lo, q_hi, obstacles)


# ═══════════════════════════════════════════════════════════════════════════
#  单进程 ForestGrower (N 维)
# ═══════════════════════════════════════════════════════════════════════════

class ForestGrowerND_SingleProcess:
    """N 维单进程 wavefront ForestGrower."""

    def __init__(self, cspace: CSpaceND, n_roots: int, max_boxes: int,
                 seed: int = 42, n_boundary_samples: int = 6,
                 boundary_epsilon: float = 0.02,
                 min_edge: float = 0.03, max_depth: int = 30,
                 max_miss: int = 200):
        self.cspace = cspace
        self.n_dims = cspace.n_dims
        self.n_roots = n_roots
        self.max_boxes = max_boxes
        self.rng = np.random.default_rng(seed)
        self.n_bnd = n_boundary_samples
        self.eps = boundary_epsilon
        self.max_miss = max_miss

        self.ffb = FFBEngineND(cspace, min_edge, max_depth)
        self.boxes: Dict[int, BoxInfoND] = {}
        self.adj = SweepAdjacencyGraphND(cspace.n_dims)
        self.next_id = 0

    def grow(self) -> dict:
        t0 = time.time()

        # Root 选取 (FPS)
        root_seeds = self._select_root_seeds()
        root_bids = []
        for ri, seed in enumerate(root_seeds):
            bid = self._try_create(seed, root_id=ri)
            if bid >= 0:
                root_bids.append(bid)

        # Subtree 分区 (简化: 对每个root用整个空间)
        # 单进程不需要真正分区, 直接用全局 FFB

        # Wavefront BFS
        bfs_q = deque(root_bids)
        miss = 0

        while len(self.boxes) < self.max_boxes and miss < self.max_miss:
            if bfs_q:
                bid = bfs_q.popleft()
                if bid not in self.boxes:
                    continue
                box = self.boxes[bid]
                for dim, side, q_seed in self._sample_boundary(box):
                    if len(self.boxes) >= self.max_boxes:
                        break
                    new_id = self._try_create(
                        q_seed, parent_id=box.box_id,
                        face_dim=dim, face_side=side,
                        root_id=box.root_id)
                    if new_id >= 0:
                        bfs_q.append(new_id)
                        miss = 0
                    else:
                        miss += 1
            else:
                # 随机回退
                seed = self._sample_near_boundary()
                if seed is None:
                    seed = self.rng.uniform(self.cspace.q_lo, self.cspace.q_hi)
                new_id = self._try_create(seed, root_id=0)
                if new_id >= 0:
                    bfs_q.append(new_id)
                    miss = 0
                else:
                    miss += 1

        # 跳过邻接图构建 (benchmark 只关心 box 增长速度)
        grow_elapsed = time.time() - t0

        return {
            "n_boxes": len(self.boxes),
            "n_edges": 0,
            "elapsed": grow_elapsed,
        }

    def _select_root_seeds(self):
        seeds = []
        for _ in range(100):
            s = self.rng.uniform(self.cspace.q_lo, self.cspace.q_hi)
            if not self.cspace.is_collision(s):
                seeds.append(s)
                break
        K = 50
        for r in range(1, self.n_roots):
            best, best_d = None, -1.0
            for _ in range(K):
                c = self.rng.uniform(self.cspace.q_lo, self.cspace.q_hi)
                if self.cspace.is_collision(c):
                    continue
                min_d = min((np.linalg.norm(c - s) for s in seeds), default=0)
                if min_d > best_d:
                    best_d = min_d
                    best = c
            if best is not None:
                seeds.append(best)
        return seeds

    def _try_create(self, seed, parent_id=-1, face_dim=-1,
                    face_side=-1, root_id=-1):
        bid = self.next_id
        result = self.ffb.find_free_box(seed, box_id=bid)
        if result is None:
            return -1
        lo, hi = result
        self.next_id += 1
        box = BoxInfoND(bid, lo, hi, seed.copy(), parent_id,
                        face_dim, face_side, root_id)
        self.boxes[bid] = box
        return bid

    def _sample_boundary(self, box):
        eps = self.eps
        q_lo, q_hi = self.cspace.q_lo, self.cspace.q_hi
        faces = []
        for d in range(self.n_dims):
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

        self.rng.shuffle(faces)
        seeds = []
        for s_idx in range(min(self.n_bnd, len(faces))):
            dim, side = faces[s_idx % len(faces)]
            seed = np.zeros(self.n_dims)
            for d in range(self.n_dims):
                if d == dim:
                    seed[d] = (box.lo[d] - eps) if side == 0 \
                              else (box.hi[d] + eps)
                else:
                    seed[d] = self.rng.uniform(box.lo[d], box.hi[d])
            seed = np.clip(seed, q_lo, q_hi)
            seeds.append((dim, side, seed))
        return seeds

    def _sample_near_boundary(self):
        if not self.boxes:
            return None
        bid = list(self.boxes.keys())[self.rng.integers(0, len(self.boxes))]
        box = self.boxes[bid]
        eps = self.eps
        faces = []
        for d in range(self.n_dims):
            if box.lo[d] - eps >= self.cspace.q_lo[d]:
                faces.append((d, 0))
            if box.hi[d] + eps <= self.cspace.q_hi[d]:
                faces.append((d, 1))
        if not faces:
            return None
        dim, side = faces[self.rng.integers(0, len(faces))]
        seed = np.zeros(self.n_dims)
        for d in range(self.n_dims):
            if d == dim:
                seed[d] = (box.lo[d] - eps) if side == 0 \
                          else (box.hi[d] + eps)
            else:
                seed[d] = self.rng.uniform(box.lo[d], box.hi[d])
        return np.clip(seed, self.cspace.q_lo, self.cspace.q_hi)


# ═══════════════════════════════════════════════════════════════════════════
#  多进程: Worker 子进程入口
# ═══════════════════════════════════════════════════════════════════════════

def _box_to_dict(box: BoxInfoND) -> dict:
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


def _dict_to_box(d: dict) -> BoxInfoND:
    return BoxInfoND(
        d["box_id"], np.array(d["lo"]), np.array(d["hi"]),
        np.array(d["seed"]), d["parent_box_id"],
        d["expand_face_dim"], d["expand_face_side"], d["root_id"])


def _worker_nd(
    worker_id: int,
    subtree_lo: list, subtree_hi: list,
    root_seed: list,
    obstacles_data: list,
    config: dict,
    result_queue,
    stop_event,
    id_base: int,
):
    """子进程: 在 subtree 区域内独立 wavefront 扩展."""
    try:
        n_dims = config["n_dims"]
        q_lo = np.array(subtree_lo)
        q_hi = np.array(subtree_hi)
        obstacles = [ObstacleND(np.array(o["lo"]), np.array(o["hi"]))
                     for o in obstacles_data]
        cspace = CSpaceND(q_lo, q_hi, obstacles)

        ffb = FFBEngineND(cspace, config["min_edge"], config["max_depth"])
        rng = np.random.default_rng(config["seed"] + worker_id * 1337)

        eps        = config["boundary_epsilon"]
        n_bnd      = config["n_boundary_samples"]
        max_miss   = config["max_consecutive_miss"]
        max_local  = config["max_local_boxes"]

        boxes: Dict[int, BoxInfoND] = {}
        next_id = id_base
        batch: List[dict] = []
        BATCH_SIZE = 5

        def flush():
            nonlocal batch
            if batch:
                result_queue.put(("batch", worker_id, list(batch)))
                batch.clear()

        def try_create(seed, parent_id=-1, face_dim=-1, face_side=-1):
            nonlocal next_id
            bid = next_id
            result = ffb.find_free_box(seed, box_id=bid)
            if result is None:
                return -1
            lo, hi = result
            next_id += 1
            box = BoxInfoND(bid, lo, hi, seed.copy(), parent_id,
                            face_dim, face_side, worker_id)
            boxes[bid] = box
            batch.append(_box_to_dict(box))
            if len(batch) >= BATCH_SIZE:
                flush()
            return bid

        def sample_boundary(box):
            faces = []
            for d in range(n_dims):
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
            rng.shuffle(faces)
            seeds = []
            for si in range(min(n_bnd, len(faces))):
                dim, side = faces[si % len(faces)]
                seed = np.zeros(n_dims)
                for d in range(n_dims):
                    if d == dim:
                        seed[d] = (box.lo[d] - eps) if side == 0 \
                                  else (box.hi[d] + eps)
                    else:
                        seed[d] = rng.uniform(box.lo[d], box.hi[d])
                seed = np.clip(seed, q_lo, q_hi)
                seeds.append((dim, side, seed))
            return seeds

        def sample_near_boundary():
            if not boxes:
                return None
            bid = list(boxes.keys())[rng.integers(0, len(boxes))]
            box = boxes[bid]
            faces = []
            for d in range(n_dims):
                if box.lo[d] - eps >= q_lo[d]:
                    faces.append((d, 0))
                if box.hi[d] + eps <= q_hi[d]:
                    faces.append((d, 1))
            if not faces:
                return None
            dim, side = faces[rng.integers(0, len(faces))]
            seed = np.zeros(n_dims)
            for d in range(n_dims):
                if d == dim:
                    seed[d] = (box.lo[d] - eps) if side == 0 \
                              else (box.hi[d] + eps)
                else:
                    seed[d] = rng.uniform(box.lo[d], box.hi[d])
            return np.clip(seed, q_lo, q_hi)

        # 创建 root box
        root_np = np.clip(np.array(root_seed), q_lo, q_hi)
        root_bid = try_create(root_np)
        if root_bid < 0:
            flush()
            result_queue.put(("done", worker_id, {"n_boxes": 0, "elapsed": 0}))
            return

        t0 = time.time()
        bfs_q = deque([root_bid])
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
                seed = sample_near_boundary()
                if seed is None:
                    seed = rng.uniform(q_lo, q_hi)
                new_id = try_create(seed)
                if new_id >= 0:
                    bfs_q.append(new_id)
                    miss = 0
                else:
                    miss += 1

        flush()
        elapsed = time.time() - t0
        result_queue.put(("done", worker_id, {
            "n_boxes": len(boxes), "elapsed": round(elapsed, 4),
        }))

    except Exception:
        import traceback
        result_queue.put(("error", worker_id, traceback.format_exc()))


# ═══════════════════════════════════════════════════════════════════════════
#  多进程 ForestGrower (N 维)
# ═══════════════════════════════════════════════════════════════════════════

class ForestGrowerND_MultiProcess:
    """N 维多进程 wavefront ForestGrower."""

    def __init__(self, cspace: CSpaceND, n_roots: int, max_boxes: int,
                 seed: int = 42, n_boundary_samples: int = 6,
                 boundary_epsilon: float = 0.02,
                 min_edge: float = 0.03, max_depth: int = 30,
                 max_miss: int = 200):
        self.cspace = cspace
        self.n_dims = cspace.n_dims
        self.n_roots = n_roots
        self.max_boxes = max_boxes
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.n_bnd = n_boundary_samples
        self.eps = boundary_epsilon
        self.min_edge = min_edge
        self.max_depth = max_depth
        self.max_miss = max_miss

        self.boxes: Dict[int, BoxInfoND] = {}
        self.adj = SweepAdjacencyGraphND(cspace.n_dims)

    def grow(self) -> dict:
        t0_total = time.time()

        # Root seed 选取
        root_seeds = self._select_root_seeds()
        n_workers = len(root_seeds)

        # Subtree 分区
        subtrees = self._partition_subtrees(root_seeds)

        # 序列化
        obstacles_data = [{"lo": o.lo.tolist(), "hi": o.hi.tolist()}
                          for o in self.cspace.obstacles]
        max_per_worker = max(self.max_boxes * 2 // n_workers, 50)
        id_stride = max_per_worker + 100

        config = {
            "n_dims": self.n_dims,
            "seed": self.seed,
            "min_edge": self.min_edge,
            "max_depth": self.max_depth,
            "boundary_epsilon": self.eps,
            "n_boundary_samples": self.n_bnd,
            "max_consecutive_miss": self.max_miss,
            "max_local_boxes": max_per_worker,
        }

        result_queue = mp.Queue()
        stop_event = mp.Event()

        workers = []
        for wi, st in enumerate(subtrees):
            p = mp.Process(
                target=_worker_nd,
                args=(wi, st["lo"].tolist(), st["hi"].tolist(),
                      st["seed"].tolist(), obstacles_data,
                      config, result_queue, stop_event,
                      wi * id_stride),
                name=f"worker-{wi}")
            workers.append(p)

        t0_grow = time.time()
        for p in workers:
            p.start()

        # 收集
        workers_done = 0
        worker_stats = {}
        while workers_done < n_workers:
            try:
                msg = result_queue.get(timeout=0.1)
            except queue.Empty:
                if all(not p.is_alive() for p in workers):
                    break
                continue

            if msg[0] == "batch":
                _, wid, batch = msg
                for d in batch:
                    if len(self.boxes) >= self.max_boxes:
                        stop_event.set()
                        break
                    box = _dict_to_box(d)
                    self.boxes[box.box_id] = box
            elif msg[0] == "done":
                _, wid, stats = msg
                workers_done += 1
                worker_stats[wid] = stats
            elif msg[0] == "error":
                _, wid, tb = msg
                workers_done += 1
                print(f"  worker-{wid} error: {tb}")

        for p in workers:
            p.join(timeout=5)
            if p.is_alive():
                p.terminate()

        grow_elapsed = time.time() - t0_grow

        total_elapsed = time.time() - t0_total

        return {
            "n_boxes": len(self.boxes),
            "n_edges": 0,
            "grow_elapsed": grow_elapsed,
            "total_elapsed": total_elapsed,
            "worker_stats": worker_stats,
        }

    def _select_root_seeds(self):
        seeds = []
        for _ in range(100):
            s = self.rng.uniform(self.cspace.q_lo, self.cspace.q_hi)
            if not self.cspace.is_collision(s):
                seeds.append(s)
                break
        K = 50
        for r in range(1, self.n_roots):
            best, best_d = None, -1.0
            for _ in range(K):
                c = self.rng.uniform(self.cspace.q_lo, self.cspace.q_hi)
                if self.cspace.is_collision(c):
                    continue
                min_d = min((np.linalg.norm(c - s) for s in seeds), default=0)
                if min_d > best_d:
                    best_d = min_d
                    best = c
            if best is not None:
                seeds.append(best)
        return seeds

    def _partition_subtrees(self, seeds):
        subtrees = []
        initial = (self.cspace.q_lo.copy(), self.cspace.q_hi.copy(),
                   list(range(len(seeds))))
        work = [initial]
        done = []
        max_depth = 8
        sc = 0
        while work and sc < max_depth:
            nw = []
            for clo, chi, indices in work:
                if len(indices) <= 1:
                    done.append((clo, chi, indices))
                    continue
                dim = sc % self.n_dims
                mid = (clo[dim] + chi[dim]) * 0.5
                le = [i for i in indices if seeds[i][dim] <= mid]
                ri = [i for i in indices if seeds[i][dim] > mid]
                if not le or not ri:
                    done.append((clo, chi, indices))
                else:
                    lhi = chi.copy(); lhi[dim] = mid
                    rlo = clo.copy(); rlo[dim] = mid
                    nw.append((clo.copy(), lhi, le))
                    nw.append((rlo, chi.copy(), ri))
            work = nw
            sc += 1
        done.extend(work)
        for clo, chi, indices in done:
            for i in indices:
                subtrees.append({
                    "root_id": i,
                    "lo": clo.copy(),
                    "hi": chi.copy(),
                    "seed": seeds[i].copy(),
                })
        return subtrees


# ═══════════════════════════════════════════════════════════════════════════
#  Benchmark Runner
# ═══════════════════════════════════════════════════════════════════════════

def run_benchmark(dims_list, max_boxes_list, n_roots, n_obstacles,
                  seed, repeats):
    """运行完整 benchmark 矩阵."""
    results = []

    for n_dims in dims_list:
        for max_boxes in max_boxes_list:
            print(f"\n{'─'*60}")
            print(f"  {n_dims}D | max_boxes={max_boxes} | "
                  f"roots={n_roots} | obstacles={n_obstacles}")
            print(f"{'─'*60}")

            # 增加高维的 boundary samples 和 max_miss
            n_bnd = min(2 * n_dims, 12)
            max_miss_scaled = 200 + 50 * n_dims

            sp_times = []
            mp_times = []
            sp_boxes = []
            mp_boxes = []

            for rep in range(repeats):
                rep_seed = seed + rep * 100

                # 生成场景
                rng = np.random.default_rng(rep_seed)
                # 高维需要更多障碍物来增加计算量
                n_obs_actual = n_obstacles + n_dims * 2
                cspace = build_nd_scene(n_dims, n_obs_actual, rng)

                # ── 单进程 ────────────────────────
                grower_sp = ForestGrowerND_SingleProcess(
                    cspace, n_roots, max_boxes, seed=rep_seed,
                    n_boundary_samples=n_bnd,
                    max_miss=max_miss_scaled,
                    min_edge=0.03,
                    max_depth=30 + n_dims * 2)
                res_sp = grower_sp.grow()
                sp_times.append(res_sp["elapsed"])
                sp_boxes.append(res_sp["n_boxes"])

                # ── 多进程 ────────────────────────
                grower_mp = ForestGrowerND_MultiProcess(
                    cspace, n_roots, max_boxes, seed=rep_seed,
                    n_boundary_samples=n_bnd,
                    max_miss=max_miss_scaled,
                    min_edge=0.03,
                    max_depth=30 + n_dims * 2)
                res_mp = grower_mp.grow()
                mp_times.append(res_mp["grow_elapsed"])
                mp_boxes.append(res_mp["n_boxes"])

                print(f"  rep {rep}: SP {res_sp['elapsed']:.3f}s "
                      f"({res_sp['n_boxes']} boxes) | "
                      f"MP {res_mp['grow_elapsed']:.3f}s "
                      f"({res_mp['n_boxes']} boxes)")

            avg_sp = np.mean(sp_times)
            avg_mp = np.mean(mp_times)
            speedup = avg_sp / max(avg_mp, 0.001)

            entry = {
                "n_dims": n_dims,
                "max_boxes": max_boxes,
                "n_roots": n_roots,
                "avg_sp_time": round(avg_sp, 4),
                "avg_mp_time": round(avg_mp, 4),
                "speedup": round(speedup, 2),
                "avg_sp_boxes": round(np.mean(sp_boxes)),
                "avg_mp_boxes": round(np.mean(mp_boxes)),
                "repeats": repeats,
            }
            results.append(entry)

            print(f"\n  平均: SP {avg_sp:.3f}s | MP {avg_mp:.3f}s | "
                  f"加速比 {speedup:.2f}x")

    return results


def print_results_table(results):
    """打印结果表格."""
    print(f"\n{'='*75}")
    print(f"  高维多进程 vs 单进程 Benchmark 结果")
    print(f"{'='*75}")
    print(f"{'维度':>4} | {'MaxBox':>6} | {'Roots':>5} | "
          f"{'SP(s)':>7} | {'MP(s)':>7} | {'加速比':>6} | "
          f"{'SP Boxes':>8} | {'MP Boxes':>8}")
    print(f"{'─'*4}-+-{'─'*6}-+-{'─'*5}-+-"
          f"{'─'*7}-+-{'─'*7}-+-{'─'*6}-+-"
          f"{'─'*8}-+-{'─'*8}")

    for r in results:
        sp_label = f"{r['speedup']:.2f}x"
        if r['speedup'] >= 1.5:
            sp_label += " ★"
        print(f"{r['n_dims']:>4} | {r['max_boxes']:>6} | {r['n_roots']:>5} | "
              f"{r['avg_sp_time']:>7.3f} | {r['avg_mp_time']:>7.3f} | "
              f"{sp_label:>6} | "
              f"{r['avg_sp_boxes']:>8.0f} | {r['avg_mp_boxes']:>8.0f}")

    print(f"{'='*75}")


# ═══════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="高维多进程 vs 单进程 ForestGrower Benchmark")
    parser.add_argument("--dims", type=int, nargs="+",
                        default=[2, 4, 6, 8],
                        help="测试维度列表 (default: 2 4 6 8)")
    parser.add_argument("--max-boxes", type=int, nargs="+",
                        default=[500, 1000],
                        help="最大 box 数量列表 (default: 500 1000)")
    parser.add_argument("--n-roots", type=int, default=4)
    parser.add_argument("--obstacles", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--repeats", type=int, default=3,
                        help="每组重复次数 (default: 3)")
    args = parser.parse_args()

    print("高维多进程 vs 单进程 ForestGrower Benchmark")
    print(f"  维度:       {args.dims}")
    print(f"  max_boxes:  {args.max_boxes}")
    print(f"  n_roots:    {args.n_roots}")
    print(f"  obstacles:  {args.obstacles}")
    print(f"  repeats:    {args.repeats}")
    print(f"  seed:       {args.seed}")

    results = run_benchmark(
        args.dims, args.max_boxes,
        args.n_roots, args.obstacles,
        args.seed, args.repeats)

    print_results_table(results)

    # 保存结果
    import json
    from pathlib import Path
    out_dir = Path(__file__).resolve().parent.parent / "results"
    out_dir.mkdir(parents=True, exist_ok=True)

    ts = time.strftime("%Y%m%d_%H%M%S")
    path = out_dir / f"bench_nd_{ts}.json"
    path.write_text(json.dumps({
        "config": {
            "dims": args.dims,
            "max_boxes": args.max_boxes,
            "n_roots": args.n_roots,
            "obstacles": args.obstacles,
            "seed": args.seed,
            "repeats": args.repeats,
        },
        "results": results,
    }, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n结果已保存: {path}")


if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    main()
