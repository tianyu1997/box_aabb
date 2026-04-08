"""
src/viz/forest_grower_2d.py — 2D ForestGrower 模拟器

提供: ForestGrower2D, build_random_scene
"""
from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from .core import GrowVizConfig, Obstacle2D, CSpace2D, BoxInfo
from .ffb_engine import FFBEngine


# ═══════════════════════════════════════════════════════════════════════════
#  ForestGrower2D — Python 2D 模拟 (与 C++ 对齐)
# ═══════════════════════════════════════════════════════════════════════════

class ForestGrower2D:
    """2D 多根 Forest 生长器, 实现与 C++ ForestGrower 相同的算法流程."""

    def __init__(self, cspace: CSpace2D, cfg: GrowVizConfig):
        self.cspace = cspace
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.ffb = FFBEngine(cspace, cfg.min_edge, cfg.max_depth)

        # adaptive: in_coarse_phase 在 grow() 中 roots 创建完毕后才开启
        # 确保 root boxes 用 fine min_edge 创建且不被标记为 coarse
        self.in_coarse_phase = False

        self.boxes: Dict[int, BoxInfo] = {}
        self.adjacency: Dict[int, Set[int]] = {}
        self.next_id = 0

        self.has_endpoints = (cfg.q_start is not None and cfg.q_goal is not None)
        self.start_config = np.array(cfg.q_start) if cfg.q_start else None
        self.goal_config = np.array(cfg.q_goal) if cfg.q_goal else None
        self.start_box_id = -1
        self.goal_box_id = -1

        # subtree partitions
        self.subtrees: List[dict] = []  # [{"root_id": int, "lo": array, "hi": array}]

        # adaptive min_edge state
        self.n_coarse_boxes = 0
        self.n_fine_boxes = 0

        # 快照
        self.snapshots: List[dict] = []

    def _effective_min_edge(self) -> float:
        """当前生效的 min_edge (adaptive 两阶段)."""
        if self.cfg.adaptive_min_edge and self.in_coarse_phase:
            return self.cfg.coarse_min_edge
        return self.cfg.min_edge

    # ── 核心入口 ────────────────────────────────────────────────────────
    def grow(self, snapshot_every: int = 1):
        """运行完整 grow 流程并收集快照. snapshot_every=1 表示每个 box 都记录."""
        # roots 用 fine min_edge 创建 (保证不被大 min_edge 阻塞)
        self._select_roots()
        self._partition_subtrees()

        # adaptive: roots 创建完毕后, 开启 coarse phase
        # 后续 wavefront/RRT 扩展将先产生大 box, 再切细
        if self.cfg.adaptive_min_edge:
            self.in_coarse_phase = True
            self.ffb.set_effective_min_edge(self.cfg.coarse_min_edge)

        if self.cfg.n_threads > 1:
            self._grow_parallel(snapshot_every)
        elif self.cfg.mode == "wavefront":
            self._grow_wavefront(snapshot_every)
        else:
            self._grow_rrt(snapshot_every)

        # 确保末尾有快照
        if not self.snapshots or self.snapshots[-1]["n_boxes"] != len(self.boxes):
            self._take_snapshot(-1, force=True)

        return self.snapshots

    # ── 根选择 (对齐 C++ select_roots) ──────────────────────────────────
    def _select_roots(self):
        if self.has_endpoints:
            self._select_roots_with_endpoints()
        else:
            self._select_roots_no_endpoints()

    def _select_roots_no_endpoints(self):
        # 第一个根: 随机
        for _ in range(100):
            seed = self._sample_random()
            bid = self._try_create_box(seed, root_id=0)
            if bid >= 0:
                break

        # FPS 选余下的根
        K = 50
        for r in range(1, self.cfg.n_roots):
            candidates = [self._sample_random() for _ in range(K)]
            root_centers = [b.center() for b in self.boxes.values()
                           if b.parent_box_id == -1]
            best_k, best_score = 0, -1.0
            for k, c in enumerate(candidates):
                min_d = min((np.linalg.norm(c - rc) for rc in root_centers),
                            default=0.0)
                if min_d > best_score:
                    best_score = min_d
                    best_k = k
            self._try_create_box(candidates[best_k], root_id=r)

    def _select_roots_with_endpoints(self):
        # root 0: start
        bid = self._try_create_box(self.start_config, root_id=0)
        if bid >= 0:
            self.start_box_id = bid

        # root 1: goal
        bid = self._try_create_box(self.goal_config, root_id=1)
        if bid >= 0:
            self.goal_box_id = bid

        # 余下根: 全空间 FPS (Farthest Point Sampling)
        # 不再沿 start→goal 线采样, 确保 roots 均匀分散在全 C-space
        if self.cfg.n_roots > 2:
            root_centers = [b.center() for b in self.boxes.values()]
            K = 80  # 候选数量
            for r in range(2, self.cfg.n_roots):
                best_cand, best_score = None, -1.0
                for _ in range(K):
                    cand = self.rng.uniform(self.cspace.q_lo, self.cspace.q_hi)
                    if self.cspace.is_collision(cand):
                        continue
                    min_d = min((np.linalg.norm(cand - rc) for rc in root_centers),
                                default=0.0)
                    if min_d > best_score:
                        best_score = min_d
                        best_cand = cand
                if best_cand is not None:
                    bid = self._try_create_box(best_cand, root_id=r)
                    if bid >= 0:
                        root_centers.append(self.boxes[bid].center())

    # ── subtree 分区 (对齐 C++ partition_subtrees) ──────────────────────
    def _partition_subtrees(self):
        self.subtrees.clear()
        roots = [(b.root_id, b.seed.copy())
                 for b in self.boxes.values() if b.parent_box_id == -1]
        if not roots:
            return

        initial = (self.cspace.q_lo.copy(), self.cspace.q_hi.copy(),
                   list(range(len(roots))))
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
                left_idx = [i for i in indices if roots[i][1][dim] <= mid]
                right_idx = [i for i in indices if roots[i][1][dim] > mid]
                if not left_idx or not right_idx:
                    done.append((clo, chi, indices))
                else:
                    lhi = chi.copy(); lhi[dim] = mid
                    rlo = clo.copy(); rlo[dim] = mid
                    next_work.append((clo.copy(), lhi, left_idx))
                    next_work.append((rlo, chi.copy(), right_idx))
            work = next_work
            split_count += 1

        done.extend(work)

        # root_id -> subtree region
        for clo, chi, indices in done:
            for i in indices:
                self.subtrees.append({
                    "root_id": roots[i][0],
                    "lo": clo.copy(),
                    "hi": chi.copy(),
                })

        print(f"[grower] partition: {len(roots)} roots -> "
              f"{len(done)} cells, {split_count} splits")

    # ── Wavefront BFS 扩展 (对齐 C++ grow_wavefront) ───────────────────
    def _grow_wavefront(self, snap_every: int):
        queue = deque()  # (box_id, from_face_dim, from_face_side)
        for bid in list(self.boxes.keys()):
            queue.append((bid, -1, -1))

        miss_count = 0
        added_since_snap = 0

        while (len(self.boxes) < self.cfg.max_boxes
               and miss_count < self.cfg.max_consecutive_miss):

            # adaptive min_edge: phase transition check
            self._check_phase_transition(queue)
            self.ffb.set_effective_min_edge(self._effective_min_edge())

            if queue:
                box_id, _, _ = queue.popleft()
                if box_id not in self.boxes:
                    continue
                box = self.boxes[box_id]

                seeds = self._sample_boundary(box)
                for dim, side, q_seed in seeds:
                    if len(self.boxes) >= self.cfg.max_boxes:
                        break
                    bid = self._try_create_box(
                        q_seed,
                        parent_box_id=box.box_id,
                        face_dim=dim, face_side=side,
                        root_id=box.root_id)
                    if bid >= 0:
                        self._update_adjacency(bid)
                        queue.append((bid, dim, 1 - side))
                        miss_count = 0
                        added_since_snap += 1
                        if added_since_snap >= snap_every:
                            self._take_snapshot(bid)
                            added_since_snap = 0
                    else:
                        miss_count += 1
            else:
                # coarse phase 且 queue 已空:
                # 先尝试随机边界采样填充(保持 coarse), 限定尝试次数
                # 只有反复失败后才 force 切换到 fine phase
                if self.cfg.adaptive_min_edge and self.in_coarse_phase:
                    coarse_refill_ok = False
                    for _ in range(30):
                        seed = self._sample_near_existing_boundary()
                        if seed is None:
                            seed = self._sample_random_in_subtree()
                        nearest = self._find_nearest_box(seed)
                        inherit_rid = nearest.root_id if nearest else -1
                        bid = self._try_create_box(seed, root_id=inherit_rid)
                        if bid >= 0:
                            self._update_adjacency(bid)
                            queue.append((bid, -1, -1))
                            miss_count = 0
                            coarse_refill_ok = True
                            added_since_snap += 1
                            if added_since_snap >= snap_every:
                                self._take_snapshot(bid)
                                added_since_snap = 0
                            break
                    if coarse_refill_ok:
                        continue  # 继续 coarse 扩展
                    # 确实无法再产出 coarse box → 切换到 fine
                    self._check_phase_transition(queue, force=True)
                    continue

                # random fallback: 从现有 box 的边界采样 (而非纯随机)
                # 这保证新 box 紧邻已有 tree, 避免在空间中产生孤立碎片
                seed = self._sample_near_existing_boundary()
                if seed is None:
                    seed = self._sample_random_in_subtree()
                # 继承最近 box 的 root_id
                nearest = self._find_nearest_box(seed)
                inherit_rid = nearest.root_id if nearest else -1
                bid = self._try_create_box(seed, root_id=inherit_rid)
                if bid >= 0:
                    self._update_adjacency(bid)
                    queue.append((bid, -1, -1))
                    miss_count = 0
                    added_since_snap += 1
                    if added_since_snap >= snap_every:
                        self._take_snapshot(bid)
                        added_since_snap = 0
                else:
                    miss_count += 1

    def _check_phase_transition(self, queue=None, force=False):
        """检查是否需要从 coarse → fine phase (对齐 C++ grow_wavefront).

        force=True 时不看 threshold, 直接切换 (当 coarse queue 耗尽时使用).
        """
        if not self.cfg.adaptive_min_edge or not self.in_coarse_phase:
            return
        threshold = int(self.cfg.coarse_fraction * self.cfg.max_boxes)
        if len(self.boxes) >= threshold or force:
            self.in_coarse_phase = False
            # n_coarse_boxes 已在 _try_create_box 中正确累计, 不再覆写
            self.ffb.set_effective_min_edge(self.cfg.min_edge)
            # re-queue all boxes for fine expansion (对齐 C++ 的 wavefront re-queue)
            if queue is not None:
                queue.clear()
                for bid in self.boxes:
                    queue.append((bid, -1, -1))

    # ── 多线程 round-robin 模拟 (对齐 C++ grow_parallel) ───────────────
    def _grow_parallel(self, snap_every: int):
        """模拟多线程: 每轮各 subtree 各扩展一步 (round-robin).

        与 C++ grow_parallel 对齐:
        - 每个 subtree 独立 queue
        - 每轮 round-robin, 每个 subtree 扩展 1 个 box
        - queue 空时用 boundary random fallback
        - global miss 计数器控制终止
        """
        # 每个 subtree 一个独立队列
        subtree_queues: Dict[int, deque] = {}
        for bid, box in self.boxes.items():
            rid = box.root_id
            if rid not in subtree_queues:
                subtree_queues[rid] = deque()
            subtree_queues[rid].append((bid, -1, -1))

        global_miss = 0
        added_since_snap = 0
        all_rids = sorted(subtree_queues.keys())

        while (len(self.boxes) < self.cfg.max_boxes
               and global_miss < self.cfg.max_consecutive_miss):

            # adaptive phase transition
            self._check_phase_transition()
            self.ffb.set_effective_min_edge(self._effective_min_edge())

            round_added = 0
            # round-robin: 每个 subtree 各扩展 1 个 box
            for rid in all_rids:
                if len(self.boxes) >= self.cfg.max_boxes:
                    break
                q = subtree_queues.setdefault(rid, deque())

                expanded = False
                # 从 queue 中尝试扩展 (只 pop 一个, 失败放回尾部)
                tried = 0
                while q and tried < len(q):
                    tried += 1
                    box_id, fd, fs = q.popleft()
                    if box_id not in self.boxes:
                        continue
                    box = self.boxes[box_id]
                    seeds = self._sample_boundary(box)
                    for dim, side, q_seed in seeds:
                        if len(self.boxes) >= self.cfg.max_boxes:
                            break
                        bid = self._try_create_box(
                            q_seed,
                            parent_box_id=box.box_id,
                            face_dim=dim, face_side=side,
                            root_id=box.root_id)
                        if bid >= 0:
                            self._update_adjacency(bid)
                            q.append((bid, dim, 1 - side))
                            expanded = True
                            round_added += 1
                            added_since_snap += 1
                            break
                    if not expanded:
                        q.append((box_id, fd, fs))  # put back
                    if expanded:
                        break

                # random fallback: 从该 subtree 的已有 box 边界采样
                if not expanded:
                    for _ in range(5):
                        seed = self._sample_near_boundary_in_subtree(rid)
                        if seed is None:
                            break
                        bid = self._try_create_box(seed, root_id=rid)
                        if bid >= 0:
                            self._update_adjacency(bid)
                            q.append((bid, -1, -1))
                            expanded = True
                            round_added += 1
                            added_since_snap += 1
                            break

            if round_added > 0:
                global_miss = 0
            else:
                global_miss += 1

            if added_since_snap >= snap_every:
                self._take_snapshot(-1)
                added_since_snap = 0

    def _sample_near_boundary_in_subtree(self, root_id: int) -> Optional[np.ndarray]:
        """从指定 subtree 的已有 box 边界采样."""
        own_boxes = [b for b in self.boxes.values() if b.root_id == root_id]
        if not own_boxes:
            return None
        box = own_boxes[self.rng.integers(0, len(own_boxes))]
        eps = self.cfg.boundary_epsilon
        q_lo, q_hi = self.cspace.q_lo, self.cspace.q_hi
        faces = []
        for d in range(2):
            if box.lo[d] - eps >= q_lo[d]:
                faces.append((d, 0))
            if box.hi[d] + eps <= q_hi[d]:
                faces.append((d, 1))
        if not faces:
            return None
        dim, side = faces[self.rng.integers(0, len(faces))]
        seed = np.zeros(2)
        for d in range(2):
            if d == dim:
                seed[d] = (box.lo[d] - eps) if side == 0 else (box.hi[d] + eps)
            else:
                seed[d] = self.rng.uniform(box.lo[d], box.hi[d])
        return np.clip(seed, q_lo, q_hi)

    # ── RRT 扩展 (对齐 C++ grow_parallel + grow_rrt, round-robin) ───────
    def _grow_rrt(self, snap_every: int):
        """多树 RRT: round-robin 每棵树各扩展一步, nearest 限定在该树内.

        对齐 C++ grow_parallel 行为 — 每棵子树拥有独立 worker,
        find_nearest_box 只搜索自身 boxes, 确保各树均衡增长.

        每棵树每轮最多尝试 max_attempts_per_tree 次 (模拟 C++ worker 独立
        运行多次迭代), 直到成功放置一个 box 或耗尽尝试. 这避免了命中率
        低的树始终被跳过, 确保各树视觉上均衡增长.
        """
        miss_count = 0
        added_since_snap = 0
        q_range = self.cspace.q_hi - self.cspace.q_lo
        step = self.cfg.rrt_step_ratio * float(np.max(q_range))

        # 每棵树每轮最多尝试次数 (模拟 C++ worker 独立跑多步)
        max_attempts_per_tree = 20

        # 收集所有 root_id
        all_rids = sorted({b.root_id for b in self.boxes.values()})
        per_tree_miss: dict = {rid: 0 for rid in all_rids}

        while (len(self.boxes) < self.cfg.max_boxes
               and miss_count < self.cfg.max_consecutive_miss):

            # adaptive min_edge: phase transition (与 wavefront 对齐)
            self._check_phase_transition()
            self.ffb.set_effective_min_edge(self._effective_min_edge())

            round_added = 0

            # ── round-robin: 每棵树各尝试放置 1 个 box ──────────────
            for rid in all_rids:
                if len(self.boxes) >= self.cfg.max_boxes:
                    break

                placed = False
                for _attempt in range(max_attempts_per_tree):
                    # sample target — 该树的 bias 目标 or 该树 subtree 区域随机
                    if (self.has_endpoints
                            and self.rng.uniform() < self.cfg.rrt_goal_bias):
                        q_rand = self._get_bias_target(rid)
                        if q_rand is None:
                            q_rand = self._sample_random_in_subtree_for(rid)
                    else:
                        q_rand = self._sample_random_in_subtree_for(rid)

                    # nearest box — 仅在该树内搜索 (对齐 C++ per-subtree worker)
                    nearest = self._find_nearest_box(q_rand, root_id=rid)
                    if nearest is None:
                        continue

                    # 计算方向向量
                    nc = nearest.center()
                    direction = q_rand - nc
                    d_norm = np.linalg.norm(direction)
                    if d_norm < 1e-12:
                        direction = self.rng.uniform(-1, 1, size=2)
                        d_norm = np.linalg.norm(direction)
                        if d_norm < 1e-12:
                            continue
                    direction = direction / d_norm

                    # boundary-snap: 对齐 C++ rrt_snap_to_face()
                    snap = self._rrt_snap_to_face(nearest, direction, step)
                    bid = self._try_create_box(
                        snap["seed"],
                        parent_box_id=nearest.box_id,
                        face_dim=snap["face_dim"],
                        face_side=snap["face_side"],
                        root_id=nearest.root_id)

                    if bid >= 0:
                        self._update_adjacency(bid)
                        per_tree_miss[rid] = 0
                        round_added += 1
                        added_since_snap += 1
                        placed = True
                        break

                if not placed:
                    per_tree_miss[rid] += 1

            # global miss 计数
            if round_added > 0:
                miss_count = 0
            else:
                miss_count += len(all_rids)  # 一轮全 miss

                # adaptive: coarse 阶段连续 miss 过多 → 强制切换到 fine
                # (对齐 C++ wavefront: coarse queue 耗尽 → 切 fine 重新开始)
                if (self.cfg.adaptive_min_edge and self.in_coarse_phase
                        and miss_count >= 30):
                    self._check_phase_transition(force=True)
                    self.ffb.set_effective_min_edge(self._effective_min_edge())
                    miss_count = 0          # 重置, 给 fine 阶段机会

            if added_since_snap >= snap_every:
                self._take_snapshot(-1)
                added_since_snap = 0

    # ── RRT boundary-snap (对齐 C++ rrt_snap_to_face) ───────────────────
    def _rrt_snap_to_face(self, nearest: BoxInfo,
                          direction: np.ndarray,
                          step: float) -> dict:
        """找到 nearest box 上与 direction 最对齐的面, 将 seed 放在该面上.

        返回 {"seed": np.ndarray, "face_dim": int, "face_side": int}.
        无可用面时回退到纯方向延伸, face_dim = -1.
        """
        eps = self.cfg.boundary_epsilon
        q_lo, q_hi = self.cspace.q_lo, self.cspace.q_hi
        nc = nearest.center()

        # 找最佳面
        best_dim, best_side, best_score = -1, -1, -float("inf")
        for d in range(2):
            for side in (0, 1):
                normal_sign = -1.0 if side == 0 else 1.0
                score = direction[d] * normal_sign
                if score <= 0:
                    continue
                if side == 0 and nearest.lo[d] - eps < q_lo[d]:
                    continue
                if side == 1 and nearest.hi[d] + eps > q_hi[d]:
                    continue
                if score > best_score:
                    best_score = score
                    best_dim, best_side = d, side

        if best_dim < 0:
            # 无可用面: 纯方向延伸
            seed = np.clip(nc + direction * step, q_lo, q_hi)
            return {"seed": seed, "face_dim": -1, "face_side": -1}

        # 在最佳面上生成 seed (70% 方向偏移 + 30% 随机)
        seed = np.zeros(2)
        for d in range(2):
            if d == best_dim:
                seed[d] = (nearest.lo[d] - eps if best_side == 0
                           else nearest.hi[d] + eps)
            else:
                face_lo = nearest.lo[d]
                face_hi = nearest.hi[d]
                target_on_face = np.clip(nc[d] + direction[d] * step,
                                         face_lo, face_hi)
                rand_on_face = self.rng.uniform(face_lo, face_hi)
                seed[d] = 0.7 * target_on_face + 0.3 * rand_on_face
        seed = np.clip(seed, q_lo, q_hi)
        return {"seed": seed, "face_dim": best_dim, "face_side": best_side}

    # ── 边界采样 (对齐 C++ sample_boundary) ─────────────────────────────
    def _sample_boundary(self, box: BoxInfo):
        """返回 [(dim, side, seed_config), ...] 列表."""
        eps = self.cfg.boundary_epsilon
        q_lo, q_hi = self.cspace.q_lo, self.cspace.q_hi
        seeds = []

        # 枚举有效面
        faces = []
        for d in range(2):
            if box.lo[d] - eps >= q_lo[d]:
                faces.append((d, 0))
            if box.hi[d] + eps <= q_hi[d]:
                faces.append((d, 1))

        # 跳过来源面
        if box.expand_face_dim >= 0:
            faces = [(d, s) for (d, s) in faces
                     if not (d == box.expand_face_dim and s == box.expand_face_side)]

        if not faces:
            return seeds

        # goal-directed 排序 — 使用该树的 bias 目标 (非全局 goal)
        bias_target = self._get_bias_target(box.root_id)
        if bias_target is not None:
            center = box.center()
            to_target = bias_target - center

            def face_priority(face):
                d, s = face
                return to_target[d] if s == 1 else -to_target[d]

            faces.sort(key=face_priority, reverse=True)
        else:
            self.rng.shuffle(faces)

        n_samples = min(self.cfg.n_boundary_samples, len(faces))
        for s_idx in range(n_samples):
            # goal face bias
            if (self.has_endpoints
                    and self.rng.uniform() < self.cfg.goal_face_bias
                    and faces):
                face_idx = 0
            else:
                face_idx = s_idx % len(faces)

            dim, side = faces[face_idx]
            seed = np.zeros(2)
            for d in range(2):
                if d == dim:
                    seed[d] = (box.lo[d] - eps) if side == 0 else (box.hi[d] + eps)
                else:
                    seed[d] = self.rng.uniform(box.lo[d], box.hi[d])

            seed = np.clip(seed, q_lo, q_hi)
            seeds.append((dim, side, seed))

        return seeds

    # ── 辅助函数 ────────────────────────────────────────────────────────
    def _get_bias_target(self, root_id: int) -> Optional[np.ndarray]:
        """根据 root_id 返回该树的 goal-bias 目标点.

        多树 RRT 中, 每棵树的 "goal" 不同:
          - root 0 (start 树): 朝 goal_config 生长
          - root 1 (goal 树):  朝 start_config 生长
          - root ≥2 (随机树):  随机选 start 或 goal (各 50%)
        无端点时返回 None.
        """
        if not self.has_endpoints:
            return None
        if root_id == 0:
            return self.goal_config.copy()
        elif root_id == 1:
            return self.start_config.copy()
        else:
            # 随机树: 50% start, 50% goal
            if self.rng.uniform() < 0.5:
                return self.start_config.copy()
            else:
                return self.goal_config.copy()

    def _sample_random(self) -> np.ndarray:
        return self.rng.uniform(self.cspace.q_lo, self.cspace.q_hi)

    def _sample_random_in_subtree(self) -> np.ndarray:
        if self.subtrees:
            st = self.subtrees[self.rng.integers(0, len(self.subtrees))]
            return self.rng.uniform(st["lo"], st["hi"])
        return self._sample_random()

    def _sample_random_in_subtree_for(self, root_id: int) -> np.ndarray:
        """在指定 root_id 的 subtree 区域内随机采样 (对齐 C++ sample_random_in_subtree)."""
        for st in self.subtrees:
            if st["root_id"] == root_id:
                return self.rng.uniform(st["lo"], st["hi"])
        return self._sample_random()

    def _sample_near_existing_boundary(self) -> Optional[np.ndarray]:
        """从现有 box 的边界面上随机采样一个 seed (紧邻已有 tree)."""
        if not self.boxes:
            return None
        # 随机选一个现有 box
        bid = list(self.boxes.keys())[self.rng.integers(0, len(self.boxes))]
        box = self.boxes[bid]
        eps = self.cfg.boundary_epsilon
        q_lo, q_hi = self.cspace.q_lo, self.cspace.q_hi
        # 枚举有效 face
        faces = []
        for d in range(2):
            if box.lo[d] - eps >= q_lo[d]:
                faces.append((d, 0))
            if box.hi[d] + eps <= q_hi[d]:
                faces.append((d, 1))
        if not faces:
            return None
        dim, side = faces[self.rng.integers(0, len(faces))]
        seed = np.zeros(2)
        for d in range(2):
            if d == dim:
                seed[d] = (box.lo[d] - eps) if side == 0 else (box.hi[d] + eps)
            else:
                seed[d] = self.rng.uniform(box.lo[d], box.hi[d])
        return np.clip(seed, q_lo, q_hi)

    def _try_create_box(self, seed: np.ndarray,
                        parent_box_id: int = -1,
                        face_dim: int = -1, face_side: int = -1,
                        root_id: int = -1) -> int:
        result = self.ffb.find_free_box(seed)
        if result is None:
            return -1
        lo, hi = result
        bid = self.next_id
        self.next_id += 1
        self.ffb.mark_box_id(lo, hi, bid)
        is_coarse = self.in_coarse_phase
        box = BoxInfo(
            box_id=bid, lo=lo, hi=hi, seed=seed.copy(),
            parent_box_id=parent_box_id,
            expand_face_dim=face_dim,
            expand_face_side=face_side,
            root_id=root_id,
            is_coarse=is_coarse)
        self.boxes[bid] = box
        self.adjacency[bid] = set()
        if is_coarse:
            self.n_coarse_boxes += 1
        else:
            self.n_fine_boxes += 1
        return bid

    def _update_adjacency(self, new_id: int):
        """增量更新邻接关系."""
        new_box = self.boxes[new_id]
        for bid, box in self.boxes.items():
            if bid == new_id:
                continue
            if new_box.is_adjacent(box):
                self.adjacency.setdefault(new_id, set()).add(bid)
                self.adjacency.setdefault(bid, set()).add(new_id)

    def _find_nearest_box(self, q: np.ndarray,
                          root_id: int = -1) -> Optional[BoxInfo]:
        """找 q 最近的 box. root_id>=0 时只搜索该子树 (对齐 C++ per-subtree worker)."""
        best, best_d = None, float("inf")
        for box in self.boxes.values():
            if root_id >= 0 and box.root_id != root_id:
                continue
            d = np.linalg.norm(q - box.center())
            if d < best_d:
                best_d = d
                best = box
        return best

    def _take_snapshot(self, new_box_id: int, force: bool = False):
        """深拷贝当前 forest 状态."""
        boxes_copy = {}
        for bid, b in self.boxes.items():
            boxes_copy[bid] = BoxInfo(
                box_id=b.box_id, lo=b.lo.copy(), hi=b.hi.copy(),
                seed=b.seed.copy(), parent_box_id=b.parent_box_id,
                expand_face_dim=b.expand_face_dim,
                expand_face_side=b.expand_face_side,
                root_id=b.root_id,
                is_coarse=b.is_coarse)
        adj_copy = {k: set(v) for k, v in self.adjacency.items()}
        self.snapshots.append({
            "n_boxes": len(self.boxes),
            "boxes": boxes_copy,
            "adjacency": adj_copy,
            "new_box_id": new_box_id,
            "in_coarse_phase": self.in_coarse_phase,
            "n_coarse_boxes": self.n_coarse_boxes,
            "n_fine_boxes": self.n_fine_boxes,
        })

    # ── Coarsening: box 合并 (对齐 C++ coarsen_forest + coarsen_greedy) ──

    def _rebuild_adjacency(self, tol: float = 1e-9):
        """全量重建邻接关系."""
        self.adjacency = {bid: set() for bid in self.boxes}
        ids = list(self.boxes.keys())
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = self.boxes[ids[i]], self.boxes[ids[j]]
                if a.is_adjacent(b, tol=tol):
                    self.adjacency[ids[i]].add(ids[j])
                    self.adjacency[ids[j]].add(ids[i])

    def coarsen_forest(self, max_rounds: int = 20) -> dict:
        """Dimension-scan merge: 合并沿某维度精确对齐的相邻 box 对.

        对齐 C++ coarsen_forest(): 对每个维度, 按 lo 排序, 找相邻且
        其余维度完全对齐的 box 对, 若 hull 无碰撞则合并.

        Returns dict with merges_performed, rounds, boxes_before, boxes_after.
        """
        n_dims = 2
        boxes_before = len(self.boxes)
        total_merges = 0
        rounds = 0

        for _round in range(max_rounds):
            rounds += 1
            merges_this_round = 0

            for dim in range(n_dims):
                # 按 dim 维度的 lo 排序
                sorted_ids = sorted(self.boxes.keys(),
                                    key=lambda bid: self.boxes[bid].lo[dim])

                i = 0
                while i + 1 < len(sorted_ids):
                    id_a, id_b = sorted_ids[i], sorted_ids[i + 1]
                    if id_a not in self.boxes or id_b not in self.boxes:
                        i += 1
                        continue
                    a, b = self.boxes[id_a], self.boxes[id_b]

                    # touch test: B.lo[dim] == A.hi[dim]
                    gap = b.lo[dim] - a.hi[dim]
                    if abs(gap) > 1e-10:
                        i += 1
                        continue

                    # exact alignment in other dims
                    exact = True
                    for d in range(n_dims):
                        if d == dim:
                            continue
                        if (abs(a.lo[d] - b.lo[d]) > 1e-10 or
                                abs(a.hi[d] - b.hi[d]) > 1e-10):
                            exact = False
                            break
                    if not exact:
                        i += 1
                        continue

                    # merged hull
                    m_lo = a.lo.copy()
                    m_hi = a.hi.copy()
                    m_hi[dim] = b.hi[dim]

                    # safety check
                    if self.cspace.box_collides(m_lo, m_hi):
                        i += 1
                        continue

                    # execute merge
                    new_id = self.next_id
                    self.next_id += 1
                    merged = BoxInfo(
                        box_id=new_id, lo=m_lo, hi=m_hi,
                        seed=a.seed.copy(), root_id=a.root_id,
                        is_coarse=False)
                    del self.boxes[id_a]
                    del self.boxes[id_b]
                    self.boxes[new_id] = merged
                    merges_this_round += 1

                    # update sorted_ids: replace pair with merged
                    sorted_ids[i] = new_id
                    sorted_ids.pop(i + 1)
                    # don't increment i — re-check with next neighbor

            total_merges += merges_this_round
            if merges_this_round == 0:
                break
            self._rebuild_adjacency()

        return {
            "merges_performed": total_merges,
            "rounds": rounds,
            "boxes_before": boxes_before,
            "boxes_after": len(self.boxes),
        }

    def coarsen_greedy(self, target_boxes: int = 0,
                       max_rounds: int = 100,
                       score_threshold: float = 50.0) -> dict:
        """Greedy adjacency-based merge: 合并任意相邻 box 对, hull 无碰撞即可.

        对齐 C++ coarsen_greedy(): 每轮收集所有相邻 pair, 计算
        score = hull_vol / (vol_a + vol_b), 按 score 升序贪心合并
        (each box 每轮最多参与一次合并).

        Returns dict with merges_performed, rounds, boxes_before, boxes_after.
        """
        boxes_before = len(self.boxes)
        total_merges = 0
        rounds = 0

        if target_boxes > 0 and len(self.boxes) <= target_boxes:
            return {"merges_performed": 0, "rounds": 0,
                    "boxes_before": boxes_before,
                    "boxes_after": len(self.boxes)}

        for _round in range(max_rounds):
            rounds += 1
            if target_boxes > 0 and len(self.boxes) <= target_boxes:
                break

            self._rebuild_adjacency()

            # collect candidates
            candidates = []  # (score, id_a, id_b, hull_lo, hull_hi)
            for id_a, neighbors in self.adjacency.items():
                a = self.boxes[id_a]
                for id_b in neighbors:
                    if id_b <= id_a:
                        continue
                    b = self.boxes[id_b]
                    hull_lo = np.minimum(a.lo, b.lo)
                    hull_hi = np.maximum(a.hi, b.hi)
                    hull_vol = float(np.prod(hull_hi - hull_lo))
                    sum_vol = a.volume + b.volume
                    score = hull_vol / sum_vol if sum_vol > 0 else 1e18
                    if score > score_threshold:
                        continue
                    candidates.append((score, id_a, id_b, hull_lo, hull_hi))

            if not candidates:
                break

            candidates.sort(key=lambda c: c[0])

            # greedy execute
            merged_this_round: Set[int] = set()
            merges_this_round = 0

            for score, id_a, id_b, hull_lo, hull_hi in candidates:
                if target_boxes > 0 and len(self.boxes) - merges_this_round <= target_boxes:
                    break
                if id_a in merged_this_round or id_b in merged_this_round:
                    continue
                if id_a not in self.boxes or id_b not in self.boxes:
                    continue

                # safety check
                if self.cspace.box_collides(hull_lo, hull_hi):
                    continue

                # execute merge
                a = self.boxes[id_a]
                new_id = self.next_id
                self.next_id += 1
                merged = BoxInfo(
                    box_id=new_id, lo=hull_lo.copy(), hi=hull_hi.copy(),
                    seed=a.seed.copy(), root_id=a.root_id,
                    is_coarse=False)
                del self.boxes[id_a]
                del self.boxes[id_b]
                self.boxes[new_id] = merged
                merged_this_round.add(id_a)
                merged_this_round.add(id_b)
                merges_this_round += 1

            total_merges += merges_this_round
            if merges_this_round == 0:
                break

        self._rebuild_adjacency()

        return {
            "merges_performed": total_merges,
            "rounds": rounds,
            "boxes_before": boxes_before,
            "boxes_after": len(self.boxes),
        }

    def run_coarsen(self) -> dict:
        """Run full coarsening pipeline: dimension-scan then greedy.

        Returns combined stats dict.
        """
        print(f"[coarsen] starting: {len(self.boxes)} boxes")
        r1 = self.coarsen_forest(
            max_rounds=self.cfg.coarsen_max_rounds)
        print(f"  dim-scan: {r1['merges_performed']} merges in "
              f"{r1['rounds']} rounds → {r1['boxes_after']} boxes")

        r2 = self.coarsen_greedy(
            target_boxes=self.cfg.coarsen_target_boxes,
            max_rounds=self.cfg.coarsen_greedy_rounds,
            score_threshold=self.cfg.coarsen_score_threshold)
        print(f"  greedy:   {r2['merges_performed']} merges in "
              f"{r2['rounds']} rounds → {r2['boxes_after']} boxes")

        total = r1["merges_performed"] + r2["merges_performed"]
        print(f"[coarsen] done: {r1['boxes_before']} → "
              f"{r2['boxes_after']} boxes ({total} merges)")

        return {
            "dim_scan": r1,
            "greedy": r2,
            "total_merges": total,
            "boxes_before": r1["boxes_before"],
            "boxes_after": r2["boxes_after"],
        }


# ═══════════════════════════════════════════════════════════════════════════
#  随机场景生成
# ═══════════════════════════════════════════════════════════════════════════

def build_random_scene(cfg: GrowVizConfig, rng) -> CSpace2D:
    """生成随机 2D C-space 障碍物, 保证 start/goal 可行且直线碰撞."""
    q_lo = np.array(cfg.q_lo)
    q_hi = np.array(cfg.q_hi)
    q_start = np.array(cfg.q_start) if cfg.q_start else None
    q_goal = np.array(cfg.q_goal) if cfg.q_goal else None

    for trial in range(500):
        obstacles = []
        for _ in range(cfg.n_obstacles):
            cx = rng.uniform(*cfg.obs_cx_range)
            cy = rng.uniform(*cfg.obs_cy_range)
            w = rng.uniform(*cfg.obs_w_range)
            h = rng.uniform(*cfg.obs_h_range)
            lo = np.array([cx - w / 2, cy - h / 2])
            hi = np.array([cx + w / 2, cy + h / 2])
            lo = np.clip(lo, q_lo, q_hi)
            hi = np.clip(hi, q_lo, q_hi)
            obstacles.append(Obstacle2D(lo, hi))

        cs = CSpace2D(q_lo, q_hi, obstacles)

        # 验证 start/goal 可行
        if q_start is not None and cs.is_collision(q_start):
            continue
        if q_goal is not None and cs.is_collision(q_goal):
            continue

        # 验证直线路径有碰撞 (否则规划无意义)
        if q_start is not None and q_goal is not None:
            blocked = False
            for t in np.linspace(0, 1, 100):
                q = q_start + t * (q_goal - q_start)
                if cs.is_collision(q):
                    blocked = True
                    break
            if not blocked:
                continue

        return cs

    raise RuntimeError("无法生成满足条件的随机场景 (500 trials)")


# ═══════════════════════════════════════════════════════════════════════════
#  Main CLI (python -m src.viz.forest_grower_2d)
# ═══════════════════════════════════════════════════════════════════════════

def main():
    import argparse, json, time
    from pathlib import Path
    from .render import (plot_snapshot, compose_gif,
                          plot_coarsen_comparison, plot_merge_comparison)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(
        description="2D ForestGrower 可视化")
    parser.add_argument("--mode", default="wavefront",
                        choices=["wavefront", "rrt"])
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--n-roots", type=int, default=3)
    parser.add_argument("--max-boxes", type=int, default=300)
    parser.add_argument("--snap-every", type=int, default=2)
    parser.add_argument("--obstacles", type=int, default=10)
    parser.add_argument("--no-endpoints", action="store_true",
                        help="不使用 start/goal 端点")
    parser.add_argument("--adaptive", action="store_true",
                        help="启用 adaptive min_edge 两阶段")
    parser.add_argument("--coarse-min-edge", type=float, default=0.15)
    parser.add_argument("--coarse-fraction", type=float, default=0.6)
    parser.add_argument("--n-threads", type=int, default=1,
                        help="模拟多线程 round-robin (>1 启用)")
    parser.add_argument("--coarsen", action="store_true",
                        help="growth 后执行 box 合并 (coarsening)")
    parser.add_argument("--coarsen-target", type=int, default=0,
                        help="greedy coarsen 目标 box 数 (0=不限)")
    args = parser.parse_args()

    cfg = GrowVizConfig(
        seed=args.seed,
        mode=args.mode,
        n_roots=args.n_roots,
        max_boxes=args.max_boxes,
        snapshot_every=args.snap_every,
        n_obstacles=args.obstacles,
        adaptive_min_edge=args.adaptive,
        coarse_min_edge=args.coarse_min_edge,
        coarse_fraction=args.coarse_fraction,
        n_threads=args.n_threads,
        coarsen_enabled=args.coarsen,
        coarsen_target_boxes=args.coarsen_target,
    )
    if args.no_endpoints:
        cfg.q_start = None
        cfg.q_goal = None

    rng = np.random.default_rng(cfg.seed)

    # ── 输出目录 ──────────────────────────────────────────────────────
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    viz_dir = Path(__file__).resolve().parent       # src/viz
    v3_root = viz_dir.parent.parent                 # v3/
    out_dir = v3_root / "results" / f"viz_2d_{timestamp}"
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    # ── 场景 ──────────────────────────────────────────────────────────
    print(f"[viz] building random scene (seed={cfg.seed}, "
          f"obs={cfg.n_obstacles}) ...")
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
    print("[viz] scanning collision map ...")
    cmap_data, extent = cspace.scan_collision_map(cfg.collision_resolution)

    # ── ForestGrower ──────────────────────────────────────────────────
    extra = ""
    if cfg.adaptive_min_edge:
        extra += f", adaptive={cfg.coarse_min_edge:.2f}→{cfg.min_edge:.2f}"
    if cfg.n_threads > 1:
        extra += f", threads={cfg.n_threads}"
    print(f"[viz] growing forest ({cfg.mode}, roots={cfg.n_roots}, "
          f"max_boxes={cfg.max_boxes}{extra}) ...")
    t0 = time.time()
    grower = ForestGrower2D(cspace, cfg)
    snapshots = grower.grow(snapshot_every=cfg.snapshot_every)
    elapsed = time.time() - t0
    print(f"  {len(snapshots)} snapshots, {len(grower.boxes)} boxes, "
          f"{elapsed:.2f} s")

    # ── 渲染帧 ────────────────────────────────────────────────────────
    print(f"[viz] rendering {len(snapshots)} frames ...")
    for idx, snap in enumerate(snapshots):
        fig = plot_snapshot(snap, cmap_data, extent, cfg, idx,
                            grower.subtrees)
        fig.savefig(frames_dir / f"frame_{idx:04d}.png",
                    dpi=cfg.dpi, bbox_inches="tight")
        plt.close(fig)

    # ── GIF ────────────────────────────────────────────────────────────
    gif_path = out_dir / "forest_grower.gif"
    print("[viz] composing GIF ...")
    ok = compose_gif(frames_dir, gif_path, cfg.gif_frame_ms)

    # ── Coarsen (box merging) ─────────────────────────────────────────
    coarsen_stats = None
    if cfg.coarsen_enabled:
        # snapshot before coarsen
        snap_before_coarsen = grower.snapshots[-1] if grower.snapshots else None
        coarsen_stats = grower.run_coarsen()
        # snapshot after coarsen
        grower._take_snapshot(-1, force=True)
        snap_after_coarsen = grower.snapshots[-1]

        # generate comparison image
        if snap_before_coarsen is not None:
            fig_merge = plot_merge_comparison(
                snap_before_coarsen, snap_after_coarsen,
                cmap_data, extent, cfg, grower.subtrees,
                coarsen_stats)
            fig_merge.savefig(out_dir / "coarsen_merge_comparison.png",
                              dpi=cfg.dpi, bbox_inches="tight")
            plt.close(fig_merge)
            print("[viz] coarsen merge comparison saved")

    # ── 碰撞底图单独保存 ──────────────────────────────────────────────
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
            snapshots[-1], cmap_data, extent, cfg, len(snapshots) - 1,
            grower.subtrees)
        fig_final.savefig(out_dir / "final_forest.png",
                          dpi=cfg.dpi, bbox_inches="tight")
        plt.close(fig_final)

    # ── Coarsen 对比图 (adaptive 模式) ────────────────────────────────
    if cfg.adaptive_min_edge and snapshots:
        result = plot_coarsen_comparison(
            snapshots, cmap_data, extent, cfg, grower.subtrees)
        if result is not None:
            fig_cmp, c_idx, f_idx = result
            fig_cmp.savefig(out_dir / "coarsen_comparison.png",
                            dpi=cfg.dpi, bbox_inches="tight")
            plt.close(fig_cmp)
            print(f"[viz] coarsen comparison saved "
                  f"(coarse@frame {c_idx}, final@frame {f_idx})")

    # ── Summary ───────────────────────────────────────────────────────
    summary = {
        "config": {
            "seed": cfg.seed, "mode": cfg.mode,
            "n_roots": cfg.n_roots, "max_boxes": cfg.max_boxes,
            "n_obstacles": cfg.n_obstacles,
            "snapshot_every": cfg.snapshot_every,
            "q_start": cfg.q_start, "q_goal": cfg.q_goal,
        },
        "result": {
            "n_boxes": len(grower.boxes),
            "n_snapshots": len(snapshots),
            "elapsed_s": round(elapsed, 3),
        },
    }
    if coarsen_stats is not None:
        summary["coarsen"] = coarsen_stats
    (out_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    md = [
        "# 2D ForestGrower 可视化",
        "",
        f"- mode: {cfg.mode}, seed: {cfg.seed}",
        f"- n_roots: {cfg.n_roots}, max_boxes: {cfg.max_boxes}",
        f"- final boxes: {len(grower.boxes)}, elapsed: {elapsed:.2f}s",
        "",
        f"![forest_grower](forest_grower.gif)",
    ]
    (out_dir / "README.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    print(f"\n{'='*60}")
    print(f"Output: {out_dir}")
    print(f"  frames: {len(snapshots)}, boxes: {len(grower.boxes)}")
    if ok:
        print(f"  gif: {gif_path}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
