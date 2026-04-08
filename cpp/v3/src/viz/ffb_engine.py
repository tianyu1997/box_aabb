"""
src/viz/ffb_engine.py — 2D 持久化 KD-tree Find-Free-Box 引擎

提供: _KDNode, FFBEngine
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from .core import CSpace2D


# ═══════════════════════════════════════════════════════════════════════════
#  KD-tree Node
# ═══════════════════════════════════════════════════════════════════════════

class _KDNode:
    """持久化 KD-tree 节点 (对齐 C++ LECT).

    三种状态:
      - 空闲叶 (leaf, box_id == -1)         → 可分配
      - 已占用叶 (leaf, box_id >= 0)         → 不可分配
      - 内部节点 (left/right != None)        → 递归
    """
    __slots__ = ("lo", "hi", "depth", "left", "right", "box_id")

    def __init__(self, lo, hi, depth=0):
        self.lo = lo
        self.hi = hi
        self.depth = depth
        self.left: Optional["_KDNode"] = None
        self.right: Optional["_KDNode"] = None
        self.box_id: int = -1          # >=0 表示已占用

    @property
    def is_leaf(self) -> bool:
        return self.left is None

    @property
    def is_occupied(self) -> bool:
        return self.box_id >= 0

    def contains(self, q: np.ndarray) -> bool:
        return bool(np.all(q >= self.lo - 1e-10) and np.all(q <= self.hi + 1e-10))


# ═══════════════════════════════════════════════════════════════════════════
#  FFBEngine
# ═══════════════════════════════════════════════════════════════════════════

class FFBEngine:
    """2D 持久化 KD-tree FFB (与 C++ LECT 对齐).

    维护一棵覆盖整个 C-space 的 KD-tree, 每个叶节点代表一个区域.
    find_free_box 沿着 tree 下行到包含 seed 的叶, 若该叶有碰撞则
    原地 split, 直到找到无碰撞的叶或达到 min_edge/max_depth 限制.

    优势:
      - box 天然铺满空间, 无重叠无间隙
      - 相邻 seed 复用已有 split, 不会产出重复的碎片
      - 障碍物边界只需在第一次遇到时 split, 后续 seed 直接复用
    """

    def __init__(self, cspace: CSpace2D, min_edge: float = 0.03,
                 max_depth: int = 20):
        self.cspace = cspace
        self.min_edge = min_edge
        self.max_depth = max_depth
        self._effective_min_edge = min_edge   # 可动态调整
        self.root = _KDNode(cspace.q_lo.copy(), cspace.q_hi.copy(), depth=0)

    def set_effective_min_edge(self, val: float):
        self._effective_min_edge = val

    def is_occupied(self, q: np.ndarray) -> bool:
        """q 是否已在某个已占用叶内."""
        node = self._find_leaf(q)
        return node is not None and node.is_occupied

    def find_free_box(self, seed: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """在 KD-tree 中为 seed 找到一个无碰撞叶. 返回 (lo, hi) 或 None."""
        if not self.root.contains(seed):
            return None
        if self.cspace.is_collision(seed):
            return None

        node = self._descend_and_split(self.root, seed)
        if node is None:
            return None

        node.box_id = 0   # 占位, 稍后由调用方设置真正 id
        return (node.lo.copy(), node.hi.copy())

    def mark_box_id(self, lo: np.ndarray, hi: np.ndarray, box_id: int):
        """将已分配叶的 box_id 更新为真正值."""
        node = self._find_leaf_by_region(self.root, lo, hi)
        if node is not None:
            node.box_id = box_id

    def _find_leaf(self, q: np.ndarray) -> Optional[_KDNode]:
        """找到包含 q 的叶节点."""
        node = self.root
        while not node.is_leaf:
            dim = node.depth % 2
            mid = (node.lo[dim] + node.hi[dim]) * 0.5
            if q[dim] <= mid:
                node = node.left
            else:
                node = node.right
        return node if node.contains(q) else None

    def _find_leaf_by_region(self, node, lo, hi):
        """按 (lo, hi) 精确匹配叶节点."""
        if node.is_leaf:
            if np.allclose(node.lo, lo, atol=1e-12) and np.allclose(node.hi, hi, atol=1e-12):
                return node
            return None
        for child in (node.left, node.right):
            # 粗过滤: child 的区域是否与目标有交集
            if np.all(child.lo <= hi + 1e-12) and np.all(lo <= child.hi + 1e-12):
                result = self._find_leaf_by_region(child, lo, hi)
                if result is not None:
                    return result
        return None

    def _descend_and_split(self, node: _KDNode, seed: np.ndarray) -> Optional[_KDNode]:
        """沿 KD-tree 下行. 若叶有碰撞则 split, 直到找到无碰撞空闲叶.

        返回可用叶, 或 None (碰撞/min_edge/max_depth 限制).
        """
        # 递归到叶
        if not node.is_leaf:
            dim = node.depth % 2
            mid = (node.lo[dim] + node.hi[dim]) * 0.5
            child = node.left if seed[dim] <= mid else node.right
            return self._descend_and_split(child, seed)

        # 叶节点
        if node.is_occupied:
            return None  # 已被占用

        # 检查该叶是否与障碍物碰撞
        if not self.cspace.box_collides(node.lo, node.hi):
            return node  # 无碰撞, 可分配

        # 需要 split
        me = self._effective_min_edge
        dim = node.depth % 2
        widths = node.hi - node.lo
        if widths[dim] < me * 2:
            # 该维度太窄, 尝试另一个维度
            dim = 1 - dim
            if widths[dim] < me * 2:
                return None  # 两个维度都太窄

        if node.depth >= self.max_depth:
            return None

        mid = (node.lo[dim] + node.hi[dim]) * 0.5

        left_lo = node.lo.copy()
        left_hi = node.hi.copy()
        left_hi[dim] = mid

        right_lo = node.lo.copy()
        right_hi = node.hi.copy()
        right_lo[dim] = mid

        node.left = _KDNode(left_lo, left_hi, node.depth + 1)
        node.right = _KDNode(right_lo, right_hi, node.depth + 1)

        # 继续下行到包含 seed 的子树
        child = node.left if seed[dim] <= mid else node.right
        return self._descend_and_split(child, seed)
