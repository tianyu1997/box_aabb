# cython: language_level=3, boundscheck=False, wraparound=False, cdivision=True
"""
aabb/_sampling_core.pyx  —  采样策略热路径 Cython 加速

提供:
  - expand_reduced_batch:   reduced-space → full-space 批量展开
  - evaluate_samples_batch: 批量 FK 求值 + 极值更新 (seg_extremes)
  - update_extremes_c:      单点极值更新
  - generate_boundary_combos: 边界组合枚举

替代 strategies/base.py 中的:
  - SamplingStrategy._expand_reduced   (纯 Python list 操作)
  - SamplingStrategy._evaluate_samples (内层循环)
  - _update_extremes (optimization.py)
"""

import numpy as np
cimport numpy as cnp
from libc.math cimport floor, ceil, fabs, cos, sin, INFINITY

cnp.import_array()

ctypedef cnp.float64_t DTYPE_t


# ═══════════════════════════════════════════
#  expand_reduced → full-space 批量展开
# ═══════════════════════════════════════════

def expand_reduced_batch(
    cnp.ndarray[DTYPE_t, ndim=2] reduced,       # (M, n_rel)
    cnp.ndarray[cnp.int32_t, ndim=1] sorted_rel, # (n_rel,)
    cnp.ndarray[DTYPE_t, ndim=1] mid_q,          # (n_joints,)
) -> cnp.ndarray:
    """将 reduced-space 采样矩阵展开为 full-space 矩阵。

    Args:
        reduced: (M, n_rel)  reduced-space 采样矩阵
        sorted_rel: (n_rel,)  相关关节索引 (sorted)
        mid_q: (n_joints,)  区间中点

    Returns:
        (M, n_joints) full-space 矩阵
    """
    cdef int M = reduced.shape[0]
    cdef int n_rel = reduced.shape[1]
    cdef int n_joints = mid_q.shape[0]
    cdef int i, j, jj
    cdef cnp.ndarray[DTYPE_t, ndim=2] full = np.empty((M, n_joints), dtype=np.float64)

    # 逐行填充: 先拷贝 mid_q, 再替换相关关节
    for i in range(M):
        for j in range(n_joints):
            full[i, j] = mid_q[j]
        for j in range(n_rel):
            jj = sorted_rel[j]
            if jj < n_joints:
                full[i, jj] = reduced[i, j]

    return full


# ═══════════════════════════════════════════
#  极值更新 (C 级别)
# ═══════════════════════════════════════════

def update_extremes_batch(
    cnp.ndarray[DTYPE_t, ndim=2] positions,      # (N, 3) — 3D 端点
    cnp.ndarray[DTYPE_t, ndim=2] configs,        # (N, n_joints)
    cnp.ndarray[DTYPE_t, ndim=1] extremes_val,   # (6,) [xmin,xmax,ymin,ymax,zmin,zmax]
    cnp.ndarray[cnp.int32_t, ndim=1] extremes_idx, # (6,) 对应的 config 索引
) -> None:
    """批量更新极值: 给定 (N, 3) 位置, 找到每个方向的极值。

    极值6维: x_min(0), x_max(1), y_min(2), y_max(3), z_min(4), z_max(5)
    extremes_idx[k] = 取得极值的配置下标 (在 configs 中的行号)。
    """
    cdef int N = positions.shape[0]
    cdef int i
    cdef double x, y, z

    for i in range(N):
        x = positions[i, 0]
        y = positions[i, 1]
        z = positions[i, 2]

        if x < extremes_val[0]:
            extremes_val[0] = x
            extremes_idx[0] = i
        if x > extremes_val[1]:
            extremes_val[1] = x
            extremes_idx[1] = i
        if y < extremes_val[2]:
            extremes_val[2] = y
            extremes_idx[2] = i
        if y > extremes_val[3]:
            extremes_val[3] = y
            extremes_idx[3] = i
        if z < extremes_val[4]:
            extremes_val[4] = z
            extremes_idx[4] = i
        if z > extremes_val[5]:
            extremes_val[5] = z
            extremes_idx[5] = i


# ═══════════════════════════════════════════
#  evaluate_samples + update_extremes 合并
# ═══════════════════════════════════════════

def evaluate_and_update(
    cnp.ndarray[DTYPE_t, ndim=2] pos_start,      # (N, 3)
    cnp.ndarray[DTYPE_t, ndim=2] pos_end,        # (N, 3)
    cnp.ndarray[DTYPE_t, ndim=2] configs,        # (N, n_joints)
    int n_sub,
    cnp.ndarray[DTYPE_t, ndim=2] seg_vals,       # (n_sub, 6) 极值
    cnp.ndarray[cnp.int32_t, ndim=2] seg_idxs,   # (n_sub, 6) 极值对应配置下标
) -> None:
    """合并 _evaluate_samples + _update_extremes 的核心循环。

    对 N 个采样点, 每个点做线性插值得到 n_sub 段的端点位置,
    然后更新各段的 6 个极值 (x_min, x_max, y_min, y_max, z_min, z_max)。

    seg_vals[k, 0..5]  = 第 k 段的极值
    seg_idxs[k, 0..5]  = 对应的 config 行下标

    初始化: seg_vals[:, 0::2] = +inf, seg_vals[:, 1::2] = -inf
    """
    cdef int N = pos_start.shape[0]
    cdef int i, k
    cdef double t0, t1, step
    cdef double sx, sy, sz, ex, ey, ez
    cdef double p0x, p0y, p0z, p1x, p1y, p1z

    step = 1.0 / n_sub

    for i in range(N):
        sx = pos_start[i, 0]; sy = pos_start[i, 1]; sz = pos_start[i, 2]
        ex = pos_end[i, 0];   ey = pos_end[i, 1];   ez = pos_end[i, 2]

        for k in range(n_sub):
            t0 = k * step
            t1 = (k + 1) * step

            # 线性插值: p = (1-t)*start + t*end
            p0x = (1.0 - t0) * sx + t0 * ex
            p0y = (1.0 - t0) * sy + t0 * ey
            p0z = (1.0 - t0) * sz + t0 * ez
            p1x = (1.0 - t1) * sx + t1 * ex
            p1y = (1.0 - t1) * sy + t1 * ey
            p1z = (1.0 - t1) * sz + t1 * ez

            # 更新 p0 的极值
            if p0x < seg_vals[k, 0]:
                seg_vals[k, 0] = p0x; seg_idxs[k, 0] = i
            if p0x > seg_vals[k, 1]:
                seg_vals[k, 1] = p0x; seg_idxs[k, 1] = i
            if p0y < seg_vals[k, 2]:
                seg_vals[k, 2] = p0y; seg_idxs[k, 2] = i
            if p0y > seg_vals[k, 3]:
                seg_vals[k, 3] = p0y; seg_idxs[k, 3] = i
            if p0z < seg_vals[k, 4]:
                seg_vals[k, 4] = p0z; seg_idxs[k, 4] = i
            if p0z > seg_vals[k, 5]:
                seg_vals[k, 5] = p0z; seg_idxs[k, 5] = i

            # 更新 p1 的极值
            if p1x < seg_vals[k, 0]:
                seg_vals[k, 0] = p1x; seg_idxs[k, 0] = i
            if p1x > seg_vals[k, 1]:
                seg_vals[k, 1] = p1x; seg_idxs[k, 1] = i
            if p1y < seg_vals[k, 2]:
                seg_vals[k, 2] = p1y; seg_idxs[k, 2] = i
            if p1y > seg_vals[k, 3]:
                seg_vals[k, 3] = p1y; seg_idxs[k, 3] = i
            if p1z < seg_vals[k, 4]:
                seg_vals[k, 4] = p1z; seg_idxs[k, 4] = i
            if p1z > seg_vals[k, 5]:
                seg_vals[k, 5] = p1z; seg_idxs[k, 5] = i


# ═══════════════════════════════════════════
#  边界组合枚举 (替代 itertools.product)
# ═══════════════════════════════════════════

def generate_boundary_combos(
    cnp.ndarray[DTYPE_t, ndim=1] lo,     # (n,) 下界
    cnp.ndarray[DTYPE_t, ndim=1] hi,     # (n,) 上界
) -> cnp.ndarray:
    """生成所有 2^n 个边界组合。

    Args:
        lo: (n,) 各维度下界
        hi: (n,) 各维度上界

    Returns:
        (2^n, n) 组合矩阵
    """
    cdef int n = lo.shape[0]
    cdef int total = 1 << n  # 2^n
    cdef int i, j
    cdef cnp.ndarray[DTYPE_t, ndim=2] out = np.empty((total, n), dtype=np.float64)

    for i in range(total):
        for j in range(n):
            if (i >> j) & 1:
                out[i, j] = hi[j]
            else:
                out[i, j] = lo[j]

    return out
