# cython: language_level=3, boundscheck=False, wraparound=False, cdivision=True
"""
forest/_coarsen_core.pyx  —  Coarsen 阶段热路径 Cython 加速

提供:
  - detect_touching_runs: 分组排序后检测 touching run (贪心扫描)
  - sweep_merge_detect: 完整单维度 run 检测 (分组 + 排序 + touching)

替代 coarsen.py 中 _sweep_merge_dim 的内层 Python 循环:
  - 分组后按 dim 排序
  - 相邻 box 的 gap 判定
  - run-length 扫描合并检测
"""

import numpy as np
cimport numpy as cnp
from libc.math cimport fabs

cnp.import_array()

ctypedef cnp.float64_t DTYPE_t
ctypedef cnp.int32_t ITYPE_t


# ═══════════════════════════════════════════
#  Touching Run 检测 (单组已排序序列)
# ═══════════════════════════════════════════

def detect_touching_runs(
    cnp.ndarray[DTYPE_t, ndim=1] sorted_lo,  # (M,) — 已按 lo 排序
    cnp.ndarray[DTYPE_t, ndim=1] sorted_hi,  # (M,) — 对应 hi
    double tol = 1e-10,
) -> list:
    """检测排序后序列中的 touching run。

    一个 run 是一段连续的 box, 相邻两个 box 的 hi[i] ≈ lo[i+1] (gap <= tol)。

    Returns:
        list of (start, end_inclusive) 元组, 表示每个 run 的索引范围。
    """
    cdef int M = sorted_lo.shape[0]
    if M < 2:
        return []

    cdef int i, run_start
    cdef list runs = []

    i = 0
    while i < M - 1:
        if fabs(sorted_hi[i] - sorted_lo[i + 1]) > tol:
            i += 1
            continue
        run_start = i
        while i < M - 1 and fabs(sorted_hi[i] - sorted_lo[i + 1]) <= tol:
            i += 1
        runs.append((run_start, i))  # inclusive range
        i += 1

    return runs


# ═══════════════════════════════════════════
#  批量 gap 计算 (向量化替代)
# ═══════════════════════════════════════════

def compute_touching_mask(
    cnp.ndarray[DTYPE_t, ndim=1] sorted_hi,  # (M,)
    cnp.ndarray[DTYPE_t, ndim=1] sorted_lo,  # (M,)
    double tol = 1e-10,
) -> cnp.ndarray:
    """计算 touching 掩码 (M-1,): touching[i] = |hi[i] - lo[i+1]| <= tol。

    比 NumPy np.abs(hi[:-1] - lo[1:]) <= tol 稍快 (避免临时数组分配)。

    Returns:
        (M-1,) bool 数组
    """
    cdef int M = sorted_hi.shape[0]
    cdef int i
    cdef cnp.ndarray[cnp.uint8_t, ndim=1] mask = np.zeros(M - 1, dtype=np.uint8)

    for i in range(M - 1):
        if fabs(sorted_hi[i] - sorted_lo[i + 1]) <= tol:
            mask[i] = 1

    return mask.view(np.bool_)


# ═══════════════════════════════════════════
#  分组 key 哈希 (避免 structured array + np.unique)
# ═══════════════════════════════════════════

def group_by_profile(
    cnp.ndarray[DTYPE_t, ndim=3] arr,      # (N, D, 2) — interval cache
    int dim,                                 # 扫描维度
    int ndim,                                # 总维度
    double round_digits = 12,               # 四舍五入精度
) -> dict:
    """按 "除 dim 外所有维度的 interval" 分组。

    Returns:
        dict: profile_key (tuple) → list of int (在 arr 中的行索引)
    """
    cdef int N = arr.shape[0]
    cdef int i, d
    cdef double lo_val, hi_val
    cdef double scale = 10.0 ** round_digits

    groups = {}
    for i in range(N):
        key_parts = []
        for d in range(ndim):
            if d == dim:
                continue
            lo_val = arr[i, d, 0]
            hi_val = arr[i, d, 1]
            # 四舍五入
            lo_val = (<long long>(lo_val * scale + 0.5)) / scale
            hi_val = (<long long>(hi_val * scale + 0.5)) / scale
            key_parts.append(lo_val)
            key_parts.append(hi_val)
        key = tuple(key_parts)
        if key not in groups:
            groups[key] = []
        groups[key].append(i)

    return groups
