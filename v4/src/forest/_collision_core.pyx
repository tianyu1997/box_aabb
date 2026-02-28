# cython: language_level=3, boundscheck=False, wraparound=False, cdivision=True
"""
forest/_collision_core.pyx  —  AABB 碰撞检测 Cython 热路径

提供:
  - aabb_overlap_3d: 3D 分离轴测试 (nogil, inline)
  - aabb_overlap_nd: 任意维度分离轴测试
  - check_links_vs_obstacles: 批量 link AABB vs obstacle AABB (nogil 循环)

替代 collision.py 中的纯 Python ``aabb_overlap`` 函数,
在单次调用上获得 ~10-20× 加速。热循环中
(如 check_config_collision, check_box_collision) 有大量调用,
累计加速效果显著。
"""

import numpy as np
cimport numpy as cnp
from libc.math cimport fabs

cnp.import_array()

ctypedef cnp.float64_t DTYPE_t
ctypedef cnp.float32_t F32_t


# ═══════════════════════════════════════════
#  3D 分离轴测试 (最常用路径: workspace 维度=3)
# ═══════════════════════════════════════════

cdef inline bint _aabb_overlap_3d(
    const double* min1, const double* max1,
    const double* min2, const double* max2,
    double eps,
) noexcept nogil:
    """纯 C 级别 3D AABB overlap 检测。

    返回 True = 重叠, False = 分离。
    """
    if max1[0] < min2[0] - eps or max2[0] < min1[0] - eps:
        return False
    if max1[1] < min2[1] - eps or max2[1] < min1[1] - eps:
        return False
    if max1[2] < min2[2] - eps or max2[2] < min1[2] - eps:
        return False
    return True


cdef inline bint _aabb_overlap_nd(
    const double* min1, const double* max1,
    const double* min2, const double* max2,
    int ndim, double eps,
) noexcept nogil:
    """任意维度 AABB 分离轴测试。"""
    cdef int i
    for i in range(ndim):
        if max1[i] < min2[i] - eps or max2[i] < min1[i] - eps:
            return False
    return True


# ═══════════════════════════════════════════
#  Python 可调用的 aabb_overlap
# ═══════════════════════════════════════════

def aabb_overlap(
    cnp.ndarray[DTYPE_t, ndim=1] min1,
    cnp.ndarray[DTYPE_t, ndim=1] max1,
    cnp.ndarray[DTYPE_t, ndim=1] min2,
    cnp.ndarray[DTYPE_t, ndim=1] max2,
) -> bint:
    """检测两个 AABB 是否重叠 (drop-in replacement for collision.aabb_overlap)。

    Args:
        min1, max1: 第一个 AABB 的最小/最大角点 (float64 ndarray)
        min2, max2: 第二个 AABB 的最小/最大角点

    Returns:
        True 表示重叠, False 表示分离。
    """
    cdef int n1 = min1.shape[0]
    cdef int n2 = min2.shape[0]
    cdef int ndim = n1 if n1 < n2 else n2
    cdef double eps = 1e-10
    cdef double* p_min1 = <double*> min1.data
    cdef double* p_max1 = <double*> max1.data
    cdef double* p_min2 = <double*> min2.data
    cdef double* p_max2 = <double*> max2.data

    if ndim == 3:
        return _aabb_overlap_3d(p_min1, p_max1, p_min2, p_max2, eps)
    return _aabb_overlap_nd(p_min1, p_max1, p_min2, p_max2, ndim, eps)


# ═══════════════════════════════════════════
#  批量 link-vs-obstacles 碰撞检测 (单配置)
# ═══════════════════════════════════════════

def check_links_vs_obstacles(
    cnp.ndarray[DTYPE_t, ndim=2] link_mins,     # (n_links, 3)
    cnp.ndarray[DTYPE_t, ndim=2] link_maxs,     # (n_links, 3)
    cnp.ndarray[DTYPE_t, ndim=2] obs_mins,      # (n_obs, 3)
    cnp.ndarray[DTYPE_t, ndim=2] obs_maxs,      # (n_obs, 3)
    cnp.ndarray[cnp.uint8_t, ndim=1] skip_mask, # (n_links,) 1=skip
) -> bint:
    """检测所有 link AABB 与所有 obstacle AABB 是否碰撞.

    对应 CollisionChecker.check_config_collision 中逐 link 逐 obs 的双重循环,
    用 C 级别循环替代 Python for + aabb_overlap 调用。

    Returns:
        True = 存在碰撞, False = 全部无碰撞。
    """
    cdef int n_links = link_mins.shape[0]
    cdef int n_obs = obs_mins.shape[0]
    cdef int li, oi
    cdef double eps = 1e-10
    cdef double* lmin
    cdef double* lmax
    cdef double* omin
    cdef double* omax

    for li in range(n_links):
        if skip_mask[li]:
            continue
        lmin = <double*> &link_mins[li, 0]
        lmax = <double*> &link_maxs[li, 0]
        for oi in range(n_obs):
            omin = <double*> &obs_mins[oi, 0]
            omax = <double*> &obs_maxs[oi, 0]
            if _aabb_overlap_3d(lmin, lmax, omin, omax, eps):
                return True
    return False


# ═══════════════════════════════════════════
#  批量配置碰撞检测 (N configs × links × obstacles)
# ═══════════════════════════════════════════

def batch_aabb_sat_check(
    cnp.ndarray[DTYPE_t, ndim=3] link_mins,     # (N, n_links, 3)
    cnp.ndarray[DTYPE_t, ndim=3] link_maxs,     # (N, n_links, 3)
    cnp.ndarray[DTYPE_t, ndim=2] obs_mins,      # (n_obs, 3)
    cnp.ndarray[DTYPE_t, ndim=2] obs_maxs,      # (n_obs, 3)
    cnp.ndarray[cnp.uint8_t, ndim=1] skip_mask, # (n_links,) 1=skip
    cnp.ndarray[cnp.uint8_t, ndim=1] result,    # (N,) 输出
) -> None:
    """批量 SAT 碰撞 (N configs × n_links × n_obs).

    对 check_config_collision_batch 中逐 obstacle 的内循环加速。
    result[i] = 1 表示第 i 个配置碰撞。
    """
    cdef int N = link_mins.shape[0]
    cdef int n_links = link_mins.shape[1]
    cdef int n_obs = obs_mins.shape[0]
    cdef int ci, li, oi
    cdef double eps = 1e-10
    cdef double* lmin
    cdef double* lmax
    cdef double* omin
    cdef double* omax

    for ci in range(N):
        if result[ci]:
            continue  # 已知碰撞, 跳过
        for li in range(n_links):
            if skip_mask[li]:
                continue
            lmin = <double*> &link_mins[ci, li, 0]
            lmax = <double*> &link_maxs[ci, li, 0]
            for oi in range(n_obs):
                omin = <double*> &obs_mins[oi, 0]
                omax = <double*> &obs_maxs[oi, 0]
                if _aabb_overlap_3d(lmin, lmax, omin, omax, eps):
                    result[ci] = 1
                    break  # 只要一个碰撞就够了
            if result[ci]:
                break
