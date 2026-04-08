"""
src.viz — SafeBoxForest 2D 可视化工具包

子模块:
    core             — 核心数据类 (GrowVizConfig, Obstacle2D, CSpace2D, BoxInfo)
    ffb_engine       — Find-Free-Box KD-tree 引擎
    forest_grower_2d — 2D ForestGrower 仿真 + 随机场景生成
    render           — matplotlib 渲染工具 (PatchCollection/LineCollection 加速)
    compare_expansion — 4-模式对比可视化 (Wavefront / RRT / Multi-thread / Adaptive)
    multiprocess     — 多进程 ForestGrower 可视化
    wavefront_step   — 单步波前扩展可视化

用法 (命令行):
    cd safeboxforest/v3
    python -m src.viz.compare_expansion --seed 42 --max-boxes 200
    python -m src.viz.forest_grower_2d --seed 42 --max-boxes 300
    python -m src.viz.multiprocess --seed 42 --n-roots 3 --max-boxes 300
    python -m src.viz.wavefront_step --seed 42 --pre-grow 20

关键改进 (2026-03-17):
    - RRT boundary-snap: 新 box 紧邻最近 box 的边界面, 消除零散小 box
    - Root FPS 全空间: root 均匀分散于整个 C-space, 不再偏向 start→goal 线
    - Adaptive 两阶段: 先 coarse (大 box) 后 fine (小 box), 效果明显
    - PatchCollection/LineCollection 渲染加速 (~2× speedup)
    - 默认 snapshot_every=1 (逐框记录), CLI 默认 snap-every=2
"""

from .core import GrowVizConfig, Obstacle2D, CSpace2D, BoxInfo
from .ffb_engine import FFBEngine
from .forest_grower_2d import ForestGrower2D, build_random_scene
from .render import ROOT_COLORS, plot_snapshot, compose_gif, _draw_boxes_on_ax, _build_title, plot_coarsen_comparison, plot_merge_comparison
from .compare_expansion import plot_4panel

__all__ = [
    # core
    "GrowVizConfig", "Obstacle2D", "CSpace2D", "BoxInfo",
    # ffb_engine
    "FFBEngine",
    # forest_grower_2d
    "ForestGrower2D", "build_random_scene",
    # render
    "ROOT_COLORS", "plot_snapshot", "compose_gif",
    "_draw_boxes_on_ax", "_build_title",
    # compare_expansion
    "plot_4panel",
]
