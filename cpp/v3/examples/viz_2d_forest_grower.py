"""
examples/viz_2d_forest_grower.py — 2D ForestGrower 可视化 (薄封装)

本文件不包含任何算法实现, 仅从 src.viz 模块导入并暴露符号,
供其他 examples/ 脚本直接 ``from viz_2d_forest_grower import ...`` 使用.

直接运行等价于 ``python -m src.viz.forest_grower_2d``.

用法:
    cd safeboxforest/v3
    python examples/viz_2d_forest_grower.py [--mode wavefront|rrt] [--seed 42]
"""
from __future__ import annotations

import sys
from pathlib import Path

# ── 路径设置: 确保 v3/ 在 sys.path 中, 使 src.viz 可导入 ─────────────
_v3_root = str(Path(__file__).resolve().parent.parent)
if _v3_root not in sys.path:
    sys.path.insert(0, _v3_root)

# ── 从 src.viz 重新导出所有公共符号 ────────────────────────────────────
# 核心数据类
from src.viz.core import GrowVizConfig, Obstacle2D, CSpace2D, BoxInfo  # noqa: F401,E402

# FFB 引擎
from src.viz.ffb_engine import FFBEngine  # noqa: F401,E402

# 生长器 + 场景生成
from src.viz.forest_grower_2d import (  # noqa: F401,E402
    ForestGrower2D,
    build_random_scene,
    main as _module_main,
)

# 渲染
from src.viz.render import (  # noqa: F401,E402
    ROOT_COLORS,
    plot_snapshot,
    compose_gif,
    _draw_boxes_on_ax,
    _build_title,
)

# 对比可视化
from src.viz.compare_expansion import plot_4panel  # noqa: F401,E402


def main():
    """委托给 src.viz.forest_grower_2d.main()."""
    _module_main()


if __name__ == "__main__":
    main()
