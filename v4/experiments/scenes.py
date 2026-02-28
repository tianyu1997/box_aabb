"""
experiments/scenes.py — 场景与规划器配置加载

支持两种模式:
1. Marcucci 场景 (shelves / bins / table) — 来自 marcucci_scenes.py
2. 随机场景 — 参数化生成
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

_DIR = Path(__file__).resolve().parent


# ═══════════════════════════════════════════════════════════════════════════
# Marcucci benchmark scenes
# ═══════════════════════════════════════════════════════════════════════════

def load_marcucci_scene(name: str, robot: str = "iiwa14") -> dict:
    """加载 Marcucci et al. 2024 基准场景.

    Args:
        name: "shelves" | "bins" | "table"
        robot: 机器人名称

    Returns:
        scene config dict (可传入 runner.load_scene_from_config)
    """
    from marcucci_scenes import (
        build_shelves_obstacles,
        build_bins_obstacles,
        build_table_obstacles,
        get_query_pairs,
    )

    builders = {
        "shelves": build_shelves_obstacles,
        "bins": build_bins_obstacles,
        "table": build_table_obstacles,
    }
    if name not in builders:
        raise ValueError(f"Unknown scene: {name}. Choose from {list(builders)}")

    obstacles = builders[name]()
    query_pairs = get_query_pairs(name)

    return {
        "name": f"marcucci_{name}",
        "robot": robot,
        "obstacles": [
            {"min": obs["min"].tolist() if hasattr(obs["min"], "tolist") else obs["min"],
             "max": obs["max"].tolist() if hasattr(obs["max"], "tolist") else obs["max"],
             "name": obs.get("name", "")}
            for obs in obstacles
        ],
        "query_pairs": [
            {"start": qp[0].tolist(), "goal": qp[1].tolist()}
            for qp in query_pairs
        ],
    }


def load_all_marcucci_scenes(robot: str = "iiwa14") -> List[dict]:
    """加载全部 Marcucci 基准场景."""
    return [load_marcucci_scene(name, robot)
            for name in ("shelves", "bins", "table")]


def load_random_scene(n_obstacles: int = 10, seed: int = 0,
                      robot: str = "iiwa14") -> dict:
    """生成随机场景配置."""
    rng = np.random.default_rng(seed)
    obstacles = []
    for i in range(n_obstacles):
        cx = rng.uniform(-0.8, 0.8)
        cy = rng.uniform(-0.8, 0.8)
        cz = rng.uniform(0.0, 1.0)
        h = rng.uniform(0.04, 0.12)
        obstacles.append({
            "min": [cx - h, cy - h, cz - h],
            "max": [cx + h, cy + h, cz + h],
            "name": f"rand_{i}",
        })
    return {
        "name": f"random_{n_obstacles}_s{seed}",
        "robot": robot,
        "obstacles": obstacles,
        "random_obstacles": None,  # 已直接生成
    }


# ═══════════════════════════════════════════════════════════════════════════
# Planner configs
# ═══════════════════════════════════════════════════════════════════════════

PAPER_PLANNERS = [
    {"type": "SBF", "method": "gcs", "name": "SBF-GCS"},
    {"type": "IRIS-NP-GCS", "n_iris_seeds": 10, "name": "IRIS-NP-GCS"},
    {"type": "C-IRIS-GCS", "n_regions": 10, "name": "C-IRIS-GCS"},
    {"type": "PRM", "n_samples": 2000, "k_neighbors": 15, "name": "PRM"},
]


def load_planners(names: Optional[List[str]] = None) -> List[dict]:
    """按名称加载 planner 配置; names=None 则加载全部."""
    if names is None:
        return list(PAPER_PLANNERS)
    name_set = set(names)
    return [p for p in PAPER_PLANNERS if p.get("name", p["type"]) in name_set]


def load_all_planners() -> Dict[str, dict]:
    """加载全部标准 planner 配置 (dict keyed by name)."""
    return {p.get("name", p["type"]): p for p in PAPER_PLANNERS}


# ═══════════════════════════════════════════════════════════════════════════
# Legacy JSON loading (backward compatible)
# ═══════════════════════════════════════════════════════════════════════════

def _load_json(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def load_all_scenes() -> Dict[str, dict]:
    """加载全部场景 (Marcucci + random supplement)."""
    scenes = {}
    for name in ("shelves", "bins", "table"):
        cfg = load_marcucci_scene(name)
        scenes[cfg["name"]] = cfg
    return scenes


def load_scenes(names: Optional[List[str]] = None) -> List[dict]:
    """按名称加载场景配置; names=None 则加载全部."""
    all_scenes = load_all_scenes()
    if names is None:
        return list(all_scenes.values())
    return [all_scenes[n] for n in names if n in all_scenes]
