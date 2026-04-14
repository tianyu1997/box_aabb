# Phase P: 功能补全 — Envelope 对比导出 + Hull16 Pipeline 集成

> 依赖: Phase I (VizExporter) + Phase N (Voxel)
> 状态: 🔲 未开始
> 产出: VizExporter 新增对比/体素导出函数 + Hull16_Grid 进入主 FFB pipeline
> 预计: ~350 LOC (C++)

---

## 目标

v4 的 VizExporter 有两个 v5 尚未迁移的高级导出函数，它们是论文配图的核心：

1. `export_envelope_comparison_json()` — 同一 box 用 5 种 envelope 方法计算，输出可切换对比
2. `export_voxel_json()` / `export_voxel_centres_json()` — 体素网格的 brick 级导出

同时，Phase N 的 Voxel 模块虽已通过单元测试，但 Hull16_Grid 还未接入 FFB 主 pipeline
的碰撞判定路径——目前 FFB 仅使用 LinkIAABB / LinkIAABB_Grid。

---

## Step P1: VizExporter — Envelope 对比导出

### 文件
修改: `include/sbf/viz/viz_exporter.h` + `src/viz/viz_exporter.cpp`

### 新增接口
```cpp
namespace sbf::viz {

/// 5 种 envelope 方法并排对比 JSON
/// methods: full_link_iaabb, link_iaabb, link_iaabb_grid, hull16_grid, voxel_hull16
void export_envelope_comparison_json(
    const std::string& path,
    const Robot& robot,
    const BoxNode& box,          // 单个 box
    const LECT& lect,
    double voxel_delta = 0.02
);

}
```

### JSON Schema
```json
{
  "type": "envelope_comparison",
  "box_id": 0,
  "intervals": [[lo0, hi0], [lo1, hi1]],
  "methods": {
    "link_iaabb": {
      "links": [{"link_idx": 0, "aabb": [[xlo,xhi],[ylo,yhi],[zlo,zhi]]}]
    },
    "link_iaabb_grid": {
      "links": [{"link_idx": 0, "sub_aabbs": [...]}]
    },
    "hull16_grid": {
      "delta": 0.02,
      "n_occupied": 1500,
      "centres": [[x,y,z], ...]
    }
  }
}
```

### 算法
```
export_envelope_comparison_json(path, robot, box, lect, delta):
    json j;
    j["type"] = "envelope_comparison";
    j["box_id"] = box.id;
    j["intervals"] = serialize(box.intervals);

    // Method 1: LinkIAABB
    ep_iaabbs = lect.compute_endpoint_iaabbs(box);
    link_env_1 = compute_link_envelope(ep_iaabbs, ..., {LinkIAABB});
    j["methods"]["link_iaabb"] = serialize_link_aabbs(link_env_1);

    // Method 2: LinkIAABB_Grid
    link_env_2 = compute_link_envelope(ep_iaabbs, ..., {LinkIAABB_Grid, grid_n=4});
    j["methods"]["link_iaabb_grid"] = serialize_sub_aabbs(link_env_2);

    // Method 3: Hull16_Grid (voxel)
    link_env_3 = compute_link_envelope(ep_iaabbs, ..., {Hull16_Grid, delta});
    centres = extract_voxel_centres(link_env_3.sparse_grid);
    j["methods"]["hull16_grid"] = {delta, n_occupied, centres};

    write_json(path, j);
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `viz_exporter.cpp` → `export_envelope_comparison_json()` (~150 LOC) |

---

## Step P2: VizExporter — Voxel 导出

### 新增接口
```cpp
namespace sbf::viz {

/// Brick 级体素导出 (hex-encoded words)
void export_voxel_json(
    const std::string& path,
    const voxel::SparseVoxelGrid& grid
);

/// Voxel 中心点导出 (Cartesian coordinates)
void export_voxel_centres_json(
    const std::string& path,
    const voxel::SparseVoxelGrid& grid
);

}
```

### JSON Schema — Brick 级
```json
{
  "type": "voxel",
  "delta": 0.02,
  "n_bricks": 42,
  "total_occupied": 1500,
  "bricks": [
    {
      "coord": [0, 0, 0],
      "popcount": 100,
      "words": ["ff00ff00ff00ff00", "00ff00ff00ff00ff", ...]
    }
  ]
}
```

### JSON Schema — Centres
```json
{
  "type": "voxel_centres",
  "delta": 0.02,
  "n_points": 1500,
  "centres": [[x, y, z], ...]
}
```

### 实现
```cpp
void export_voxel_json(path, grid) {
    json j;
    j["type"] = "voxel";
    j["delta"] = grid.delta();
    int total = 0;
    json bricks_arr = json::array();
    for (auto it = grid.bricks().begin(); it != grid.bricks().end(); ++it) {
        json b;
        b["coord"] = {it->key.bx, it->key.by, it->key.bz};
        b["popcount"] = it->brick.popcount();
        total += it->brick.popcount();
        json words = json::array();
        for (int w = 0; w < 8; ++w)
            words.push_back(hex64(it->brick.words[w]));
        b["words"] = words;
        bricks_arr.push_back(b);
    }
    j["n_bricks"] = grid.bricks().size();
    j["total_occupied"] = total;
    j["bricks"] = bricks_arr;
    write_json(path, j);
}

void export_voxel_centres_json(path, grid) {
    json j;
    j["type"] = "voxel_centres";
    j["delta"] = grid.delta();
    json pts = json::array();
    for (auto it = grid.bricks().begin(); it != grid.bricks().end(); ++it) {
        for (int z = 0; z < 8; ++z)
          for (int y = 0; y < 8; ++y)
            for (int x = 0; x < 8; ++x)
              if (it->brick.test(x, y, z)) {
                  int gx = it->key.bx * 8 + x;
                  int gy = it->key.by * 8 + y;
                  int gz = it->key.bz * 8 + z;
                  auto c = grid.cell_center({gx, gy, gz});
                  pts.push_back({c.x(), c.y(), c.z()});
              }
    }
    j["n_points"] = pts.size();
    j["centres"] = pts;
    write_json(path, j);
}
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `export_voxel_json()`, `export_voxel_centres_json()` |

---

## Step P3: Hull16_Grid 接入 FFB 碰撞路径

### 当前状态

`envelope_type.cpp` 已实现 `Hull16_Grid` 的 envelope 计算（Step N6）。
但 `ffb.cpp` 中的 `ffb_check()` 函数目前只处理 `LinkIAABB` 和 `LinkIAABB_Grid`。

### 修改文件
`src/ffb/ffb.cpp`

### 变更
```cpp
// 现有: 仅检查 link_iaabbs 与 obstacle iAABBs 重叠
// 新增: 当 envelope 有 sparse_grid 时, 使用 voxel collides

bool ffb_check(const LinkEnvelope& env,
               const Obstacle* obs, int n_obs,
               ...) {
    if (env.has_grid() && env.sparse_grid) {
        // Hull16_Grid 模式: voxel 碰撞
        voxel::SparseVoxelGrid obs_grid(env.sparse_grid->delta());
        for (int i = 0; i < n_obs; ++i)
            voxel::rasterise_box_obstacle(obs[i], obs_grid);
        return !env.sparse_grid->collides(obs_grid);  // true = free
    }
    // 原有 iAABB SAT 路径
    ...
}
```

### 影响
- `GrowerConfig` 中可通过 `EnvelopeTypeConfig{Hull16_Grid, delta}` 选择体素模式
- 现有默认 (`LinkIAABB`) 不受影响
- 新增 voxel 依赖到 ffb → 需确保 `#include "sbf/voxel/hull_rasteriser.h"`

---

## Step P4: Python 可视化层适配

### 修改文件
`python/sbf5_viz/load_data.py` — 新增 voxel 数据类 + 加载

```python
@dataclass
class VoxelData:
    delta: float
    n_bricks: int
    total_occupied: int
    centres: np.ndarray        # (N, 3)

def load_voxel_data(data: dict) -> VoxelData:
    """从 voxel_centres JSON 加载."""
```

### 修改文件
`python/sbf5_viz/` — 新增 `voxel_viz.py`

```python
def plot_voxel_scatter(voxel_data: VoxelData,
                       opacity: float = 0.3,
                       marker_size: float = 2.0) -> go.Figure:
    """稀疏体素点云散点图."""

def add_voxel_traces(fig: go.Figure, voxel_data: VoxelData) -> None:
    """向 combined 图添加 voxel 层."""
```

---

## Step P5: 测试

### C++ 测试
追加到 `test/test_viz_exporter.cpp`:

| 用例 | 描述 |
|------|------|
| `envelope_comparison_json` | 导出 comparison → 检查 3 种 method 均存在 |
| `voxel_json_roundtrip` | 导出 voxel → 检查 bricks 数组 + popcount |
| `voxel_centres_json` | 导出 centres → 检查 n_points > 0 |

追加到 `test/test_full_pipeline.cpp`:

| 用例 | 描述 |
|------|------|
| `ffb_with_hull16_grid` | GrowerConfig 使用 Hull16_Grid → FFB 正确判定 |

---

## 验收标准

- [ ] `export_envelope_comparison_json()` 输出包含 3 种 method 对比
- [ ] `export_voxel_json()` brick 级输出, hex-encoded words 正确
- [ ] `export_voxel_centres_json()` 输出 Cartesian 点集
- [ ] `Hull16_Grid` 在 FFB 碰撞路径中正确工作
- [ ] Python `plot_voxel_scatter()` 生成 Plotly 散点图
- [ ] 所有新增测试通过
- [ ] 默认 LinkIAABB 模式不受任何影响
