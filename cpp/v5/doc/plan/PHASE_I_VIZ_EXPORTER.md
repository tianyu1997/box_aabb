# Phase I: VizExporter — C++ JSON 导出层

> 依赖: Phase A (Core) + Phase B-C (Envelope) + Phase F (Forest)
> 状态: ✅ 已完成 (2026-04-04)
> 产出: `viz/` 模块 — 将 Robot/Envelope/Forest/Scene 数据导出为结构化 JSON，供 Python 可视化消费
> 预计: ~500 LOC (精简版)，~800 LOC (含 voxel 支持)

---

## 目标

为 v5 建立 C++ → JSON 导出管线，使调试和论文配图所需的可视化数据可以直接由 C++ 端生成。
JSON 文件随后由 Phase K 的 Python 可视化层消费。

### 为什么先做导出而非可视化

1. 调试优先 — 在 pybind11 (Phase J) 之前就能 dump 中间数据
2. 解耦 — C++ 只管导出，Python 只管渲染，职责清晰
3. 复用 v4 — v4 `viz_exporter.cpp` 成熟 (~800 LOC)，可直接适配

---

## 管线概览

```
Robot / FKState / boxes[] / obstacles[]
    ↓
VizExporter  (C++ → JSON files)
    ↓
output/*.json
    ↓
Python sbf5_viz (Phase K)
    ↓
Plotly HTML
```

---

## Step I1: Robot JSON 导出

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/viz/viz_exporter.h` | `src/viz/viz_exporter.cpp` |

### 接口
```cpp
namespace sbf::viz {

/// 导出 robot FK 配置 (多组 q → link positions)
void export_robot_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<Eigen::VectorXd>& configs
);

}  // namespace sbf::viz
```

### JSON Schema
```json
{
  "type": "robot",
  "name": "<robot_name>",
  "n_joints": 2,
  "link_radii": [0.0, 0.0],
  "configs": [
    {
      "q": [0.1, 0.2],
      "link_positions": [[x0,y0,z0], [x1,y1,z1], ...]
    }
  ]
}
```

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v4 | `v4/src/viz/viz_exporter.cpp` → `export_robot_json()` | JSON 结构, FK 计算 |

---

## Step I2: Envelope iAABB 导出

### 接口
```cpp
namespace sbf::viz {

/// 导出 per-node link iAABB envelope
void export_envelope_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<BoxNode>& boxes,
    const LECT& lect
);

/// 导出 FK-based envelope (snapshot)
void export_envelope_from_boxes_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<BoxNode>& boxes,
    int samples_per_box = 8
);

}
```

### JSON Schema
```json
{
  "type": "envelope",
  "method": "link_iaabb",
  "boxes": [
    {
      "box_id": 0,
      "intervals": [[lo0, hi0], [lo1, hi1]],
      "links": [
        {"link_idx": 0, "aabb": [[xlo, xhi], [ylo, yhi], [zlo, zhi]]}
      ]
    }
  ]
}
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `export_envelope_json()`, `export_envelope_from_boxes_json()` |

---

## Step I3: Scene / Obstacle 导出

### 接口
```cpp
namespace sbf::viz {

/// 导出障碍物列表
void export_scene_json(
    const std::string& path,
    const Obstacle* obs, int n_obs
);

}
```

### JSON Schema
```json
{
  "type": "scene",
  "obstacles": [
    {"center": [x,y,z], "half_sizes": [hx,hy,hz]}
  ]
}
```

---

## Step I4: Snapshot 统一导出

### 接口
```cpp
namespace sbf::viz {

/// 一次性导出所有层: robot + envelope + scene + forest
void export_snapshot_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<BoxNode>& boxes,
    const LECT& lect,
    const Obstacle* obs, int n_obs,
    const std::vector<Eigen::VectorXd>& sample_configs = {}
);

}
```

### JSON Schema
```json
{
  "type": "snapshot",
  "robot": { ... },
  "envelope": { ... },
  "scene": { ... },
  "forest": {
    "n_boxes": 150,
    "boxes": [
      {"id": 0, "intervals": [[lo,hi],...], "volume": 0.01}
    ]
  }
}
```

---

## Step I5: 辅助 + 写入

### 内部工具函数
```cpp
namespace sbf::viz::detail {

/// 写 JSON 到文件 (pretty-print, indent=2)
void write_json(const std::string& path, const nlohmann::json& j);

/// 计算 box center (interval midpoints)
Eigen::VectorXd box_center(const BoxNode& box);

}
```

---

## Step I6: CMake 集成

### CMakeLists.txt 变更
```cmake
# 在 sbf5 library 中新增:
    src/viz/viz_exporter.cpp
```

无额外依赖 — 已依赖 `nlohmann_json`。

---

## Step I7: 测试

### 文件
`test/test_viz_exporter.cpp`

### 测试用例
| 用例 | 描述 |
|------|------|
| `robot_json_roundtrip` | 导出 2DOF robot JSON → 解析回 → 检查 link_positions 匹配 |
| `envelope_json_structure` | 导出 envelope → 检查 JSON 有 boxes/links 字段 |
| `scene_json_structure` | 导出 obstacles → 检查 JSON obstacles 数组长度 |
| `snapshot_has_all_layers` | 导出 snapshot → 检查 type/robot/envelope/scene/forest 字段均存在 |

### CMake 新增
```cmake
# SBF5_TESTS list 中追加:
    test_viz_exporter
```

---

## 验收标准

- [x] `export_robot_json()` 输出合法 JSON，可被 Python `json.load()` 读取
- [x] `export_snapshot_json()` 包含所有 4 个层 (robot/envelope/scene/forest)
- [x] 所有 JSON schema 与 v4 `sbf4_viz` 兼容 (保持字段名一致)
- [x] 4 个测试通过
- [x] 无额外依赖引入
