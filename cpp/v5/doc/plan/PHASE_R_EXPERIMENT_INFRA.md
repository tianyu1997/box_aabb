# Phase R: 实验基础设施扩展

> 依赖: Phase A-Q (全部完成)
> 状态: 🔲 未开始
> 产出: Python 可控制 4×3=12 pipeline 配置 + GCPC cache + envelope-level metrics
> 预计: ~500 LOC (C++ 改动 ~200, Python 改动 ~300)

---

## 目标

当前 Python benchmark 框架 **无法指定 EndpointSource 和 EnvelopeType**:
- `SBFPlannerConfig` 仅有 grower / coarsen / smoother / gcs 四个字段
- `sbf_planner.cpp` 中硬编码 `EndpointSourceConfig ep_cfg; EnvelopeTypeConfig env_cfg;` (默认值)
- pybind11 未绑定 `EndpointSource`、`EnvelopeType` 枚举及其 Config 结构体

本阶段打通完整配置链路：

```
Python (SBFPlannerAdapter)
  → pybind11 (EndpointSource/EnvelopeType enums + configs)
  → SBFPlannerConfig (新增 endpoint_source_config / envelope_type_config 字段)
  → SBFPlanner::build()
  → LECT 构造函数 (ep_config, env_config)
  → compute_endpoint_iaabb() / compute_link_envelope()
```

---

## Step R0: GCPC Cache 生成工具

### 背景
`data/` 目录下无任何 `.gcpc` 文件。GCPC endpoint source 需要预计算的 cache 文件。
`bench_endpoint_iaabb.cpp` 中已有 `make_gcpc_cache()` 逻辑 (随机采样关节空间)。

### 文件
| 新建 | 说明 |
|------|------|
| `tools/generate_gcpc_cache.cpp` | 独立可执行文件, 生成 `.gcpc` 二进制 cache |

### 接口

```cpp
// tools/generate_gcpc_cache.cpp
// Usage: generate_gcpc_cache <robot.json> <output.gcpc> [n_points=5000] [seed=42]

#include <sbf/core/robot.h>
#include <sbf/envelope/gcpc_source.h>
#include <random>
#include <cstdio>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::fprintf(stderr, "Usage: %s <robot.json> <output.gcpc> [n_points] [seed]\n", argv[0]);
        return 1;
    }
    auto robot = sbf::Robot::from_json(argv[1]);
    int n_points = (argc > 3) ? std::atoi(argv[3]) : 5000;
    uint64_t seed = (argc > 4) ? std::stoull(argv[4]) : 42;

    sbf::GcpcCache cache;
    cache.set_n_dims(robot.n_joints());

    std::mt19937_64 gen(seed);
    const auto& lim = robot.joint_limits().limits;
    for (int i = 0; i < n_points; ++i) {
        Eigen::VectorXd pt(robot.n_joints());
        for (int j = 0; j < robot.n_joints(); ++j) {
            std::uniform_real_distribution<double> d(lim[j].lo, lim[j].hi);
            pt[j] = d(gen);
        }
        cache.add_point(pt);
    }
    cache.save(argv[2]);
    std::printf("Generated %d points → %s\n", n_points, argv[2]);
    return 0;
}
```

### CMake

```cmake
# 在 CMakeLists.txt 中追加 (SBF_BUILD_EXPERIMENTS 区域内):
add_executable(generate_gcpc_cache tools/generate_gcpc_cache.cpp)
target_link_libraries(generate_gcpc_cache PRIVATE sbf5)
```

### 运行

```powershell
cd build_x64
cmake --build . --config Release --target generate_gcpc_cache
.\Release\generate_gcpc_cache.exe ..\data\panda.json ..\data\panda_5000.gcpc 5000 42
.\Release\generate_gcpc_cache.exe ..\data\2dof_planar.json ..\data\2dof_500.gcpc 500 42
```

### 产出
| 文件 | 内容 |
|------|------|
| `data/panda_5000.gcpc` | 7DOF, 5000 点, ~280KB |
| `data/2dof_500.gcpc` | 2DOF, 500 点, ~8KB |

---

## Step R1: SBFPlannerConfig 扩展 + 配置穿透

### 当前问题

```cpp
// sbf_planner.cpp:22-23 — 硬编码默认配置
EndpointSourceConfig ep_cfg;    // 永远是 IFK
EnvelopeTypeConfig env_cfg;     // 永远是 LinkIAABB
lect_ = std::make_unique<LECT>(robot_, root_ivs, ep_cfg, env_cfg);
```

### 修改文件

| 文件 | 修改内容 |
|------|----------|
| `include/sbf/planner/sbf_planner.h` | `SBFPlannerConfig` 增加 2 个字段 |
| `src/planner/sbf_planner.cpp` | `build()` 使用 config 中的新字段 |

### 改动 1: sbf_planner.h

```cpp
// 在 SBFPlannerConfig 中新增:
struct SBFPlannerConfig {
    GrowerConfig grower;
    GreedyCoarsenConfig coarsen;
    SmootherConfig smoother;
    bool use_gcs = false;
    GCSConfig gcs;

    // ── 新增 (Phase R1) ──
    EndpointSourceConfig endpoint_source;   // default: IFK
    EnvelopeTypeConfig   envelope_type;     // default: LinkIAABB
};
```

需要新增 `#include`:
```cpp
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
```

### 改动 2: sbf_planner.cpp

```cpp
void SBFPlanner::build(...) {
    auto root_ivs = robot_.joint_limits().limits;
    // 替换硬编码:
    lect_ = std::make_unique<LECT>(
        robot_, root_ivs,
        config_.endpoint_source,    // ← 使用 config
        config_.envelope_type       // ← 使用 config
    );
    // ... 其余不变
}
```

### 验证
- `ctest -C Release` 13/13 仍通过 (默认值与原硬编码一致)
- 手动测试: 构造 `SBFPlannerConfig` 设置 `endpoint_source.source = EndpointSource::Analytical`, 验证 plan() 成功

---

## Step R2: pybind11 扩展绑定

### 修改文件
`python/sbf5_bindings.cpp`

### 新增绑定

```cpp
// ── EndpointSource 枚举 ─────────────────────────────────────
py::enum_<sbf::EndpointSource>(m, "EndpointSource")
    .value("IFK",        sbf::EndpointSource::IFK)
    .value("CritSample", sbf::EndpointSource::CritSample)
    .value("Analytical", sbf::EndpointSource::Analytical)
    .value("GCPC",       sbf::EndpointSource::GCPC);

// ── EnvelopeType 枚举 ───────────────────────────────────────
py::enum_<sbf::EnvelopeType>(m, "EnvelopeType")
    .value("LinkIAABB",      sbf::EnvelopeType::LinkIAABB)
    .value("LinkIAABB_Grid", sbf::EnvelopeType::LinkIAABB_Grid)
    .value("Hull16_Grid",    sbf::EnvelopeType::Hull16_Grid);

// ── EndpointSourceConfig ────────────────────────────────────
py::class_<sbf::EndpointSourceConfig>(m, "EndpointSourceConfig")
    .def(py::init<>())
    .def_readwrite("source",               &sbf::EndpointSourceConfig::source)
    .def_readwrite("n_samples_crit",        &sbf::EndpointSourceConfig::n_samples_crit)
    .def_readwrite("max_phase_analytical",  &sbf::EndpointSourceConfig::max_phase_analytical);
    // 注意: gcpc_cache 指针不直接绑定, 通过 path 加载 (见 R2b)

// ── EnvelopeTypeConfig ──────────────────────────────────────
py::class_<sbf::EnvelopeTypeConfig>(m, "EnvelopeTypeConfig")
    .def(py::init<>())
    .def_readwrite("type",           &sbf::EnvelopeTypeConfig::type)
    .def_readwrite("n_subdivisions", &sbf::EnvelopeTypeConfig::n_subdivisions);
    // grid_config 的 voxel_delta 可通过 lambda 暴露

// ── GcpcCache ───────────────────────────────────────────────
py::class_<sbf::GcpcCache>(m, "GcpcCache")
    .def(py::init<>())
    .def_static("load", &sbf::GcpcCache::load, py::arg("path"))
    .def("save",     &sbf::GcpcCache::save, py::arg("path"))
    .def("n_points", &sbf::GcpcCache::n_points)
    .def("n_dims",   &sbf::GcpcCache::n_dims)
    .def("empty",    &sbf::GcpcCache::empty);

// ── SBFPlannerConfig 扩展 ───────────────────────────────────
// 在现有 SBFPlannerConfig 绑定中追加:
    .def_readwrite("endpoint_source", &sbf::SBFPlannerConfig::endpoint_source)
    .def_readwrite("envelope_type",   &sbf::SBFPlannerConfig::envelope_type);
```

### R2b: GCPC Cache 在 Planner 中的生命周期

GCPC 需要 `const GcpcCache*` 指针。方案:

```cpp
// 在 SBFPlanner 中添加:
class SBFPlanner : public IPlanner {
    // ...
    std::unique_ptr<GcpcCache> owned_cache_;  // ← 新增: 持有 cache
};
```

```cpp
// sbf_planner.cpp build() 中:
if (config_.endpoint_source.source == EndpointSource::GCPC) {
    if (!owned_cache_ && !config_.endpoint_source.gcpc_cache) {
        // 自动从 data/ 加载默认 cache
        // 或抛出错误, 要求用户传入 cache_path
    }
    config_.endpoint_source.gcpc_cache = owned_cache_.get();
}
```

Python 侧:
```python
# 用户代码:
cache = sbf5.GcpcCache.load("data/panda_5000.gcpc")

cfg = sbf5.SBFPlannerConfig()
cfg.endpoint_source.source = sbf5.EndpointSource.GCPC
# cache 需要通过 planner 方法传入 (避免悬垂指针):
planner = sbf5.SBFPlanner(robot, cfg)
planner.set_gcpc_cache(cache)  # ← 新增方法, 内部持有 shared_ptr/unique_ptr
```

### includes 更新
```cpp
// sbf5_bindings.cpp 顶部新增:
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/gcpc_source.h>
```

---

## Step R3: SBFPlannerAdapter 支持 Pipeline 配置

### 修改文件
`python/sbf5_bench/sbf_adapter.py`

### 改动

```python
class SBFPlannerAdapter(BasePlanner):

    def __init__(
        self,
        endpoint_source: str = "IFK",        # "IFK"|"CritSample"|"Analytical"|"GCPC"
        envelope_type: str = "LinkIAABB",     # "LinkIAABB"|"LinkIAABB_Grid"|"Hull16_Grid"
        use_gcs: bool = False,
        gcpc_cache_path: str | None = None,   # .gcpc 文件路径
        extra_config: dict | None = None,
    ):
        self._endpoint_source = endpoint_source
        self._envelope_type = envelope_type
        self._use_gcs = use_gcs
        self._gcpc_cache_path = gcpc_cache_path
        self._extra_config = extra_config or {}
        self._planner = None
        self._gcpc_cache = None

    @property
    def name(self) -> str:
        gcs_tag = "-GCS" if self._use_gcs else ""
        return f"SBF-{self._endpoint_source}-{self._envelope_type}{gcs_tag}"

    def setup(self, robot, scene, config=None) -> None:
        import sbf5

        self._robot = robot
        self._obstacles = scene

        cfg = sbf5.SBFPlannerConfig()
        cfg.use_gcs = self._use_gcs

        # Endpoint source
        src_map = {
            "IFK":        sbf5.EndpointSource.IFK,
            "CritSample": sbf5.EndpointSource.CritSample,
            "Analytical": sbf5.EndpointSource.Analytical,
            "GCPC":       sbf5.EndpointSource.GCPC,
        }
        cfg.endpoint_source.source = src_map[self._endpoint_source]

        # Envelope type
        env_map = {
            "LinkIAABB":      sbf5.EnvelopeType.LinkIAABB,
            "LinkIAABB_Grid": sbf5.EnvelopeType.LinkIAABB_Grid,
            "Hull16_Grid":    sbf5.EnvelopeType.Hull16_Grid,
        }
        cfg.envelope_type.type = env_map[self._envelope_type]

        # Extra
        c = {**self._extra_config, **(config or {})}
        if "max_boxes" in c:
            cfg.grower.max_boxes = c["max_boxes"]
        if "rng_seed" in c:
            cfg.grower.rng_seed = c["rng_seed"]
        if "target_boxes" in c:
            cfg.coarsen.target_boxes = c["target_boxes"]
        if "n_subdivisions" in c:
            cfg.envelope_type.n_subdivisions = c["n_subdivisions"]
        if "n_samples_crit" in c:
            cfg.endpoint_source.n_samples_crit = c["n_samples_crit"]
        if "max_phase_analytical" in c:
            cfg.endpoint_source.max_phase_analytical = c["max_phase_analytical"]

        self._planner = sbf5.SBFPlanner(robot, cfg)

        # GCPC cache
        if self._endpoint_source == "GCPC" and self._gcpc_cache_path:
            self._gcpc_cache = sbf5.GcpcCache.load(self._gcpc_cache_path)
            self._planner.set_gcpc_cache(self._gcpc_cache)
```

---

## Step R4: Runner 支持配置矩阵

### 修改文件
`python/sbf5_bench/runner.py`

### 新增: PipelineConfig

```python
@dataclass
class PipelineConfig:
    """One pipeline = endpoint_source × envelope_type."""
    endpoint_source: str
    envelope_type: str

    @property
    def label(self) -> str:
        return f"{self.endpoint_source}-{self.envelope_type}"


# 全部 12 种组合:
ALL_PIPELINE_CONFIGS = [
    PipelineConfig(ep, env)
    for ep in ["IFK", "CritSample", "Analytical", "GCPC"]
    for env in ["LinkIAABB", "LinkIAABB_Grid", "Hull16_Grid"]
]
```

### ExperimentConfig 扩展

```python
@dataclass
class ExperimentConfig:
    scenes: List[str]
    planners: List[BasePlanner]           # 已有
    pipeline_configs: List[PipelineConfig] | None = None  # 新增
    gcpc_cache_path: str | None = None    # 新增: GCPC cache 共享路径
    n_trials: int = 10
    seeds: List[int] | None = None
    timeout: float = 30.0
    output_dir: str = "results/"
```

### run_experiment 扩展

当 `pipeline_configs` 不为 None 时, 自动用 `SBFPlannerAdapter` 生成 planner 列表:

```python
def run_experiment(config: ExperimentConfig) -> ExperimentResults:
    # 如果指定了 pipeline_configs, 自动构建 planner 列表
    planners = config.planners
    if config.pipeline_configs:
        from .sbf_adapter import SBFPlannerAdapter
        planners = [
            SBFPlannerAdapter(
                endpoint_source=pc.endpoint_source,
                envelope_type=pc.envelope_type,
                gcpc_cache_path=config.gcpc_cache_path,
            )
            for pc in config.pipeline_configs
        ] + planners  # 保留额外传入的 baseline planners

    # ... 其余循环逻辑不变, 使用 planners
```

---

## Step R5: Envelope-Level Metrics

### R5a: PlanResult 扩展 (C++)

```cpp
// sbf_planner.h — PlanResult 新增字段:
struct PlanResult {
    // ... 已有字段 ...

    // ── 新增 (Phase R5) ──
    double envelope_volume_total = 0.0;    // 所有 box 的 envelope volume 之和
    double build_time_ms = 0.0;            // forest build (不含 search/smooth)
    double lect_time_ms = 0.0;             // LECT construct + envelope computation
};
```

`sbf_planner.cpp` 中 `build()` 记录时间:
```cpp
auto t_lect = steady_clock::now();
lect_ = std::make_unique<LECT>(...);
result.lect_time_ms = duration<double, milli>(steady_clock::now() - t_lect).count();
```

### R5b: pybind11 绑定新字段

```cpp
py::class_<sbf::PlanResult>(m, "PlanResult")
    // ... 已有 ...
    .def_readonly("envelope_volume_total", &sbf::PlanResult::envelope_volume_total)
    .def_readonly("build_time_ms",         &sbf::PlanResult::build_time_ms)
    .def_readonly("lect_time_ms",          &sbf::PlanResult::lect_time_ms);
```

### R5c: Python metrics 更新

```python
# sbf_adapter.py — plan() 的 metadata 新增:
metadata={
    "n_boxes": result.n_boxes,
    "n_coarsen_merges": result.n_coarsen_merges,
    "planning_time_ms_cpp": result.planning_time_ms,
    "envelope_volume_total": result.envelope_volume_total,  # 新增
    "build_time_ms": result.build_time_ms,                  # 新增
    "lect_time_ms": result.lect_time_ms,                    # 新增
}
```

---

## 测试

### C++ 测试
| 用例 | 文件 | 描述 |
|------|------|------|
| `planner_with_analytical` | `test/test_planner.cpp` (追加) | 设置 `EndpointSource::Analytical`, 验证 plan() 成功 |
| `planner_with_linkiaabb_grid` | `test/test_planner.cpp` (追加) | 设置 `EnvelopeType::LinkIAABB_Grid`, 验证 plan() 成功 |
| `gcpc_cache_roundtrip` | 已有 `test_endpoint_iaabb.cpp` | 无需新增 |

### Python 测试
| 用例 | 文件 | 描述 |
|------|------|------|
| `test_endpoint_source_enum` | `python/tests/test_sbf5.py` (追加) | 验证 `sbf5.EndpointSource.IFK` 等可访问 |
| `test_envelope_type_enum` | `python/tests/test_sbf5.py` (追加) | 验证 `sbf5.EnvelopeType.LinkIAABB` 等可访问 |
| `test_config_pipeline` | `python/tests/test_sbf5.py` (追加) | 构造 12 种 SBFPlannerConfig 不崩溃 |
| `test_gcpc_load` | `python/tests/test_sbf5.py` (追加) | `GcpcCache.load()` 加载测试 cache |
| `test_adapter_12_configs` | `python/tests/test_bench.py` (追加) | 12 种 SBFPlannerAdapter 均可 setup + plan (2DOF) |

---

## 验收标准

- [ ] `data/panda_5000.gcpc` 和 `data/2dof_500.gcpc` 已生成
- [ ] `SBFPlannerConfig` 包含 `endpoint_source` + `envelope_type` 字段
- [ ] `SBFPlanner::build()` 正确传递配置到 LECT
- [ ] `ctest -C Release` 13/13 + 新增 test → 全部通过
- [ ] Python 可创建 12 种 `SBFPlannerAdapter` 并在 2DOF 场景成功 plan 
- [ ] `PlanResult` 包含 `build_time_ms`、`lect_time_ms`、`envelope_volume_total`
- [ ] `runner.py` 接受 `pipeline_configs` 参数自动生成 12 planner 实例
- [ ] Checkpoint/resume 对新字段兼容

---

## 文件改动清单

| 文件 | 类型 | 改动量 |
|------|------|--------|
| `tools/generate_gcpc_cache.cpp` | 新建 | ~50 LOC |
| `include/sbf/planner/sbf_planner.h` | 修改 | +8 LOC |
| `src/planner/sbf_planner.cpp` | 修改 | +20 LOC |
| `python/sbf5_bindings.cpp` | 修改 | +60 LOC |
| `python/sbf5_bench/sbf_adapter.py` | 修改 | +40 LOC |
| `python/sbf5_bench/runner.py` | 修改 | +30 LOC |
| `test/test_planner.cpp` | 修改 | +20 LOC |
| `python/tests/test_sbf5.py` | 修改 | +25 LOC |
| `python/tests/test_bench.py` | 修改 | +20 LOC |
| `CMakeLists.txt` | 修改 | +3 LOC |
| **总计** | | **~276 LOC** |
