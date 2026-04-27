# 2D Build Trace + Multithreaded Interactive Replay — 详细执行计划

**目标**: 开发 2-DOF Planar 臂的 SBF 构建过程可视化工具，支持多线程日志、帧级回放、GIF/MP4 导出。

**更新时间**: 2026-04-22

**状态**: Phase A-B 规划完毕；Phase C (Python) 详细设计；Phase D 执行清单

---

## 0. 总体架构

```
C++ 侧                               Python 侧
├─ exp_2d_trace                     ├─ viz_2d_build_replay.py
│  ├─ 2dof_planar.json robot        │  ├─ parse log → frames[]
│  ├─ 3 scenes (simple/narrow/      │  ├─ state machine (boxes, FFB descent)
│  │  cluttered)                    │  ├─ matplotlib dual/quad-panel layout
│  ├─ threads=1|4 (configurable)    │  ├─ keyboard: space/←/→/n/b/home/end/q
│  ├─ max_boxes=20|40               │  ├─ GIF/MP4 export (imageio+ffmpeg)
│  ├─ Log: [2D-META] header         │  └─ --frame-mode PRIMITIVE|THREAD|BOX
│  ├─ Log: [tid=N] all traces       │
│  └─ Log: [GRW-FFB] OK + intervals└─ log file (finished)
│
└─ Logger enhancements
   ├─ fmt_intervals() → "[lo,hi],[lo,hi]"
   ├─ [GRW-FFB] OK line + intervals + tid
   └─ [FFB] begin + root_iv once
```

---

## Phase A — C++ Logger Plumbing (Log format + TID)

### A1. `include/sbf/core/log_format.h` — Add `fmt_intervals()`

**Change**: Add function after `fmt_vec()`:

```cpp
/// Format interval vector as "[(-3.14,3.14),(-3.14,1.57)]"
inline std::string fmt_intervals(const std::vector<Interval>& ivs,
                                 int precision = 3) {
    char buf[64];
    std::string out;
    out.reserve(ivs.size() * 20 + 4);
    out.push_back('[');
    for (size_t i = 0; i < ivs.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "(%.*f,%.*f)",
                      precision, ivs[i].lo, precision, ivs[i].hi);
        out += buf;
        if (i + 1 < ivs.size()) out.push_back(',');
    }
    out.push_back(']');
    return out;
}
```

**Include**: `#include <sbf/core/types.h>` at top of header (already has Interval def).

---

### A2. `src/forest/grower.h` — Add worker TID tracking

**Changes**:
1. Add field to `ForestGrower` class:
   ```cpp
   int worker_tid_ = -1;  // Assigned by grow_parallel; -1 = serial mode
   ```

2. Add public setter:
   ```cpp
   void set_worker_tid(int tid) { worker_tid_ = tid; }
   int  get_worker_tid() const { return worker_tid_; }
   ```

**Rationale**: Each parallel worker gets a stable TID [0, n_threads-1] assigned before
it calls grow_rrt() or similar. Grower threads use this in TRACE lines.

---

### A3. `src/forest/grower.cpp` — Enhance `try_create_box()` OK line

**Location**: After line ~186 (the existing `SBF_TRACE("[GRW-FFB] OK ...")` line).

**Change**: Replace the OK TRACE with:

```cpp
const auto& box_intervals = box.joint_intervals;
SBF_TRACE("[GRW-FFB] OK tid=%d box=%d leaf=%d depth=%d steps=%d t=%.2fms "
          "new_nodes=%d cache_h/m=%d/%d parent=%d root=%d seed=%s "
          "intervals=%s",
          worker_tid_, box.id, ffb.node_idx, (int)ffb.path.size() - 1,
          ffb.n_steps, ffb.total_ms, ffb.n_new_nodes,
          ffb.n_cache_hits, ffb.n_cache_misses,
          parent_box_id, root_id, fmt_vec(q).c_str(),
          fmt_intervals(box_intervals).c_str());
```

(Also add `#include <sbf/core/log_format.h>` if not already present.)

**Also inject TID into other GRW traces**:
- `[GRW-FFB] fail code=...` → prepend `tid=%d worker_tid_`
- `[GRW-FFB] reject(occupied)` → prepend `tid=%d worker_tid_`
- `[GRW-ROOT] multi-goal seed ...` → prepend `tid=%d worker_tid_`
- `[GRW-ROOT] endpoint start/goal` → prepend `tid=%d worker_tid_`
- `[GRW-ROOT] diversity ...` → prepend `tid=%d worker_tid_`
- `[GRW-RRT] sample q=...` → prepend `tid=%d worker_tid_`

---

### A4. `src/ffb/ffb.cpp` — Emit `[FFB] begin` with root intervals

**Location**: Entry to `find_free_box()`, right after result init.

**Challenge**: `find_free_box()` doesn't know the worker TID. **Solution**: Have
the *caller* (grower) emit `[FFB] begin` before calling `find_free_box()`.

**New approach**:
- Grower emits: `SBF_TRACE("[FFB] begin tid=%d seed=... root_iv=...", worker_tid_, ...)`
- Then calls `find_free_box(...)`
- FFB emits step lines as usual (step lines don't need tid; Python infers thread from header or sequential read)

**Location to add in grower.cpp::try_create_box()**:
Right before `find_free_box()` call (~line 145):

```cpp
const auto root_iv = lect_.root_intervals();
SBF_TRACE("[FFB] begin tid=%d seed=%s root_iv=%s max_depth=%d",
          worker_tid_, fmt_vec(q).c_str(), fmt_intervals(root_iv).c_str(),
          config_.ffb_config.max_depth);

FFBResult ffb = find_free_box(lect_, q, obs, n_obs, config_.ffb_config);
```

---

### A5. `src/forest/grower_parallel.cpp` — Assign TID in grow_worker()

**Location**: In `ForestGrower::grow_parallel()`, when spawning workers.

**Change**: Before calling `grow_worker(i)`, set TID:

```cpp
void grow_worker(int worker_id) {
    set_worker_tid(worker_id);  // ← Add this line at entry
    // ... rest of grow_rrt, grow_wavefront, etc.
}
```

Or in the lambda:
```cpp
for (int i = 0; i < n_workers; ++i) {
    workers.push_back(std::thread([this, i]() {
        set_worker_tid(i);  // ← Add this line
        grow_worker(i);
    }));
}
```

---

## Phase B — 2D Experiment Binary

### B1. Create `experiments/exp_2d_trace.cpp`

**Features**:
- Hardcoded `data/2dof_planar.json`
- 3 inline obstacle scenes (simple/narrow/cluttered)
- Endpoint-based (start/goal) root creation
- Single run per scene
- Configurable `--scene NAME --threads N --max-boxes M`
- Force TRACE log level + default path `log/2d_trace_<scene>_<ts>_tid<N>.log` (or shared if sequential)
- Emit `[2D-META]` header once at start

**Headers & Utils**:

```cpp
#include <sbf/planner/sbf_planner.h>
#include <sbf/forest/connectivity.h>
#include <sbf/core/robot.h>
#include <sbf/core/log.h>
#include "marcucci_scenes.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

using namespace sbf;
```

**Scene definitions** (hardcoded; match `python/sbf5_bench/scenes.py`):

```cpp
struct Scene2D {
    std::string name;
    std::vector<Obstacle> obstacles;
    Eigen::VectorXd start_config;
    Eigen::VectorXd goal_config;
    std::array<double, 2> link_lengths;  // For workspace viz
};

Scene2D make_2dof_simple() {
    return Scene2D{
        "2dof_simple",
        {Obstacle(0.f - 0.3f, 1.5f - 0.15f, 0.f - 0.3f,
                  0.f + 0.3f, 1.5f + 0.15f, 0.f + 0.3f)},
        Eigen::Vector2d(0.5, 0.5),
        Eigen::Vector2d(2.0, 1.0),
        {1.0, 1.0}
    };
}

Scene2D make_2dof_narrow() {
    return Scene2D{
        "2dof_narrow",
        {Obstacle(0.f - 0.8f, 1.8f - 0.1f, 0.f - 0.8f,
                  0.f + 0.8f, 1.8f + 0.1f, 0.f + 0.8f),
         Obstacle(0.f - 0.8f, 1.2f - 0.1f, 0.f - 0.8f,
                  0.f + 0.8f, 1.2f + 0.1f, 0.f + 0.8f)},
        Eigen::Vector2d(0.5, 0.3),
        Eigen::Vector2d(2.5, 1.5),
        {1.0, 1.0}
    };
}

Scene2D make_2dof_cluttered() {
    return Scene2D{
        "2dof_cluttered",
        {Obstacle(0.f - 0.15f, 1.5f - 0.15f, 0.f - 0.15f,
                  0.f + 0.15f, 1.5f + 0.15f, 0.f + 0.15f),
         Obstacle(-0.8f - 0.12f, 1.0f - 0.12f, 0.f - 0.12f,
                  -0.8f + 0.12f, 1.0f + 0.12f, 0.f + 0.12f)},
        Eigen::Vector2d(0.5, 0.3),
        Eigen::Vector2d(2.5, 1.5),
        {1.0, 1.0}
    };
}
```

**main() logic**:

```cpp
int main(int argc, char** argv) {
    sbf::set_log_level(sbf::LogLevel::TRACE);
    if (const char* f = std::getenv("SBF_LOG_FILE"); f && *f) {
        sbf::set_log_file(f);
    } else {
        // Use default: log/2d_trace_<scene>_<ts>.log
        std::time_t t = std::time(nullptr);
        std::tm tm{};
        localtime_r(&t, &tm);
        char ts[32], path[1024];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
        ::mkdir(SBF_LOG_DIR, 0755);
        std::snprintf(path, sizeof(path), "%s/2d_trace_%s_%s.log",
                      SBF_LOG_DIR, scene_name.c_str(), ts);
        sbf::set_log_file(path);
        std::fprintf(stderr, "[exp_2d_trace] log -> %s\n", path);
    }

    // Parse CLI args
    std::string robot_path = std::string(SBF_DATA_DIR) + "/2dof_planar.json";
    std::string scene_name = "simple";
    int max_boxes = 40;
    int n_threads = 1;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--scene" && i + 1 < argc) scene_name = argv[++i];
        else if (a == "--max-boxes" && i + 1 < argc) max_boxes = std::atoi(argv[++i]);
        else if (a == "--threads" && i + 1 < argc) n_threads = std::atoi(argv[++i]);
    }

    Robot robot = Robot::from_json(robot_path);
    Scene2D scene;
    if (scene_name == "narrow") scene = make_2dof_narrow();
    else if (scene_name == "cluttered") scene = make_2dof_cluttered();
    else scene = make_2dof_simple();

    // Emit 2D metadata header
    std::string obstacles_str = "[";
    for (size_t k = 0; k < scene.obstacles.size(); ++k) {
        const auto& obs = scene.obstacles[k];
        if (k > 0) obstacles_str += ",";
        char buf[256];
        std::snprintf(buf, sizeof(buf), "(%.3f,%.3f,%.3f,%.3f)",
                      obs.bounds[0], obs.bounds[1], obs.bounds[3], obs.bounds[4]);
        obstacles_str += buf;
    }
    obstacles_str += "]";

    char limits_str[256];
    std::snprintf(limits_str, sizeof(limits_str),
                  "[(-%.4f,%.4f),(-%.4f,%.4f)]",
                  M_PI, M_PI, M_PI, M_PI);

    SBF_INFO("[2D-META] scene=%s dof=%d robot=%s n_threads=%d max_boxes=%d "
             "limits=%s obstacles=%s start=[%.3f,%.3f] goal=[%.3f,%.3f] "
             "link_lengths=[%.3f,%.3f]",
             scene_name.c_str(), 2, robot.name().c_str(), n_threads, max_boxes,
             limits_str, obstacles_str.c_str(),
             scene.start_config[0], scene.start_config[1],
             scene.goal_config[0], scene.goal_config[1],
             scene.link_lengths[0], scene.link_lengths[1]);

    // Config + build
    SBFPlannerConfig cfg;
    cfg.z4_enabled = true;
    cfg.split_order = SplitOrder::BEST_TIGHTEN;
    cfg.lect_no_cache = false;

    cfg.grower.mode = GrowerConfig::Mode::RRT;
    cfg.grower.max_boxes = max_boxes;
    cfg.grower.timeout_ms = 30000.0;
    cfg.grower.n_threads = n_threads;
    cfg.grower.bridge_n_threads = 1;
    cfg.grower.rng_seed = 0;
    cfg.grower.max_consecutive_miss = 200;
    cfg.grower.rrt_goal_bias = 0.5;
    cfg.grower.rrt_step_ratio = 0.05;
    cfg.grower.connect_mode = true;  // Enable bridge
    cfg.grower.enable_promotion = false;
    cfg.grower.ffb_config.max_depth = 100;
    cfg.coarsen.target_boxes = max_boxes;
    cfg.coarsen.max_rounds = 0;

    SBFPlanner planner(robot, cfg);
    std::vector<Eigen::VectorXd> seed_points = {
        scene.start_config, scene.goal_config
    };

    auto t0 = std::chrono::steady_clock::now();
    planner.build_coverage(scene.obstacles.data(), scene.obstacles.size(),
                           cfg.grower.timeout_ms, seed_points);
    double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

    int n_boxes = planner.n_boxes();
    const auto& adj = planner.adjacency();
    int n_edges = 0;
    for (auto& kv : adj) n_edges += static_cast<int>(kv.second.size());
    n_edges /= 2;
    auto islands = find_islands(adj);

    SBF_INFO("[2D-DONE] elapsed=%.2fs boxes=%d edges=%d islands=%zu",
             elapsed_s, n_boxes, n_edges, islands.size());

    std::cout << "2D trace done. See log: " << SBF_LOG_DIR << "/\n";
    return 0;
}
```

### B2. Register in `experiments/CMakeLists.txt`

**Add line**:
```cmake
sbf_add_experiment(exp_2d_trace exp_2d_trace.cpp)
```

---

## Phase C — Python Replay Viewer

### C1. Create `scripts/viz_2d_build_replay.py`

**High-level structure**:

```python
#!/usr/bin/env python3
"""
viz_2d_build_replay.py — 2D SBF build trace interactive replay + video export.

Features:
  - Parse multi-threaded trace log into frames
  - State machine: boxes, FFB descent rectangle, bridge pairs
  - Dual/quad-panel matplotlib viewer (C-space + workspace per thread)
  - Keyboard: space/→/←/n/b/home/end/q to navigate
  - GIF/MP4 export: --video out.mp4 [--fps 10] [--bitrate 2500]
  - Frame aggregation: --frame-mode PRIMITIVE | THREAD | BOX
  - Optional: --save-frames outdir/ for PNG dump

Usage:
  python3 viz_2d_build_replay.py LOGFILE [--video out.mp4] [--fps 15]
  python3 viz_2d_build_replay.py LOGFILE --frame-mode THREAD --threads 4
"""

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation


@dataclass
class Interval:
    """1D interval [lo, hi]."""
    lo: float
    hi: float

    def center(self) -> float:
        return 0.5 * (self.lo + self.hi)

    def width(self) -> float:
        return self.hi - self.lo


@dataclass
class BoxInfo:
    """C-space box."""
    box_id: int
    intervals: List[Interval]
    seed: np.ndarray
    root_id: int
    parent_id: int
    frame_idx: int  # When created

    def get_rect(self) -> Tuple[float, float, float, float]:
        """Return (q1_lo, q1_hi, q2_lo, q2_hi) for matplotlib Rectangle."""
        if len(self.intervals) >= 2:
            return (self.intervals[0].lo, self.intervals[1].lo,
                    self.intervals[0].width(), self.intervals[1].width())
        return (0, 0, 0, 0)


@dataclass
class FFBState:
    """Current FFB descent."""
    seed: np.ndarray
    intervals: List[Interval]  # Running intervals (shrink over steps)
    step: int = 0
    max_depth: int = 0
    history: List[dict] = field(default_factory=list)
    active: bool = False
    outcome: Optional[str] = None  # 'free', 'fail', 'occupied', None


@dataclass
class Frame:
    """A single replay frame."""
    idx: int
    kind: str  # 'metadata', 'ffb-begin', 'ffb-step', 'ffb-result', 'box-ok', 'rrt-sample', 'bridge', ...
    tid: int  # Thread ID (-1 for serial)
    payload: dict = field(default_factory=dict)
    raw_line: str = ""


class LogParser:
    """Parse 2D trace log into frames."""

    def __init__(self, logfile: str):
        self.logfile = logfile
        self.frames: List[Frame] = []
        self.metadata: dict = {}
        self.parse()

    def parse(self):
        """Parse log file into frames."""
        with open(self.logfile, 'r') as f:
            for line_no, raw_line in enumerate(f):
                raw_line = raw_line.rstrip('\n')
                if not raw_line:
                    continue

                # Extract TID if present
                tid = -1
                tid_match = re.search(r'\[tid=(\d+)\]', raw_line)
                if tid_match:
                    tid = int(tid_match.group(1))

                # Route by pattern
                if '[2D-META]' in raw_line:
                    self._parse_meta(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='metadata', tid=tid,
                        payload={'line': raw_line}, raw_line=raw_line
                    ))
                elif '[GRW-ROOT] endpoint start' in raw_line:
                    seed = self._extract_seed(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='root-seed-start', tid=tid,
                        payload={'seed': seed}, raw_line=raw_line
                    ))
                elif '[GRW-ROOT] endpoint goal' in raw_line:
                    seed = self._extract_seed(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='root-seed-goal', tid=tid,
                        payload={'seed': seed}, raw_line=raw_line
                    ))
                elif '[GRW-ROOT] diversity' in raw_line:
                    payload = self._parse_diversity_root(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='root-seed-diversity', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[FFB] begin' in raw_line:
                    payload = self._parse_ffb_begin(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='ffb-begin', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[FFB]  step=' in raw_line:
                    payload = self._parse_ffb_step(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='ffb-step', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[GRW-FFB] OK' in raw_line:
                    payload = self._parse_box_ok(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='box-ok', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[GRW-FFB] fail' in raw_line or '[GRW-FFB] reject' in raw_line:
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='box-fail', tid=tid,
                        payload={'line': raw_line}, raw_line=raw_line
                    ))
                elif '[CHN]' in raw_line:
                    payload = self._parse_chain_step(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='bridge-step', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[BRG]' in raw_line and 'pair' in raw_line:
                    payload = self._parse_bridge_pair(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='bridge-pair', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[GRW-RRT] sample' in raw_line:
                    payload = self._parse_rrt_sample(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='rrt-sample', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))

    def _parse_meta(self, line: str):
        """Extract [2D-META] info."""
        # Format: [2D-META] scene=foo dof=2 robot=... limits=... obstacles=... start=... goal=... link_lengths=...
        m = re.search(r'scene=(\w+)', line)
        if m:
            self.metadata['scene'] = m.group(1)
        # ... extract other fields as needed

    def _extract_seed(self, line: str) -> Optional[np.ndarray]:
        """Extract `seed=[...]` from line."""
        m = re.search(r'seed=\[([\d\.\-,]+)\]', line)
        if m:
            vals = [float(x) for x in m.group(1).split(',')]
            return np.array(vals)
        return None

    def _parse_diversity_root(self, line: str) -> dict:
        # Parse: [GRW-ROOT] diversity r=R/K=30 best_k=K best_min_dist=D seed=[...]
        payload = {'seed': self._extract_seed(line)}
        m = re.search(r'best_min_dist=([\d\.]+)', line)
        if m:
            payload['distance'] = float(m.group(1))
        return payload

    def _parse_ffb_begin(self, line: str) -> dict:
        # Parse: [FFB] begin tid=N seed=... root_iv=... max_depth=...
        payload = {'seed': self._extract_seed(line)}
        m = re.search(r'root_iv=\[(.*?)\]', line)
        if m:
            payload['root_iv'] = self._parse_intervals(m.group(1))
        m = re.search(r'max_depth=(\d+)', line)
        if m:
            payload['max_depth'] = int(m.group(1))
        return payload

    def _parse_ffb_step(self, line: str) -> dict:
        # Parse: [FFB]  step=N node=X depth=D ... COLLIDE descend split_dim=K split_val=V -> L|R
        payload = {}
        m = re.search(r'step=(\d+)', line)
        if m:
            payload['step'] = int(m.group(1))
        m = re.search(r'depth=(\d+)', line)
        if m:
            payload['depth'] = int(m.group(1))
        m = re.search(r'split_dim=(\d+) split_val=([\d\.\-]+)', line)
        if m:
            payload['split_dim'] = int(m.group(1))
            payload['split_val'] = float(m.group(2))
        m = re.search(r'-> (L|R)', line)
        if m:
            payload['side'] = m.group(1)
        if 'EXPAND' in line:
            payload['action'] = 'expand'
        elif 'descend' in line:
            payload['action'] = 'descend'
        elif 'FREE' in line:
            payload['action'] = 'free'
        elif 'FAIL' in line:
            payload['action'] = 'fail'
        return payload

    def _parse_box_ok(self, line: str) -> dict:
        # Parse: [GRW-FFB] OK ... seed=... intervals=...
        payload = {'seed': self._extract_seed(line)}
        m = re.search(r'box=(\d+)', line)
        if m:
            payload['box_id'] = int(m.group(1))
        m = re.search(r'intervals=\[(.*?)\]', line)
        if m:
            payload['intervals'] = self._parse_intervals(m.group(1))
        m = re.search(r'parent=(\d+)', line)
        if m:
            payload['parent_id'] = int(m.group(1))
        m = re.search(r'root=(\d+)', line)
        if m:
            payload['root_id'] = int(m.group(1))
        return payload

    def _parse_intervals(self, s: str) -> List[Interval]:
        """Parse "(-3.14,3.14),(-3.14,1.57)" into Interval list."""
        intervals = []
        for match in re.finditer(r'\(([\d\.\-]+),([\d\.\-]+)\)', s):
            intervals.append(Interval(float(match.group(1)), float(match.group(2))))
        return intervals

    def _parse_rrt_sample(self, line: str) -> dict:
        payload = {'q': self._extract_seed(line)}
        m = re.search(r'nearest box=(\d+)', line)
        if m:
            payload['nearest_box'] = int(m.group(1))
        return payload

    def _parse_chain_step(self, line: str) -> dict:
        # [CHN] ...
        return {'raw': line}

    def _parse_bridge_pair(self, line: str) -> dict:
        # [BRG] pair i/N: box A->B, dist=D, ...
        m = re.search(r'box (\d+)->(\d+)', line)
        if m:
            return {'from_box': int(m.group(1)), 'to_box': int(m.group(2))}
        return {}


class ViewerState:
    """Replay state machine."""

    def __init__(self, metadata: dict):
        self.metadata = metadata
        self.boxes: Dict[int, BoxInfo] = {}
        self.ffb_state: Optional[FFBState] = None
        self.rrt_sample: Optional[np.ndarray] = None
        self.bridge_pairs: List[Tuple[int, int]] = []
        self.last_box_id = -1

    def apply(self, frame: Frame):
        """Update state given frame."""
        if frame.kind == 'ffb-begin':
            seed = frame.payload.get('seed')
            root_iv = frame.payload.get('root_iv', [])
            max_depth = frame.payload.get('max_depth', 100)
            self.ffb_state = FFBState(
                seed=seed,
                intervals=root_iv,
                max_depth=max_depth,
                active=True
            )
        elif frame.kind == 'ffb-step':
            if self.ffb_state:
                action = frame.payload.get('action')
                if action == 'descend':
                    dim = frame.payload.get('split_dim')
                    val = frame.payload.get('split_val')
                    side = frame.payload.get('side')
                    if dim is not None and val is not None:
                        if side == 'L':
                            self.ffb_state.intervals[dim].hi = val
                        else:
                            self.ffb_state.intervals[dim].lo = val
        elif frame.kind == 'box-ok':
            box_id = frame.payload.get('box_id', len(self.boxes))
            seed = frame.payload.get('seed')
            intervals = frame.payload.get('intervals', [])
            parent_id = frame.payload.get('parent_id', -1)
            root_id = frame.payload.get('root_id', -1)
            box = BoxInfo(
                box_id=box_id, intervals=intervals, seed=seed,
                root_id=root_id, parent_id=parent_id, frame_idx=frame.idx
            )
            self.boxes[box_id] = box
            self.last_box_id = box_id
            self.ffb_state = None
        elif frame.kind == 'bridge-pair':
            a = frame.payload.get('from_box')
            b = frame.payload.get('to_box')
            if a is not None and b is not None:
                self.bridge_pairs.append((a, b))
        elif frame.kind == 'rrt-sample':
            self.rrt_sample = frame.payload.get('q')


class Viewer2D:
    """Interactive 2D SBF trace viewer."""

    def __init__(self, frames: List[Frame], metadata: dict, save_frames_dir: Optional[str] = None,
                 video_path: Optional[str] = None, fps: int = 10):
        self.frames = frames
        self.metadata = metadata
        self.state = ViewerState(metadata)
        self.current_frame_idx = 0
        self.save_frames_dir = save_frames_dir
        self.video_path = video_path
        self.fps = fps

        # Parse metadata
        limits_str = metadata.get('limits', '[(-3.14,3.14),(-3.14,3.14)]')
        limits_match = re.findall(r'\(([\d\.\-]+),([\d\.\-]+)\)', limits_str)
        self.joint_limits = [Interval(float(m[0]), float(m[1])) for m in limits_match]

        obstacles_str = metadata.get('obstacles', '')
        obstacles_match = re.findall(r'\(([\d\.\-]+),([\d\.\-]+),([\d\.\-]+),([\d\.\-]+)\)', obstacles_str)
        self.ws_obstacles = []  # [(x_min, y_min, x_max, y_max), ...]
        for m in obstacles_match:
            self.ws_obstacles.append((float(m[0]), float(m[1]), float(m[2]), float(m[3])))

        self.link_lengths = [1.0, 1.0]  # Default
        links_str = metadata.get('link_lengths', '[1.0,1.0]')
        links_match = re.findall(r'([\d\.]+)', links_str)
        if len(links_match) >= 2:
            self.link_lengths = [float(links_match[0]), float(links_match[1])]

        # Setup matplotlib
        self.fig, (self.ax_cs, self.ax_ws) = plt.subplots(1, 2, figsize=(14, 6))
        self.setup_axes()

        # Render loop
        self.draw_frame(0)

    def setup_axes(self):
        """Configure axes."""
        # C-space
        if len(self.joint_limits) >= 2:
            lo_q1, hi_q1 = self.joint_limits[0].lo, self.joint_limits[0].hi
            lo_q2, hi_q2 = self.joint_limits[1].lo, self.joint_limits[1].hi
            self.ax_cs.set_xlim(lo_q1, hi_q1)
            self.ax_cs.set_ylim(lo_q2, hi_q2)
            self.ax_cs.set_aspect('equal')
            self.ax_cs.set_xlabel('q1 (rad)')
            self.ax_cs.set_ylabel('q2 (rad)')
            self.ax_cs.set_title('Configuration Space')
            self.ax_cs.grid(alpha=0.3)

        # Workspace
        self.ax_ws.set_xlim(-0.5, 2.5)
        self.ax_ws.set_ylim(-0.5, 2.5)
        self.ax_ws.set_aspect('equal')
        self.ax_ws.set_xlabel('x (m)')
        self.ax_ws.set_ylabel('y (m)')
        self.ax_ws.set_title('Workspace')
        self.ax_ws.grid(alpha=0.3)

    def draw_frame(self, frame_idx: int):
        """Render frame with index frame_idx."""
        if frame_idx < 0 or frame_idx >= len(self.frames):
            return

        # Replay up to frame_idx
        self.state = ViewerState(self.metadata)
        for i in range(frame_idx + 1):
            self.state.apply(self.frames[i])

        # Clear axes
        self.ax_cs.clear()
        self.ax_ws.clear()
        self.setup_axes()

        # Draw C-space boxes
        colors = plt.cm.tab10(np.linspace(0, 1, 10))
        for box_id, box in self.state.boxes.items():
            x, y, w, h = box.get_rect()
            color_id = box.root_id % 10 if box.root_id >= 0 else 0
            alpha = 0.7 if box_id != self.state.last_box_id else 1.0
            edge = 'black' if box_id == self.state.last_box_id else None
            linewidth = 3 if box_id == self.state.last_box_id else 1
            rect = mpatches.Rectangle((x, y), w, h,
                                       facecolor=colors[color_id], alpha=alpha,
                                       edgecolor=edge, linewidth=linewidth)
            self.ax_cs.add_patch(rect)

        # Draw FFB descent rectangle
        if self.state.ffb_state and len(self.state.ffb_state.intervals) >= 2:
            iv = self.state.ffb_state.intervals
            x, y = iv[0].lo, iv[1].lo
            w, h = iv[0].width(), iv[1].width()
            rect = mpatches.Rectangle((x, y), w, h,
                                       facecolor='none', edgecolor='orange',
                                       linestyle='--', linewidth=2)
            self.ax_cs.add_patch(rect)

        # Draw workspace obstacles
        for x_min, y_min, x_max, y_max in self.ws_obstacles:
            rect = mpatches.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                                       facecolor='grey', alpha=0.5)
            self.ax_ws.add_patch(rect)

        # Draw arm at last box seed
        arm_q = None
        if self.state.last_box_id >= 0 and self.state.last_box_id in self.state.boxes:
            arm_q = self.state.boxes[self.state.last_box_id].seed
        if arm_q is not None and len(arm_q) >= 2:
            self._draw_arm(arm_q)

        # Status/title
        frame = self.frames[frame_idx]
        title_cs = f"C-space | Frame {frame_idx}/{len(self.frames)} | " \
                   f"Boxes: {len(self.state.boxes)} | Kind: {frame.kind}"
        self.ax_cs.set_title(title_cs)
        self.ax_ws.set_title(f"Workspace | Thread: {frame.tid}")

        self.fig.suptitle(f"[{frame_idx}] {frame.raw_line[:80]}", fontsize=8)
        self.fig.canvas.draw()

    def _draw_arm(self, q: np.ndarray):
        """Draw 2-link arm in workspace."""
        q1, q2 = q[0], q[1]
        a1, a2 = self.link_lengths
        # Joint 0
        p0 = np.array([0, 0])
        # Joint 1
        p1 = p0 + a1 * np.array([np.cos(q1), np.sin(q1)])
        # End-effector
        p2 = p1 + a2 * np.array([np.cos(q1 + q2), np.sin(q1 + q2)])

        # Draw links
        self.ax_ws.plot([p0[0], p1[0]], [p0[1], p1[1]], 'b-', linewidth=2)
        self.ax_ws.plot([p1[0], p2[0]], [p1[1], p2[1]], 'r-', linewidth=2)
        # Joints
        self.ax_ws.plot([p0[0]], [p0[1]], 'ko', markersize=6)
        self.ax_ws.plot([p1[0]], [p1[1]], 'ko', markersize=5)
        # EE
        self.ax_ws.plot([p2[0]], [p2[1]], 'r*', markersize=12)

    def on_key(self, event):
        """Keyboard event handler."""
        if event.key == ' ' or event.key == 'right':
            self.current_frame_idx = min(self.current_frame_idx + 1, len(self.frames) - 1)
        elif event.key == 'left':
            self.current_frame_idx = max(self.current_frame_idx - 1, 0)
        elif event.key == 'n':
            # Next box-ok
            for i in range(self.current_frame_idx + 1, len(self.frames)):
                if self.frames[i].kind == 'box-ok':
                    self.current_frame_idx = i
                    break
        elif event.key == 'b':
            # Next FFB begin
            for i in range(self.current_frame_idx + 1, len(self.frames)):
                if self.frames[i].kind == 'ffb-begin':
                    self.current_frame_idx = i
                    break
        elif event.key == 'home':
            self.current_frame_idx = 0
        elif event.key == 'end':
            self.current_frame_idx = len(self.frames) - 1
        elif event.key == 'q':
            plt.close(self.fig)
            return

        self.draw_frame(self.current_frame_idx)

    def run_interactive(self):
        """Launch interactive viewer."""
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        print(f"Loaded {len(self.frames)} frames. Keys: space/→/← n b home end q")
        plt.show()

    def export_video(self):
        """Export frames to video."""
        if not self.video_path:
            return

        try:
            import imageio
        except ImportError:
            print("Error: imageio not installed. Skipping video export.")
            return

        print(f"Exporting {len(self.frames)} frames to {self.video_path}...")
        frames_rgb = []
        for i in range(len(self.frames)):
            self.draw_frame(i)
            # Capture canvas
            canvas = self.fig.canvas
            canvas.draw()
            buf = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
            buf = buf.reshape(canvas.get_width_height()[::-1] + (3,))
            frames_rgb.append(buf)

        imageio.mimsave(self.video_path, frames_rgb, fps=self.fps)
        print(f"Video saved: {self.video_path}")


def main():
    parser = argparse.ArgumentParser(description="2D SBF build trace replay viewer.")
    parser.add_argument('logfile', help="Trace log file")
    parser.add_argument('--video', help="Export to video file (MP4 or GIF)")
    parser.add_argument('--fps', type=int, default=10, help="Video FPS (default 10)")
    parser.add_argument('--save-frames', help="Save PNG frames to directory")
    parser.add_argument('--frame-mode', default='PRIMITIVE', choices=['PRIMITIVE', 'THREAD', 'BOX'],
                        help="Frame aggregation mode")
    args = parser.parse_args()

    if not Path(args.logfile).exists():
        print(f"Error: log file not found: {args.logfile}")
        sys.exit(1)

    print(f"Parsing {args.logfile}...")
    parser_obj = LogParser(args.logfile)
    print(f"Parsed {len(parser_obj.frames)} frames. Metadata: {parser_obj.metadata}")

    viewer = Viewer2D(parser_obj.frames, parser_obj.metadata,
                      save_frames_dir=args.save_frames,
                      video_path=args.video,
                      fps=args.fps)

    if args.video:
        viewer.export_video()
    else:
        viewer.run_interactive()


if __name__ == '__main__':
    main()
```

---

## Phase D — Implementation Execution Checklist

### Subtask D1: C++ Logger Changes (2-3 hours)

- [ ] **D1.1** Edit [include/sbf/core/log_format.h](cpp/v6/include/sbf/core/log_format.h):
  - Add `#include <sbf/core/types.h>`
  - Add `fmt_intervals()` function

- [ ] **D1.2** Edit [include/sbf/forest/grower.h](cpp/v6/include/sbf/forest/grower.h):
  - Add `int worker_tid_ = -1`
  - Add `set_worker_tid(int)` and `get_worker_tid()` methods

- [ ] **D1.3** Edit [src/forest/grower.cpp](cpp/v6/src/forest/grower.cpp):
  - Add `#include <sbf/core/log_format.h>` if missing
  - Inject `worker_tid_` into all GRW-FFB TRACE lines (6 locations: ok, fail, reject, root-*goals, root-diversity, rrt-sample)
  - Append `intervals=%s` to `[GRW-FFB] OK` line, computed from `fmt_intervals(box.joint_intervals)`
  - Add `[FFB] begin` emission before `find_free_box()` call in `try_create_box()`

- [ ] **D1.4** Edit [src/forest/grower_parallel.cpp](cpp/v6/src/forest/grower_parallel.cpp):
  - Call `set_worker_tid(i)` in the worker lambda before `grow_worker()`

- [ ] **D1.5** (Optional) Edit [src/ffb/ffb.cpp](cpp/v6/src/ffb/ffb.cpp):
  - Confirm `[FFB]  step=...` lines are still correct (no changes needed if grower emits [FFB] begin)

### Subtask D2: Exp_2d_trace Binary (1-2 hours)

- [ ] **D2.1** Create [experiments/exp_2d_trace.cpp](cpp/v6/experiments/exp_2d_trace.cpp):
  - Full file as in Phase B1 above

- [ ] **D2.2** Edit [experiments/CMakeLists.txt](cpp/v6/experiments/CMakeLists.txt):
  - Add `sbf_add_experiment(exp_2d_trace exp_2d_trace.cpp)`

- [ ] **D2.3** Build:
  ```bash
  cd cpp/v6/build
  cmake ..
  cmake --build . --target exp_2d_trace -j
  ```

- [ ] **D2.4** Smoke test:
  ```bash
  ./experiments/exp_2d_trace --scene simple --threads 1 --max-boxes 20
  ls -lh ../log/2d_trace_*.log
  grep -c '\[GRW-ROOT\]' ../log/2d_trace_*.log
  ```

### Subtask D3: Python Viewer (2-3 hours)

- [ ] **D3.1** Create [scripts/viz_2d_build_replay.py](cpp/v6/scripts/viz_2d_build_replay.py):
  - Full file as in Phase C1 above

- [ ] **D3.2** Test imports:
  ```bash
  python3 -c "import numpy, matplotlib, matplotlib.pyplot; print('OK')"
  ```

- [ ] **D3.3** Run viewer (interactive mode):
  ```bash
  cd cpp/v6
  python3 scripts/viz_2d_build_replay.py log/2d_trace_2dof_simple_*.log
  ```
  - Verify frame 0 loads, bars/keys work

- [ ] **D3.4** Export MP4:
  ```bash
  python3 scripts/viz_2d_build_replay.py log/2d_trace_2dof_simple_*.log \
    --video /tmp/2d_simple.mp4 --fps 15
  ```
  - Check file size (~5-50 MB for 300 frames @ 15 fps)

### Subtask D4: Multithreaded Smoke Test (1 hour)

- [ ] **D4.1** Re-run exp_2d_trace with 4 threads:
  ```bash
  ./build/experiments/exp_2d_trace --scene narrow --threads 4 --max-boxes 40
  ```

- [ ] **D4.2** Verify TID in log:
  ```bash
  grep -E '\[tid=[0-3]\]' ../log/2d_trace_*.log | head -20
  ```

- [ ] **D4.3** Export multithreaded video:
  ```bash
  python3 scripts/viz_2d_build_replay.py log/2d_trace_2dof_narrow_*.log \
    --video /tmp/2d_narrow_mt.mp4 --fps 10
  ```

### Subtask D5: Documentation & Cleanup (30 min)

- [ ] **D5.1** Update [experiments/README.md](cpp/v6/experiments/README.md) or create new doc with usage:
  ```
  ## exp_2d_trace — 2D Build Trace Replay
  
  Build single-threaded or multithreaded 2-DOF Planar traces with full frame logging.
  
  Usage:
    ./exp_2d_trace [--scene simple|narrow|cluttered] [--threads N] [--max-boxes M]
  
  Viewer:
    python3 ../scripts/viz_2d_build_replay.py log/2d_trace_*.log [--video out.mp4]
  ```

- [ ] **D5.2** Verify all log files created and viewer parses successfully

---

## Video Export Troubleshooting

- **imageio not found**: `pip install imageio imageio-ffmpeg`
- **ffmpeg not found**: fallback to GIF (slower, larger files)
- **Canvas size mismatch**: ensure figure DPI and size are consistent (use `self.fig.set_dpi(100)`)

---

## Timeline

| Phase | Task | Est. Time |
|-------|------|-----------|
| D1 | C++ logger changes | 2–3h |
| D2 | Exp_2d_trace binary + CMake | 1–2h |
| D3 | Python viewer (full) | 2–3h |
| D4 | Smoke tests (both serial + MT) | 1h |
| D5 | Docs + cleanup | 0.5h |
| **Total** | | **6.5–9.5h** |

**Next step**: Begin D1.1 (log_format.h update).
