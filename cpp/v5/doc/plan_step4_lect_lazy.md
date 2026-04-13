# Step 4: LECT 懒加载优化（Issue 1）

## 问题描述

SBF planner 在 `build()` 阶段调用 `lect_load_binary()` 一次性将整个 LECT 文件读入内存。

当前 LECT 缓存文件统计：
- 格式：V4 dual-channel AoS + optional grid section
- 节点数：~335K nodes
- 每节点记录大小：32 bytes tree struct + 2 × ep_stride × sizeof(float)
  - ep_stride = n_active_links × 2 × 6 = 8 × 12 = 96 floats
  - 所以每节点 = 32 + 2 × 96 × 4 = 32 + 768 = **800 bytes**
- 总大小（纯节点）：335K × 800 = ~256MB
- 额外：z4_cache、grid section、link_iaabb_cache 等
- 加载时间：~169ms（主要是 IO + unpack + rebuild_derived）

### 为什么是优化目标

虽然 169ms 在总规划时间（~30s build + ~2s query）中不算瓶颈，但随着 LECT 缓存增长，此开销会线性增加。更重要的是，实际使用中只有一小部分节点被 FFB（Find Free Box）查询命中——大多数节点的 ep_data 从未被访问。

## 根因分析

文件：`src/lect/lect_io.cpp`，`load_v4()` 方法（约 L538-L577）

### 当前代码

```cpp
static bool load_v4(LECT& lect, const Robot& robot,
                    std::ifstream& in, const LectFileHeaderV4& hdr) {
    const int nn = hdr.n_nodes;

    // 读取 root intervals
    std::vector<Interval> root_iv;
    if (!read_root_intervals(in, hdr.n_dims, root_iv)) return false;
    set_lect_state(lect, robot, ...);

    alloc_buffers(lect, nn);  // ← 分配所有 vector 内存

    // 一次性读取所有节点记录
    int st = stride_v4(hdr.ep_stride);
    std::vector<uint8_t> bulk(static_cast<size_t>(st) * nn);
    in.read(reinterpret_cast<char*>(bulk.data()),
            static_cast<std::streamsize>(st) * nn);
    if (!in.good()) return false;
    for (int i = 0; i < nn; ++i)
        unpack_node_v4(bulk.data() + static_cast<size_t>(i) * st, lect, i);

    // 后续操作...
    rebuild_derived(lect, robot);
    read_cache_trailer(in, lect, true);
    if (hdr.grid_section) read_grid_section(in, lect);
    return true;
}
```

### 内存使用分析

`alloc_buffers()` 分配的主要 buffer：
- `channels_[2].ep_data`: 2 × 335K × 96 × 4 bytes = **244MB**
- `channels_[2].has_data`, `source_quality`: 2 × 335K × 1 = ~640KB
- `left_, right_, parent_, depth_, split_dim_`: 5 × 335K × 4 = **6.4MB**
- `split_val_`: 335K × 8 = **2.56MB**
- `link_iaabb_cache_`: 335K × 48 × 4 = **61.4MB** (但被标记 dirty）

总计约 **315MB** 堆内存。

## 修改方案：mmap 替代 bulk read

### 核心思路

使用 Linux `mmap()` 将 LECT 文件直接映射到虚拟地址空间，**延迟实际 IO 到首次访问时**（page fault）。树结构（left/right/parent/depth/split_dim/split_val）仍然一次性读入（FFB 遍历需要全部树结构），但 **ep_data 按需加载**。

### 架构设计

```
┌─────────────────────────────────────────────────┐
│  LECT (modified)                                │
│                                                  │
│  tree struct: left_[], right_[], ... split_val_[]│  ← 正常 vector（~10MB）
│  ep_data: mmap pointer → file region             │  ← mmap（~244MB 虚拟）
│  has_data: mmap pointer → file region            │  ← mmap
│                                                  │
│  access_ep(node_i, channel):                     │
│    return mmap_base + node_offset(i, ch)         │
│                                                  │
└─────────────────────────────────────────────────┘
```

### 具体修改

#### 1. 新增 mmap 管理类

文件：`include/sbf/lect/lect_mmap.h`（新建）

```cpp
#pragma once
#include <cstddef>
#include <string>

namespace sbf {

/// RAII wrapper for memory-mapped LECT file
class LectMmap {
public:
    LectMmap() = default;
    ~LectMmap();

    LectMmap(const LectMmap&) = delete;
    LectMmap& operator=(const LectMmap&) = delete;
    LectMmap(LectMmap&& o) noexcept;
    LectMmap& operator=(LectMmap&& o) noexcept;

    bool open(const std::string& path);
    void close();
    bool is_open() const { return data_ != nullptr; }

    const uint8_t* data() const { return data_; }
    size_t size() const { return size_; }

private:
    uint8_t* data_ = nullptr;
    size_t size_ = 0;
    int fd_ = -1;
};

}  // namespace sbf
```

#### 2. mmap 实现

文件：`src/lect/lect_mmap.cpp`（新建）

```cpp
#include <sbf/lect/lect_mmap.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace sbf {

LectMmap::~LectMmap() { close(); }

LectMmap::LectMmap(LectMmap&& o) noexcept
    : data_(o.data_), size_(o.size_), fd_(o.fd_) {
    o.data_ = nullptr; o.size_ = 0; o.fd_ = -1;
}

LectMmap& LectMmap::operator=(LectMmap&& o) noexcept {
    if (this != &o) {
        close();
        data_ = o.data_; size_ = o.size_; fd_ = o.fd_;
        o.data_ = nullptr; o.size_ = 0; o.fd_ = -1;
    }
    return *this;
}

bool LectMmap::open(const std::string& path) {
    close();
    fd_ = ::open(path.c_str(), O_RDONLY);
    if (fd_ < 0) return false;

    struct stat st;
    if (fstat(fd_, &st) < 0) { ::close(fd_); fd_ = -1; return false; }
    size_ = static_cast<size_t>(st.st_size);

    void* ptr = mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd_, 0);
    if (ptr == MAP_FAILED) { ::close(fd_); fd_ = -1; size_ = 0; return false; }

    data_ = static_cast<uint8_t*>(ptr);
    // Advise kernel for sequential read pattern initially
    madvise(data_, size_, MADV_SEQUENTIAL);
    return true;
}

void LectMmap::close() {
    if (data_) { munmap(data_, size_); data_ = nullptr; }
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    size_ = 0;
}

}  // namespace sbf
```

#### 3. 修改 LECT 类

文件：`include/sbf/lect/lect.h`

```cpp
// 新增成员
private:
    LectMmap mmap_;                // mmap handle
    const uint8_t* mmap_data_base_ = nullptr;  // base of node AoS data
    int mmap_node_stride_ = 0;    // bytes per node
    bool use_mmap_ = false;       // whether ep_data comes from mmap

public:
    /// Access ep_data for channel ch at node i (const float*)
    const float* ep_data(int node_i, int ch = CH_SAFE) const;
```

#### 4. 修改 load_v4

```cpp
static bool load_v4(LECT& lect, const Robot& robot,
                    std::ifstream& in, const LectFileHeaderV4& hdr) {
    // ... 不变：read root intervals, set_lect_state ...

    // 新策略：只分配树结构 buffer，ep_data 由 mmap 提供
    const int nn = hdr.n_nodes;
    int st = stride_v4(hdr.ep_stride);

    // 仍然 bulk read 树结构（10MB 级别）
    // 但 ep_data 不再分配 vector，改用 mmap 指针

    alloc_tree_structure(lect, nn);  // 只分配 left/right/parent/depth/split_dim/split_val + has_data
    
    // Bulk read 所有节点，但只 unpack 树结构字段 + has_data
    std::vector<uint8_t> bulk(static_cast<size_t>(st) * nn);
    in.read(reinterpret_cast<char*>(bulk.data()), st * nn);
    for (int i = 0; i < nn; ++i)
        unpack_node_tree_only(bulk.data() + i * st, lect, i);

    // ep_data 通过 mmap 按需访问
    lect.mmap_data_base_ = lect.mmap_.data() + doff_v4(hdr.n_dims);
    lect.mmap_node_stride_ = st;
    lect.use_mmap_ = true;

    // ... 不变：rebuild_derived, read_cache_trailer, read_grid_section ...
}
```

#### 5. ep_data 访问函数

```cpp
const float* LECT::ep_data(int node_i, int ch) const {
    if (use_mmap_) {
        const uint8_t* base = mmap_data_base_ + 
            static_cast<size_t>(node_i) * mmap_node_stride_ + 32;
        if (ch == CH_UNSAFE) base += ep_stride_ * sizeof(float);
        return reinterpret_cast<const float*>(base);
    }
    return channels_[ch].ep_data.data() + node_i * ep_stride_;
}
```

### 不修改的部分

- save（写入）路径不变
- V1/V2/V3 加载不变（仍用 bulk read）
- FFB 查询逻辑不变（只是数据来源从 vector 变为 mmap）
- LECT expand（写入新节点）仍用 vector（mmap 只用于只读加载）

### 平台兼容性

- Linux：`mmap()` / `munmap()` / `madvise()`
- macOS：同上（POSIX 标准）
- Windows：需 `CreateFileMapping` / `MapViewOfFile`（本项目仅 Linux，暂不考虑）

## 预期效果

| 指标 | 修改前 | 修改后预期 |
|------|--------|-----------|
| LECT 加载时间 | ~169ms | <10ms |
| 初始内存占用 | ~315MB | ~15MB（树结构 + 元数据） |
| 峰值内存 | ~315MB | ~315MB（如果所有节点被访问） |
| 首次 FFB 查询 | ~0ms (数据已在内存) | ~1-2ms (page fault) |
| 后续 FFB 查询 | ~0ms | ~0ms (OS page cache) |

## 验证步骤

```bash
cd /home/tian/桌面/box_aabb/cpp/build
cmake --build . --target exp2_e2e_planning -j$(nproc)
./experiments/exp2_e2e_planning --seeds 1 2>&1 | grep -E "lect=|PLN.*ms"
```

检查：
1. `[PLN] lect=Xms` 时间 < 10ms
2. `build_coverage` 总时间不增加（FFB page fault 可被 OS prefetch 覆盖）
3. SR=100%
4. 路径质量不变（Drake collision check clean）
5. `free -m` 确认初始内存占用低于 100MB

## 风险与回退

- **风险**：mmap page fault 在 FFB 密集查询时可能产生抖动 → `madvise(MADV_WILLNEED)` 预热
- **风险**：save 后再 load 的 round-trip 可能需要 sync → 确保 save 后 close fd
- **风险**：修改量大，涉及 LECT 核心数据访问路径 → 需要全面测试（所有 exp）
- **回退**：保留 `use_mmap_` 开关，编译时或运行时选择 mmap vs bulk read
- **渐进方案**：如果 mmap 改动太大，可先只做"skip ep_data alloc + 按需 read/seek"，避免 mmap 系统调用

## 实现复杂度

- 新增文件：2（lect_mmap.h, lect_mmap.cpp）
- 修改文件：2（lect.h, lect_io.cpp）
- 涉及接口：需审计所有 `channels_[ch].ep_data[i * ep_stride_]` 的使用点，替换为 `ep_data(i, ch)` 调用
- 预计修改行数：~200 行新增 + ~100 行修改
- 优先级：**最低**（169ms 不是关键瓶颈，但随缓存增长会变成问题）
