# SafeBoxForest (SBF) v4 重构与升级计划

> **目标**：将 `v3` 代码库平滑迁移至 `v4`，并在 `v4` 中实现架构与算法的深度优化（B、C、E 项）。
> **核心原则**：**绝对不能损失现有代码性能**。所有修改必须经过严格的基准测试（Benchmark）验证。
> **日期**：2026-02-23

---

## 1. 核心改进目标

在 v4 版本中，我们将重点实现以下三个核心改进：

### B. RRT Bridge 路径的“Box 化” (Paving the Bridge)
*   **动机**：目前当 forest 不连通时，会调用 RRT-Connect 作为 fallback 寻找桥接路径。但这条路径有时仅作为一次性解返回，未能完全反哺给 forest。
*   **方案**：将 RRT 找到的无碰撞路径（waypoints）作为 seed，强制在其周围生成一系列 box（Paving），并将这些 box 注入到 forest 中。
*   **收益**：未来的查询可以直接复用这条被 RRT 证明可行的通道，真正发挥 SBF “单调增长、越用越快”的终极优势。

### C. 邻接图构建的进一步并行化 (Parallel Adjacency Graph)
*   **动机**：虽然 `v3` 中 `_build_adjacency_and_islands` 已经通过 numpy 分块向量化（Chunking）解决了内存爆炸问题，但对于超大规模的 forest（>10,000 boxes），单线程计算仍有优化空间。
*   **方案**：利用分块计算天然的数据独立性，引入多进程（`ProcessPoolExecutor`）或在 Cython 中释放 GIL 并使用 OpenMP，将不同 chunk 的 overlap 计算分配到多核 CPU 上。
*   **收益**：将大规模 forest 的邻接图构建时间进一步压缩至毫秒级。

### E. Cython 缓存的动态扩容 (Dynamic Mmap Resizing)
*   **动机**：`NodeStore` 目前使用预分配的 mmap 缓冲区（固定 capacity）。在极端复杂场景下，如果节点数超过预设上限，会导致程序崩溃。
*   **方案**：实现类似 `std::vector` 的动态扩容机制。当节点数达到容量上限时，自动重新分配更大的文件并拷贝/映射数据。
*   **收益**：彻底消除容量上限带来的隐患，提升工业级应用的稳定性和鲁棒性。

---

## 2. 迁移与实施路线图

为了保证性能不退化，实施过程将严格遵循“**迁移 -> 基准测试 -> 逐项修改 -> 回归测试**”的流程。

### 阶段 1：基础迁移、框架精简与基准测试 (Base Migration, Cleanup & Benchmarking)
1.  **代码复制与重命名**：将 `v3` 目录完整复制为 `v4`。根据命名规范，将 `v4/src/planner/box_planner.py` 重命名为 `sbf_planner.py`，将 `BoxForest` 重命名为 `SafeBoxForest` 等。
2.  **废弃代码清理 (Dead Code Removal)**：
    *   删除 `v4/src/baselines/` 中不再使用的旧适配器（如 `box_aabb_adapter.py`，统一使用 `sbf_adapter.py`）。
    *   清理 `v4/src/planner/` 中遗留的旧类（如 `BoxRRT` 别名、`AffineForm`、`HierAABBNode` 等）。
    *   精简 `v4/examples/` 目录，删除重复的 demo 脚本（如 `gcs_planner_panda.py`），将核心管线逻辑（如 `grow_and_prepare`）从 example 移入 `src/planner/pipeline.py`。
3.  **目录结构优化**：
    *   将所有实验配置统一移至 `v4/experiments/configs/`。
    *   将所有绘图和可视化脚本统一移至 `v4/viz/`。
4.  **环境隔离**：确保 `v4` 拥有独立的测试入口和 `pyproject.toml`。
5.  **建立性能基线**：运行 `v4/benchmarks/` 下的核心测试（如 `bench_hier_cache.py`, `bench_panda_forest.py`），记录 `v3` 架构下的各项耗时（Grow, Adjacency, Solve, Cache IO）作为基线数据。

### 阶段 2：实现 E - Cython 缓存动态扩容
1.  **修改 `_hier_core.pyx`**：
    *   在 `NodeStore` 中添加扩容逻辑（如 `_resize_buffer`）。
    *   当 `next_idx >= _cap` 时，触发扩容：解除当前 mmap 绑定，扩展底层文件大小（如翻倍），重新 mmap 并绑定新指针。
2.  **修改 `hier_aabb_tree.py`**：
    *   配合 `NodeStore` 的扩容，更新 Python 层的拓扑列表（`_left`, `_right`, `_parent` 等）的容量。
3.  **性能验证**：测试触发扩容时的开销，确保未触发扩容时的常规访问性能（~20ns/node）无任何下降。

### 阶段 3：实现 C - 邻接图构建并行化
1.  **重构 `_build_adjacency_and_islands`**：
    *   将双重循环的 chunk 计算逻辑提取为独立的 worker 函数。
    *   使用 `concurrent.futures.ThreadPoolExecutor`（如果 numpy 运算能释放 GIL）或 `ProcessPoolExecutor` 并发执行 chunk 块的 overlap 计算。
    *   合并各 worker 返回的局部邻接表和并查集（UnionFind）。
2.  **性能验证**：对比单线程与多线程/多进程在 2000, 5000, 10000 个 boxes 下的构建时间。确保小规模 forest 下并行开销不会导致性能倒退（可设置阈值，小规模回退单线程）。

### 阶段 4：实现 B - RRT Bridge 路径 Box 化
1.  **修改 `bridge_islands`**：
    *   当 RRT-Connect 成功找到 `rrt_fallback_path` 时，不再直接返回路径。
    *   遍历路径上的 waypoints，调用 `find_free_box` 生成新的 boxes。
    *   将这些新 boxes 添加到 `bridge_boxes_res` 中，并建立它们之间的邻接关系。
2.  **修改 `run_method_with_bridge`**：
    *   移除直接使用 `rrt_fallback_path` 绕过图搜索的逻辑。
    *   确保新生成的 bridge boxes 被正确注入到全局 `adj` 和 `boxes` 字典中，随后统一走 Dijkstra 图搜索。
3.  **性能验证**：测试二次查询（Reuse Query）的性能。确保第一次查询通过 RRT 桥接后，第二次相同或相近的查询能够直接在 forest 中以 $O(\text{Dijkstra})$ 的极快速度完成，而无需再次调用 RRT。

---

## 3. 性能保障机制 (Performance Guarantee)

1.  **微基准测试 (Micro-benchmarking)**：针对 Cython 层的修改（E项），必须使用 `timeit` 验证单次节点访问的纳秒级开销。
2.  **端到端测试 (End-to-End Testing)**：每次提交前，运行完整的 Panda 7-DOF 10-obs 场景测试，对比总规划时间、Forest 生长时间和路径质量。
3.  **A/B 对比脚本**：编写专门的脚本，同时加载 `v3` 和 `v4` 的模块，在相同随机种子下运行，断言输出结果的一致性并打印耗时差异。

---

## 4. 预期交付物

1.  完整的 `v4` 代码目录。
2.  支持动态扩容的 `_hier_core.pyx` 和 `hier_aabb_tree.py`。
3.  并行化的 `_build_adjacency_and_islands` 实现。
4.  具备 Paving 能力的 `bridge_islands` 逻辑。
5.  一份详细的性能对比报告（v3 vs v4），证明性能未受损且在特定场景下有显著提升。