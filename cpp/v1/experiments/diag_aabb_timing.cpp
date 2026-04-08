/**
 * diag_aabb_timing.cpp — 测量 interval FK 计算 AABB vs 缓存读取 AABB 耗时
 *
 * 用法: ./diag_aabb_timing [robot.json]
 */
#include "sbf/aabb/interval_fk.h"
#include "sbf/forest/collision.h"
#include "sbf/forest/node_store.h"
#include "sbf/core/robot.h"
#include "marcucci_scenes.h"
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace sbf;

int main(int argc, char** argv) {
    std::string robot_path = "configs/iiwa14.json";
    if (argc > 1 && argv[1][0] != '-') robot_path = argv[1];

    Robot robot = Robot::from_json(robot_path);
    auto obstacles = make_combined_obstacles();
    CollisionChecker checker(robot, obstacles);

    int n_dims = robot.n_joints();
    int n_active = robot.n_active_links();
    auto jl = robot.joint_limits();

    std::cout << "Robot: DOF=" << n_dims << "  active_links=" << n_active
              << "  obstacles=" << checker.n_obs() << "\n\n";

    // 生成 N 组随机小区间 (模拟典型 box: 全域 1% 宽)
    constexpr int N = 100000;
    std::mt19937 rng(42);
    std::vector<std::vector<Interval>> all_ivs(N);
    for (int i = 0; i < N; ++i) {
        all_ivs[i].resize(n_dims);
        for (int d = 0; d < n_dims; ++d) {
            double lo = jl.limits[d].lo;
            double hi = jl.limits[d].hi;
            double w = (hi - lo) * 0.01;
            std::uniform_real_distribution<> dist(lo, hi - w);
            double base = dist(rng);
            all_ivs[i][d] = {base, base + w};
        }
    }

    std::vector<float> aabb_buf(n_active * 6);
    volatile float sink = 0;
    volatile bool bsink = false;

    // ─── Benchmark 1: compute_link_aabbs (full interval FK) ──────────
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < N; ++i) {
        compute_link_aabbs(robot, all_ivs[i], aabb_buf.data());
        sink += aabb_buf[0];
    }
    auto t1 = std::chrono::steady_clock::now();
    double ns_full = std::chrono::duration<double, std::nano>(t1 - t0).count() / N;

    // ─── Benchmark 2: incremental FK (改变 dim 0) ────────────────────
    // 模拟 lazy split: parent full FK + child incremental FK + extract
    auto t2 = std::chrono::steady_clock::now();
    for (int i = 0; i < N; ++i) {
        FKState parent = compute_fk_full(robot, all_ivs[i]);
        auto ivs_copy = all_ivs[i];
        double mid = 0.5 * (ivs_copy[0].lo + ivs_copy[0].hi);
        ivs_copy[0].hi = mid;
        FKState child = compute_fk_incremental(parent, robot, ivs_copy, 0);
        extract_link_aabbs(child, robot.active_link_map(), n_active,
                           aabb_buf.data(), robot.active_link_radii());
        sink += aabb_buf[0];
    }
    auto t3 = std::chrono::steady_clock::now();
    double ns_incr = std::chrono::duration<double, std::nano>(t3 - t2).count() / N;

    // ─── Benchmark 3: incremental FK (只 incremental 部分, 不含 parent) ──
    {
        FKState parent = compute_fk_full(robot, all_ivs[0]);
        auto t4 = std::chrono::steady_clock::now();
        for (int i = 0; i < N; ++i) {
            auto ivs_copy = all_ivs[i];
            double mid = 0.5 * (ivs_copy[0].lo + ivs_copy[0].hi);
            ivs_copy[0].hi = mid;
            FKState child = compute_fk_incremental(parent, robot, ivs_copy, 0);
            extract_link_aabbs(child, robot.active_link_map(), n_active,
                               aabb_buf.data(), robot.active_link_radii());
            sink += aabb_buf[0];
        }
        auto t5 = std::chrono::steady_clock::now();
        double ns_incr_only = std::chrono::duration<double, std::nano>(t5 - t4).count() / N;
        std::cout << "  incremental FK only (no parent): " << std::fixed
                  << std::setprecision(0) << ns_incr_only << " ns\n";
    }

    // ─── Benchmark 4: 预填充 NodeStore, 测缓存读取 + SAT ────────────
    NodeStore store(n_active, n_dims, N);
    store.set_active_link_map(robot.active_link_map(), n_active);
    // 预计算所有 AABB 到 store
    for (int i = 0; i < N; ++i) {
        int idx = store.alloc_node(-1, 0);
        compute_link_aabbs(robot, all_ivs[i], store.aabb(idx));
        store.set_has_aabb(idx, true);
    }

    const float* obs_compact = checker.obs_compact();
    int n_obs = checker.n_obs();
    int n_slots = checker.n_aabb_slots();

    // 4a. Sequential 顺序访问
    auto t6 = std::chrono::steady_clock::now();
    for (int i = 0; i < N; ++i) {
        bsink = aabbs_collide_obs(store.aabb(i), n_slots, obs_compact, n_obs);
    }
    auto t7 = std::chrono::steady_clock::now();
    double ns_cache_seq = std::chrono::duration<double, std::nano>(t7 - t6).count() / N;

    // 4b. Random 随机访问 (模拟真实树遍历)
    std::vector<int> perm(N);
    std::iota(perm.begin(), perm.end(), 0);
    std::shuffle(perm.begin(), perm.end(), rng);
    auto t8 = std::chrono::steady_clock::now();
    for (int i = 0; i < N; ++i) {
        bsink = aabbs_collide_obs(store.aabb(perm[i]), n_slots, obs_compact, n_obs);
    }
    auto t9 = std::chrono::steady_clock::now();
    double ns_cache_rand = std::chrono::duration<double, std::nano>(t9 - t8).count() / N;

    // 4c. 纯 memcpy 读取 (不含碰撞检测)
    auto t10 = std::chrono::steady_clock::now();
    for (int i = 0; i < N; ++i) {
        std::memcpy(aabb_buf.data(), store.aabb(i), n_active * 6 * sizeof(float));
        sink += aabb_buf[0];
    }
    auto t11 = std::chrono::steady_clock::now();
    double ns_memcpy = std::chrono::duration<double, std::nano>(t11 - t10).count() / N;

    // ─── 输出汇总 ───────────────────────────────────────────────────
    std::cout << std::fixed << std::setprecision(0);
    std::cout << "\nAABB Timing Benchmark (N=" << N << ")\n";
    std::cout << "══════════════════════════════════════════════\n";
    std::cout << "  compute_link_aabbs (full FK):       " << ns_full << " ns\n";
    std::cout << "  parent full FK + child incr + extract: " << ns_incr << " ns\n";
    std::cout << "  cache read + SAT (sequential):      " << ns_cache_seq << " ns\n";
    std::cout << "  cache read + SAT (random access):   " << ns_cache_rand << " ns\n";
    std::cout << "  cache read only (memcpy):            " << ns_memcpy << " ns\n";
    std::cout << "══════════════════════════════════════════════\n";
    std::cout << std::setprecision(1);
    std::cout << "  full_FK / cache_seq ratio:           " << ns_full / ns_cache_seq << "×\n";
    std::cout << "  full_FK / cache_rand ratio:          " << ns_full / ns_cache_rand << "×\n";
    std::cout << "  incr_FK / cache_seq ratio:           " << ns_incr / ns_cache_seq << "×\n";

    // ─── 缓存大小分析 ───────────────────────────────────────────────
    int stride = node_layout::compute_stride(n_active);
    int aabb_bytes = n_active * 6 * 4;
    std::cout << "\nMemory analysis:\n";
    std::cout << "  NodeStore stride: " << stride << " bytes/node\n";
    std::cout << "  AABB data: " << aabb_bytes << " bytes/node (" << n_active << " links × 6 × float32)\n";
    std::cout << "  Topology overhead: " << node_layout::TOPO_SIZE << " bytes/node\n";
    std::cout << "  For N=100k tree nodes: " << (100000LL * stride / 1024 / 1024) << " MB\n";
    std::cout << "  For N=1M tree nodes: " << (1000000LL * stride / 1024 / 1024) << " MB\n";

    (void)sink;
    (void)bsink;
    return 0;
}
