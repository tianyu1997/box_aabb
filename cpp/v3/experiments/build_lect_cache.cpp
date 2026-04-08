// ═══════════════════════════════════════════════════════════════════════════
//  build_lect_cache.cpp — Offline LECT cache builder
//
//  Pre-expands a LECT KD-tree to a specified depth, computing FK +
//  link AABBs + hull grids for every node.  Saves the result to disk
//  for fast loading by ForestGrower (bypassing expensive FK + envelope
//  computation during root_select).
//
//  Usage:
//    build_lect_cache [depth] [output_dir]
//
//  Defaults:
//    depth      = 12   (4095 nodes, ~2-4 MB on disk)
//    output_dir = lect_cache
//
//  The cache is robot-specific but scene-independent.  Collision checking
//  is done at query time, so the same cache works for any obstacle layout.
//
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/forest/lect.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

using namespace sbf;
using namespace sbf::envelope;
using namespace sbf::forest;

static const char* ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

int main(int argc, char* argv[])
{
    int target_depth = 12;
    std::string output_dir = "lect_cache";

    if (argc >= 2) target_depth = std::atoi(argv[1]);
    if (argc >= 3) output_dir   = argv[2];

    std::printf("═══ LECT Cache Builder ═══\n");
    std::printf("  robot:       %s\n", ROBOT_PATH);
    std::printf("  target_depth: %d  (%d nodes max)\n",
                target_depth, (1 << (target_depth + 1)) - 1);
    std::printf("  output_dir:  %s\n", output_dir.c_str());

    // ── Load robot ──────────────────────────────────────────────────────
    Robot robot = Robot::from_json(ROBOT_PATH);
    std::printf("  n_joints=%d  n_active_links=%d\n",
                robot.n_joints(), robot.n_active_links());

    // ── Build LECT with recommended pipeline ────────────────────────────
    auto pipeline = PipelineConfig::recommended();  // CritSample + Hull16_Grid
    LECT lect(robot, pipeline);

    std::printf("\n  Pre-expanding to depth %d...\n", target_depth);
    auto t0 = std::chrono::high_resolution_clock::now();

    int n_new = lect.pre_expand(target_depth);

    auto t1 = std::chrono::high_resolution_clock::now();
    double build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::printf("  Done: %d new nodes, %d total nodes\n",
                n_new, lect.n_nodes());
    std::printf("  Build time (expand): %.1f ms (%.2f s)\n", build_ms, build_ms / 1000.0);

    // ── Compute hull grids for all nodes ────────────────────────────────
    std::printf("  Computing hull grids for all nodes...\n");
    auto t1b = std::chrono::high_resolution_clock::now();

    int n_hulls_added = lect.compute_all_hull_grids();

    auto t1c = std::chrono::high_resolution_clock::now();
    double hull_ms = std::chrono::duration<double, std::milli>(t1c - t1b).count();
    std::printf("  Hull grids computed: %d  (%.1f ms)\n", n_hulls_added, hull_ms);

    // Statistics
    int n_aabb = lect.count_nodes_with_aabb();
    int n_hull = lect.count_nodes_with_hull();
    int hull_voxels = lect.total_hull_voxels();
    std::printf("  Nodes with AABB: %d\n", n_aabb);
    std::printf("  Nodes with hull: %d\n", n_hull);
    std::printf("  Total hull voxels: %d\n", hull_voxels);

    // ── Save to disk ────────────────────────────────────────────────────
    namespace fs = std::filesystem;
    fs::create_directories(output_dir);

    std::printf("\n  Saving to %s...\n", output_dir.c_str());
    auto t2 = std::chrono::high_resolution_clock::now();

    lect.save(output_dir);

    auto t3 = std::chrono::high_resolution_clock::now();
    double save_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // Report file sizes
    uint64_t total_bytes = 0;
    for (auto& entry : fs::directory_iterator(output_dir)) {
        uint64_t sz = fs::file_size(entry.path());
        total_bytes += sz;
        std::printf("    %-20s %8.1f KB\n",
                    entry.path().filename().string().c_str(),
                    sz / 1024.0);
    }
    std::printf("    %-20s %8.1f KB\n", "TOTAL", total_bytes / 1024.0);
    std::printf("  Save time: %.1f ms\n", save_ms);

    // ── Verify: test load ───────────────────────────────────────────────
    std::printf("\n  Verifying load...\n");
    auto t4 = std::chrono::high_resolution_clock::now();

    LECT lect2(robot, pipeline);
    lect2.load(output_dir, robot);

    auto t5 = std::chrono::high_resolution_clock::now();
    double load_ms = std::chrono::duration<double, std::milli>(t5 - t4).count();

    std::printf("  Loaded: %d nodes, %d AABBs, %d hulls\n",
                lect2.n_nodes(),
                lect2.count_nodes_with_aabb(),
                lect2.count_nodes_with_hull());
    std::printf("  Load time: %.1f ms\n", load_ms);

    if (lect2.n_nodes() != lect.n_nodes()) {
        std::printf("  ERROR: node count mismatch! saved=%d loaded=%d\n",
                    lect.n_nodes(), lect2.n_nodes());
        return 1;
    }

    std::printf("\n═══ Cache build complete ═══\n");
    std::printf("  Build: %.2f s   Save: %.1f ms   Load: %.1f ms\n",
                build_ms / 1000.0, save_ms, load_ms);
    std::printf("  Speedup potential: root_select FK calls eliminated for\n"
                "  all %d cached internal nodes.\n", lect.n_nodes());

    return 0;
}
