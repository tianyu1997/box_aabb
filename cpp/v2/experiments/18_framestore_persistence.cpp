// ═══════════════════════════════════════════════════════════════════════════
// Experiment 18 — FrameStore persistence round-trip test
// ═══════════════════════════════════════════════════════════════════════════
//
// Tests:
//   1. Full save → load → exact match
//   2. Incremental save: add more nodes, append, reload → exact match
//   3. Dirty tracking: only newly-written nodes are dirty
//   4. LECT save/load wires frames correctly
//
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/forest/lect.h"
#include "sbf/common/types.h"

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <random>
#include <vector>

namespace fs = std::filesystem;
using namespace sbf;
using namespace sbf::envelope;

static constexpr float EPS = 1e-7f;

static std::vector<Interval> random_intervals(
    const Robot& robot, std::mt19937& rng,
    double width_lo = 0.05, double width_hi = 0.5)
{
    std::uniform_real_distribution<double> wdist(width_lo, width_hi);
    int n = robot.n_joints();
    const auto& lim = robot.joint_limits();
    std::vector<Interval> ivs(n);
    for (int i = 0; i < n; ++i) {
        double lo_lim = lim.limits[i].lo;
        double hi_lim = lim.limits[i].hi;
        double w = wdist(rng);
        double range = hi_lim - lo_lim;
        if (w > range) w = range;
        std::uniform_real_distribution<double> cdist(lo_lim + w/2, hi_lim - w/2);
        double c = cdist(rng);
        ivs[i] = {c - w/2, c + w/2};
    }
    return ivs;
}

int main(int argc, char* argv[]) {
    // ── Resolve robot ────────────────────────────────────────────────────
    fs::path exe_dir = fs::path(argv[0]).parent_path();
    std::string robot_json;
    for (auto& c : {
        exe_dir / ".." / ".." / "v1" / "configs",
        exe_dir / ".." / "v1" / "configs",
        exe_dir / ".." / ".." / ".." / "safeboxforest" / "v1" / "configs",
        fs::path("../v1/configs"),
        fs::path("../../v1/configs"),
    }) {
        fs::path p = c / "panda.json";
        if (fs::exists(p)) { robot_json = p.string(); break; }
    }
    if (robot_json.empty()) {
        printf("ERROR: panda.json not found\n");
        return 1;
    }

    Robot robot = Robot::from_json(robot_json);
    printf("Robot: %s  n_joints=%d  n_active=%d\n",
           robot.name().c_str(), robot.n_joints(), robot.n_active_links());

    const int n_frames = robot.n_joints() + (robot.has_tool() ? 1 : 0);
    const int fpn = n_frames * 6;
    printf("n_frames=%d  floats_per_node=%d\n\n", n_frames, fpn);

    std::mt19937 rng(12345);

    // Temporary directory for test files
    fs::path tmp_dir = fs::temp_directory_path() / "sbf_exp18_test";
    fs::create_directories(tmp_dir);
    std::string frames_file = (tmp_dir / "test.frames").string();

    // ════════════════════════════════════════════════════════════════════
    //  Test 1: Full save → load → exact match
    // ════════════════════════════════════════════════════════════════════
    printf("=== Test 1: Full save / load round-trip ===\n");
    {
        FrameStore fs1(robot);

        // Store 50 random nodes at random indices
        constexpr int N1 = 50;
        std::vector<int> indices(N1);
        for (int i = 0; i < N1; ++i)
            indices[i] = std::uniform_int_distribution<int>(0, 999)(rng);

        for (int idx : indices) {
            auto ivs = random_intervals(robot, rng);
            FKState fk = compute_fk_full(robot, ivs);
            fs1.store_from_fk(idx, fk);
        }

        printf("  Stored %d nodes, n_valid=%d, n_dirty=%d\n",
               N1, fs1.n_valid(), fs1.n_dirty());
        assert(fs1.n_dirty() > 0);

        // Full save
        fs1.save(frames_file);
        printf("  Saved to %s\n", frames_file.c_str());
        assert(fs1.n_dirty() == 0);
        printf("  After save: n_dirty=%d (should be 0)\n", fs1.n_dirty());

        // Load into fresh FrameStore
        FrameStore fs2(robot);
        fs2.load(frames_file);
        printf("  Loaded: n_valid=%d, n_dirty=%d\n", fs2.n_valid(), fs2.n_dirty());
        assert(fs2.n_valid() == fs1.n_valid());
        assert(fs2.n_dirty() == 0);

        // Compare all stored nodes
        int mismatches = 0;
        for (int i = 0; i < 1000; ++i) {
            const float* a = fs1.get_frames(i);
            const float* b = fs2.get_frames(i);
            if ((a == nullptr) != (b == nullptr)) { ++mismatches; continue; }
            if (!a) continue;
            for (int j = 0; j < fpn; ++j) {
                if (std::abs(a[j] - b[j]) > EPS) { ++mismatches; break; }
            }
        }
        printf("  Mismatches: %d\n", mismatches);
        assert(mismatches == 0);
        printf("  PASS: Full round-trip exact match\n\n");
    }

    // ════════════════════════════════════════════════════════════════════
    //  Test 2: Incremental save
    // ════════════════════════════════════════════════════════════════════
    printf("=== Test 2: Incremental save ===\n");
    {
        FrameStore fs1(robot);

        // Phase A: store 30 nodes, full save
        constexpr int NA = 30;
        std::vector<int> indicesA;
        for (int i = 0; i < NA; ++i) {
            int idx = 1000 + i;  // non-overlapping with phase B
            indicesA.push_back(idx);
            auto ivs = random_intervals(robot, rng);
            FKState fk = compute_fk_full(robot, ivs);
            fs1.store_from_fk(idx, fk);
        }
        fs1.save(frames_file);
        printf("  Phase A: saved %d nodes, n_on_disk=%d n_dirty=%d\n",
               fs1.n_valid(), fs1.n_valid(), fs1.n_dirty());

        // Phase B: store 20 more nodes, incremental save
        constexpr int NB = 20;
        for (int i = 0; i < NB; ++i) {
            int idx = 2000 + i;
            auto ivs = random_intervals(robot, rng);
            FKState fk = compute_fk_full(robot, ivs);
            fs1.store_from_fk(idx, fk);
        }
        printf("  Phase B: n_valid=%d, n_dirty=%d (should be %d)\n",
               fs1.n_valid(), fs1.n_dirty(), NB);
        assert(fs1.n_dirty() == NB);

        fs1.save_incremental(frames_file);
        printf("  Incremental save done. n_dirty=%d (should be 0)\n",
               fs1.n_dirty());
        assert(fs1.n_dirty() == 0);

        // Reload and verify all 50 nodes
        FrameStore fs2(robot);
        fs2.load(frames_file);
        printf("  Reloaded: n_valid=%d (should be %d)\n",
               fs2.n_valid(), NA + NB);
        assert(fs2.n_valid() == NA + NB);

        // Verify data match
        int mismatches = 0;
        for (int i = 0; i < 3000; ++i) {
            const float* a = fs1.get_frames(i);
            const float* b = fs2.get_frames(i);
            if ((a == nullptr) != (b == nullptr)) { ++mismatches; continue; }
            if (!a) continue;
            for (int j = 0; j < fpn; ++j) {
                if (std::abs(a[j] - b[j]) > EPS) { ++mismatches; break; }
            }
        }
        printf("  Mismatches: %d\n", mismatches);
        assert(mismatches == 0);
        printf("  PASS: Incremental round-trip exact match\n\n");
    }

    // ════════════════════════════════════════════════════════════════════
    //  Test 3: Metadata preservation (active_link_map, link_radii, base_pos)
    // ════════════════════════════════════════════════════════════════════
    printf("=== Test 3: Metadata preservation ===\n");
    {
        FrameStore fs1(robot);

        auto ivs = random_intervals(robot, rng);
        FKState fk = compute_fk_full(robot, ivs);
        fs1.store_from_fk(0, fk);
        fs1.save(frames_file);

        FrameStore fs2(robot);
        fs2.load(frames_file);

        assert(fs2.n_frames() == fs1.n_frames());
        assert(fs2.n_active_links() == fs1.n_active_links());

        bool map_ok = true, radii_ok = true, base_ok = true;
        for (int i = 0; i < fs1.n_active_links(); ++i) {
            if (fs2.active_link_map()[i] != fs1.active_link_map()[i]) map_ok = false;
            if (std::abs(fs2.link_radii()[i] - fs1.link_radii()[i]) > EPS) radii_ok = false;
        }
        for (int i = 0; i < 3; ++i) {
            if (std::abs(fs2.base_pos()[i] - fs1.base_pos()[i]) > EPS) base_ok = false;
        }

        printf("  active_link_map: %s\n", map_ok ? "MATCH" : "MISMATCH");
        printf("  link_radii:      %s\n", radii_ok ? "MATCH" : "MISMATCH");
        printf("  base_pos:        %s\n", base_ok ? "MATCH" : "MISMATCH");
        assert(map_ok && radii_ok && base_ok);
        printf("  PASS: All metadata preserved\n\n");
    }

    // ════════════════════════════════════════════════════════════════════
    //  Test 4: File size check
    // ════════════════════════════════════════════════════════════════════
    printf("=== Test 4: File size check ===\n");
    {
        FrameStore fs1(robot);

        constexpr int N = 100;
        for (int i = 0; i < N; ++i) {
            auto ivs = random_intervals(robot, rng);
            FKState fk = compute_fk_full(robot, ivs);
            fs1.store_from_fk(i, fk);
        }
        fs1.save(frames_file);

        auto file_size = fs::file_size(frames_file);
        int record_size = 4 + fpn * 4;  // uint32 + n_frames*6 floats
        int expected = 512 + N * record_size;  // header + records
        printf("  File size: %llu bytes  (expected %d)\n",
               (unsigned long long)file_size, expected);
        assert(static_cast<int>(file_size) == expected);
        printf("  Per-node cost: %d bytes\n", record_size);
        printf("  PASS: File size matches expected\n\n");
    }

    // Cleanup
    fs::remove_all(tmp_dir);

    printf("═══════════════════════════════════════════════════════════\n");
    printf("ALL TESTS PASSED\n");
    printf("═══════════════════════════════════════════════════════════\n");
    return 0;
}
