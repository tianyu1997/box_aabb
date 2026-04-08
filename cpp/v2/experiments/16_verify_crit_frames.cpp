// ═══════════════════════════════════════════════════════════════════════════
// Experiment 16 — Verify derive_crit_frames() correctness and subdiv reuse
// ═══════════════════════════════════════════════════════════════════════════
//
// Tests that:
//   1. derive_crit_frames + derive_aabb == derive_aabb_critical (sub=1)
//   2. derive_crit_frames + derive_aabb_subdivided ⊇ derive_aabb_critical (sub>1)
//   3. Point cloud storage cost calculation
//
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_derive_critical.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <random>
#include <vector>

namespace fs = std::filesystem;
using namespace sbf;
using namespace sbf::envelope;

static std::vector<Interval> random_intervals(
    const Robot& robot, std::mt19937& rng,
    double width_lo, double width_hi)
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
        std::fprintf(stderr, "ERROR: Cannot find panda.json\n");
        return 1;
    }
    Robot robot = Robot::from_json(robot_json);
    int n_active = robot.n_active_links();
    int n_frames = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    printf("Robot: %s  (%d joints, %d active links, %d frames)\n",
           robot.name().c_str(), robot.n_joints(), n_active, n_frames);

    // ── Prepare FrameStore for crit frames ──────────────────────────────
    // We use FrameStore's active_link_map, link_radii, base_pos
    FrameStore fs_crit(robot, 16);

    const int N_TRIALS = 100;
    const int SUBS[] = {1, 2, 4, 8};
    constexpr int N_SUB = 4;

    std::mt19937 rng(42);

    int n_pass_sub1 = 0, n_fail_sub1 = 0;
    int n_superset_ok[N_SUB] = {};
    int n_superset_fail[N_SUB] = {};
    double max_gap[N_SUB] = {};  // tightest crit_frames - aabb_critical (should be >= 0)
    double max_margin[N_SUB] = {};  // max(crit_frames - aabb_critical) per sub

    // Track combos across trials
    long long total_combos = 0;

    // Buffers
    std::vector<float> crit_frames(n_frames * 6);
    int max_sub = 8;
    std::vector<float> aabb_from_crit_frames(n_active * max_sub * 6);
    std::vector<float> aabb_direct(n_active * max_sub * 6);

    printf("\n=== Test 1: derive_crit_frames + derive_aabb vs derive_aabb_critical ===\n");
    printf("Running %d trials x 3 widths x 4 subdivisions ...\n\n", N_TRIALS);

    const double WIDTHS[][2] = {{0.05, 0.15}, {0.25, 0.50}, {0.70, 1.40}};
    const char* WNAMES[] = {"small", "medium", "large"};

    for (int wi = 0; wi < 3; ++wi) {
        for (int t = 0; t < N_TRIALS; ++t) {
            auto ivs = random_intervals(robot, rng, WIDTHS[wi][0], WIDTHS[wi][1]);

            // Compute crit frames
            int n_combos = 0;
            derive_crit_frames(robot, ivs, crit_frames.data(), &n_combos);
            total_combos += n_combos;

            // Store into FrameStore slot 0
            fs_crit.store_frames(0, crit_frames.data());
            const float* frames = fs_crit.get_frames(0);

            for (int si = 0; si < N_SUB; ++si) {
                int sub_n = SUBS[si];

                // Method A: derive_crit_frames + derive_aabb[_subdivided]
                if (sub_n == 1) {
                    derive_aabb(frames, n_frames,
                        fs_crit.active_link_map(), n_active,
                        fs_crit.link_radii(), fs_crit.base_pos(),
                        aabb_from_crit_frames.data());
                } else {
                    for (int li = 0; li < n_active; ++li) {
                        int fidx = fs_crit.active_link_map()[li];
                        float rad = fs_crit.link_radii() ?
                            fs_crit.link_radii()[li] : 0.f;
                        derive_aabb_subdivided(frames, n_frames,
                            fidx - 1, fidx, sub_n, rad,
                            fs_crit.base_pos(),
                            aabb_from_crit_frames.data() + li * sub_n * 6);
                    }
                }

                // Method B: derive_aabb_critical (direct)
                derive_aabb_critical(robot, ivs, sub_n, aabb_direct.data());

                // Compare
                int total_slots = n_active * sub_n;
                bool exact_match = true;
                bool is_superset = true;  // crit_frames result >= direct (wider)
                double max_diff = 0;

                for (int s = 0; s < total_slots; ++s) {
                    const float* a = aabb_from_crit_frames.data() + s * 6;
                    const float* b = aabb_direct.data() + s * 6;
                    for (int k = 0; k < 3; ++k) {
                        float diff_lo = b[k] - a[k];       // direct_lo - crit_lo (should be >= 0 if crit is wider)
                        float diff_hi = a[k+3] - b[k+3];   // crit_hi - direct_hi (should be >= 0 if crit is wider)
                        if (diff_lo < -1e-5f || diff_hi < -1e-5f)
                            is_superset = false;
                        if (std::fabs(a[k] - b[k]) > 1e-5f || std::fabs(a[k+3] - b[k+3]) > 1e-5f)
                            exact_match = false;
                        max_diff = std::max(max_diff, (double)std::fabs(a[k] - b[k]));
                        max_diff = std::max(max_diff, (double)std::fabs(a[k+3] - b[k+3]));
                    }
                }

                if (sub_n == 1) {
                    if (exact_match) n_pass_sub1++;
                    else {
                        n_fail_sub1++;
                        if (n_fail_sub1 <= 3) {
                            printf("  FAIL sub=1 trial=%d width=%s max_diff=%.6f\n",
                                   t, WNAMES[wi], max_diff);
                            // Print first mismatched slot
                            for (int s = 0; s < total_slots && n_fail_sub1 <= 3; ++s) {
                                const float* a = aabb_from_crit_frames.data() + s * 6;
                                const float* b = aabb_direct.data() + s * 6;
                                printf("    slot %d: from_frames=[%.4f,%.4f,%.4f | %.4f,%.4f,%.4f]\n",
                                       s, a[0],a[1],a[2],a[3],a[4],a[5]);
                                printf("    slot %d: direct     =[%.4f,%.4f,%.4f | %.4f,%.4f,%.4f]\n",
                                       s, b[0],b[1],b[2],b[3],b[4],b[5]);
                            }
                        }
                    }
                }
                if (is_superset) n_superset_ok[si]++;
                else {
                    n_superset_fail[si]++;
                    if (n_superset_fail[si] <= 2) {
                        printf("  SUPERSET FAIL sub=%d trial=%d width=%s\n",
                               sub_n, t, WNAMES[wi]);
                    }
                }
                max_margin[si] = std::max(max_margin[si], max_diff);
            }
        }
    }

    int total_tests = N_TRIALS * 3;
    printf("\n── Results ──\n");
    printf("Sub=1 exact match: %d/%d (FAIL=%d)\n", n_pass_sub1, total_tests, n_fail_sub1);
    for (int si = 0; si < N_SUB; ++si) {
        printf("Sub=%d superset:    %d/%d (FAIL=%d, max_margin=%.6f)\n",
               SUBS[si], n_superset_ok[si], total_tests,
               n_superset_fail[si], max_margin[si]);
    }

    // ── Storage cost calculation ─────────────────────────────────────────
    double avg_combos = (double)total_combos / (N_TRIALS * 3);
    // Point cloud: per config, per frame, store one 3D point = 3 floats = 12 bytes
    // Actually we need proximal + distal per active link = 2 points
    // But crit_frames stores per-frame, and fk_link_positions returns n_joints+1+(has_tool) points
    // Per config: n_frames+1 positions (including base), but base is constant
    // Relevant: n_frames * 3 doubles = n_frames * 24 bytes per config (as double)
    // Or n_frames * 3 floats = n_frames * 12 bytes per config (as float)
    printf("\n── Storage Cost Estimates ──\n");
    printf("Average critical combos per node: %.0f\n", avg_combos);
    printf("n_frames = %d\n", n_frames);
    printf("\nOption A: FrameStore format (intervals)\n");
    printf("  Per node: %d frames x 6 floats x 4 bytes = %d bytes = %.1f KB\n",
           n_frames, n_frames * 6 * 4, n_frames * 6 * 4 / 1024.0);
    printf("\nOption B: Full point cloud (float32)\n");
    printf("  Per node: %.0f configs x %d frames x 3 floats x 4 bytes = %.0f bytes = %.1f KB\n",
           avg_combos, n_frames, avg_combos * n_frames * 3 * 4,
           avg_combos * n_frames * 3 * 4 / 1024.0);
    printf("\nOption C: Full point cloud (float64)\n");
    printf("  Per node: %.0f configs x %d frames x 3 doubles x 8 bytes = %.0f bytes = %.1f KB\n",
           avg_combos, n_frames, avg_combos * n_frames * 3 * 8,
           avg_combos * n_frames * 3 * 8 / 1024.0);
    printf("\nOption D: Per-active-link endpoint pairs (float32)\n");
    printf("  Per node: %.0f configs x %d active_links x 2 endpoints x 3 floats x 4 bytes = %.0f bytes = %.1f KB\n",
           avg_combos, n_active, avg_combos * n_active * 2 * 3 * 4,
           avg_combos * n_active * 2 * 3 * 4 / 1024.0);

    // Typical tree has ~1000-10000 nodes
    printf("\n── Scaling (for 10000-node tree) ──\n");
    double nodes = 10000;
    printf("  Option A (intervals):          %.1f MB\n",
           nodes * n_frames * 6 * 4 / 1e6);
    printf("  Option B (point cloud float):  %.1f MB\n",
           nodes * avg_combos * n_frames * 3 * 4 / 1e6);
    printf("  Option D (link endpoints):     %.1f MB\n",
           nodes * avg_combos * n_active * 2 * 3 * 4 / 1e6);

    bool all_pass = (n_fail_sub1 == 0);
    for (int si = 0; si < N_SUB; ++si)
        if (n_superset_fail[si] > 0) all_pass = false;

    printf("\n%s\n", all_pass ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    return all_pass ? 0 : 1;
}
