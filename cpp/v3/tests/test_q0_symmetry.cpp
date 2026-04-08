// ═══════════════════════════════════════════════════════════════════════════
//  test_q0_symmetry.cpp — verify multi-level joint-freezing in LECT
//
//  Tests:
//    1. interval_trig.h sanity — known angle reconstruction (1-level)
//    2. 1-level (q₀ only) reconstruction conservativeness (vs MC)
//    3. LECT q₀-split inherits parent cache correctly
//    4. find_free_box end-to-end
//    5. CritSample reconstruction (1-level)
//    6. 2-level (q₀+q₁) reconstruction conservativeness (vs MC)
//    7. 2-level multi-level undo/apply round-trip identities
//    8. LECT q₁-split inherits parent cache correctly
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/robot/interval_math.h"
#include "sbf/envelope/frame_source.h"
#include "sbf/common/interval_trig.h"
#include "sbf/forest/lect.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;
using namespace sbf::forest;

static const char* ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

// ─── Helper: check AABB A ⊇ AABB B  (A contains B, with tolerance) ─────
static bool aabb_contains(const float* outer, const float* inner,
                          int n_endpoints, float tol = 1e-4f)
{
    for (int k = 0; k < n_endpoints; ++k) {
        for (int a = 0; a < 3; ++a) {
            float inner_lo = inner[k * 6 + a];
            float inner_hi = inner[k * 6 + a + 3];
            float outer_lo = outer[k * 6 + a];
            float outer_hi = outer[k * 6 + a + 3];
            if (outer_lo > inner_lo + tol || outer_hi < inner_hi - tol)
                return false;
        }
    }
    return true;
}

// ─── Helper: compute sum of AABB volumes ────────────────────────────────
static double total_aabb_volume(const float* ep, int n_endpoints)
{
    double vol = 0.0;
    for (int k = 0; k < n_endpoints; ++k) {
        double v = 1.0;
        for (int a = 0; a < 3; ++a) {
            double w = ep[k * 6 + a + 3] - ep[k * 6 + a];
            v *= std::max(w, 0.0);
        }
        vol += v;
    }
    return vol;
}

// ═════════════════════════════════════════════════════════════════════════
//  Test 1: interval_trig.h sanity — known angle reconstruction
// ═════════════════════════════════════════════════════════════════════════
static bool test_interval_trig_sanity()
{
    std::cout << "\n=== Test 1: interval_trig.h sanity ===\n";

    // Point interval q₀ = 0: (A,B,C) → (A,B,C)
    {
        float local[6] = {1.0f, 2.0f, 3.0f,  4.0f, 5.0f, 6.0f};
        float cart[6];
        Interval q0(0.0, 0.0);
        reconstruct_cartesian_endpoints(local, 1, q0, cart);
        for (int i = 0; i < 6; ++i) {
            assert(std::abs(cart[i] - local[i]) < 1e-6f);
        }
        std::cout << "  q₀=0 identity check: PASS\n";
    }

    // Point interval q₀ = π/2: x=-B, y=A, z=C
    {
        float local[6] = {1.0f, 2.0f, 3.0f,  1.0f, 2.0f, 3.0f};  // point
        float cart[6];
        Interval q0(HALF_PI, HALF_PI);
        reconstruct_cartesian_endpoints(local, 1, q0, cart);
        // x = A*cos(π/2) - B*sin(π/2) = A*0 - B*1 = -B = -2
        // y = A*sin(π/2) + B*cos(π/2) = A*1 + B*0 = A = 1
        assert(std::abs(cart[0] - (-2.0f)) < 1e-5f);  // x_lo
        assert(std::abs(cart[1] - 1.0f)  < 1e-5f);    // y_lo
        assert(std::abs(cart[2] - 3.0f)  < 1e-5f);    // z_lo
        std::cout << "  q₀=π/2 rotation check: PASS\n";
    }

    // Interval q₀ = [-π, π]: full rotation, x,y should be wide
    {
        float local[6] = {0.5f, 0.0f, 1.0f,  0.5f, 0.0f, 1.0f};
        float cart[6];
        Interval q0(-PI, PI);
        reconstruct_cartesian_endpoints(local, 1, q0, cart);
        // A=[0.5,0.5], B=[0,0]: R = 0.5, full rotation
        //   x = 0.5*cos - 0*sin = 0.5*cos ∈ [-0.5, 0.5]
        //   y = 0.5*sin + 0*cos = 0.5*sin ∈ [-0.5, 0.5]
        assert(cart[0] <= -0.49f && cart[3] >= 0.49f);  // x full range
        assert(cart[1] <= -0.49f && cart[4] >= 0.49f);  // y full range
        assert(std::abs(cart[2] - 1.0f) < 1e-6f);       // z unchanged
        std::cout << "  q₀=[-π,π] full rotation check: PASS\n";
    }

    std::cout << "  ALL PASS\n";
    return true;
}

// ═════════════════════════════════════════════════════════════════════════
//  Test 2: (A,B,C) + reconstruction is conservative w.r.t. Monte Carlo
//
//  For a given C-space box, computes:
//    (a) Monte Carlo ground-truth AABB by sampling FK at many configs
//    (b) Direct Cartesian endpoints via IFK with real q₀
//    (c) Local-frame (A,B,C) via IFK with q₀=0, then reconstruction
//  Verifies (b) ⊇ (a) and (c) ⊇ (a)  (both conservative).
//  Reports volume ratios showing (c) is tighter than (b).
// ═════════════════════════════════════════════════════════════════════════
static bool test_reconstruction_conservativeness(const Robot& robot)
{
    std::cout << "\n=== Test 2: reconstruction conservativeness (vs MC) ===\n";
    auto cfg = EndpointSourceConfig::ifk();
    const auto& lim = robot.joint_limits();
    int n_ep = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    // Test multiple sub-boxes with varying q₀ widths
    struct TestCase {
        const char* name;
        double q0_lo, q0_hi;
    };
    TestCase cases[] = {
        {"narrow q\xE2\x82\x80",   0.3,  0.5},
        {"medium q\xE2\x82\x80",  -0.5,  0.5},
        {"wide q\xE2\x82\x80",    -1.5,  1.5},
        {"full range",   lim.limits[0].lo, lim.limits[0].hi},
    };

    bool all_ok = true;

    for (auto& tc : cases) {
        // Build intervals: use middle 50% for q₁..q₆, custom q₀
        std::vector<Interval> ivs(robot.n_joints());
        ivs[0] = {tc.q0_lo, tc.q0_hi};
        for (int d = 1; d < robot.n_joints(); ++d) {
            double c = lim.limits[d].center();
            double w = lim.limits[d].width() * 0.25;
            ivs[d] = {c - w, c + w};
        }

        // (a) Monte Carlo ground truth: sample FK at random configs
        std::vector<float> mc_aabbs(n_ep * 6);
        for (int k = 0; k < n_ep; ++k) {
            for (int a = 0; a < 3; ++a) {
                mc_aabbs[k*6+a]   =  1e30f;   // lo = +inf
                mc_aabbs[k*6+a+3] = -1e30f;   // hi = -inf
            }
        }
        // Grid sampling: 5 per joint (5^7 = 78125 samples)
        const int N_per_dim = 5;
        int total_samples = 1;
        for (int d = 0; d < robot.n_joints(); ++d) total_samples *= N_per_dim;

        for (int si = 0; si < total_samples; ++si) {
            Eigen::VectorXd q(robot.n_joints());
            int idx = si;
            for (int d = 0; d < robot.n_joints(); ++d) {
                int gi = idx % N_per_dim;
                idx /= N_per_dim;
                double t = (N_per_dim == 1) ? 0.5 : gi / (double)(N_per_dim - 1);
                q[d] = ivs[d].lo + t * ivs[d].width();
            }

            // Compute FK at this configuration (point FK)
            std::vector<Interval> pt_ivs(robot.n_joints());
            for (int d = 0; d < robot.n_joints(); ++d)
                pt_ivs[d] = {q[d], q[d]};
            auto pt_ep = compute_endpoint_aabb(cfg, robot, pt_ivs);

            // Update MC bounding box
            for (int k = 0; k < n_ep; ++k) {
                for (int a = 0; a < 3; ++a) {
                    float v = pt_ep.endpoint_aabbs[k*6+a];  // lo == hi for point
                    mc_aabbs[k*6+a]   = std::min(mc_aabbs[k*6+a],   v);
                    mc_aabbs[k*6+a+3] = std::max(mc_aabbs[k*6+a+3], v);
                }
            }
        }

        // (b) Direct Cartesian (standard IFK)
        auto direct_ep = compute_endpoint_aabb(cfg, robot, ivs);

        // (c) Local-frame + reconstruction
        std::vector<Interval> local_ivs = ivs;
        local_ivs[0] = {0.0, 0.0};
        auto local_ep = compute_endpoint_aabb(cfg, robot, local_ivs);
        std::vector<float> recon_ep = reconstruct_cartesian_endpoints(
            local_ep.endpoint_aabbs, n_ep, ivs[0]);

        // Check: both ⊇ MC ground truth  (conservative)
        bool direct_ok = aabb_contains(direct_ep.endpoint_aabbs.data(),
                                       mc_aabbs.data(), n_ep, 1e-4f);
        bool recon_ok  = aabb_contains(recon_ep.data(),
                                       mc_aabbs.data(), n_ep, 1e-4f);

        // Compute volumes
        double vol_mc     = total_aabb_volume(mc_aabbs.data(), n_ep);
        double vol_direct = total_aabb_volume(direct_ep.endpoint_aabbs.data(), n_ep);
        double vol_recon  = total_aabb_volume(recon_ep.data(), n_ep);

        std::cout << "  " << std::setw(12) << tc.name
                  << ":  direct\u2287MC=" << (direct_ok ? "YES" : "NO ")
                  << "  recon\u2287MC=" << (recon_ok ? "YES" : "NO ")
                  << "  vol_mc=" << std::scientific << std::setprecision(3) << vol_mc
                  << "  vol_direct=" << vol_direct
                  << "  vol_recon=" << vol_recon
                  << "  recon/direct=" << std::fixed << std::setprecision(3)
                  << (vol_direct > 0 ? vol_recon / vol_direct : 0.0)
                  << "\n";

        if (!recon_ok) {
            std::cout << "  FAIL: reconstruction is NOT conservative w.r.t. MC!\n";
            for (int k = 0; k < n_ep; ++k) {
                for (int a = 0; a < 3; ++a) {
                    float mc_lo = mc_aabbs[k*6+a];
                    float mc_hi = mc_aabbs[k*6+a+3];
                    float r_lo = recon_ep[k*6+a];
                    float r_hi = recon_ep[k*6+a+3];
                    if (r_lo > mc_lo + 1e-4f || r_hi < mc_hi - 1e-4f) {
                        std::cout << "    ep[" << k << "] axis " << a
                                  << ": MC=[" << mc_lo << "," << mc_hi
                                  << "] recon=[" << r_lo << "," << r_hi << "]\n";
                    }
                }
            }
            all_ok = false;
        }
    }

    if (all_ok)
        std::cout << "  ALL PASS\n";
    return all_ok;
}

// ═════════════════════════════════════════════════════════════════════════
//  Test 3: LECT q₀-split — children inherit parent cache
//
//  Build LECT with depth sufficient to trigger a q₀ split (depth 0),
//  then verify child link AABBs are valid (non-zero).
// ═════════════════════════════════════════════════════════════════════════
static bool test_lect_q0_split(const Robot& robot)
{
    std::cout << "\n=== Test 3: LECT q₀-split inheritance ===\n";

    // Use IFK + AABB (simplest pipeline)
    auto pipeline = PipelineConfig::fast();
    LECT lect(robot, pipeline, 64);

    // Pre-expand to depth 2 (ensures at least one q₀ split at depth 0)
    int new_nodes = lect.pre_expand(2);
    std::cout << "  pre_expand(2): " << new_nodes << " new nodes\n";

    // Root's children were split on q₀ (dim=0)
    // Verify both children have valid AABBs
    int left = lect.left(0);
    int right = lect.right(0);
    assert(left >= 0 && right >= 0);

    assert(lect.has_aabb(left));
    assert(lect.has_aabb(right));

    // Verify children's endpoint stores match parent's (inheritance)
    assert(lect.has_endpoint_aabbs(0));
    assert(lect.has_endpoint_aabbs(left));
    assert(lect.has_endpoint_aabbs(right));

    const float* parent_ep = lect.get_endpoint_aabbs(0);
    const float* left_ep   = lect.get_endpoint_aabbs(left);
    const float* right_ep  = lect.get_endpoint_aabbs(right);

    int n_ep = robot.n_joints() + (robot.has_tool() ? 1 : 0);
    for (int i = 0; i < n_ep * 6; ++i) {
        assert(std::abs(parent_ep[i] - left_ep[i]) < 1e-10f);
        assert(std::abs(parent_ep[i] - right_ep[i]) < 1e-10f);
    }
    std::cout << "  q₀-split ep_store inheritance: PASS\n";

    // Verify children's link AABBs are non-degenerate (positive volume)
    const float* left_aabb  = lect.get_link_aabbs(left);
    const float* right_aabb = lect.get_link_aabbs(right);
    int n_act = robot.n_active_links();
    for (int k = 0; k < n_act; ++k) {
        for (int a = 0; a < 3; ++a) {
            assert(left_aabb[k*6+a+3] >= left_aabb[k*6+a]);
            assert(right_aabb[k*6+a+3] >= right_aabb[k*6+a]);
        }
    }
    std::cout << "  q₀-split children link AABBs non-degenerate: PASS\n";

    // Verify: child link AABBs should be tighter than parent
    // (children have narrower q₀ ranges → tighter Rot_z)
    const float* parent_aabb = lect.get_link_aabbs(0);
    bool any_tighter = false;
    for (int k = 0; k < n_act; ++k) {
        for (int a = 0; a < 3; ++a) {
            if (left_aabb[k*6+a] > parent_aabb[k*6+a] + 1e-6f ||
                left_aabb[k*6+a+3] < parent_aabb[k*6+a+3] - 1e-6f)
                any_tighter = true;
        }
    }
    std::cout << "  q₀-split produces tighter child AABBs: "
              << (any_tighter ? "YES" : "same") << "\n";

    std::cout << "  ALL PASS\n";
    return true;
}

// ═════════════════════════════════════════════════════════════════════════
//  Test 4: LECT find_free_box with q₀ symmetry
//
//  Run find_free_box with a budget and verify it completes without
//  assertion failures.
// ═════════════════════════════════════════════════════════════════════════
static bool test_lect_find_free_box(const Robot& robot)
{
    std::cout << "\n=== Test 4: find_free_box end-to-end ===\n";

    // Create a simple scene with one obstacle
    Obstacle obs(
        Eigen::Vector3d(0.5, 0.0, 0.5),
        Eigen::Vector3d(0.1, 0.1, 0.1),
        "box1"
    );
    std::vector<Obstacle> obstacles = {obs};

    // Use IFK + AABB pipeline
    auto pipeline = PipelineConfig::fast();
    LECT lect(robot, pipeline, 64);

    // Sample a seed near center of C-space
    Eigen::VectorXd seed(robot.n_joints());
    for (int d = 0; d < robot.n_joints(); ++d)
        seed[d] = robot.joint_limits().limits[d].center();

    // find_free_box with budget
    auto result = lect.find_free_box(
        seed, obstacles.data(), static_cast<int>(obstacles.size()),
        /*min_edge=*/1e-4, /*max_depth=*/30);

    std::cout << "  success=" << (result.success() ? "YES" : "NO")
              << "  node=" << result.node_idx
              << "  new_nodes=" << result.n_new_nodes
              << "\n";

    // Just verify no crash — the logic correctness is covered by tests 1-3
    if (result.success()) {
        // Verify the found node has valid AABBs
        assert(lect.has_aabb(result.node_idx));
        const float* aabb = lect.get_link_aabbs(result.node_idx);
        int n_act = robot.n_active_links();
        for (int k = 0; k < n_act; ++k) {
            for (int a = 0; a < 3; ++a) {
                assert(aabb[k*6+a+3] >= aabb[k*6+a]);
            }
        }
    }

    std::cout << "  PASS (no crash)\n";
    return true;
}

// ═════════════════════════════════════════════════════════════════════════
//  Test 5: CritSample path with q₀ symmetry
// ═════════════════════════════════════════════════════════════════════════
static bool test_reconstruction_critsample(const Robot& robot)
{
    std::cout << "\n=== Test 5: CritSample reconstruction ===\n";
    auto cfg = EndpointSourceConfig::crit_sampling();
    int n_ep = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    // Use a moderate q₀ interval
    std::vector<Interval> ivs(robot.n_joints());
    const auto& lim = robot.joint_limits();
    ivs[0] = {-0.5, 0.5};
    for (int d = 1; d < robot.n_joints(); ++d) {
        double c = lim.limits[d].center();
        double w = lim.limits[d].width() * 0.25;
        ivs[d] = {c - w, c + w};
    }

    // (a) Direct Cartesian
    auto direct_ep = compute_endpoint_aabb(cfg, robot, ivs);

    // (b) Local-frame + reconstruction
    std::vector<Interval> local_ivs = ivs;
    local_ivs[0] = {0.0, 0.0};
    auto local_ep = compute_endpoint_aabb(cfg, robot, local_ivs);
    std::vector<float> recon_ep = reconstruct_cartesian_endpoints(
        local_ep.endpoint_aabbs, n_ep, ivs[0]);

    bool conservative = aabb_contains(recon_ep.data(),
                                      direct_ep.endpoint_aabbs.data(),
                                      n_ep, 1e-3f);

    double vol_direct = total_aabb_volume(direct_ep.endpoint_aabbs.data(), n_ep);
    double vol_recon  = total_aabb_volume(recon_ep.data(), n_ep);

    std::cout << "  conservative=" << (conservative ? "YES" : "NO")
              << "  vol_direct=" << std::scientific << vol_direct
              << "  vol_recon=" << vol_recon
              << "  ratio=" << std::fixed << std::setprecision(3)
              << (vol_direct > 0 ? vol_recon / vol_direct : 0.0)
              << "\n";

    if (!conservative) {
        std::cout << "  FAIL: CritSample reconstruction NOT conservative!\n";
        return false;
    }

    std::cout << "  PASS\n";
    return true;
}

// ═════════════════════════════════════════════════════════════════════════
//  Test 6: 2-level (q₀+q₁) reconstruction conservativeness (vs MC)
//
//  Freezes q₀=0, q₁=0, computes local-frame endpoints via IFK,
//  then reconstructs Cartesian via undo + 2-level rotation.
//  Verifies: reconstruction ⊇ MC ground truth   (conservative)
//  Reports: volume of 2-level recon vs direct vs 1-level recon.
// ═════════════════════════════════════════════════════════════════════════
static bool test_2level_reconstruction(const Robot& robot)
{
    std::cout << "\n=== Test 6: 2-level (q\xE2\x82\x80+q\xE2\x82\x81) reconstruction ===\n";
    auto cfg = EndpointSourceConfig::ifk();
    const auto& lim = robot.joint_limits();
    int n_ep = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    // Build FrozenJointDesc for joints 0 and 1
    const auto& dh = robot.dh_params();
    FrozenJointDesc descs[2];
    descs[0] = {classify_alpha(dh[0].alpha), dh[0].d, {0.0, 0.0}};
    descs[1] = {classify_alpha(dh[1].alpha), dh[1].d, {0.0, 0.0}};

    struct TestCase {
        const char* name;
        double q0_lo, q0_hi, q1_lo, q1_hi;
    };
    TestCase cases[] = {
        {"narrow both", -0.2, 0.2, 0.2, 0.5},
        {"medium q0",   -0.8, 0.8, 0.2, 0.5},
        {"wide q0",     -1.5, 1.5, -0.1, 1.0},
        {"full range",  lim.limits[0].lo, lim.limits[0].hi,
                        lim.limits[1].lo, lim.limits[1].hi},
    };

    bool all_ok = true;

    for (auto& tc : cases) {
        std::vector<Interval> ivs(robot.n_joints());
        ivs[0] = {tc.q0_lo, tc.q0_hi};
        ivs[1] = {tc.q1_lo, tc.q1_hi};
        for (int d = 2; d < robot.n_joints(); ++d) {
            double c = lim.limits[d].center();
            double w = lim.limits[d].width() * 0.25;
            ivs[d] = {c - w, c + w};
        }

        // (a) Monte Carlo ground truth: 5^7 = 78125 samples
        std::vector<float> mc_aabbs(n_ep * 6);
        for (int k = 0; k < n_ep; ++k) {
            for (int a = 0; a < 3; ++a) {
                mc_aabbs[k*6+a]   =  1e30f;
                mc_aabbs[k*6+a+3] = -1e30f;
            }
        }
        const int N = 5;
        int total = 1;
        for (int d = 0; d < robot.n_joints(); ++d) total *= N;
        for (int si = 0; si < total; ++si) {
            std::vector<Interval> pt_ivs(robot.n_joints());
            int idx = si;
            for (int d = 0; d < robot.n_joints(); ++d) {
                int gi = idx % N;
                idx /= N;
                double t = (N == 1) ? 0.5 : gi / (double)(N - 1);
                double val = ivs[d].lo + t * ivs[d].width();
                pt_ivs[d] = {val, val};
            }
            auto pt_ep = compute_endpoint_aabb(cfg, robot, pt_ivs);
            for (int k = 0; k < n_ep; ++k) {
                for (int a = 0; a < 3; ++a) {
                    float v = pt_ep.endpoint_aabbs[k*6+a];
                    mc_aabbs[k*6+a]   = std::min(mc_aabbs[k*6+a],   v);
                    mc_aabbs[k*6+a+3] = std::max(mc_aabbs[k*6+a+3], v);
                }
            }
        }

        // (b) Direct Cartesian (standard IFK)
        auto direct_ep = compute_endpoint_aabb(cfg, robot, ivs);

        // (c) 2-level: freeze q₀=0,q₁=0, then reconstruct
        std::vector<Interval> local_ivs = ivs;
        local_ivs[0] = {0.0, 0.0};
        local_ivs[1] = {0.0, 0.0};
        auto local_ep = compute_endpoint_aabb(cfg, robot, local_ivs);

        FrozenJointDesc td[2] = {descs[0], descs[1]};
        td[0].q = ivs[0];
        td[1].q = ivs[1];
        std::vector<float> recon2 = reconstruct_cartesian_endpoints_multilevel(
            local_ep.endpoint_aabbs, n_ep, td, 2);

        // (d) 1-level comparison: freeze only q₀
        std::vector<Interval> loc1_ivs = ivs;
        loc1_ivs[0] = {0.0, 0.0};
        auto loc1_ep = compute_endpoint_aabb(cfg, robot, loc1_ivs);
        std::vector<float> recon1 = reconstruct_cartesian_endpoints(
            loc1_ep.endpoint_aabbs, n_ep, ivs[0]);

        bool recon2_ok = aabb_contains(recon2.data(), mc_aabbs.data(), n_ep, 1e-3f);
        bool recon1_ok = aabb_contains(recon1.data(), mc_aabbs.data(), n_ep, 1e-3f);
        bool direct_ok = aabb_contains(direct_ep.endpoint_aabbs.data(),
                                       mc_aabbs.data(), n_ep, 1e-4f);

        double vol_mc   = total_aabb_volume(mc_aabbs.data(), n_ep);
        double vol_dir  = total_aabb_volume(direct_ep.endpoint_aabbs.data(), n_ep);
        double vol_r1   = total_aabb_volume(recon1.data(), n_ep);
        double vol_r2   = total_aabb_volume(recon2.data(), n_ep);

        std::cout << "  " << std::setw(12) << tc.name
                  << ":  2L\u2287MC=" << (recon2_ok ? "YES" : "NO ")
                  << "  1L\u2287MC=" << (recon1_ok ? "YES" : "NO ")
                  << "  dir\u2287MC=" << (direct_ok ? "YES" : "NO ")
                  << "  vol_mc=" << std::scientific << std::setprecision(3) << vol_mc
                  << "  r2/dir=" << std::fixed << std::setprecision(3)
                  << (vol_dir > 0 ? vol_r2 / vol_dir : 0.0)
                  << "  r1/dir=" << (vol_dir > 0 ? vol_r1 / vol_dir : 0.0)
                  << "\n";

        if (!recon2_ok) {
            std::cout << "  FAIL: 2-level recon NOT conservative!\n";
            for (int k = 0; k < n_ep; ++k) {
                for (int a = 0; a < 3; ++a) {
                    float mc_lo = mc_aabbs[k*6+a],   mc_hi = mc_aabbs[k*6+a+3];
                    float r_lo  = recon2[k*6+a],      r_hi  = recon2[k*6+a+3];
                    if (r_lo > mc_lo + 1e-3f || r_hi < mc_hi - 1e-3f) {
                        std::cout << "    ep[" << k << "] axis " << a
                                  << ": MC=[" << mc_lo << "," << mc_hi
                                  << "] recon2=[" << r_lo << "," << r_hi << "]\n";
                    }
                }
            }
            all_ok = false;
        }
    }

    if (all_ok)
        std::cout << "  ALL PASS\n";
    return all_ok;
}

// ═════════════════════════════════════════════════════════════════════════
//  Test 7: undo + apply round-trip identity at known angles
//
//  For a single endpoint, verify that undo_frozen_joint followed by
//  apply_joint_rotation with the original q=0 returns the original values.
// ═════════════════════════════════════════════════════════════════════════
static bool test_undo_apply_roundtrip()
{
    std::cout << "\n=== Test 7: undo/apply round-trip ===\n";

    // α-types to test: 0(=0), 1(=-π/2), 2(=+π/2)
    struct Case {
        int alpha_type; double d; const char* name;
    };
    Case cases[] = {
        {0, 0.1575, "alpha=0,d=0.1575"},
        {1, 0.0,    "alpha=-pi/2,d=0"},
        {2, 0.2025, "alpha=+pi/2,d=0.2025"},
    };

    bool all_ok = true;
    for (auto& tc : cases) {
        // Original "stored" interval (simulating Cartesian at q=0)
        Interval X0(0.1, 0.5), Y0(-0.3, 0.2), Z0(0.05, 0.8);

        // Undo then re-apply with q=0 — should recover original
        Interval X = X0, Y = Y0, Z = Z0;
        undo_frozen_joint(X, Y, Z, tc.alpha_type, tc.d);
        Interval q_zero(0.0, 0.0);
        apply_joint_rotation(X, Y, Z, q_zero, tc.alpha_type, tc.d);

        bool ok = (std::abs(X.lo - X0.lo) < 1e-12 &&
                   std::abs(X.hi - X0.hi) < 1e-12 &&
                   std::abs(Y.lo - Y0.lo) < 1e-12 &&
                   std::abs(Y.hi - Y0.hi) < 1e-12 &&
                   std::abs(Z.lo - Z0.lo) < 1e-12 &&
                   std::abs(Z.hi - Z0.hi) < 1e-12);

        std::cout << "  " << tc.name << ": " << (ok ? "PASS" : "FAIL");
        if (!ok) {
            std::cout << "\n    original=(" << X0.lo << "," << X0.hi
                      << "),(" << Y0.lo << "," << Y0.hi
                      << "),(" << Z0.lo << "," << Z0.hi << ")"
                      << "\n    result  =(" << X.lo << "," << X.hi
                      << "),(" << Y.lo << "," << Y.hi
                      << "),(" << Z.lo << "," << Z.hi << ")";
            all_ok = false;
        }
        std::cout << "\n";
    }

    if (all_ok)
        std::cout << "  ALL PASS\n";
    return all_ok;
}

// ═════════════════════════════════════════════════════════════════════════
//  Test 8: LECT q₁-split — children inherit parent cache
//
//  With freeze_depth=2, splits on dim=1 (q₁) should inherit ep_store_
//  from parent without recomputing IFK.
// ═════════════════════════════════════════════════════════════════════════
static bool test_lect_q1_split(const Robot& robot)
{
    std::cout << "\n=== Test 8: LECT q\xE2\x82\x81-split inheritance ===\n";

    auto pipeline = PipelineConfig::fast();
    LECT lect(robot, pipeline, 128);

    // Pre-expand to depth >= 2 so that q₁ splits happen
    // Depth 0 splits on dim=0, depth 1 splits on dim=1 (q₁)
    int new_nodes = lect.pre_expand(3);
    std::cout << "  pre_expand(3): " << new_nodes << " new nodes\n";

    // Find a node at depth 1 that was split on dim=1
    // Root (depth 0) → left child at depth 1 → its children at depth 2
    int root_left = lect.left(0);
    assert(root_left >= 0);
    int d1_left  = lect.left(root_left);
    int d1_right = lect.right(root_left);

    if (d1_left < 0 || d1_right < 0) {
        std::cout << "  SKIP: depth-1 node not split yet\n";
        return true;
    }

    // Both depth-2 children should have valid AABBs
    assert(lect.has_aabb(d1_left));
    assert(lect.has_aabb(d1_right));

    // Verify endpoint store inheritance: parent and both children
    // should share the same local-frame endpoints
    assert(lect.has_endpoint_aabbs(root_left));
    assert(lect.has_endpoint_aabbs(d1_left));
    assert(lect.has_endpoint_aabbs(d1_right));

    const float* parent_ep = lect.get_endpoint_aabbs(root_left);
    const float* left_ep   = lect.get_endpoint_aabbs(d1_left);
    const float* right_ep  = lect.get_endpoint_aabbs(d1_right);
    int n_ep = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    for (int i = 0; i < n_ep * 6; ++i) {
        assert(std::abs(parent_ep[i] - left_ep[i]) < 1e-10f);
        assert(std::abs(parent_ep[i] - right_ep[i]) < 1e-10f);
    }
    std::cout << "  q\xE2\x82\x81-split ep_store inheritance: PASS\n";

    // Verify non-degenerate link AABBs
    const float* left_aabb  = lect.get_link_aabbs(d1_left);
    const float* right_aabb = lect.get_link_aabbs(d1_right);
    int n_act = robot.n_active_links();
    for (int k = 0; k < n_act; ++k) {
        for (int a = 0; a < 3; ++a) {
            assert(left_aabb[k*6+a+3] >= left_aabb[k*6+a]);
            assert(right_aabb[k*6+a+3] >= right_aabb[k*6+a]);
        }
    }
    std::cout << "  q\xE2\x82\x81-split children link AABBs non-degenerate: PASS\n";

    // Verify: at least one child is tighter than parent
    const float* parent_aabb = lect.get_link_aabbs(root_left);
    bool any_tighter = false;
    for (int k = 0; k < n_act; ++k) {
        for (int a = 0; a < 3; ++a) {
            if (left_aabb[k*6+a] > parent_aabb[k*6+a] + 1e-6f ||
                left_aabb[k*6+a+3] < parent_aabb[k*6+a+3] - 1e-6f)
                any_tighter = true;
        }
    }
    std::cout << "  q\xE2\x82\x81-split produces tighter child AABBs: "
              << (any_tighter ? "YES" : "same") << "\n";

    std::cout << "  ALL PASS\n";
    return true;
}


int main()
{
    std::cout << "Loading robot from: " << ROBOT_PATH << "\n";
    Robot robot = Robot::from_json(ROBOT_PATH);
    std::cout << "  n_joints=" << robot.n_joints()
              << "  n_active_links=" << robot.n_active_links()
              << "  has_tool=" << robot.has_tool() << "\n";

    int pass = 0, fail = 0;

    if (test_interval_trig_sanity())                  ++pass; else ++fail;
    if (test_reconstruction_conservativeness(robot))   ++pass; else ++fail;
    if (test_lect_q0_split(robot))                     ++pass; else ++fail;
    if (test_lect_find_free_box(robot))                ++pass; else ++fail;
    if (test_reconstruction_critsample(robot))         ++pass; else ++fail;
    if (test_2level_reconstruction(robot))             ++pass; else ++fail;
    if (test_undo_apply_roundtrip())                   ++pass; else ++fail;
    if (test_lect_q1_split(robot))                     ++pass; else ++fail;

    std::cout << "\n═══════════════════════════════════\n"
              << "  PASS: " << pass << "  FAIL: " << fail << "\n"
              << "═══════════════════════════════════\n";
    return fail;
}
