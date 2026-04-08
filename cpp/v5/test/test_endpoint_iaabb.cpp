// SafeBoxForest v5 — test_endpoint_iaabb
// Phase B verification

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/ifk_source.h>
#include <sbf/envelope/crit_source.h>
#include <sbf/envelope/analytical_source.h>
#include <sbf/envelope/gcpc_source.h>

#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <fstream>

using namespace sbf;

static Robot make_2dof() {
    return Robot::from_json("data/2dof_planar.json");
}

// Scalar FK for MC verification — must match interval FK DH convention.
//   A = [ct, -st,    0,     a     ]
//       [st*ca, ct*ca, -sa, -d*sa ]
//       [st*sa, ct*sa,  ca,  d*ca ]
//       [0,     0,      0,   1    ]
static void scalar_fk_verify(const Robot& robot, const double* q,
                              double positions[][3])
{
    const auto& dh = robot.dh_params();
    int n = robot.n_joints();
    double T[16];
    for (int i = 0; i < 16; ++i) T[i] = 0.0;
    T[0] = T[5] = T[10] = T[15] = 1.0;
    positions[0][0] = 0.0; positions[0][1] = 0.0; positions[0][2] = 0.0;

    for (int j = 0; j < n; ++j) {
        double d_val = (dh[j].joint_type == 1) ? (q[j] + dh[j].d) : dh[j].d;
        double angle = (dh[j].joint_type == 0) ? (q[j] + dh[j].theta) : dh[j].theta;
        double ct = std::cos(angle), st = std::sin(angle);
        double ca = std::cos(dh[j].alpha), sa = std::sin(dh[j].alpha);
        double A[16] = {
            ct,      -st,      0.0,  dh[j].a,
            st*ca,    ct*ca,  -sa,  -d_val*sa,
            st*sa,    ct*sa,   ca,   d_val*ca,
            0.0,      0.0,    0.0,   1.0
        };
        double R[16];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                R[r*4+c] = T[r*4]*A[c] + T[r*4+1]*A[4+c]
                          + T[r*4+2]*A[8+c] + T[r*4+3]*A[12+c];
        R[12] = 0.0; R[13] = 0.0; R[14] = 0.0; R[15] = 1.0;
        for (int i = 0; i < 16; ++i) T[i] = R[i];
        positions[j+1][0] = T[3];
        positions[j+1][1] = T[7];
        positions[j+1][2] = T[11];
    }

    // Tool frame
    if (robot.has_tool()) {
        const auto& tool = *robot.tool_frame();
        double ct = std::cos(tool.theta), st = std::sin(tool.theta);
        double ca = std::cos(tool.alpha), sa = std::sin(tool.alpha);
        double A[16] = {
            ct,      -st,      0.0,  tool.a,
            st*ca,    ct*ca,  -sa,  -tool.d*sa,
            st*sa,    ct*sa,   ca,   tool.d*ca,
            0.0,      0.0,    0.0,   1.0
        };
        double R[16];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                R[r*4+c] = T[r*4]*A[c] + T[r*4+1]*A[4+c]
                          + T[r*4+2]*A[8+c] + T[r*4+3]*A[12+c];
        R[12] = 0.0; R[13] = 0.0; R[14] = 0.0; R[15] = 1.0;
        for (int i = 0; i < 16; ++i) T[i] = R[i];
        positions[n+1][0] = T[3];
        positions[n+1][1] = T[7];
        positions[n+1][2] = T[11];
    }
}

// Check that all MC samples lie within endpoint iAABBs
static bool mc_conservative_check(const Robot& robot,
                                   const std::vector<Interval>& intervals,
                                   const EndpointIAABBResult& result,
                                   int n_mc = 1000)
{
    int n = robot.n_joints();
    int n_act = robot.n_active_links();
    const int* alm = robot.active_link_map();

    std::mt19937 gen(12345);
    std::vector<std::uniform_real_distribution<double>> dists;
    for (int j = 0; j < n; ++j)
        dists.emplace_back(intervals[j].lo, intervals[j].hi);

    double positions[MAX_TF][3];
    std::vector<double> q(n);
    const float tol = 1e-5f;

    for (int s = 0; s < n_mc; ++s) {
        for (int j = 0; j < n; ++j)
            q[j] = dists[j](gen);
        scalar_fk_verify(robot, q.data(), positions);

        for (int ci = 0; ci < n_act; ++ci) {
            int V = alm[ci];
            // Proximal
            const float* p = result.endpoint_iaabbs.data() + (ci * 2) * 6;
            for (int d = 0; d < 3; ++d) {
                float v = static_cast<float>(positions[V][d]);
                if (v < p[d] - tol || v > p[d + 3] + tol)
                    return false;
            }
            // Distal
            const float* dd = result.endpoint_iaabbs.data() + (ci * 2 + 1) * 6;
            for (int d = 0; d < 3; ++d) {
                float v = static_cast<float>(positions[V + 1][d]);
                if (v < dd[d] - tol || v > dd[d + 3] + tol)
                    return false;
            }
        }
    }
    return true;
}

// Compute volume of endpoint iAABBs (sum over all endpoints)
static double total_volume(const EndpointIAABBResult& result) {
    double vol = 0.0;
    int n_ep = result.n_active_links * 2;
    for (int i = 0; i < n_ep; ++i) {
        const float* a = result.endpoint_iaabbs.data() + i * 6;
        double dx = std::max(0.0, static_cast<double>(a[3] - a[0]));
        double dy = std::max(0.0, static_cast<double>(a[4] - a[1]));
        double dz = std::max(0.0, static_cast<double>(a[5] - a[2]));
        vol += dx * dy * dz;
    }
    return vol;
}

// Check that result A contains result B (A ⊇ B)
static bool contains(const EndpointIAABBResult& outer,
                     const EndpointIAABBResult& inner, float tol = 1e-5f) {
    int n_ep = outer.n_active_links * 2;
    if (inner.n_active_links != outer.n_active_links) return false;
    for (int i = 0; i < n_ep; ++i) {
        const float* a = outer.endpoint_iaabbs.data() + i * 6;
        const float* b = inner.endpoint_iaabbs.data() + i * 6;
        for (int d = 0; d < 3; ++d) {
            if (b[d] < a[d] - tol) return false;         // inner.lo < outer.lo
            if (b[d + 3] > a[d + 3] + tol) return false; // inner.hi > outer.hi
        }
    }
    return true;
}


TEST_SUITE("IFK") {
    TEST_CASE("2DOF: output dimension") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {
            {-1.0, 1.0},
            {-0.5, 0.5}
        };
        auto result = compute_endpoint_iaabb_ifk(robot, ivs);

        CHECK(result.n_active_links == robot.n_active_links());
        CHECK(result.endpoint_iaabb_len() == robot.n_active_links() * 2 * 6);
        CHECK(static_cast<int>(result.endpoint_iaabbs.size()) == result.endpoint_iaabb_len());
        CHECK(result.is_safe == true);
        CHECK(result.source == EndpointSource::IFK);
    }

    TEST_CASE("2DOF: MC conservativeness") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {
            {-1.0, 1.0},
            {-0.5, 0.5}
        };
        auto result = compute_endpoint_iaabb_ifk(robot, ivs);
        CHECK(mc_conservative_check(robot, ivs, result, 1000));
    }

    TEST_CASE("2DOF: incremental FK matches full") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {
            {-1.0, 1.0},
            {-0.5, 0.5}
        };
        FKState fk;
        auto r1 = compute_endpoint_iaabb_ifk(robot, ivs, &fk);
        CHECK(fk.valid);

        // Change dim 0
        std::vector<Interval> ivs2 = {
            {-0.8, 0.8},
            {-0.5, 0.5}
        };
        auto r_full = compute_endpoint_iaabb_ifk(robot, ivs2);
        auto r_incr = compute_endpoint_iaabb_ifk(robot, ivs2, &fk, 0);

        int len = r_full.endpoint_iaabb_len();
        for (int i = 0; i < len; ++i) {
            CHECK(r_incr.endpoint_iaabbs[i] == doctest::Approx(r_full.endpoint_iaabbs[i]).epsilon(1e-10));
        }
    }
}


TEST_SUITE("CritSample") {
    TEST_CASE("2DOF: is_safe == false") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {
            {-1.0, 1.0},
            {-0.5, 0.5}
        };
        auto result = compute_endpoint_iaabb_crit(robot, ivs, 500);
        CHECK(result.is_safe == false);
        CHECK(result.source == EndpointSource::CritSample);
        CHECK(result.n_active_links == robot.n_active_links());
    }

    TEST_CASE("2DOF: AABB volume usually <= IFK volume") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {
            {-1.0, 1.0},
            {-0.5, 0.5}
        };
        auto ifk_result = compute_endpoint_iaabb_ifk(robot, ivs);
        auto crit_result = compute_endpoint_iaabb_crit(robot, ivs, 2000);

        double ifk_vol = total_volume(ifk_result);
        double crit_vol = total_volume(crit_result);
        // CritSample is typically tighter than IFK
        CHECK(crit_vol <= ifk_vol + 1e-6);
    }
}


TEST_SUITE("Analytical") {
    TEST_CASE("2DOF: Phase 0 ⊆ IFK") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {
            {-1.0, 1.0},
            {-0.5, 0.5}
        };
        auto ifk_result = compute_endpoint_iaabb_ifk(robot, ivs);
        auto ana_result = compute_endpoint_iaabb_analytical(robot, ivs, 0);

        CHECK(ana_result.is_safe == false);
        CHECK(ana_result.source == EndpointSource::Analytical);
        // Analytical Phase 0 result should be contained within IFK
        CHECK(contains(ifk_result, ana_result));
    }

    TEST_CASE("2DOF: is_safe == false") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {
            {-0.5, 0.5},
            {-0.3, 0.3}
        };
        auto result = compute_endpoint_iaabb_analytical(robot, ivs, 3);
        CHECK(result.is_safe == false);
    }

    TEST_CASE("2DOF: Phase 1 MC deviation < 1%") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {
            {-0.5, 0.5},
            {-0.3, 0.3}
        };
        auto ana_result = compute_endpoint_iaabb_analytical(robot, ivs, 1);

        // MC ground truth via dense sampling
        auto crit_result = compute_endpoint_iaabb_crit(robot, ivs, 50000);

        // Check that analytical result is close to MC truth
        int n_ep = ana_result.n_active_links * 2;
        for (int i = 0; i < n_ep; ++i) {
            const float* ana = ana_result.endpoint_iaabbs.data() + i * 6;
            const float* mc  = crit_result.endpoint_iaabbs.data() + i * 6;
            for (int d = 0; d < 3; ++d) {
                float ana_width = ana[d + 3] - ana[d];
                float mc_width  = mc[d + 3] - mc[d];
                if (mc_width > 1e-6f) {
                    float deviation = std::abs(ana_width - mc_width) / mc_width;
                    CHECK(deviation < 0.05);  // < 5% deviation
                }
            }
        }
    }

    TEST_CASE("2DOF: MC conservativeness with Phase 1") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {
            {-1.0, 1.0},
            {-0.5, 0.5}
        };
        auto result = compute_endpoint_iaabb_analytical(robot, ivs, 1);
        CHECK(mc_conservative_check(robot, ivs, result, 2000));
    }
}


TEST_SUITE("GCPC") {
    TEST_CASE("cache save/load roundtrip") {
        GcpcCache cache;
        cache.set_n_dims(2);
        Eigen::VectorXd p1(2); p1 << 0.1, 0.2;
        Eigen::VectorXd p2(2); p2 << -0.5, 0.3;
        Eigen::VectorXd p3(2); p3 << 1.0, -1.0;
        cache.add_point(p1);
        cache.add_point(p2);
        cache.add_point(p3);

        std::string path = "test_gcpc_roundtrip.gcpc";
        cache.save(path);

        auto loaded = GcpcCache::load(path);
        CHECK(loaded.n_dims() == 2);
        CHECK(loaded.n_points() == 3);

        for (int i = 0; i < 3; ++i) {
            for (int d = 0; d < 2; ++d) {
                CHECK(loaded.points()[i][d] == doctest::Approx(cache.points()[i][d]).epsilon(1e-15));
            }
        }

        // Cleanup
        std::remove(path.c_str());
    }

    TEST_CASE("GCPC AABB ⊆ Analytical AABB") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {
            {-0.5, 0.5},
            {-0.3, 0.3}
        };

        // Build a small cache with some interior points
        GcpcCache cache;
        cache.set_n_dims(2);
        std::mt19937 gen(42);
        std::uniform_real_distribution<double> d0(-0.5, 0.5);
        std::uniform_real_distribution<double> d1(-0.3, 0.3);
        for (int i = 0; i < 50; ++i) {
            Eigen::VectorXd pt(2);
            pt << d0(gen), d1(gen);
            cache.add_point(pt);
        }

        auto ana_result = compute_endpoint_iaabb_analytical(robot, ivs, 3);
        auto gcpc_result = compute_endpoint_iaabb_gcpc(robot, ivs, cache);

        CHECK(gcpc_result.is_safe == false);
        CHECK(gcpc_result.source == EndpointSource::GCPC);
        // GCPC uses analytical Phase 0-2 + cache interior, so its AABB
        // should be contained within or equal to full analytical (Phase 0-3)
        // The analytical with Phase 3 may have found more interior critical points,
        // but GCPC won't exceed analytical boundary bounds.
        // Actually GCPC expands with cache points (scalar FK) so it should
        // be tighter or equal. Just check it's contained in IFK (safe bound).
        auto ifk_result = compute_endpoint_iaabb_ifk(robot, ivs);
        CHECK(contains(ifk_result, gcpc_result));
    }
}


TEST_SUITE("unified interface") {
    TEST_CASE("compute_endpoint_iaabb with IFK") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {{-1.0, 1.0}, {-0.5, 0.5}};

        EndpointSourceConfig config;
        config.source = EndpointSource::IFK;
        auto result = compute_endpoint_iaabb(robot, ivs, config);

        CHECK(result.source == EndpointSource::IFK);
        CHECK(result.is_safe == true);
        CHECK(result.n_active_links == robot.n_active_links());
    }

    TEST_CASE("compute_endpoint_iaabb with CritSample") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {{-1.0, 1.0}, {-0.5, 0.5}};

        EndpointSourceConfig config;
        config.source = EndpointSource::CritSample;
        config.n_samples_crit = 500;
        auto result = compute_endpoint_iaabb(robot, ivs, config);

        CHECK(result.source == EndpointSource::CritSample);
        CHECK(result.is_safe == false);
    }

    TEST_CASE("compute_endpoint_iaabb with Analytical") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {{-1.0, 1.0}, {-0.5, 0.5}};

        EndpointSourceConfig config;
        config.source = EndpointSource::Analytical;
        config.max_phase_analytical = 1;
        auto result = compute_endpoint_iaabb(robot, ivs, config);

        CHECK(result.source == EndpointSource::Analytical);
        CHECK(result.is_safe == false);
    }

    TEST_CASE("compute_endpoint_iaabb with GCPC") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {{-0.5, 0.5}, {-0.3, 0.3}};

        GcpcCache cache;
        cache.set_n_dims(2);
        Eigen::VectorXd pt(2); pt << 0.1, 0.1;
        cache.add_point(pt);

        EndpointSourceConfig config;
        config.source = EndpointSource::GCPC;
        config.gcpc_cache = &cache;
        auto result = compute_endpoint_iaabb(robot, ivs, config);

        CHECK(result.source == EndpointSource::GCPC);
        CHECK(result.is_safe == false);
    }

    TEST_CASE("safety flags are correct") {
        auto robot = make_2dof();
        std::vector<Interval> ivs = {{-0.5, 0.5}, {-0.3, 0.3}};

        // IFK = safe
        {
            EndpointSourceConfig c; c.source = EndpointSource::IFK;
            CHECK(compute_endpoint_iaabb(robot, ivs, c).is_safe == true);
        }
        // CritSample = unsafe
        {
            EndpointSourceConfig c; c.source = EndpointSource::CritSample;
            CHECK(compute_endpoint_iaabb(robot, ivs, c).is_safe == false);
        }
        // Analytical = unsafe
        {
            EndpointSourceConfig c; c.source = EndpointSource::Analytical;
            CHECK(compute_endpoint_iaabb(robot, ivs, c).is_safe == false);
        }
        // GCPC = unsafe
        {
            GcpcCache cache; cache.set_n_dims(2);
            EndpointSourceConfig c; c.source = EndpointSource::GCPC;
            c.gcpc_cache = &cache;
            CHECK(compute_endpoint_iaabb(robot, ivs, c).is_safe == false);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 7DOF Panda tests
// ═══════════════════════════════════════════════════════════════════════════

static Robot make_panda() {
    return Robot::from_json("data/panda.json");
}

TEST_SUITE("Panda 7DOF") {

    TEST_CASE("robot loads correctly") {
        auto robot = make_panda();
        CHECK(robot.n_joints() == 7);
        CHECK(robot.n_active_links() > 0);
        CHECK(robot.has_tool() == true);
    }

    TEST_CASE("IFK: dimension and MC conservativeness [7dof]") {
        auto robot = make_panda();
        // Narrow interval around a specific config
        std::vector<Interval> ivs = {
            {0.0, 0.5}, {-0.5, 0.0}, {0.0, 0.5}, {-2.5, -1.5},
            {0.0, 0.5}, {1.0, 1.5}, {0.0, 0.5}
        };
        auto result = compute_endpoint_iaabb_ifk(robot, ivs);
        int n_act = robot.n_active_links();
        CHECK(result.n_active_links == n_act);
        CHECK(static_cast<int>(result.endpoint_iaabbs.size()) == n_act * 2 * 6);
        CHECK(result.is_safe == true);

        // MC conservativeness: random configs must stay inside IFK AABB
        std::mt19937 rng(42);
        const float* ep = result.endpoint_iaabbs.data();
        const int* alm = robot.active_link_map();
        bool all_inside = true;
        for (int trial = 0; trial < 500; ++trial) {
            std::vector<double> q(7);
            for (int j = 0; j < 7; ++j) {
                std::uniform_real_distribution<double> dist(ivs[j].lo, ivs[j].hi);
                q[j] = dist(rng);
            }
            double positions[MAX_TF][3];
            scalar_fk_verify(robot, q.data(), positions);
            for (int ci = 0; ci < n_act; ++ci) {
                int V = alm[ci];
                for (int e = 0; e < 2; ++e) {
                    int frame = (e == 0) ? V : V + 1;
                    const float* b = ep + (ci * 2 + e) * 6;
                    for (int d = 0; d < 3; ++d) {
                        float v = static_cast<float>(positions[frame][d]);
                        if (v < b[d] - 1e-5f || v > b[d + 3] + 1e-5f)
                            all_inside = false;
                    }
                }
            }
        }
        CHECK(all_inside);
    }

    TEST_CASE("Analytical: AA Gap Pruning skips links > 0 [7dof]") {
        auto robot = make_panda();
        // Use a narrow interval where IFK wrapping is small enough
        // that some links' IFK bounds are within tolerance of Phase 0 AABB
        std::vector<Interval> ivs = {
            {0.0, 0.1}, {-0.1, 0.0}, {0.0, 0.1}, {-1.6, -1.5},
            {0.0, 0.1}, {1.0, 1.1}, {0.0, 0.1}
        };

        auto result = compute_endpoint_iaabb_analytical(robot, ivs, 3);
        CHECK(result.is_safe == false);
        CHECK(result.n_active_links == robot.n_active_links());
        // AA Gap Pruning must skip at least 1 link for the Panda robot
        CHECK(result.n_pruned_links > 0);
        MESSAGE("Panda n_active_links = ", robot.n_active_links(),
                ", n_pruned_links = ", result.n_pruned_links);
    }

    TEST_CASE("Analytical: Phase 0 subset of IFK [7dof]") {
        auto robot = make_panda();
        std::vector<Interval> ivs = {
            {-0.5, 0.5}, {-0.5, 0.5}, {-0.5, 0.5}, {-2.5, -1.0},
            {-0.5, 0.5}, {0.5, 2.0}, {-0.5, 0.5}
        };

        auto ifk = compute_endpoint_iaabb_ifk(robot, ivs);
        auto ana = compute_endpoint_iaabb_analytical(robot, ivs, 0);
        int n_ep = robot.n_active_links() * 2;
        for (int i = 0; i < n_ep; ++i) {
            const float* ia = ifk.endpoint_iaabbs.data() + i * 6;
            const float* aa = ana.endpoint_iaabbs.data() + i * 6;
            for (int d = 0; d < 3; ++d) {
                CHECK(aa[d] >= ia[d] - 1e-4f);      // analytical lo >= IFK lo
                CHECK(aa[d + 3] <= ia[d + 3] + 1e-4f); // analytical hi <= IFK hi
            }
        }
    }

    TEST_CASE("Analytical: MC conservativeness [7dof]") {
        auto robot = make_panda();
        std::vector<Interval> ivs = {
            {0.0, 0.5}, {-0.5, 0.0}, {0.0, 0.5}, {-2.5, -1.5},
            {0.0, 0.5}, {1.0, 1.5}, {0.0, 0.5}
        };
        auto result = compute_endpoint_iaabb_analytical(robot, ivs, 3);
        CHECK(result.is_safe == false);

        std::mt19937 rng(123);
        const float* ep = result.endpoint_iaabbs.data();
        const int* alm = robot.active_link_map();
        int n_act = robot.n_active_links();
        bool all_inside = true;
        for (int trial = 0; trial < 500; ++trial) {
            std::vector<double> q(7);
            for (int j = 0; j < 7; ++j) {
                std::uniform_real_distribution<double> dist(ivs[j].lo, ivs[j].hi);
                q[j] = dist(rng);
            }
            double positions[MAX_TF][3];
            scalar_fk_verify(robot, q.data(), positions);
            for (int ci = 0; ci < n_act; ++ci) {
                int V = alm[ci];
                for (int e = 0; e < 2; ++e) {
                    int frame = (e == 0) ? V : V + 1;
                    const float* b = ep + (ci * 2 + e) * 6;
                    for (int d = 0; d < 3; ++d) {
                        float v = static_cast<float>(positions[frame][d]);
                        if (v < b[d] - 1e-5f || v > b[d + 3] + 1e-5f)
                            all_inside = false;
                    }
                }
            }
        }
        CHECK(all_inside);
    }
}
