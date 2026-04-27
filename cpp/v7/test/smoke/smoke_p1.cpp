#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "sbf/core/endpoint_iaabb.h"
#include "sbf/core/fk_state.h"
#include "sbf/core/interval_math.h"
#include "sbf/core/robot.h"

#ifndef SBF_DATA_DIR
#define SBF_DATA_DIR "data"
#endif

namespace {

using sbf::core::Interval;
using sbf::core::Robot;

std::string data_path(const char* name) {
    return std::string(SBF_DATA_DIR) + "/" + name;
}

}  // namespace

TEST(P1, RobotLoadsIIWA) {
    Robot r = Robot::from_json(data_path("iiwa14.json"));
    EXPECT_EQ(r.name(), "kuka_iiwa14_r820");
    EXPECT_EQ(r.n_joints(), 7);
    EXPECT_TRUE(r.has_tool());
    EXPECT_TRUE(r.has_link_radii());
    // Active links: skip base (0), skip a=d=0 joints (1, 3, 5).
    // IIWA14: link 2 (d=0.42), 4 (d=0.4), 6 (d=0.081), tool (d=0.2).
    // Plus joint 0 itself (d=0.36) is skipped (base). So expected map = {2,4,6,7}.
    EXPECT_EQ(r.n_active_links(), 4);
    const int* m = r.active_link_map();
    EXPECT_EQ(m[0], 2);
    EXPECT_EQ(m[1], 4);
    EXPECT_EQ(m[2], 6);
    EXPECT_EQ(m[3], 7);
}

TEST(P1, IntervalSinCos) {
    auto s = sbf::core::I_sin(0.0, sbf::core::PI);
    EXPECT_NEAR(s.lo, 0.0, 1e-12);
    EXPECT_NEAR(s.hi, 1.0, 1e-12);

    auto c = sbf::core::I_cos(-sbf::core::PI, sbf::core::PI);
    EXPECT_NEAR(c.lo, -1.0, 1e-12);
    EXPECT_NEAR(c.hi,  1.0, 1e-12);

    auto wide = sbf::core::I_sin(0.0, 10.0);  // > 2π
    EXPECT_DOUBLE_EQ(wide.lo, -1.0);
    EXPECT_DOUBLE_EQ(wide.hi,  1.0);
}

TEST(P1, FkAtZeroConfigIIWA) {
    Robot r = Robot::from_json(data_path("iiwa14.json"));
    std::vector<Interval> ivs(r.n_joints(), {0.0, 0.0});
    auto fk = sbf::core::compute_fk_full(r, ivs);
    ASSERT_TRUE(fk.valid);

    // At zero config, FK is point-valued: lo == hi.
    // Tool frame (last prefix) z translation should be sum of d_i for all
    // links in the chain: 0.36 + 0 + 0.42 + 0 + 0.4 + 0 + 0.081 + 0.2 = 1.461.
    int last = fk.n_tf - 1;
    double z_lo = fk.prefix_lo[last][11];
    double z_hi = fk.prefix_hi[last][11];
    EXPECT_NEAR(z_lo, 1.461, 1e-12);
    EXPECT_NEAR(z_hi, 1.461, 1e-12);
}

TEST(P1, IncrementalFkMatchesFull) {
    Robot r = Robot::from_json(data_path("iiwa14.json"));
    std::vector<Interval> ivs;
    ivs.reserve(r.n_joints());
    for (int i = 0; i < r.n_joints(); ++i) {
        double c = 0.1 * (i + 1);
        ivs.push_back({c - 0.05, c + 0.05});
    }
    auto fk_parent = sbf::core::compute_fk_full(r, ivs);

    // Change joint 3.
    int changed = 3;
    ivs[changed] = {0.7, 0.9};

    auto fk_full = sbf::core::compute_fk_full(r, ivs);
    auto fk_inc  = sbf::core::compute_fk_incremental(fk_parent, r, ivs, changed);

    ASSERT_EQ(fk_full.n_tf, fk_inc.n_tf);
    for (int t = 0; t < fk_full.n_tf; ++t) {
        for (int k = 0; k < 16; ++k) {
            EXPECT_NEAR(fk_full.prefix_lo[t][k], fk_inc.prefix_lo[t][k], 1e-12)
                << "lo mismatch at tf=" << t << " idx=" << k;
            EXPECT_NEAR(fk_full.prefix_hi[t][k], fk_inc.prefix_hi[t][k], 1e-12)
                << "hi mismatch at tf=" << t << " idx=" << k;
        }
    }
}

TEST(P1, EndpointIaabbZeroRadius) {
    Robot r = Robot::from_json(data_path("iiwa14.json"));
    std::vector<Interval> ivs(r.n_joints(), {-0.1, 0.1});
    auto res = sbf::core::compute_endpoint_iaabb_ifk(r, ivs);

    EXPECT_TRUE(res.is_safe);
    EXPECT_EQ(res.n_active_links, r.n_active_links());
    ASSERT_EQ(res.endpoint_iaabbs.size(),
              static_cast<size_t>(res.n_active_links * 2 * 6));

    // First active link (link 2) proximal endpoint: location of frame 2.
    // At joints near zero, this should be at z ≈ 0.36 (after first joint d).
    const float* prox0 = res.endpoint_iaabbs.data() + 0 * 6;
    EXPECT_NEAR(prox0[2], 0.36f, 1e-3f);  // z lo
    EXPECT_NEAR(prox0[5], 0.36f, 1e-3f);  // z hi

    // Sanity: for tiny joint range, AABB is small (< 0.2m on each axis),
    // confirming radii are NOT inflated into the output (D1).
    for (int i = 0; i < res.n_active_links * 2; ++i) {
        const float* box = res.endpoint_iaabbs.data() + i * 6;
        for (int d = 0; d < 3; ++d) {
            float w = box[3 + d] - box[d];
            EXPECT_LE(w, 0.5f) << "link " << i << " axis " << d
                               << " unexpectedly wide; radii leaked?";
        }
    }
}
