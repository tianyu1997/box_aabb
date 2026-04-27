// SafeBoxForest v7 — Smoke tests for P6 (experiment system).
#include "experiments/common.h"

#include "sbf/scene/scene_config.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

namespace {

std::string cfg_path(const std::string& fname) {
    return std::string(SBF_V7_ROOT) + "/experiments/configs/" + fname;
}

}  // namespace

// ───── 1. Scene loader: 2dof_box ────────────────────────────────────────────
TEST(P6, SceneLoad2DoF) {
    auto sc = sbf::scene::load_scene_json(cfg_path("2dof_box.json"));
    EXPECT_EQ(sc.name, "2dof_box");
    EXPECT_EQ(sc.robot, "2dof_planar");
    EXPECT_EQ(sc.q_start.size(), 2);
    EXPECT_EQ(sc.q_goal.size(), 2);
    EXPECT_EQ(sc.obstacles.size(), 1u);
    auto packed = sc.packed_obstacles();
    EXPECT_EQ(packed.size(), 6u);
    EXPECT_FLOAT_EQ(packed[0], 20.0f);
    EXPECT_FLOAT_EQ(packed[3], 21.0f);
}

// ───── 2. Scene loader: iiwa14_far ──────────────────────────────────────────
TEST(P6, SceneLoadIIWA14) {
    auto sc = sbf::scene::load_scene_json(cfg_path("iiwa14_far.json"));
    EXPECT_EQ(sc.robot, "iiwa14");
    EXPECT_EQ(sc.q_start.size(), 7);
    EXPECT_EQ(sc.q_goal.size(),  7);
    EXPECT_EQ(sc.obstacles.size(), 1u);
}

// ───── 3. Run a quick trial through run_trial() — iiwa14_far should solve ──
TEST(P6, RunTrialIIWA14Quick) {
    auto sc = sbf::scene::load_scene_json(cfg_path("iiwa14_far.json"));
    auto r  = sbf::exp::run_trial(sc, /*seed=*/42, /*n_threads=*/4,
                                  /*timeout_s=*/30, /*quick=*/true);
    ASSERT_TRUE(r.success) << r.fail_reason;
    EXPECT_GT(r.opt_length, 0.0);
    EXPECT_LE(r.opt_length, r.raw_length + 1e-6);
    EXPECT_LT(r.total_time_ms, 10000.0);
}

// ───── 4. plan_to_json round-trip exposes the expected keys ─────────────────
TEST(P6, PlanToJsonShape) {
    auto sc = sbf::scene::load_scene_json(cfg_path("iiwa14_far.json"));
    auto r  = sbf::exp::run_trial(sc, /*seed=*/42, /*n_threads=*/4,
                                  /*timeout_s=*/30, /*quick=*/true);
    auto j = sbf::exp::plan_to_json(r);
    for (const char* k : {"success", "n_boxes", "opt_length", "raw_length",
                          "total_time_ms", "step_lengths", "fail_reason"}) {
        EXPECT_TRUE(j.contains(k)) << "missing key: " << k;
    }
}
