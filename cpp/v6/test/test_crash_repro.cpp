// Standalone crash reproduction: multiple plan() calls with IFK on 2dof_narrow
// Build with: cmake --build build_x64 --config Release --target test_crash_repro

#include <sbf/planner/sbf_planner.h>
#include <sbf/core/robot.h>
#include <sbf/core/types.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>

#include <Eigen/Dense>
#include <cstdio>
#include <vector>

using namespace sbf;

static Robot make_2dof_robot() {
    // Must match data/2dof_planar.json exactly!
    // Joint 0: alpha=0, a=0 (zero-length base), d=0, theta=0
    // Joint 1: alpha=0, a=1, d=0, theta=0
    // Tool: alpha=0, a=1, d=0, theta=0
    // 3 link_radii: [0.05, 0.05, 0.05]
    std::vector<DHParam> dh = {
        {0.0, 0.0, 0.0, 0.0, 0},  // joint 0: a=0
        {0.0, 1.0, 0.0, 0.0, 0},  // joint 1: a=1
    };
    JointLimits limits;
    limits.limits = {{-PI, PI}, {-PI, PI}};
    DHParam tool{0.0, 1.0, 0.0, 0.0, 0};  // tool: a=1
    std::vector<double> radii = {0.05, 0.05, 0.05};
    return Robot("2dof_planar", dh, limits, tool, radii);
}

int main() {
    auto robot = make_2dof_robot();

    // 2dof_narrow obstacles: two walls
    // wall1: center=[0,1.8,0] half=[0.8,0.1,0.8]
    // wall2: center=[0,1.2,0] half=[0.8,0.1,0.8]
    std::vector<Obstacle> obstacles = {
        {-0.8f, 1.7f, -0.8f, 0.8f, 1.9f, 0.8f},
        {-0.8f, 1.1f, -0.8f, 0.8f, 1.3f, 0.8f},
    };

    Eigen::VectorXd start(2); start << 0.5, 0.3;
    Eigen::VectorXd goal(2);  goal  << 2.5, 1.5;

    // Test all 3 IFK envelope types
    struct TestConfig {
        const char* name;
        EnvelopeType env_type;
    };

    TestConfig configs[] = {
        {"IFK-LinkIAABB",      EnvelopeType::LinkIAABB},
        {"IFK-LinkIAABB_Grid", EnvelopeType::LinkIAABB_Grid},
        {"IFK-Hull16_Grid",    EnvelopeType::Hull16_Grid},
    };

    const int N_TRIALS = 30;

    for (auto& tc : configs) {
        fprintf(stderr, "\n=== %s ===\n", tc.name);
        for (int trial = 0; trial < N_TRIALS; ++trial) {
            SBFPlannerConfig cfg;
            cfg.grower.max_boxes = 500;
            cfg.grower.mode = GrowerConfig::Mode::WAVEFRONT;
            cfg.endpoint_source.source = EndpointSource::IFK;
            cfg.envelope_type.type = tc.env_type;
            cfg.lect_no_cache = true;

            SBFPlanner planner(robot, cfg);

            fprintf(stderr, "[%s trial=%d] planning...\n", tc.name, trial);
            auto result = planner.plan(start, goal,
                                       obstacles.data(),
                                       static_cast<int>(obstacles.size()),
                                       30000.0);
            fprintf(stderr, "[%s trial=%d] success=%d boxes=%d time=%.0fms\n",
                    tc.name, trial, result.success,
                    result.n_boxes, result.planning_time_ms);
        }
    }

    fprintf(stderr, "\nAll trials complete!\n");
    return 0;
}
