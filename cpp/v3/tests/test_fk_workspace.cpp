// Minimal test to isolate FKWorkspace crash
#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/frame_source.h"
#include <iostream>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

static const char* ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

int main() {
    std::cout << "Step 1: Loading robot..." << std::flush;
    Robot robot = Robot::from_json(ROBOT_PATH);
    std::cout << " OK (" << robot.n_joints() << " joints)\n";

    int n = robot.n_joints();
    auto lim = robot.joint_limits().limits;
    std::vector<Interval> ivs(n);
    for (int j = 0; j < n; ++j) {
        double c = (lim[j].lo + lim[j].hi) * 0.5;
        double hw = (lim[j].hi - lim[j].lo) * 0.05;
        ivs[j] = {c - hw, c + hw};
    }

    std::cout << "Step 2: Testing fk_transforms_inplace..." << std::flush;
    {
        Eigen::VectorXd q(n);
        for (int j = 0; j < n; ++j) q[j] = ivs[j].lo;
        std::vector<Eigen::Matrix4d> tf(n + 2);
        int np = fk_transforms_inplace(robot, q, tf.data());
        std::cout << " OK (np=" << np << ")\n";
    }

    std::cout << "Step 3: Testing IFK source..." << std::flush;
    {
        auto cfg = FrameSourceConfig::ifk();
        auto res = compute_frame_source(cfg, robot, ivs);
        std::cout << " OK (" << res.endpoint_aabbs.size() / 6 << " endpoint_aabbs)\n";
    }

    std::cout << "Step 4: Testing CritSample source..." << std::flush;
    {
        auto cfg = FrameSourceConfig::crit_sampling();
        auto res = compute_frame_source(cfg, robot, ivs);
        std::cout << " OK (" << res.endpoint_aabbs.size() / 6 << " endpoint_aabbs)\n";
    }

    std::cout << "Step 5: Testing AnalyticalCrit source..." << std::flush;
    {
        auto cfg = FrameSourceConfig::analytical_critical();
        auto res = compute_frame_source(cfg, robot, ivs, 4);
        std::cout << " OK (" << res.endpoint_aabbs.size() / 6 << " endpoint_aabbs)\n";
    }

    std::cout << "ALL STEPS PASSED\n";
    return 0;
}
