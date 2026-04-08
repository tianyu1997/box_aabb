// SafeBoxForest v5 — GCPC Cache Generator (Phase R0)
// Usage: generate_gcpc_cache <robot.json> <output.gcpc> [n_points=5000] [seed=42]

#include <sbf/core/robot.h>
#include <sbf/envelope/gcpc_source.h>

#include <Eigen/Dense>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::fprintf(stderr,
            "Usage: %s <robot.json> <output.gcpc> [n_points] [seed]\n",
            argv[0]);
        return 1;
    }

    const std::string robot_path = argv[1];
    const std::string output_path = argv[2];
    int n_points = (argc > 3) ? std::atoi(argv[3]) : 5000;
    uint64_t seed = (argc > 4) ? std::stoull(argv[4]) : 42;

    auto robot = sbf::Robot::from_json(robot_path);
    int nj = robot.n_joints();

    sbf::GcpcCache cache;
    cache.set_n_dims(nj);

    std::mt19937_64 gen(seed);
    const auto& lim = robot.joint_limits().limits;

    for (int i = 0; i < n_points; ++i) {
        Eigen::VectorXd pt(nj);
        for (int j = 0; j < nj; ++j) {
            std::uniform_real_distribution<double> d(lim[j].lo, lim[j].hi);
            pt[j] = d(gen);
        }
        cache.add_point(pt);
    }

    cache.save(output_path);
    std::printf("Generated %d points (%d dims) -> %s\n",
                n_points, nj, output_path.c_str());
    return 0;
}
