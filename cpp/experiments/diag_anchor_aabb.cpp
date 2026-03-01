/**
 * diag_anchor_aabb.cpp
 *
 * 诊断 LB/RB anchor box 的 interval FK AABB 与障碍物的重叠情况：
 *   - find_free_box 找到 anchor node
 *   - 获取该节点及路径上每个祖先的 C-space intervals
 *   - 对每个节点计算 interval FK AABB，报告与哪个障碍物重叠多少
 *   - 也检查兄弟节点（sibling），看为什么扩展被阻断
 *
 * 用法:
 *   ./diag_anchor_aabb [robot.json] [--seed N]
 */
#include "sbf/forest/hier_aabb_tree.h"
#include "sbf/forest/collision.h"
#include "sbf/aabb/interval_fk.h"
#include "sbf/aabb/fk_scalar.h"
#include "sbf/core/robot.h"
#include "sbf/core/types.h"
#include "marcucci_scenes.h"
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace sbf;

// Compute interval FK AABB (raw, no inflation) for given intervals
std::vector<float> compute_raw_aabb(const Robot& robot,
                                     const std::vector<Interval>& ivs) {
    FKState state = compute_fk_full(robot, ivs);
    int n = robot.n_active_links();
    std::vector<float> aabb(n * 6);
    extract_link_aabbs(state, robot.active_link_map(), n, aabb.data(), nullptr);
    return aabb;
}

// Print world-space AABB for each active link
void print_aabb_world(const Robot& robot,
                      const std::vector<float>& aabb,
                      const std::string& label) {
    int n = robot.n_active_links();
    const double* radii = robot.active_link_radii();
    std::cout << "  " << label << " world AABB (raw, before obstacle inflation):\n";
    std::cout << "    " << std::setw(6) << "link"
              << "  " << std::setw(8) << "r"
              << "  [lo_x, hi_x]          [lo_y, hi_y]          [lo_z, hi_z]\n";
    for (int ci = 0; ci < n; ++ci) {
        double r = radii ? radii[ci] : 0.0;
        float lx=aabb[ci*6+0], hx=aabb[ci*6+3];
        float ly=aabb[ci*6+1], hy=aabb[ci*6+4];
        float lz=aabb[ci*6+2], hz=aabb[ci*6+5];
        std::cout << std::fixed << std::setprecision(4)
                  << "    link" << ci
                  << "  r=" << r
                  << "  x[" << lx << ", " << hx << "]"
                  << "  y[" << ly << ", " << hy << "]"
                  << "  z[" << lz << ", " << hz << "]"
                  << "  (inflated: x[" << lx-r << ", " << hx+r << "]"
                  << " y[" << ly-r << ", " << hy+r << "]"
                  << " z[" << lz-r << ", " << hz+r << "])\n";
    }
}

// Report overlap of raw AABB vs inflated obstacles
void report_aabb_vs_obs(const Robot& robot,
                         const std::vector<float>& aabb,
                         const std::vector<Obstacle>& obstacles,
                         const std::string& label) {
    int n = robot.n_active_links();
    const double* radii = robot.active_link_radii();
    bool any = false;

    for (auto& obs : obstacles) {
        auto obs_lo = obs.lo();
        auto obs_hi = obs.hi();
        for (int ci = 0; ci < n; ++ci) {
            double r = radii ? radii[ci] : 0.0;
            float lo_x = aabb[ci*6+0], lo_y = aabb[ci*6+1], lo_z = aabb[ci*6+2];
            float hi_x = aabb[ci*6+3], hi_y = aabb[ci*6+4], hi_z = aabb[ci*6+5];
            // Obstacle inflated by r
            double olx = obs_lo.x()-r, ohx = obs_hi.x()+r;
            double oly = obs_lo.y()-r, ohy = obs_hi.y()+r;
            double olz = obs_lo.z()-r, ohz = obs_hi.z()+r;
            double ox = std::min((double)hi_x, ohx) - std::max((double)lo_x, olx);
            double oy = std::min((double)hi_y, ohy) - std::max((double)lo_y, oly);
            double oz = std::min((double)hi_z, ohz) - std::max((double)lo_z, olz);
            if (ox > 0 && oy > 0 && oz > 0) {
                double pen = std::min({ox, oy, oz});
                if (!any) {
                    std::cout << "  " << label << ":\n";
                    any = true;
                }
                std::cout << "    link" << ci << "(r=" << r << ")"
                          << " vs " << obs.name
                          << "  pen=" << std::fixed << std::setprecision(4) << pen
                          << "  overlap=(" << ox << "," << oy << "," << oz << ")\n";
            }
        }
    }
    if (!any)
        std::cout << "  " << label << ": collision-free\n";
}

// Print C-space intervals summary
void print_intervals(const std::vector<Interval>& ivs, const std::string& label) {
    std::cout << "  " << label << " C-space intervals:\n";
    for (int d = 0; d < (int)ivs.size(); ++d) {
        double w = ivs[d].hi - ivs[d].lo;
        std::cout << "    j" << d
                  << " [" << std::fixed << std::setprecision(3) << ivs[d].lo
                  << ", " << ivs[d].hi << "]"
                  << "  width=" << w << "\n";
    }
}

void analyze_seed(const std::string& name,
                  const Eigen::VectorXd& q,
                  const Robot& robot,
                  const std::vector<Obstacle>& obstacles)
{
    std::cout << "\n╔══════════════════════════════════════════╗\n";
    std::cout << "║  " << name << " anchor AABB diagnosis\n";
    std::cout << "╚══════════════════════════════════════════╝\n";

    // Build a fresh tree
    HierAABBTree tree(robot);
    CollisionChecker checker(robot, obstacles);

    auto ffb = tree.find_free_box(q, checker.obs_flat(), checker.n_obs(),
                                   400, 0.005);

    std::cout << "  find_free_box: ";
    if (ffb.success()) std::cout << "SUCCESS  node=" << ffb.node_idx;
    else               std::cout << "FAILED   fail_code=" << ffb.fail_code;
    std::cout << "  path_depth=" << ffb.path.size()
              << "  new_nodes=" << ffb.n_new_nodes << "\n";

    if (!ffb.success()) return;

    // Anchor node intervals + AABB
    auto anchor_ivs = tree.get_node_intervals(ffb.node_idx);
    print_intervals(anchor_ivs, "anchor");
    {
        auto raw_aabb = compute_raw_aabb(robot, anchor_ivs);
        print_aabb_world(robot, raw_aabb, "anchor");
        std::cout << "\n";
        report_aabb_vs_obs(robot, raw_aabb, obstacles, "anchor AABB");
    }

    // Use interval-FK computed AABB for the anchor (no empirical sampling)
    std::cout << "\n  ── interval-FK anchor AABB (no sampling) ──\n";
    auto interval_aabb = compute_raw_aabb(robot, anchor_ivs);
    print_aabb_world(robot, interval_aabb, "interval anchor");
    std::cout << "\n";
    report_aabb_vs_obs(robot, interval_aabb, obstacles, "interval anchor AABB");

    // Walk up the path: report each ancestor's AABB collision
    std::cout << "\n  ── path ancestors (root→anchor) ──\n";
    for (int depth = 0; depth < (int)ffb.path.size(); ++depth) {
        int node = ffb.path[depth];
        auto ivs = tree.get_node_intervals(node);
        auto raw_aabb = compute_raw_aabb(robot, ivs);

        // Check collision with inflated obstacles
        bool collide = false;
        for (auto& obs : obstacles) {
            auto obs_lo = obs.lo();
            auto obs_hi = obs.hi();
            int n = robot.n_active_links();
            const double* radii = robot.active_link_radii();
            for (int ci = 0; ci < n && !collide; ++ci) {
                double r = radii ? radii[ci] : 0.0;
                float lo_x = raw_aabb[ci*6+0], lo_y = raw_aabb[ci*6+1], lo_z = raw_aabb[ci*6+2];
                float hi_x = raw_aabb[ci*6+3], hi_y = raw_aabb[ci*6+4], hi_z = raw_aabb[ci*6+5];
                double olx = obs_lo.x()-r, ohx = obs_hi.x()+r;
                double oly = obs_lo.y()-r, ohy = obs_hi.y()+r;
                double olz = obs_lo.z()-r, ohz = obs_hi.z()+r;
                if (std::min((double)hi_x,ohx) > std::max((double)lo_x,olx) &&
                    std::min((double)hi_y,ohy) > std::max((double)lo_y,oly) &&
                    std::min((double)hi_z,ohz) > std::max((double)lo_z,olz))
                    collide = true;
            }
        }

        // Width of narrowest joint dim
        double min_w = 1e18;
        for (auto& iv : ivs) min_w = std::min(min_w, iv.hi - iv.lo);

        std::cout << "    depth=" << std::setw(3) << depth
                  << "  node=" << std::setw(5) << node
                  << "  min_w=" << std::fixed << std::setprecision(3) << min_w
                  << "  " << (collide ? "COLLIDE" : "free") << "\n";
    }

    // Find first free ancestor (first node in path that's collision-free)
    std::cout << "\n  ── first free node in path (interval FK) ──\n";
    for (int depth = 0; depth < (int)ffb.path.size(); ++depth) {
        int node = ffb.path[depth];
        auto ivs = tree.get_node_intervals(node);
        auto raw_aabb = compute_raw_aabb(robot, ivs);
        bool collide = link_aabbs_collide_flat(raw_aabb.data(),
                                               checker.obs_flat(), checker.n_obs());
        if (!collide) {
            std::cout << "    depth=" << depth << "  node=" << node
                      << "  C-space widths: [";
            for (int d = 0; d < (int)ivs.size(); ++d)
                std::cout << std::fixed << std::setprecision(2)
                          << (ivs[d].hi - ivs[d].lo) << (d+1<(int)ivs.size()?",":"");
            std::cout << "]\n";
            break;
        }
    }
}

int main(int argc, char** argv)
{
    std::string robot_path = "configs/iiwa14.json";
    int seed = 0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seed" && i+1 < argc) seed = std::atoi(argv[++i]);
        else if (a[0] != '-') robot_path = a;
    }
    (void)seed;

    Robot robot = Robot::from_json(robot_path);
    auto obstacles = make_combined_obstacles();

    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints() << "\n";
    std::cout << "Active links: " << robot.n_active_links()
              << "  radii: [";
    for (int i = 0; i < robot.n_active_links(); ++i)
        std::cout << robot.active_link_radii()[i]
                  << (i+1<robot.n_active_links()?",":"");
    std::cout << "]\n";
    std::cout << "Obstacles: " << obstacles.size() << "\n";

    analyze_seed("LB", config_LB(), robot, obstacles);
    analyze_seed("RB", config_RB(), robot, obstacles);

    return 0;
}
