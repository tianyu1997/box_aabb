/**
 * verify_collision.cpp — Check configs from a text file against our AABB collision model.
 * Also check interpolated CS→LB straight line.
 */
#include <sbf/core/robot.h>
#include <sbf/core/types.h>
#include <sbf/scene/collision_checker.h>
#include "marcucci_scenes.h"

#include <Eigen/Core>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

using namespace sbf;

int main() {
    // ── Load robot with narrow limits ──
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    Robot robot = Robot::from_json(robot_path);
    {
        auto& lim = const_cast<JointLimits&>(robot.joint_limits());
        const double planning_limits[7][2] = {
            {-1.865488,  1.865691},
            {-0.100000,  1.086648},
            {-0.662656,  0.662338},
            {-2.094400, -0.371673},
            {-0.619251,  0.619534},
            {-1.095222,  1.257951},
            { 1.050209,  2.091190},
        };
        for (int j = 0; j < 7; ++j) {
            lim.limits[j].lo = planning_limits[j][0];
            lim.limits[j].hi = planning_limits[j][1];
        }
    }

    auto obstacles = make_combined_obstacles();
    CollisionChecker cc(robot, obstacles);

    // Print obstacles
    printf("Obstacles: %d\n", (int)obstacles.size());
    for (int i = 0; i < (int)obstacles.size(); ++i) {
        printf("  obs[%d]: [%.4f,%.4f,%.4f] - [%.4f,%.4f,%.4f]\n", i,
               obstacles[i].bounds[0], obstacles[i].bounds[1], obstacles[i].bounds[2],
               obstacles[i].bounds[3], obstacles[i].bounds[4], obstacles[i].bounds[5]);
    }

    // Print robot info
    printf("\nRobot: %s  DOF=%d\n", robot.name().c_str(), robot.n_joints());
    printf("Active links: %d\n", robot.n_active_links());
    const int* amap = robot.active_link_map();
    const double* radii = robot.active_link_radii();
    for (int i = 0; i < robot.n_active_links(); ++i) {
        printf("  active[%d] = link %d, radius=%.4f\n", i, amap[i],
               radii ? radii[i] : -1.0);
    }

    // Tool frame
    if (robot.has_tool()) {
        auto& tool = *robot.tool_frame();
        printf("Tool: alpha=%.4f a=%.4f d=%.4f theta=%.4f\n",
               tool.alpha, tool.a, tool.d, tool.theta);
    }

    // 5 configs
    auto q_AS = config_AS();
    auto q_TS = config_TS();
    auto q_CS = config_CS();
    auto q_LB = config_LB();
    auto q_RB = config_RB();

    printf("\n═══ Config collision checks ═══\n");
    printf("AS: %s\n", cc.check_config(q_AS) ? "COLLISION" : "free");
    printf("TS: %s\n", cc.check_config(q_TS) ? "COLLISION" : "free");
    printf("CS: %s\n", cc.check_config(q_CS) ? "COLLISION" : "free");
    printf("LB: %s\n", cc.check_config(q_LB) ? "COLLISION" : "free");
    printf("RB: %s\n", cc.check_config(q_RB) ? "COLLISION" : "free");

    // ── Interpolate CS→LB (100 samples) ──
    printf("\n═══ CS → LB straight line (101 samples) ═══\n");
    int n_free = 0, n_collision = 0;
    double first_collision_t = -1;
    for (int i = 0; i <= 100; ++i) {
        double t = i / 100.0;
        Eigen::VectorXd q = (1.0 - t) * q_CS + t * q_LB;
        bool col = cc.check_config(q);
        if (col) {
            n_collision++;
            if (first_collision_t < 0) first_collision_t = t;
        } else {
            n_free++;
        }
    }
    printf("  Free: %d  Collision: %d  First collision at t=%.2f\n",
           n_free, n_collision, first_collision_t);

    // ── Interpolate CS→RB (100 samples) ──
    printf("\n═══ CS → RB straight line (101 samples) ═══\n");
    n_free = 0; n_collision = 0; first_collision_t = -1;
    for (int i = 0; i <= 100; ++i) {
        double t = i / 100.0;
        Eigen::VectorXd q = (1.0 - t) * q_CS + t * q_RB;
        bool col = cc.check_config(q);
        if (col) {
            n_collision++;
            if (first_collision_t < 0) first_collision_t = t;
        } else {
            n_free++;
        }
    }
    printf("  Free: %d  Collision: %d  First collision at t=%.2f\n",
           n_free, n_collision, first_collision_t);

    // ── Interpolate TS→CS (100 samples) ──
    printf("\n═══ TS → CS straight line (101 samples) ═══\n");
    n_free = 0; n_collision = 0; first_collision_t = -1;
    for (int i = 0; i <= 100; ++i) {
        double t = i / 100.0;
        Eigen::VectorXd q = (1.0 - t) * q_TS + t * q_CS;
        bool col = cc.check_config(q);
        if (col) {
            n_collision++;
            if (first_collision_t < 0) first_collision_t = t;
        } else {
            n_free++;
        }
    }
    printf("  Free: %d  Collision: %d  First collision at t=%.2f\n",
           n_free, n_collision, first_collision_t);

    // ── Interpolate AS→CS (100 samples) ──
    printf("\n═══ AS → CS straight line (101 samples) ═══\n");
    n_free = 0; n_collision = 0; first_collision_t = -1;
    for (int i = 0; i <= 100; ++i) {
        double t = i / 100.0;
        Eigen::VectorXd q = (1.0 - t) * q_AS + t * q_CS;
        bool col = cc.check_config(q);
        if (col) {
            n_collision++;
            if (first_collision_t < 0) first_collision_t = t;
        } else {
            n_free++;
        }
    }
    printf("  Free: %d  Collision: %d  First collision at t=%.2f\n",
           n_free, n_collision, first_collision_t);

    // ── Check the Drake AS→CS path ──
    printf("\n═══ Drake AS→CS path check ═══\n");
    std::ifstream fin("/tmp/drake_as_cs_path.txt");
    if (fin.good()) {
        std::vector<Eigen::VectorXd> path;
        std::string line;
        while (std::getline(fin, line)) {
            Eigen::VectorXd q(7);
            sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf",
                   &q[0], &q[1], &q[2], &q[3], &q[4], &q[5], &q[6]);
            path.push_back(q);
        }
        printf("  Loaded %d waypoints\n", (int)path.size());
        
        int n_free_p = 0, n_col_p = 0;
        int first_col_idx = -1;
        for (int i = 0; i < (int)path.size(); ++i) {
            bool col = cc.check_config(path[i]);
            if (col) {
                n_col_p++;
                if (first_col_idx < 0) {
                    first_col_idx = i;
                    printf("  First collision at waypoint %d: [", i);
                    for (int d = 0; d < 7; ++d) printf("%.6f%s", path[i][d], d<6?", ":"");
                    printf("]\n");
                }
            } else {
                n_free_p++;
            }
        }
        printf("  Free: %d  Collision: %d\n", n_free_p, n_col_p);
    } else {
        printf("  Could not open /tmp/drake_as_cs_path.txt\n");
    }

    // ── Detailed: at first collision point on CS→LB, show FK and AABB ──
    printf("\n═══ Detailed collision diagnosis on CS→LB line ═══\n");
    {
        const int nd = robot.n_joints();
        const auto& dh = robot.dh_params();
        const int n_active = robot.n_active_links();
        const int* amap = robot.active_link_map();
        const double* rad = robot.active_link_radii();

        // Check a few t values around the collision boundary
        for (double t : {0.0, 0.05, 0.10, 0.11, 0.12, 0.15, 0.20, 0.30, 0.50}) {
            Eigen::VectorXd q = (1.0 - t) * q_CS + t * q_LB;

            // Compute FK
            double prefix[16][16];
            std::memset(prefix[0], 0, 16 * sizeof(double));
            prefix[0][0] = prefix[0][5] = prefix[0][10] = prefix[0][15] = 1.0;

            double A[16];
            for (int i = 0; i < nd; ++i) {
                double theta = q[i] + dh[i].theta;
                double d_val = dh[i].d;
                double ca = std::cos(dh[i].alpha), sa = std::sin(dh[i].alpha);
                double ct = std::cos(theta), st = std::sin(theta);
                // Standard DH matrix
                A[0] = ct;      A[1] = -st;     A[2] = 0;    A[3] = dh[i].a;
                A[4] = st*ca;   A[5] = ct*ca;   A[6] = -sa;  A[7] = -sa*d_val;
                A[8] = st*sa;   A[9] = ct*sa;   A[10] = ca;  A[11] = ca*d_val;
                A[12] = 0; A[13] = 0; A[14] = 0; A[15] = 1.0;

                // prefix[i+1] = prefix[i] * A
                double out[16];
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c)
                        out[r*4+c] = prefix[i][r*4+0]*A[0*4+c] + prefix[i][r*4+1]*A[1*4+c]
                                   + prefix[i][r*4+2]*A[2*4+c] + prefix[i][r*4+3]*A[3*4+c];
                std::memcpy(prefix[i+1], out, 16*sizeof(double));
            }
            // Tool
            if (robot.has_tool()) {
                auto& tool = *robot.tool_frame();
                double ct = std::cos(tool.theta), st = std::sin(tool.theta);
                double ca = std::cos(tool.alpha), sa = std::sin(tool.alpha);
                A[0] = ct;      A[1] = -st;     A[2] = 0;    A[3] = tool.a;
                A[4] = st*ca;   A[5] = ct*ca;   A[6] = -sa;  A[7] = -sa*tool.d;
                A[8] = st*sa;   A[9] = ct*sa;   A[10] = ca;  A[11] = ca*tool.d;
                A[12] = 0; A[13] = 0; A[14] = 0; A[15] = 1.0;
                double out[16];
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c)
                        out[r*4+c] = prefix[nd][r*4+0]*A[0*4+c] + prefix[nd][r*4+1]*A[1*4+c]
                                   + prefix[nd][r*4+2]*A[2*4+c] + prefix[nd][r*4+3]*A[3*4+c];
                std::memcpy(prefix[nd+1], out, 16*sizeof(double));
            }

            bool any_col = false;
            printf("  t=%.2f  q=[", t);
            for (int d = 0; d < 7; ++d) printf("%.4f%s", q[d], d<6?",":"");
            printf("]\n");

            for (int li = 0; li < n_active; ++li) {
                int idx = amap[li];
                float sx = prefix[idx][3], ex = prefix[idx+1][3];
                float sy = prefix[idx][7], ey = prefix[idx+1][7];
                float sz = prefix[idx][11], ez = prefix[idx+1][11];
                float r = rad ? (float)rad[li] : 0.0f;

                float lo_x = std::min(sx, ex) - r, hi_x = std::max(sx, ex) + r;
                float lo_y = std::min(sy, ey) - r, hi_y = std::max(sy, ey) + r;
                float lo_z = std::min(sz, ez) - r, hi_z = std::max(sz, ez) + r;

                printf("    link[%d] (frame %d→%d) r=%.3f: "
                       "(%.3f,%.3f,%.3f)→(%.3f,%.3f,%.3f) "
                       "AABB=[%.3f,%.3f]x[%.3f,%.3f]x[%.3f,%.3f]\n",
                       li, idx, idx+1, r, sx, sy, sz, ex, ey, ez,
                       lo_x, hi_x, lo_y, hi_y, lo_z, hi_z);

                // Check against each obstacle
                for (int oi = 0; oi < (int)obstacles.size(); ++oi) {
                    float ox0 = obstacles[oi].bounds[0], oy0 = obstacles[oi].bounds[1], oz0 = obstacles[oi].bounds[2];
                    float ox1 = obstacles[oi].bounds[3], oy1 = obstacles[oi].bounds[4], oz1 = obstacles[oi].bounds[5];

                    if (lo_x <= ox1 && hi_x >= ox0 &&
                        lo_y <= oy1 && hi_y >= oy0 &&
                        lo_z <= oz1 && hi_z >= oz0) {
                        printf("      ** COLLIDES with obs[%d]: [%.3f,%.3f]x[%.3f,%.3f]x[%.3f,%.3f]\n",
                               oi, ox0, ox1, oy0, oy1, oz0, oz1);
                        any_col = true;
                    }
                }
            }
            printf("    → %s\n\n", any_col ? "COLLISION" : "free");
        }
    }

    return 0;
}
