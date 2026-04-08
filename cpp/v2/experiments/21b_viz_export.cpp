// ═══════════════════════════════════════════════════════════════════════════
// Experiment 21b — AABB / Grid 3D 可视化数据导出
// ═══════════════════════════════════════════════════════════════════════════
//
// 为单个随机 C-space 区间导出：
//   • 各变体 (IFK/CRIT × n_sub=1,4,8) 的 AABB 几何
//   • Grid 体素化结果（occupied voxel centres）
//   • 多个采样臂型 (scalar FK link positions)
//   • 中点臂型
//
// 输出: JSON 文件 → 供交互式 Plotly 3D HTML 读取
//
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/grid_store.h"
#include "sbf/common/config.h"

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace sbf;
using namespace sbf::envelope;

// ─── World bounds ────────────────────────────────────────────────────────
static const float WORLD_BOUNDS[6] = {
    -0.8f, -1.2f, 0.0f,
     1.8f,  1.2f, 1.4f
};

// ─── Robot config finder ─────────────────────────────────────────────────
static std::string find_robot_config(const std::string& name) {
    std::vector<std::string> bases = {
        "../../../safeboxforest/v1/configs",
        "../../safeboxforest/v1/configs",
        "../safeboxforest/v1/configs",
        "../../../../safeboxforest/v1/configs",
    };
    for (auto& b : bases) {
        std::string p = b + "/" + name + ".json";
        if (fs::exists(p)) return p;
    }
    return "";
}

// ─── JSON helpers ────────────────────────────────────────────────────────
static std::string json_arr3(double x, double y, double z) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(6) << "[" << x << "," << y << "," << z << "]";
    return o.str();
}

static std::string json_aabb(const float* a) {
    // a = [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
    std::ostringstream o;
    o << std::fixed << std::setprecision(6)
      << "{\"lo\":[" << a[0] << "," << a[1] << "," << a[2]
      << "],\"hi\":[" << a[3] << "," << a[4] << "," << a[5] << "]}";
    return o.str();
}

// ═════════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    // Parse args
    std::string robot_name = "panda";
    std::string width_regime = "medium";
    int seed = 42;
    std::string output_dir;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--robot" && i+1 < argc) robot_name = argv[++i];
        if (a == "--width" && i+1 < argc) width_regime = argv[++i];
        if (a == "--seed"  && i+1 < argc) seed = std::stoi(argv[++i]);
        if (a == "--output" && i+1 < argc) output_dir = argv[++i];
    }

    // Width regime
    double w_lo = 0.15, w_hi = 0.50;
    if (width_regime == "tiny")   { w_lo = 0.01; w_hi = 0.04; }
    if (width_regime == "small")  { w_lo = 0.05; w_hi = 0.15; }
    if (width_regime == "large")  { w_lo = 0.50; w_hi = 1.50; }

    // Load robot
    std::string config_path = find_robot_config(robot_name);
    if (config_path.empty()) {
        std::cerr << "Robot config not found: " << robot_name << "\n";
        return 1;
    }
    Robot robot = Robot::from_json(config_path);
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int n_frames = n + (robot.has_tool() ? 1 : 0);

    // Radii
    std::vector<float> radii_f(n_act, 0.f);
    if (robot.has_link_radii()) {
        const double* rd = robot.active_link_radii();
        for (int i = 0; i < n_act; ++i)
            radii_f[i] = static_cast<float>(rd[i]);
    }
    float base_pos[3] = {0.f, 0.f, 0.f};

    // Generate random interval
    std::mt19937 rng(seed);
    std::vector<Interval> ivls(n);
    for (int j = 0; j < n; ++j) {
        double lo_j = robot.joint_limits().limits[j].lo;
        double hi_j = robot.joint_limits().limits[j].hi;
        double range = hi_j - lo_j;
        double width = std::uniform_real_distribution<double>(w_lo, w_hi)(rng) * range;
        double center = std::uniform_real_distribution<double>(
            lo_j + width/2, hi_j - width/2)(rng);
        ivls[j].lo = center - width/2;
        ivls[j].hi = center + width/2;
    }

    std::cout << "Robot: " << robot_name << "  Width: " << width_regime
              << "  Seed: " << seed << "\n";
    std::cout << "Intervals:\n";
    for (int j = 0; j < n; ++j)
        std::cout << "  q" << j << ": [" << ivls[j].lo << ", " << ivls[j].hi << "]\n";

    // ─── Compute frames ──────────────────────────────────────────────────

    // IFK frames
    std::vector<float> ifk_frames(n_frames * 6);
    FKState fk = compute_fk_full(robot, ivls);
    for (int k = 0; k < n_frames; ++k) {
        const double* plo = fk.prefix_lo[k + 1];
        const double* phi = fk.prefix_hi[k + 1];
        float* f = ifk_frames.data() + k * 6;
        f[0] = static_cast<float>(plo[3]);
        f[1] = static_cast<float>(plo[7]);
        f[2] = static_cast<float>(plo[11]);
        f[3] = static_cast<float>(phi[3]);
        f[4] = static_cast<float>(phi[7]);
        f[5] = static_cast<float>(phi[11]);
    }

    // CRIT frames
    std::vector<float> crit_frames(n_frames * 6);
    derive_crit_frames(robot, ivls, crit_frames.data(), nullptr);

    // ─── Compute AABBs for n_sub = 1, 4, 8 ──────────────────────────────

    struct EnvData {
        std::string name;
        std::string frame_src;
        int n_sub;
        std::vector<float> aabbs;  // [n_act * n_sub * 6]
    };

    std::vector<EnvData> envs;
    const int subs[] = {1, 4, 8};

    for (int si = 0; si < 3; ++si) {
        int ns = subs[si];
        for (int src = 0; src < 2; ++src) {
            const float* frames = (src == 0) ? ifk_frames.data() : crit_frames.data();
            const char* src_name = (src == 0) ? "IFK" : "CRIT";

            EnvData ed;
            ed.frame_src = src_name;
            ed.n_sub = ns;
            ed.name = std::string(src_name) + "_sub" + std::to_string(ns);
            ed.aabbs.resize(n_act * ns * 6);

            for (int i = 0; i < n_act; ++i) {
                int frame_idx = robot.active_link_map()[i];
                int parent_idx = frame_idx - 1;
                derive_aabb_subdivided(
                    frames, n_frames,
                    parent_idx, frame_idx,
                    ns, radii_f[i], base_pos,
                    ed.aabbs.data() + i * ns * 6);
            }
            envs.push_back(std::move(ed));
        }
    }

    // ─── Compute Grid voxels for n_sub = 8 ──────────────────────────────

    struct GridData {
        std::string name;
        std::vector<std::array<float,3>> centres;  // occupied voxel centres
        int occ_count;
    };

    std::vector<GridData> grids;

    for (int src = 0; src < 2; ++src) {
        const float* frames = (src == 0) ? ifk_frames.data() : crit_frames.data();
        const char* src_name = (src == 0) ? "IFK" : "CRIT";

        GridStore gs(n_frames, n_act,
                     robot.active_link_map(), radii_f.data(),
                     base_pos, WORLD_BOUNDS, 4);
        gs.derive_from_frames(0, frames, 8);

        GridData gd;
        gd.name = std::string(src_name) + "_grid";
        gd.occ_count = gs.occupied_count(0);

        const uint64_t* grid = gs.get_grid(0);
        float cell_x = (WORLD_BOUNDS[3] - WORLD_BOUNDS[0]) / GRID_R;
        float cell_y = (WORLD_BOUNDS[4] - WORLD_BOUNDS[1]) / GRID_R;
        float cell_z = (WORLD_BOUNDS[5] - WORLD_BOUNDS[2]) / GRID_R;

        for (int x = 0; x < GRID_R; ++x) {
            for (int y = 0; y < GRID_R; ++y) {
                for (int z = 0; z < GRID_R; ++z) {
                    int lin = x * GRID_R * GRID_R + y * GRID_R + z;
                    int word = lin >> 6;
                    int bit = lin & 63;
                    if (grid[word] & (uint64_t(1) << bit)) {
                        float cx = WORLD_BOUNDS[0] + (x + 0.5f) * cell_x;
                        float cy = WORLD_BOUNDS[1] + (y + 0.5f) * cell_y;
                        float cz = WORLD_BOUNDS[2] + (z + 0.5f) * cell_z;
                        gd.centres.push_back({cx, cy, cz});
                    }
                }
            }
        }
        grids.push_back(std::move(gd));
    }

    // ─── Sample arm poses via scalar FK ──────────────────────────────────

    struct ArmPose {
        std::string label;
        std::vector<Eigen::Vector3d> positions;
    };

    std::vector<ArmPose> arm_poses;

    // Midpoint pose
    {
        Eigen::VectorXd q_mid(n);
        for (int j = 0; j < n; ++j) q_mid[j] = (ivls[j].lo + ivls[j].hi) / 2.0;
        auto pos = fk_link_positions(robot, q_mid);
        arm_poses.push_back({"midpoint", pos});
    }

    // Corner poses (lo/hi per joint)
    for (int c = 0; c < std::min(8, 1 << n); ++c) {
        Eigen::VectorXd q(n);
        for (int j = 0; j < n; ++j)
            q[j] = (c & (1 << j)) ? ivls[j].hi : ivls[j].lo;
        auto pos = fk_link_positions(robot, q);
        arm_poses.push_back({"corner_" + std::to_string(c), pos});
    }

    // Random sample poses
    for (int s = 0; s < 30; ++s) {
        Eigen::VectorXd q(n);
        for (int j = 0; j < n; ++j)
            q[j] = std::uniform_real_distribution<double>(ivls[j].lo, ivls[j].hi)(rng);
        auto pos = fk_link_positions(robot, q);
        arm_poses.push_back({"sample_" + std::to_string(s), pos});
    }

    // ─── Write JSON ──────────────────────────────────────────────────────

    if (output_dir.empty()) {
        output_dir = "../results/exp21b_viz";
    }
    fs::create_directories(output_dir);
    std::string json_path = output_dir + "/viz_data.json";

    std::ofstream js(json_path);
    js << std::fixed << std::setprecision(6);
    js << "{\n";

    // Metadata
    js << "\"robot\":\"" << robot_name << "\",\n";
    js << "\"width\":\"" << width_regime << "\",\n";
    js << "\"seed\":" << seed << ",\n";
    js << "\"n_joints\":" << n << ",\n";
    js << "\"n_active\":" << n_act << ",\n";
    js << "\"grid_R\":" << GRID_R << ",\n";

    // Intervals
    js << "\"intervals\":[";
    for (int j = 0; j < n; ++j) {
        if (j) js << ",";
        js << "[" << ivls[j].lo << "," << ivls[j].hi << "]";
    }
    js << "],\n";

    // World bounds
    js << "\"world_bounds\":[" << WORLD_BOUNDS[0] << "," << WORLD_BOUNDS[1]
       << "," << WORLD_BOUNDS[2] << "," << WORLD_BOUNDS[3]
       << "," << WORLD_BOUNDS[4] << "," << WORLD_BOUNDS[5] << "],\n";

    // AABB envelopes
    js << "\"envelopes\":[\n";
    for (size_t ei = 0; ei < envs.size(); ++ei) {
        auto& ed = envs[ei];
        js << "  {\"name\":\"" << ed.name << "\","
           << "\"frame_src\":\"" << ed.frame_src << "\","
           << "\"n_sub\":" << ed.n_sub << ","
           << "\"boxes\":[";
        int total = n_act * ed.n_sub;
        for (int b = 0; b < total; ++b) {
            if (b) js << ",";
            js << json_aabb(ed.aabbs.data() + b * 6);
        }
        js << "]}";
        if (ei + 1 < envs.size()) js << ",";
        js << "\n";
    }
    js << "],\n";

    // Grid voxels (occupied centres)
    js << "\"grids\":[\n";
    for (size_t gi = 0; gi < grids.size(); ++gi) {
        auto& gd = grids[gi];
        js << "  {\"name\":\"" << gd.name << "\","
           << "\"occ_count\":" << gd.occ_count << ","
           << "\"centres\":[";
        for (size_t ci = 0; ci < gd.centres.size(); ++ci) {
            if (ci) js << ",";
            js << "[" << gd.centres[ci][0] << "," << gd.centres[ci][1]
               << "," << gd.centres[ci][2] << "]";
        }
        js << "]}";
        if (gi + 1 < grids.size()) js << ",";
        js << "\n";
    }
    js << "],\n";

    // Arm poses
    js << "\"arm_poses\":[\n";
    for (size_t ai = 0; ai < arm_poses.size(); ++ai) {
        auto& ap = arm_poses[ai];
        js << "  {\"label\":\"" << ap.label << "\",\"positions\":[";
        for (size_t pi = 0; pi < ap.positions.size(); ++pi) {
            if (pi) js << ",";
            js << json_arr3(ap.positions[pi].x(), ap.positions[pi].y(),
                            ap.positions[pi].z());
        }
        js << "]}";
        if (ai + 1 < arm_poses.size()) js << ",";
        js << "\n";
    }
    js << "]\n";

    js << "}\n";
    js.close();

    std::cout << "JSON written to: " << json_path << "\n";
    std::cout << "Envelopes: " << envs.size() << "\n";
    std::cout << "Grids: " << grids.size() << "\n";
    std::cout << "Arm poses: " << arm_poses.size() << "\n";

    return 0;
}
