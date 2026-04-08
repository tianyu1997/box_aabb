// ═══════════════════════════════════════════════════════════════════════════
// Experiment 21 — Grid vs AABB 对比实验 (IFK + CRIT 两种帧源)
// ═══════════════════════════════════════════════════════════════════════════
//
// 四种组合:
//   IFK  + AABB   — 区间FK帧 → 细分AABB
//   IFK  + Grid   — 区间FK帧 → 位域栅格 (GridStore, R=32)
//   CRIT + AABB   — 临界点帧 → 细分AABB
//   CRIT + Grid   — 临界点帧 → 位域栅格 (GridStore, R=32)
//
// Grid 在 IFK / CRIT 两种模式下独立储存 (各自的 GridStore 实例)。
// 仅储存 sub 最大时 (n_sub = max_sub) 的最紧密结果。
//
// 度量:
//   • 体积 (AABB: 各 sub-AABB 体积之和; Grid: occupied_count × cell_vol)
//   • 推导时间 (微秒)
//   • Grid 占用体素数
//
// 扫描变量:
//   • Robot:  panda, iiwa14
//   • Width:  small (0.05-0.15), medium (0.15-0.50), large (0.50-1.50)
//   • n_sub:  1, 2, 4, 8 (对比不同细分级别)
//
// Build:
//   cmake .. -DSBF_BUILD_EXPERIMENTS=ON
//   cmake --build . --target exp_21_grid_vs_aabb --config Release
//
// Run:
//   ./exp_21_grid_vs_aabb [--trials 50] [--repeats 3]
//
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/grid_store.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/common/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace sbf;
using namespace sbf::envelope;
using Clock = std::chrono::high_resolution_clock;

// ─── CLI config ──────────────────────────────────────────────────────────

struct ExpConfig {
    int n_trials  = 50;
    int n_repeats = 3;
    std::string output_dir;
};

static ExpConfig parse_args(int argc, char** argv) {
    ExpConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--trials"  && i + 1 < argc) cfg.n_trials  = std::stoi(argv[++i]);
        if (a == "--repeats" && i + 1 < argc) cfg.n_repeats = std::stoi(argv[++i]);
        if (a == "--output"  && i + 1 < argc) cfg.output_dir = argv[++i];
    }
    return cfg;
}

// ─── Width regimes ───────────────────────────────────────────────────────

struct WidthRegime {
    const char* name;
    double lo, hi;
};

static const WidthRegime WIDTHS[] = {
    {"small",  0.05, 0.15},
    {"medium", 0.15, 0.50},
    {"large",  0.50, 1.50},
};

// ─── Robot configs ───────────────────────────────────────────────────────

struct RobotSetup {
    std::string name;
    std::string json_path;
};

static std::vector<RobotSetup> find_robots() {
    std::vector<RobotSetup> robots;
    std::vector<std::string> search_paths = {
        "../../../safeboxforest/v1/configs",
        "../../safeboxforest/v1/configs",
        "../safeboxforest/v1/configs",
        "../../../../safeboxforest/v1/configs",
    };
    for (auto& base : search_paths) {
        std::string panda_path = base + "/panda.json";
        std::string iiwa_path  = base + "/iiwa14.json";
        if (fs::exists(panda_path)) {
            robots.push_back({"panda", panda_path});
            if (fs::exists(iiwa_path))
                robots.push_back({"iiwa14", iiwa_path});
            break;
        }
    }
    if (robots.empty()) {
        std::cerr << "Warning: No robot configs found. "
                  << "Please run from build/ dir.\n";
    }
    return robots;
}

// ─── Volume helpers ──────────────────────────────────────────────────────

// AABB 体积: 各 sub-slot 的 dx*dy*dz 之和
static double aabb_volume(const float* aabbs, int n_slots) {
    double total = 0.0;
    for (int i = 0; i < n_slots; ++i) {
        const float* a = aabbs + i * 6;
        double dx = std::max(0.0, double(a[3] - a[0]));
        double dy = std::max(0.0, double(a[4] - a[1]));
        double dz = std::max(0.0, double(a[5] - a[2]));
        total += dx * dy * dz;
    }
    return total;
}

// Grid 体积: occupied_count × cell_volume
static double grid_volume(int occupied_count, const float world_bounds[6]) {
    double cell_vol = 1.0;
    for (int c = 0; c < 3; ++c) {
        double span = double(world_bounds[3 + c]) - double(world_bounds[c]);
        cell_vol *= span / GRID_R;
    }
    return occupied_count * cell_vol;
}

// ─── Variant definitions ─────────────────────────────────────────────────

enum class FrameSource { IFK, CRIT };
enum class EnvelopeType { AABB, GRID };

struct Variant {
    const char* name;
    const char* short_name;
    FrameSource frame_src;
    EnvelopeType env_type;
};

static const Variant VARIANTS[] = {
    {"IFK+AABB",  "ifk_aabb",  FrameSource::IFK,  EnvelopeType::AABB},
    {"IFK+Grid",  "ifk_grid",  FrameSource::IFK,  EnvelopeType::GRID},
    {"CRIT+AABB", "crit_aabb", FrameSource::CRIT, EnvelopeType::AABB},
    {"CRIT+Grid", "crit_grid", FrameSource::CRIT, EnvelopeType::GRID},
};
static constexpr int N_VARIANTS = 4;

// ─── World bounds (workspace extent for grid) ────────────────────────────

static const float WORLD_BOUNDS[6] = {
    -0.8f, -1.2f, 0.0f,   // lo: x, y, z
     1.8f,  1.2f, 1.4f    // hi: x, y, z
};

// ─── Result record ───────────────────────────────────────────────────────

struct TrialResult {
    std::string robot;
    std::string width_regime;
    int n_sub;
    std::string variant;
    double volume_m3;
    double derive_us;
    int grid_occupied;      // 0 for AABB variants
};

// ═════════════════════════════════════════════════════════════════════════════
//  Main
// ═════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    auto exp_cfg = parse_args(argc, argv);
    auto robots  = find_robots();

    if (robots.empty()) {
        std::cerr << "No robots found. Aborting.\n";
        return 1;
    }

    const std::vector<int> N_SUBS = {1, 2, 4, 8};  // 扫描不同细分级别
    constexpr int MAX_SUB = 8;   // 最大细分 (用于缓冲分配)

    std::vector<TrialResult> all_results;
    std::mt19937 rng(42);

    std::cout << "================================================================\n"
              << " Experiment 21: Grid vs AABB  (IFK / CRIT × AABB / Grid)\n"
              << "================================================================\n"
              << " Trials: " << exp_cfg.n_trials
              << "  Repeats: " << exp_cfg.n_repeats
              << "  n_sub: {1,2,4,8}"
              << "  Grid R: " << GRID_R
              << "\n\n";

    for (auto& rs : robots) {
        Robot robot = Robot::from_json(rs.json_path);
        const int n       = robot.n_joints();
        const int n_act   = robot.n_active_links();
        const int n_frames = n + (robot.has_tool() ? 1 : 0);
        const int total_aabb_slots = n_act * MAX_SUB;

        // 准备 float 版 link_radii (for derive_aabb / GridStore)
        std::vector<float> radii_f(n_act, 0.f);
        if (robot.has_link_radii()) {
            const double* rd = robot.active_link_radii();
            for (int i = 0; i < n_act; ++i)
                radii_f[i] = static_cast<float>(rd[i]);
        }

        float base_pos[3] = {0.f, 0.f, 0.f};

        // 两个独立的 GridStore ── IFK 和 CRIT 各自储存
        GridStore grid_ifk(n_frames, n_act,
                           robot.active_link_map(), radii_f.data(),
                           base_pos, WORLD_BOUNDS, /*initial_cap=*/4);

        GridStore grid_crit(n_frames, n_act,
                            robot.active_link_map(), radii_f.data(),
                            base_pos, WORLD_BOUNDS, /*initial_cap=*/4);

        // 帧缓冲
        std::vector<float> ifk_frames(n_frames * 6);
        std::vector<float> crit_frames(n_frames * 6);

        // AABB 缓冲
        std::vector<float> aabb_buf(total_aabb_slots * 6);

        // CRIT 配置 — 使用 full_fast (全流水线 + 加速)
        auto crit_cfg = CriticalSamplingConfig::full_fast();

        std::cout << "── Robot: " << rs.name
                  << " (" << n << " joints, " << n_act << " active links)\n\n";

        for (auto& wr : WIDTHS) {
          for (int n_sub : N_SUBS) {
            const int total_slots = n_act * n_sub;

            // 每个 (robot, width, n_sub) 的变体累积器
            // key = variant short_name
            std::map<std::string, std::vector<double>> vol_acc, time_acc;
            std::map<std::string, std::vector<int>> occ_acc;

            for (int trial = 0; trial < exp_cfg.n_trials; ++trial) {
                // 生成随机 C-space 区间
                std::vector<Interval> ivls(n);
                for (int j = 0; j < n; ++j) {
                    double lo_j = robot.joint_limits().limits[j].lo;
                    double hi_j = robot.joint_limits().limits[j].hi;
                    double range = hi_j - lo_j;
                    double width = std::uniform_real_distribution<double>(
                        wr.lo, wr.hi)(rng) * range;
                    double center = std::uniform_real_distribution<double>(
                        lo_j + width / 2.0, hi_j - width / 2.0)(rng);
                    ivls[j].lo = center - width / 2.0;
                    ivls[j].hi = center + width / 2.0;
                }

                // ──────────────────────────────────────────────────────────
                // 1) IFK 帧
                // ──────────────────────────────────────────────────────────
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

                // ──────────────────────────────────────────────────────────
                // 2) CRIT 帧
                // ──────────────────────────────────────────────────────────
                derive_crit_frames(robot, ivls, crit_frames.data(), nullptr);

                // ──────────────────────────────────────────────────────────
                // 测量各变体
                // ──────────────────────────────────────────────────────────
                for (int vi = 0; vi < N_VARIANTS; ++vi) {
                    const auto& var = VARIANTS[vi];
                    const float* frames = (var.frame_src == FrameSource::IFK)
                                          ? ifk_frames.data()
                                          : crit_frames.data();

                    double vol = 0.0;
                    int occ = 0;
                    double best_us = 1e18;

                    if (var.env_type == EnvelopeType::AABB) {
                        // ── AABB 细分 ──
                        // 预热
                        for (int i = 0; i < n_act; ++i) {
                            int frame_idx  = robot.active_link_map()[i];
                            int parent_idx = frame_idx - 1;
                            derive_aabb_subdivided(
                                frames, n_frames,
                                parent_idx, frame_idx,
                                n_sub, radii_f[i], base_pos,
                                aabb_buf.data() + i * n_sub * 6);
                        }

                        // 计时
                        for (int rep = 0; rep < exp_cfg.n_repeats; ++rep) {
                            auto t0 = Clock::now();
                            for (int i = 0; i < n_act; ++i) {
                                int frame_idx  = robot.active_link_map()[i];
                                int parent_idx = frame_idx - 1;
                                derive_aabb_subdivided(
                                    frames, n_frames,
                                    parent_idx, frame_idx,
                                    n_sub, radii_f[i], base_pos,
                                    aabb_buf.data() + i * n_sub * 6);
                            }
                            auto t1 = Clock::now();
                            double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
                            best_us = std::min(best_us, us);
                        }

                        vol = aabb_volume(aabb_buf.data(), total_slots);

                    } else {
                        // ── Grid 位域 ──
                        GridStore& gs = (var.frame_src == FrameSource::IFK)
                                        ? grid_ifk : grid_crit;
                        constexpr int NODE_IDX = 0;

                        // 预热
                        gs.derive_from_frames(NODE_IDX, frames, n_sub);

                        // 计时
                        for (int rep = 0; rep < exp_cfg.n_repeats; ++rep) {
                            auto t0 = Clock::now();
                            gs.derive_from_frames(NODE_IDX, frames, n_sub);
                            auto t1 = Clock::now();
                            double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
                            best_us = std::min(best_us, us);
                        }

                        occ = gs.occupied_count(NODE_IDX);
                        vol = grid_volume(occ, WORLD_BOUNDS);
                    }

                    vol_acc[var.short_name].push_back(vol);
                    time_acc[var.short_name].push_back(best_us);
                    occ_acc[var.short_name].push_back(occ);

                    TrialResult tr;
                    tr.robot        = rs.name;
                    tr.width_regime = wr.name;
                    tr.n_sub        = n_sub;
                    tr.variant      = var.short_name;
                    tr.volume_m3    = vol;
                    tr.derive_us    = best_us;
                    tr.grid_occupied = occ;
                    all_results.push_back(tr);
                }
            }

            // ── 打印本 (robot, width, n_sub) 的汇总 ─────────────────────
            std::cout << "  " << wr.name << " | n_sub=" << n_sub << "\n";
            std::cout << "  " << std::string(95, '-') << "\n";
            std::cout << "  " << std::left << std::setw(15) << "Variant"
                      << std::right
                      << std::setw(14) << "Vol(m3)"
                      << std::setw(14) << "Time(us)"
                      << std::setw(12) << "GridOcc"
                      << std::setw(14) << "VolRatio"
                      << "\n";

            // 基准: IFK+AABB 的平均体积
            double base_vol = 0;
            if (!vol_acc["ifk_aabb"].empty()) {
                for (double v : vol_acc["ifk_aabb"]) base_vol += v;
                base_vol /= vol_acc["ifk_aabb"].size();
            }

            for (int vi = 0; vi < N_VARIANTS; ++vi) {
                const auto& var = VARIANTS[vi];
                auto& vols  = vol_acc[var.short_name];
                auto& times = time_acc[var.short_name];
                auto& occs  = occ_acc[var.short_name];
                if (vols.empty()) continue;

                double vol_mean  = 0, time_mean = 0;
                double occ_mean  = 0;
                for (double v : vols) vol_mean += v;
                for (double t : times) time_mean += t;
                for (int o : occs) occ_mean += o;
                vol_mean  /= vols.size();
                time_mean /= times.size();
                occ_mean  /= occs.size();

                double ratio = (base_vol > 0) ? vol_mean / base_vol : 1.0;

                std::cout << "  " << std::left << std::setw(15) << var.name
                          << std::right << std::fixed
                          << std::setprecision(6) << std::setw(14) << vol_mean
                          << std::setprecision(1) << std::setw(14) << time_mean
                          << std::setw(12) << static_cast<int>(occ_mean)
                          << std::setprecision(4) << std::setw(14) << ratio
                          << "\n";
            }
            std::cout << "\n";
          } // n_sub loop
        } // width loop
    }

    // ═════════════════════════════════════════════════════════════════════
    // Write CSV
    // ═════════════════════════════════════════════════════════════════════

    std::string csv_path;
    if (!exp_cfg.output_dir.empty()) {
        fs::create_directories(exp_cfg.output_dir);
        csv_path = exp_cfg.output_dir + "/exp21_grid_vs_aabb.csv";
    } else {
        auto now = std::chrono::system_clock::now();
        auto t   = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
#ifdef _WIN32
        localtime_s(&tm_buf, &t);
#else
        localtime_r(&t, &tm_buf);
#endif
        std::ostringstream oss;
        oss << "../results/exp21_grid_vs_aabb_"
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        std::string dir = oss.str();
        fs::create_directories(dir);
        csv_path = dir + "/exp21_grid_vs_aabb.csv";
    }

    std::ofstream csv(csv_path);
    if (csv.is_open()) {
        csv << "robot,width,n_sub,variant,volume_m3,derive_us,grid_occupied\n";
        for (auto& r : all_results) {
            csv << r.robot << "," << r.width_regime << "," << r.n_sub << ","
                << r.variant << ","
                << std::scientific << std::setprecision(8) << r.volume_m3 << ","
                << std::fixed << std::setprecision(1) << r.derive_us << ","
                << r.grid_occupied << "\n";
        }
        csv.close();
        std::cout << "CSV written to: " << csv_path << "\n";
    }

    // ═════════════════════════════════════════════════════════════════════
    // Write Markdown report
    // ═════════════════════════════════════════════════════════════════════

    std::string md_path = csv_path.substr(0, csv_path.size() - 4) + "_report.md";
    std::ofstream md(md_path);
    if (md.is_open()) {
        md << "# Experiment 21 — Grid vs AABB 对比\n\n";
        md << "**Date**: " << __DATE__ << "  \n";
        md << "**Trials**: " << exp_cfg.n_trials
           << " | **Repeats**: " << exp_cfg.n_repeats
           << " | **n_sub**: {1,2,4,8}"
           << " | **Grid R**: " << GRID_R << "\n\n";

        md << "## 变体说明\n\n";
        md << "| 变体 | 帧源 | 包络类型 | 说明 |\n";
        md << "|------|------|----------|------|\n";
        md << "| IFK+AABB  | 区间FK | 细分AABB | 基准 |\n";
        md << "| IFK+Grid  | 区间FK | 位域栅格 | Grid 独立储存 (IFK) |\n";
        md << "| CRIT+AABB | 临界点 | 细分AABB | 更紧帧 → 更紧 AABB |\n";
        md << "| CRIT+Grid | 临界点 | 位域栅格 | Grid 独立储存 (CRIT) |\n";
        md << "\n";

        md << "## 总结\n\n";

        // 按 (robot, width) 汇总
        struct Agg {
            double vol_sum = 0, time_sum = 0;
            long long occ_sum = 0;
            int count = 0;
        };
        std::map<std::string, Agg> agg;
        for (auto& r : all_results) {
            std::string key = r.robot + " | " + r.width_regime + " | " + std::to_string(r.n_sub) + " | " + r.variant;
            agg[key].vol_sum  += r.volume_m3;
            agg[key].time_sum += r.derive_us;
            agg[key].occ_sum  += r.grid_occupied;
            agg[key].count++;
        }

        md << "| Robot | Width | n_sub | Variant | Mean Vol (m³) | Mean Time (µs) | Mean GridOcc |\n";
        md << "|-------|-------|-------|---------|---------------|----------------|-------------|\n";
        for (auto& [key, a] : agg) {
            md << "| " << key << " | "
               << std::scientific << std::setprecision(4)
               << a.vol_sum / a.count << " | "
               << std::fixed << std::setprecision(1)
               << a.time_sum / a.count << " | "
               << a.occ_sum / a.count << " |\n";
        }
        md << "\n";

        md.close();
        std::cout << "Report written to: " << md_path << "\n";
    }

    std::cout << "\nDone. " << all_results.size() << " total trial results.\n";
    return 0;
}
