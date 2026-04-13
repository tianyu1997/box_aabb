/**
 * exp1_coverage.cpp — 实验 1: Region 生成效率与自由空间覆盖
 *
 * v5 pipeline: LECT → Wavefront grow → Coarsen → MC coverage
 *
 * 设计:
 *   合并场景 (shelves + bins + table = 16 obstacles)
 *   使用 build_coverage() (无 start/goal, 纯 wavefront 扩展)
 *   多组 preset 配置: 不同 max_boxes / wavefront stages / coarsen target
 *   N_seeds 次重复, 测量: n_boxes, total_volume, build_time, coverage_rate (MC)
 *
 * v5 优化:
 *   - BEST_TIGHTEN split order (LECT 维度分割)
 *   - Z4 symmetry cache (自动对称性加速)
 *   - LECT persistent cache (磁盘缓存, 热重建加速)
 *   - Wavefront multi-stage growth
 *   - Multi-threaded parallel grow
 *
 * 用法:
 *   ./exp1_coverage [--seeds N] [--mc N] [--preset NAME] [--threads N] [--quick]
 */

#include <sbf/planner/sbf_planner.h>
#include <sbf/scene/collision_checker.h>
#include <sbf/forest/connectivity.h>
#include <sbf/core/robot.h>
#include "marcucci_scenes.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════
// Output directory helpers
// ═══════════════════════════════════════════════════════════════════════════

static std::string make_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    struct tm tm_buf;
    localtime_r(&t, &tm_buf);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
    return std::string(buf);
}

static void ensure_dir(const std::string& path) {
    ::mkdir(path.c_str(), 0755);
}

/// Tee-stream: writes to both a file and an existing ostream (e.g. cout/cerr).
/// Auto-flushes both destinations on newline for reliable log capture.
class TeeBuf : public std::streambuf {
public:
    TeeBuf(std::streambuf* sb1, std::streambuf* sb2)
        : sb1_(sb1), sb2_(sb2) {}
protected:
    int overflow(int c) override {
        if (c == EOF) return !EOF;
        if (sb1_->sputc(c) == EOF) return EOF;
        if (sb2_->sputc(c) == EOF) return EOF;
        // Auto-flush on newline for crash safety
        if (c == '\n') {
            sb1_->pubsync();
            sb2_->pubsync();
        }
        return c;
    }
    int sync() override {
        sb1_->pubsync();
        sb2_->pubsync();
        return 0;
    }
private:
    std::streambuf* sb1_;
    std::streambuf* sb2_;
};

/// Tee stderr (fd 2) to both the original console AND a log file,
/// using a pipe + background reader thread.  Returns the pipe-write fd
/// (this is now the new stderr) so the caller can close it when done.
static int g_stderr_pipe_read_fd  = -1;
static int g_stderr_saved_fd      = -1;
static std::thread g_stderr_thread;

static void start_stderr_tee(const std::string& path) {
    int pipefd[2];
    if (::pipe(pipefd) != 0) return;
    g_stderr_saved_fd = ::dup(STDERR_FILENO);  // keep original console fd
    ::dup2(pipefd[1], STDERR_FILENO);          // stderr now goes into pipe
    ::close(pipefd[1]);
    g_stderr_pipe_read_fd = pipefd[0];

    g_stderr_thread = std::thread([read_fd = pipefd[0],
                                   console_fd = g_stderr_saved_fd,
                                   path]() {
        FILE* logf = ::fopen(path.c_str(), "w");
        char buf[4096];
        ssize_t n;
        while ((n = ::read(read_fd, buf, sizeof(buf))) > 0) {
            ::write(console_fd, buf, n);        // → terminal
            if (logf) { ::fwrite(buf, 1, n, logf); ::fflush(logf); }
        }
        ::close(read_fd);
        if (logf) ::fclose(logf);
    });
}

static void stop_stderr_tee() {
    if (g_stderr_saved_fd < 0) return;
    ::fflush(stderr);
    ::dup2(g_stderr_saved_fd, STDERR_FILENO);  // restore original stderr
    ::close(g_stderr_saved_fd);
    g_stderr_saved_fd = -1;
    if (g_stderr_pipe_read_fd >= 0) {
        // The pipe read end is closed by the thread when write-end EOF
        // But dup2 above closed the write end (old STDERR_FILENO copy)
        // Thread will see EOF and exit
    }
    if (g_stderr_thread.joinable()) g_stderr_thread.join();
}

// ═══════════════════════════════════════════════════════════════════════════
// Coverage estimation — analytical + box-validated MC
// ═══════════════════════════════════════════════════════════════════════════

struct CoverageResult {
    double vol_coverage;     // sum(box_vol) / V_free  (analytical upper bound)
    double valid_rate;       // fraction of in-box samples that are truly collision-free
    double vol_total;        // sum of box volumes
    double v_free_est;       // estimated free-space volume from MC
    int n_free, n_total;
};

/**
 * Hybrid coverage estimation for high-dimensional C-spaces.
 *
 * 1) Uniform MC to estimate V_free = (n_free / n_total) * V_total
 * 2) Analytical volume coverage = sum(box_vol) / V_free
 * 3) Box-interior MC to validate: sample uniformly inside each box,
 *    check collision → valid_rate = fraction truly collision-free
 *    (valid_rate should be ~100% if FFB is correct)
 */
CoverageResult estimate_coverage(
    const Robot& robot,
    const CollisionChecker& checker,
    const std::vector<BoxNode>& boxes,
    int n_mc_free, int n_mc_validate, int mc_seed)
{
    std::mt19937 rng(mc_seed);
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    const int ndim = robot.n_joints();
    const auto& jl = robot.joint_limits();

    // 1) Estimate free fraction via uniform MC
    int n_free = 0;
    Eigen::VectorXd q(ndim);
    for (int s = 0; s < n_mc_free; ++s) {
        for (int d = 0; d < ndim; ++d)
            q[d] = jl.limits[d].lo + unif(rng) * jl.limits[d].width();
        if (!checker.check_config(q)) n_free++;
    }

    // Compute V_total (product of joint widths)
    double v_total = 1.0;
    for (int d = 0; d < ndim; ++d)
        v_total *= jl.limits[d].width();

    double free_frac = n_mc_free > 0 ? double(n_free) / n_mc_free : 0.5;
    double v_free = v_total * free_frac;

    // 2) Sum box volumes (analytical)
    double vol_sum = 0.0;
    for (auto& b : boxes) vol_sum += b.volume;

    double vol_cov = v_free > 0 ? vol_sum / v_free : 0.0;

    // 3) Validate boxes: sample inside each box, check collision
    int n_valid = 0, n_checked = 0;
    if (!boxes.empty() && n_mc_validate > 0) {
        int per_box = std::max(1, n_mc_validate / static_cast<int>(boxes.size()));
        for (auto& box : boxes) {
            for (int s = 0; s < per_box; ++s) {
                for (int d = 0; d < ndim; ++d) {
                    double lo = box.joint_intervals[d].lo;
                    double hi = box.joint_intervals[d].hi;
                    q[d] = lo + unif(rng) * (hi - lo);
                }
                n_checked++;
                if (!checker.check_config(q)) n_valid++;
            }
        }
    }
    double valid_rate = n_checked > 0 ? double(n_valid) / n_checked : 1.0;

    return {vol_cov, valid_rate, vol_sum, v_free, n_free, n_mc_free};
}

// ═══════════════════════════════════════════════════════════════════════════
// Statistics
// ═══════════════════════════════════════════════════════════════════════════

struct Stats { double median, q25, q75, mean; };

Stats compute_stats(std::vector<double>& data) {
    Stats s{};
    if (data.empty()) return s;
    std::sort(data.begin(), data.end());
    int n = static_cast<int>(data.size());
    s.median = data[n / 2];
    s.q25 = data[n / 4];
    s.q75 = data[3 * n / 4];
    s.mean = std::accumulate(data.begin(), data.end(), 0.0) / n;
    return s;
}

// ═══════════════════════════════════════════════════════════════════════════
// Preset configurations (mapped to v5 SBFPlannerConfig)
// ═══════════════════════════════════════════════════════════════════════════

struct Preset {
    std::string name;
    int max_boxes;
    std::vector<GrowerConfig::WavefrontStage> stages;
    int coarsen_target;     // 0 = no greedy coarsen
    int max_consecutive_miss;
};

// A: 快速 — 粗粒度, 少量 box
Preset preset_A() {
    return {"A-fast", 500,
            {{100}, {300}, {500}},
            0, 50};
}

// B: 基线 — 默认 v5 wavefront 配置
Preset preset_B() {
    return {"B-baseline", 500,
            {{50}, {150}, {300}, {500}},
            0, 50};
}

// C: 高覆盖 — 更多 box, 更多 stage
Preset preset_C() {
    return {"C-high", 2000,
            {{100}, {300}, {800}, {1500}, {2000}},
            0, 100};
}

// D: 极致覆盖 — 大量 box
Preset preset_D() {
    return {"D-max", 5000,
            {{200}, {500}, {1500}, {3000}, {4000}, {5000}},
            0, 200};
}

// E: 基线 + greedy coarsen → 200 boxes
Preset preset_E() {
    return {"E-coarsen200", 2000,
            {{100}, {300}, {800}, {1500}, {2000}},
            200, 100};
}

// F: 基线 + greedy coarsen → 500 boxes
Preset preset_F() {
    return {"F-coarsen500", 2000,
            {{100}, {300}, {800}, {1500}, {2000}},
            500, 100};
}

// G: 最优 — 10s 预算, 高体积覆盖 + 强连通 + 少 box
//    Key: more initial boxes + aggressive multi-pass coarsen
Preset preset_G() {
    return {"G-optimal", 2500,
            {{200}, {800}, {1800}, {2500}},
            300, 300};
}

// H: 轻量 — 更少 box, 牺牲一些覆盖率
Preset preset_H() {
    return {"H-lite", 2000,
            {{150}, {600}, {1400}, {2000}},
            200, 200};
}

// I: CritSample Optimal — 补偿 CritSample 体积缩小, 更多 box + 激进 coarsen
Preset preset_I() {
    return {"I-crit-opt", 3000,
            {{300}, {1200}, {2200}, {3000}},
            250, 300};
}

// J: CritSample Lite — 少量 box, 快速収敛
Preset preset_J() {
    return {"J-crit-lite", 2000,
            {{200}, {800}, {1500}, {2000}},
            150, 200};
}

SBFPlannerConfig make_config(const Preset& p, int n_threads, uint64_t seed,
                             bool lect_cold) {
    SBFPlannerConfig cfg;
    cfg.z4_enabled = true;
    cfg.split_order = SplitOrder::BEST_TIGHTEN;
    cfg.lect_no_cache = lect_cold;

    // Envelope: CritSample + Hull16_Grid (tighter, V6 cache persistent)
    cfg.endpoint_source.source = EndpointSource::CritSample;
    cfg.endpoint_source.n_samples_crit = 64;     // 64 samples: good balance of speed vs tightness
    cfg.envelope_type.type = EnvelopeType::Hull16_Grid;
    cfg.envelope_type.n_subdivisions = 1;
    cfg.envelope_type.grid_config.voxel_delta = 0.04;  // 0.04 = 2x coarser → 8x fewer voxels

    cfg.grower.mode = GrowerConfig::Mode::WAVEFRONT;
    cfg.grower.max_boxes = p.max_boxes;
    cfg.grower.max_consecutive_miss = p.max_consecutive_miss;
    cfg.grower.wavefront_stages = p.stages;
    // Grow phase: always serial (1 thread) — unified wavefront allocates
    // boxes where they're most useful, yielding 10-20× better coverage than
    // parallel per-tree split (which caps each tree at max_boxes/n_trees).
    // Bridge phase uses the user's --threads setting via config_.grower.n_threads.
    cfg.grower.n_threads = 1;
    cfg.grower.bridge_n_threads = n_threads;  // for bridge_all_islands
    cfg.grower.rng_seed = seed;
    cfg.grower.enable_promotion = true;
    cfg.grower.ffb_config.max_depth = 70;  // deeper for CritSample's tighter envelopes

    cfg.coarsen.target_boxes = p.coarsen_target;
    cfg.coarsen.max_rounds = 100;
    cfg.coarsen.score_threshold = 200.0;   // more aggressive merge (CritSample = precise)
    cfg.coarsen.max_lect_fk_per_round = 30000;

    return cfg;
}

// Build seed_points from all 8 Marcucci key configurations.
// Multi-goal RRT grows from each seed simultaneously → uniform coverage + fewer islands.
std::vector<Eigen::VectorXd> make_seed_points() {
    std::vector<Eigen::VectorXd> seeds;
    seeds.push_back(config_AS());
    seeds.push_back(config_TS());
    seeds.push_back(config_CS());
    seeds.push_back(config_LB());
    seeds.push_back(config_RB());
    seeds.push_back(config_C());
    seeds.push_back(config_L());
    seeds.push_back(config_R());
    return seeds;
}

void run_preset(const Preset& p, const Robot& robot,
                const std::vector<Obstacle>& obstacles,
                int n_seeds, int n_mc, int n_threads, bool lect_cold) {
    std::cout << "\n╔════════════════════════════════════════════════╗\n"
              << "║  Preset: " << std::left << std::setw(38) << p.name << "║\n"
              << "╚════════════════════════════════════════════════╝\n"
              << "  max_boxes=" << p.max_boxes
              << "  coarsen_target=" << p.coarsen_target
              << "  threads=" << n_threads
              << "  lect_cache=" << (lect_cold ? "OFF" : "ON") << "\n"
              << "  stages: [";
    for (size_t i = 0; i < p.stages.size(); ++i)
        std::cout << (i ? ", " : "")
                  << "{" << p.stages[i].box_limit << "}";
    std::cout << "]\n\n";

    auto seed_points = make_seed_points();

    std::vector<double> v_reg, v_cov, v_time, v_vol, v_islands, v_edges;

    for (int seed = 0; seed < n_seeds; ++seed) {
        auto cfg = make_config(p, n_threads, static_cast<uint64_t>(seed), lect_cold);
        SBFPlanner planner(robot, cfg);

        auto t0 = std::chrono::steady_clock::now();
        planner.build_coverage(obstacles.data(),
                               static_cast<int>(obstacles.size()),
                               cfg.grower.timeout_ms,
                               seed_points);
        double wall = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        int nreg = planner.n_boxes();
        double vol = 0;
        for (auto& b : planner.boxes()) vol += b.volume;

        // Connectivity metrics
        const auto& adj = planner.adjacency();
        auto islands = find_islands(adj);
        int n_islands = static_cast<int>(islands.size());
        int n_edges = 0;
        for (auto& kv : adj) n_edges += static_cast<int>(kv.second.size());
        n_edges /= 2;

        int mc_seed = seed + 1000000;
        CollisionChecker checker(robot, obstacles);
        int n_validate = std::min(n_mc, static_cast<int>(planner.boxes().size()) * 10);
        auto cov = estimate_coverage(robot, checker, planner.boxes(),
                                      n_mc, n_validate, mc_seed);

        v_reg.push_back(nreg);
        v_cov.push_back(cov.vol_coverage * 100.0);
        v_time.push_back(wall);
        v_vol.push_back(vol);
        v_islands.push_back(n_islands);
        v_edges.push_back(n_edges);

        double avg_deg = nreg > 0 ? 2.0 * n_edges / nreg : 0.0;
        std::cout << "    s" << std::setw(2) << seed
                  << "  boxes=" << std::setw(5) << nreg
                  << "  vol=" << std::scientific << std::setprecision(3) << vol
                  << "  vcov=" << std::fixed << std::setprecision(4)
                  << cov.vol_coverage * 100.0 << "%"
                  << "  valid=" << std::setprecision(1) << cov.valid_rate * 100 << "%"
                  << "  t=" << std::setprecision(2) << wall << "s"
                  << "  isl=" << n_islands
                  << "  E=" << n_edges
                  << "  deg=" << std::setprecision(1) << avg_deg
                  << "\n";
    }

    auto sr = compute_stats(v_reg);
    auto sc = compute_stats(v_cov);
    auto st = compute_stats(v_time);
    auto sv = compute_stats(v_vol);
    auto si = compute_stats(v_islands);
    auto se = compute_stats(v_edges);
    std::cout << "\n  ── " << p.name << " ──\n"
              << "    Boxes:    med=" << std::setprecision(0) << sr.median
              << "  mean=" << std::setprecision(1) << sr.mean
              << "  [" << sr.q25 << "," << sr.q75 << "]\n"
              << "    Volume:   med=" << std::scientific << std::setprecision(3) << sv.median
              << "  mean=" << sv.mean << std::fixed << "\n"
              << "    VolCov:   med=" << std::setprecision(4) << sc.median << "%"
              << "  mean=" << sc.mean << "%"
              << "  [" << sc.q25 << "," << sc.q75 << "]\n"
              << "    Islands:  med=" << std::setprecision(0) << si.median
              << "  mean=" << std::setprecision(1) << si.mean << "\n"
              << "    Edges:    med=" << std::setprecision(0) << se.median
              << "  mean=" << std::setprecision(1) << se.mean << "\n"
              << "    Time:     med=" << std::setprecision(3) << st.median << "s"
              << "  mean=" << st.mean << "s\n";
}

// ═══════════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    std::string output_dir = std::string(SBF_RESULT_DIR);
    int n_seeds = 10;
    int n_mc = 100000;
    int n_threads = static_cast<int>(std::thread::hardware_concurrency());
    bool quick = false;
    bool lect_cold = false;
    std::string preset_name = "all";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i+1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--mc" && i+1 < argc) n_mc = std::atoi(argv[++i]);
        else if (a == "--preset" && i+1 < argc) preset_name = argv[++i];
        else if (a == "--threads" && i+1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--output-dir" && i+1 < argc) output_dir = argv[++i];
        else if (a == "--cold") lect_cold = true;
        else if (a == "--quick") quick = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_seeds = 3; n_mc = 50000; }
    if (n_threads < 1) n_threads = 1;

    // ── Setup output directory & log files ──────────────────────────────
    ensure_dir(output_dir);
    std::string ts = make_timestamp();
    std::string run_tag = "exp1_" + ts + "_" + preset_name + "_s" + std::to_string(n_seeds);
    std::string run_dir = output_dir + "/" + run_tag;
    ensure_dir(run_dir);

    // Tee stdout → console + file
    std::string stdout_path = run_dir + "/stdout.log";
    std::string stderr_path = run_dir + "/stderr.log";
    std::ofstream stdout_file(stdout_path);

    // Save original streambufs
    std::streambuf* orig_cout = std::cout.rdbuf();

    // Create tee buffer: write to both file and original console
    TeeBuf tee_cout(orig_cout, stdout_file.rdbuf());
    std::cout.rdbuf(&tee_cout);

    // Tee C-level stderr → both console AND stderr.log via pipe+thread
    start_stderr_tee(stderr_path);

    fprintf(stderr, "[exp1] output_dir: %s\n", run_dir.c_str());
    fprintf(stderr, "[exp1] stdout → %s\n", stdout_path.c_str());
    fprintf(stderr, "[exp1] stderr → %s\n", stderr_path.c_str());
    fflush(stderr);

    Robot robot = Robot::from_json(robot_path);

    // Override with v4 planning joint limits (narrowed to Marcucci scene coverage)
    // Original IIWA14 limits are [-2.97, 2.97] etc — far too wide.
    // These v4 limits reduce C-space volume by ~3000x, drastically cutting
    // CritSample combinatorial enumeration (kπ/2 candidates per joint).
    {
        auto& lim = const_cast<JointLimits&>(robot.joint_limits());
        const double planning_limits[7][2] = {
            {-1.865488,  1.865691},   // j0  width=3.73
            {-0.100000,  1.086648},   // j1  width=1.19
            {-0.662656,  0.662338},   // j2  width=1.33
            {-2.094400, -0.371673},   // j3  width=1.72
            {-0.619251,  0.619534},   // j4  width=1.24
            {-1.095222,  1.257951},   // j5  width=2.35
            { 1.050209,  2.091190},   // j6  width=1.04
        };
        for (int j = 0; j < 7; ++j) {
            lim.limits[j].lo = planning_limits[j][0];
            lim.limits[j].hi = planning_limits[j][1];
        }
    }

    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "  active_links=" << robot.n_active_links() << "\n";
    std::cout << "  Joint limits (v4 planning):";
    for (int j = 0; j < robot.n_joints(); ++j)
        std::cout << " [" << std::setprecision(3)
                  << robot.joint_limits().limits[j].lo << ","
                  << robot.joint_limits().limits[j].hi << "]";
    std::cout << "\n";

    auto obstacles = make_combined_obstacles();
    std::cout << "  Scene: combined  (" << obstacles.size() << " obstacles)\n"
              << "  seeds=" << n_seeds << "  MC=" << n_mc
              << "  threads=" << n_threads
              << "  preset=" << preset_name
              << "  lect_cache=" << (lect_cold ? "OFF" : "ON") << "\n";
    std::cout << std::fixed;

    // Collect presets
    std::vector<Preset> presets;
    if (preset_name == "all") {
        presets = {preset_A(), preset_B(), preset_C(), preset_D()};
    } else if (preset_name == "coarsen") {
        presets = {preset_E(), preset_F()};
    } else if (preset_name == "optimal") {
        presets = {preset_G(), preset_H(), preset_I(), preset_J()};
    } else if (preset_name == "crit") {
        presets = {preset_I(), preset_J()};
    } else if (preset_name == "full") {
        presets = {preset_A(), preset_B(), preset_C(), preset_G(), preset_H(),
                   preset_E(), preset_F()};
    } else if (preset_name == "A") presets = {preset_A()};
    else if (preset_name == "B") presets = {preset_B()};
    else if (preset_name == "C") presets = {preset_C()};
    else if (preset_name == "D") presets = {preset_D()};
    else if (preset_name == "E") presets = {preset_E()};
    else if (preset_name == "F") presets = {preset_F()};
    else if (preset_name == "G") presets = {preset_G()};
    else if (preset_name == "H") presets = {preset_H()};
    else if (preset_name == "I") presets = {preset_I()};
    else if (preset_name == "J") presets = {preset_J()};
    else {
        std::cerr << "Unknown preset: " << preset_name << "\n";
        return 1;
    }

    for (auto& p : presets) {
        run_preset(p, robot, obstacles, n_seeds, n_mc, n_threads, lect_cold);
    }

    if (presets.size() > 1) {
        std::cout << "\n╔════════════════════════════════════════════════════════╗\n"
                  << "║  SWEEP COMPLETE — " << presets.size() << " presets done"
                  << std::string(33 - std::to_string(presets.size()).size(), ' ') << "║\n"
                  << "╚════════════════════════════════════════════════════════╝\n";
    }

    std::cout << "\n  Exp 1 complete.\n";
    std::cout << "  Results saved to: " << run_dir << "\n";

    // Restore streams
    std::cout.rdbuf(orig_cout);
    stdout_file.close();
    stop_stderr_tee();

    return 0;
}
