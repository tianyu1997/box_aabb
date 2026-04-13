/**
 * exp_box_connect.cpp — Box-Only 5-Tree Connection Experiment
 *
 * Plants 5 trees at the Marcucci poststone configurations (AS, TS, CS, LB, RB)
 * and grows them in parallel (one RRT thread per tree) with goal_bias pointing
 * at the other trees' roots.  connect_mode checks cross-tree box adjacency
 * after parallel merge and reports when all 5 form a single component.
 *
 * Usage:
 *   ./exp_box_connect [--seeds N] [--mc N] [--max-boxes N] [--threads N]
 *                     [--timeout MS] [--output-dir DIR]
 */

#include <sbf/forest/grower.h>
#include <sbf/forest/adjacency.h>
#include <sbf/forest/coarsen.h>
#include <sbf/forest/connectivity.h>
#include <sbf/lect/lect.h>
#include <sbf/lect/lect_io.h>
#include <sbf/lect/lect_cache_manager.h>
#include <sbf/core/robot.h>
#include <sbf/scene/collision_checker.h>
#include "marcucci_scenes.h"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
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
// Helpers
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

/// Tee-stream: writes to both a file and an existing ostream.
class TeeBuf : public std::streambuf {
public:
    TeeBuf(std::streambuf* sb1, std::streambuf* sb2)
        : sb1_(sb1), sb2_(sb2) {}
protected:
    int overflow(int c) override {
        if (c == EOF) return !EOF;
        if (sb1_->sputc(c) == EOF) return EOF;
        if (sb2_->sputc(c) == EOF) return EOF;
        if (c == '\n') { sb1_->pubsync(); sb2_->pubsync(); }
        return c;
    }
    int sync() override { sb1_->pubsync(); sb2_->pubsync(); return 0; }
private:
    std::streambuf *sb1_, *sb2_;
};

static int g_stderr_pipe_read_fd = -1;
static int g_stderr_saved_fd = -1;
static std::thread g_stderr_thread;

static void start_stderr_tee(const std::string& path) {
    int pipefd[2];
    if (::pipe(pipefd) != 0) return;
    g_stderr_saved_fd = ::dup(STDERR_FILENO);
    ::dup2(pipefd[1], STDERR_FILENO);
    ::close(pipefd[1]);
    g_stderr_pipe_read_fd = pipefd[0];
    g_stderr_thread = std::thread([read_fd = pipefd[0],
                                   console_fd = g_stderr_saved_fd, path]() {
        FILE* logf = ::fopen(path.c_str(), "w");
        char buf[4096];
        ssize_t n;
        while ((n = ::read(read_fd, buf, sizeof(buf))) > 0) {
            ::write(console_fd, buf, n);
            if (logf) { ::fwrite(buf, 1, n, logf); ::fflush(logf); }
        }
        ::close(read_fd);
        if (logf) ::fclose(logf);
    });
}

static void stop_stderr_tee() {
    if (g_stderr_saved_fd < 0) return;
    ::fflush(stderr);
    ::dup2(g_stderr_saved_fd, STDERR_FILENO);
    ::close(g_stderr_saved_fd);
    g_stderr_saved_fd = -1;
    if (g_stderr_thread.joinable()) g_stderr_thread.join();
}

// ═══════════════════════════════════════════════════════════════════════════
// Coverage estimation (copied from exp1)
// ═══════════════════════════════════════════════════════════════════════════

struct CoverageResult {
    double vol_coverage;
    double valid_rate;
    double vol_total;
    double v_free_est;
};

CoverageResult estimate_coverage(
    const Robot& robot, const CollisionChecker& checker,
    const std::vector<BoxNode>& boxes,
    int n_mc_free, int n_mc_validate, int mc_seed)
{
    std::mt19937 rng(mc_seed);
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    const int ndim = robot.n_joints();
    const auto& jl = robot.joint_limits();

    int n_free = 0;
    Eigen::VectorXd q(ndim);
    for (int s = 0; s < n_mc_free; ++s) {
        for (int d = 0; d < ndim; ++d)
            q[d] = jl.limits[d].lo + unif(rng) * jl.limits[d].width();
        if (!checker.check_config(q)) n_free++;
    }

    double v_total = 1.0;
    for (int d = 0; d < ndim; ++d)
        v_total *= jl.limits[d].width();
    double free_frac = n_mc_free > 0 ? double(n_free) / n_mc_free : 0.5;
    double v_free = v_total * free_frac;

    double vol_sum = 0.0;
    for (auto& b : boxes) vol_sum += b.volume;
    double vol_cov = v_free > 0 ? vol_sum / v_free : 0.0;

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

    return {vol_cov, valid_rate, vol_sum, v_free};
}

// ═══════════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    std::string output_dir = std::string(SBF_RESULT_DIR);
    int n_seeds = 5;
    int n_mc = 50000;
    int max_boxes = 200000;
    double timeout_ms = 600000.0;  // 600s = 10min
    int n_threads = 5;  // one thread per tree for parallel RRT
    int ffb_depth = 300;
    double goal_bias = 0.5;
    double step_ratio = 0.05;
    int max_miss = 2000;
    bool do_coarsen = true;           // Phase C: post-growth coarsening
    double coarsen_score = 5.0;       // greedy merge score threshold
    int coarsen_rounds = 50;          // greedy max rounds
    int fill_boxes = 0;               // Phase B: extra boxes after connect (0=skip)

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i+1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--mc" && i+1 < argc) n_mc = std::atoi(argv[++i]);
        else if (a == "--max-boxes" && i+1 < argc) max_boxes = std::atoi(argv[++i]);
        else if (a == "--timeout" && i+1 < argc) timeout_ms = std::atof(argv[++i]);
        else if (a == "--threads" && i+1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--output-dir" && i+1 < argc) output_dir = argv[++i];
        else if (a == "--ffb-depth" && i+1 < argc) ffb_depth = std::atoi(argv[++i]);
        else if (a == "--goal-bias" && i+1 < argc) goal_bias = std::atof(argv[++i]);
        else if (a == "--step-ratio" && i+1 < argc) step_ratio = std::atof(argv[++i]);
        else if (a == "--max-miss" && i+1 < argc) max_miss = std::atoi(argv[++i]);
        else if (a == "--no-coarsen") do_coarsen = false;
        else if (a == "--coarsen-score" && i+1 < argc) coarsen_score = std::atof(argv[++i]);
        else if (a == "--coarsen-rounds" && i+1 < argc) coarsen_rounds = std::atoi(argv[++i]);
        else if (a == "--fill-boxes" && i+1 < argc) fill_boxes = std::atoi(argv[++i]);
        else if (a[0] != '-') robot_path = a;
    }

    // ── Setup output ────────────────────────────────────────────────────
    ensure_dir(output_dir);
    std::string ts = make_timestamp();
    std::string run_dir = output_dir + "/box_connect_" + ts;
    ensure_dir(run_dir);

    std::string stdout_path = run_dir + "/stdout.log";
    std::string stderr_path = run_dir + "/stderr.log";
    std::ofstream stdout_file(stdout_path);
    std::streambuf* orig_cout = std::cout.rdbuf();
    TeeBuf tee_cout(orig_cout, stdout_file.rdbuf());
    std::cout.rdbuf(&tee_cout);
    start_stderr_tee(stderr_path);

    fprintf(stderr, "[box_connect] output_dir: %s\n", run_dir.c_str());
    fflush(stderr);

    // ── Load robot + narrow limits ──────────────────────────────────────
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
    const int n_obs = static_cast<int>(obstacles.size());
    const Obstacle* obs = obstacles.data();

    // ── 5 Poststone seeds (Marcucci query endpoints) ────────────────────
    std::vector<Eigen::VectorXd> seeds = {
        config_AS(), config_TS(), config_CS(), config_LB(), config_RB()
    };
    const char* seed_names[] = {"AS", "TS", "CS", "LB", "RB"};

    std::cout << "╔════════════════════════════════════════════════════════╗\n"
              << "║  Box-Only 5-Tree Connection Experiment                ║\n"
              << "╚════════════════════════════════════════════════════════╝\n"
              << "  Robot: " << robot.name() << "  DOF=" << robot.n_joints() << "\n"
              << "  Seeds: ";
    for (int i = 0; i < 5; i++)
        std::cout << seed_names[i] << (i < 4 ? ", " : "\n");
    std::cout << "  max_boxes=" << max_boxes
              << "  timeout=" << timeout_ms << "ms"
              << "  n_seeds=" << n_seeds
              << "  MC=" << n_mc << "\n"
              << "  ffb_depth=" << ffb_depth
              << "  goal_bias=" << goal_bias
              << "  step_ratio=" << step_ratio
              << "  max_miss=" << max_miss << "\n"
              << "  coarsen=" << (do_coarsen ? "ON" : "OFF")
              << "  coarsen_score=" << coarsen_score
              << "  fill_boxes=" << fill_boxes << "\n\n";

    // ── Configuration ────────────────────────────────────────────────────
    // Envelope: CritSample + Hull16_Grid
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::CritSample;
    ep_cfg.n_samples_crit = 64;

    EnvelopeTypeConfig env_cfg;
    env_cfg.type = EnvelopeType::Hull16_Grid;
    env_cfg.n_subdivisions = 1;
    env_cfg.grid_config.voxel_delta = 0.04;

    // (wavefront stages not used in RRT mode)

    // ── Run seeds ────────────────────────────────────────────────────────
    std::vector<double> v_time, v_boxes, v_connected, v_ctime, v_cboxes, v_vol, v_cov;

    for (int s = 0; s < n_seeds; s++) {
        uint64_t rng_seed = static_cast<uint64_t>(s);
        fprintf(stderr, "\n═══════════ seed %d ═══════════\n", s);

        // 1. Build LECT
        auto t0 = std::chrono::steady_clock::now();
        auto root_ivs = robot.joint_limits().limits;
        LECT lect(robot, root_ivs, ep_cfg, env_cfg);
        lect.set_split_order(SplitOrder::BEST_TIGHTEN);

        // V6 persistent cache (no .lect file load — OOM fix)
        std::string cache_dir = std::string(getenv("HOME") ? getenv("HOME") : "/tmp") + "/.sbf_cache";
        uint64_t fp = robot.fingerprint();

        LectCacheManager cache_mgr;
        if (cache_mgr.init(fp, robot.name(),
                           lect.n_active_links() * 2 * 6, cache_dir)) {
            lect.set_cache_manager(&cache_mgr);
            fprintf(stderr, "[LECT] V6 cache: EP safe=%d/%d unsafe=%d/%d\n",
                    cache_mgr.ep_cache(0).size(), cache_mgr.ep_cache(0).capacity(),
                    cache_mgr.ep_cache(1).size(), cache_mgr.ep_cache(1).capacity());
        }

        double lect_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        fprintf(stderr, "[LECT] init=%.0fms nodes=%d\n", lect_ms, lect.n_nodes());

        // 2. Configure grower — PARALLEL RRT + CONNECT_MODE
        GrowerConfig gcfg;
        gcfg.mode = GrowerConfig::Mode::RRT;
        gcfg.max_boxes = max_boxes;
        gcfg.timeout_ms = timeout_ms;
        gcfg.max_consecutive_miss = max_miss;
        gcfg.ffb_config.max_depth = ffb_depth;
        gcfg.n_threads = n_threads;        // parallel: one thread per tree
        gcfg.rng_seed = rng_seed;
        gcfg.enable_promotion = true;
        gcfg.connect_mode = true;          // ★ Check cross-tree box adjacency after merge
        gcfg.rrt_goal_bias = goal_bias;
        gcfg.rrt_step_ratio = step_ratio;

        // 3. Grow
        ForestGrower grower(robot, lect, gcfg);
        grower.set_multi_goals(seeds);

        auto t_grow = std::chrono::steady_clock::now();
        auto gr = grower.grow(obs, n_obs);
        double grow_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_grow).count();

        int pre_coarsen_boxes = static_cast<int>(gr.boxes.size());

        // ── Phase B: Fill-growth after connectivity (TODO: needs import_boxes) ──
        // Skipped for now — coarsening alone should be sufficient.

        // ── Phase C: Post-growth Coarsening ─────────────────────────────
        double coarsen_ms = 0.0;
        int coarsen_merges = 0;
        if (do_coarsen && !gr.boxes.empty()) {
            // Pre-coarsen diagnostics
            {
                auto pre_adj = compute_adjacency(gr.boxes, 1e-6);
                auto pre_islands = find_islands(pre_adj);
                fprintf(stderr, "[Coarsen] PRE-coarsen: %d boxes, %d islands (tol=1e-6)\n",
                        (int)gr.boxes.size(), (int)pre_islands.size());
                // Also try wider tolerance
                auto pre_adj2 = compute_adjacency(gr.boxes, 1e-3);
                auto pre_islands2 = find_islands(pre_adj2);
                fprintf(stderr, "[Coarsen] PRE-coarsen: %d boxes, %d islands (tol=1e-3)\n",
                        (int)gr.boxes.size(), (int)pre_islands2.size());
            }

            CollisionChecker coarsen_checker(robot, obstacles);
            auto t_coarsen = std::chrono::steady_clock::now();

            // C1: Exact dimension-sweep merge (fast, no collision check)
            auto cr1 = coarsen_forest(gr.boxes, coarsen_checker, 20);
            fprintf(stderr, "[Coarsen] sweep: %d→%d boxes (%d merges, %d rounds)\n",
                    cr1.boxes_before, cr1.boxes_after,
                    cr1.merges_performed, cr1.rounds);

            // Post-sweep diagnostics
            {
                auto ps_adj = compute_adjacency(gr.boxes, 1e-6);
                auto ps_islands = find_islands(ps_adj);
                fprintf(stderr, "[Coarsen] POST-sweep: %d boxes, %d islands\n",
                        (int)gr.boxes.size(), (int)ps_islands.size());
            }

            // C2: Greedy adjacency merge — protect articulation points!
            {
                auto pre_adj = compute_adjacency(gr.boxes, 1e-6);
                auto bridge_ids = find_articulation_points(pre_adj);
                fprintf(stderr, "[Coarsen] articulation points: %d\n",
                        (int)bridge_ids.size());

                GreedyCoarsenConfig gc;
                gc.target_boxes = 0;
                gc.max_rounds = coarsen_rounds;
                gc.score_threshold = coarsen_score;
                gc.adjacency_tol = 1e-6;
                gc.max_lect_fk_per_round = 20000;
                auto cr2 = coarsen_greedy(gr.boxes, coarsen_checker, gc,
                                          &lect, &bridge_ids);
                fprintf(stderr, "[Coarsen] greedy: %d→%d boxes (%d merges, %d rounds, %.1fs)\n",
                        cr2.boxes_before, cr2.boxes_after,
                        cr2.merges_performed, cr2.rounds, cr2.elapsed_sec);
                coarsen_merges += cr2.merges_performed;
            }

            // C3: Remove fully-contained overlapping boxes — protect bridges!
            // NOTE: Disabled — overlap filter tends to break connectivity
            // even with articulation-point protection (cascading removal).
            // {
            //     auto post_adj = compute_adjacency(gr.boxes);
            //     auto bridge_ids2 = find_articulation_points(post_adj);
            //     int removed = filter_coarsen_overlaps(gr.boxes, 1e-4, &bridge_ids2);
            //     fprintf(stderr, "[Coarsen] overlap filter: removed %d boxes → %d remain\n",
            //             removed, (int)gr.boxes.size());
            //     coarsen_merges += removed;
            // }

            coarsen_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_coarsen).count();
            coarsen_merges += cr1.merges_performed;
        }

        // 4. Compute adjacency + islands (post-grow verification)
        auto adj = compute_adjacency(gr.boxes, 1e-6);
        auto islands = find_islands(adj);
        int n_islands = static_cast<int>(islands.size());
        int n_edges = 0;
        for (auto& kv : adj) n_edges += (int)kv.second.size();
        n_edges /= 2;

        // Per-tree box count
        std::unordered_map<int, int> tree_counts;
        for (auto& b : gr.boxes) tree_counts[b.root_id]++;

        double total_wall = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        // 5. MC coverage
        CollisionChecker checker(robot, obstacles);
        auto cov = estimate_coverage(robot, checker, gr.boxes,
                                      n_mc, std::min(n_mc, (int)gr.boxes.size() * 10),
                                      s + 1000000);

        // Record
        v_time.push_back(total_wall);
        v_boxes.push_back(gr.boxes.size());
        v_connected.push_back(gr.all_connected ? 1.0 : 0.0);
        v_ctime.push_back(gr.connect_time_ms);
        v_cboxes.push_back(gr.connect_n_boxes);
        v_vol.push_back(gr.total_volume);
        v_cov.push_back(cov.vol_coverage * 100.0);

        // Print per-seed result
        std::cout << "  s" << s
                  << "  connected=" << (gr.all_connected ? "YES" : "NO")
                  << "  connect_time=" << std::fixed << std::setprecision(0)
                  << gr.connect_time_ms << "ms"
                  << "  connect_boxes=" << gr.connect_n_boxes
                  << "  pre_coarsen=" << pre_coarsen_boxes
                  << "  total_boxes=" << gr.boxes.size()
                  << "  islands=" << n_islands
                  << "  edges=" << n_edges
                  << "  vcov=" << std::setprecision(2) << cov.vol_coverage * 100.0 << "%"
                  << "  valid=" << std::setprecision(1) << cov.valid_rate * 100 << "%"
                  << "  grow=" << std::setprecision(1) << grow_ms << "ms"
                  << "  coarsen=" << std::setprecision(1) << coarsen_ms << "ms"
                  << "  total=" << std::setprecision(2) << total_wall << "s"
                  << "\n";

        // Print tree sizes
        std::cout << "    trees:";
        for (int t = 0; t < 5; t++) {
            auto it = tree_counts.find(t);
            std::cout << " " << seed_names[t] << "=" << (it != tree_counts.end() ? it->second : 0);
        }
        std::cout << "\n";

        // Skip .lect save — using V6 persistent cache only
        // if (loaded < lect.n_nodes()) {
        //     lect_save_incremental(lect, cache_path, loaded);
        // }
    }

    // ── Summary ──────────────────────────────────────────────────────────
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n"
              << "║  Summary: Box-Only 5-Tree Connection                  ║\n"
              << "╚════════════════════════════════════════════════════════╝\n";

    int n_ok = 0;
    for (auto v : v_connected) if (v > 0.5) n_ok++;
    std::cout << "  Connected: " << n_ok << "/" << n_seeds << "\n";

    if (!v_ctime.empty()) {
        std::sort(v_ctime.begin(), v_ctime.end());
        std::sort(v_cboxes.begin(), v_cboxes.end());
        std::sort(v_time.begin(), v_time.end());
        std::sort(v_boxes.begin(), v_boxes.end());
        std::sort(v_cov.begin(), v_cov.end());
        int m = n_seeds / 2;
        std::cout << "  Connect time (ms): med=" << std::setprecision(0) << v_ctime[m]
                  << "  min=" << v_ctime[0] << "  max=" << v_ctime.back() << "\n"
                  << "  Connect boxes:     med=" << v_cboxes[m]
                  << "  min=" << v_cboxes[0] << "  max=" << v_cboxes.back() << "\n"
                  << "  Total boxes:       med=" << v_boxes[m]
                  << "  min=" << v_boxes[0] << "  max=" << v_boxes.back() << "\n"
                  << "  Total time (s):    med=" << std::setprecision(2) << v_time[m]
                  << "  min=" << v_time[0] << "  max=" << v_time.back() << "\n"
                  << "  VolCov (%):        med=" << std::setprecision(2) << v_cov[m]
                  << "  min=" << v_cov[0] << "  max=" << v_cov.back() << "\n";
    }

    std::cout << "\n  Results saved to: " << run_dir << "\n";

    std::cout.rdbuf(orig_cout);
    stdout_file.close();
    stop_stderr_tee();
    return 0;
}
