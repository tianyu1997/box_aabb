// SafeBoxForest v6 — 2D Planar Build Trace Experiment
//
// 3 hardcoded obstacle scenes (simple/narrow/cluttered)
// Multithreaded construction with detailed TRACE logging
// Output: log/2d_trace_<scene>_<timestamp>.log

#include <sbf/planner/sbf_planner.h>
#include <sbf/forest/connectivity.h>
#include <sbf/core/robot.h>
#include <sbf/core/log.h>
#include <sbf/scene/collision_checker.h>
#include <sbf/ffb/ffb.h>
#include <sbf/lect/lect.h>
#include "marcucci_scenes.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <random>
#include <string>
#include <sys/stat.h>
#include <vector>

using namespace sbf;

// ─── Scene definitions (inline, matching pbf5_bench/scenes.py) ────────────

struct Scene2D {
    std::string name;
    std::vector<Obstacle> obstacles;
    Eigen::VectorXd start_config;
    Eigen::VectorXd goal_config;
    std::array<double, 2> link_lengths;
};

Scene2D make_2dof_simple() {
    return Scene2D{
        "simple",
        {Obstacle(0.f - 0.3f, 1.5f - 0.15f, 0.f - 0.3f,
                  0.f + 0.3f, 1.5f + 0.15f, 0.f + 0.3f)},
        Eigen::Vector2d(0.5, 0.5),
        Eigen::Vector2d(2.0, 1.0),
        {1.0, 1.0}
    };
}

Scene2D make_2dof_narrow() {
    return Scene2D{
        "narrow",
        {Obstacle(0.f - 0.8f, 1.8f - 0.1f, 0.f - 0.8f,
                  0.f + 0.8f, 1.8f + 0.1f, 0.f + 0.8f),
         Obstacle(0.f - 0.8f, 1.2f - 0.1f, 0.f - 0.8f,
                  0.f + 0.8f, 1.2f + 0.1f, 0.f + 0.8f)},
        Eigen::Vector2d(0.5, 0.3),
        Eigen::Vector2d(2.5, 1.5),
        {1.0, 1.0}
    };
}

Scene2D make_2dof_cluttered() {
    return Scene2D{
        "cluttered",
        {Obstacle(0.f - 0.15f, 1.5f - 0.15f, 0.f - 0.15f,
                  0.f + 0.15f, 1.5f + 0.15f, 0.f + 0.15f),
         Obstacle(-0.8f - 0.12f, 1.0f - 0.12f, 0.f - 0.12f,
                  -0.8f + 0.12f, 1.0f + 0.12f, 0.f + 0.12f)},
        Eigen::Vector2d(0.5, 0.3),
        Eigen::Vector2d(2.5, 1.5),
        {1.0, 1.0}
    };
}

// 6 obstacles: narrow-passage barriers (identical to 'narrow' scene) +
// 4 decorative-only obstacles beyond the arm's max reach (arm max = 2.0m).
// Out-of-reach obstacles cannot be touched by any arm configuration, so they
// do not change C-space connectivity; they only add visual scatter.
Scene2D make_2dof_scattered() {
    return Scene2D{
        "scattered",
        {
            // 4 scattered obstacles, all centers within 2m of origin.
            Obstacle( 0.9f,-0.45f, -0.25f,  1.5f, 0.15f, 0.25f),   // center≈(1.2,-0.15)
            Obstacle(-1.4f,-1.0f,  -0.25f, -0.8f,-0.4f,  0.25f),   // center≈(-1.1,-0.7)
            Obstacle(-0.6f, 1.5f,  -0.25f,  0.6f, 1.9f,  0.25f),   // center≈(0,1.7)
            Obstacle( 1.2f, 0.8f,  -0.25f,  1.8f, 1.4f,  0.25f),   // center≈(1.5,1.1)
        },
        Eigen::Vector2d(0.3, 0.15),
        Eigen::Vector2d(2.5, 1.5),
        {1.0, 1.0}
    };
}

Scene2D make_2dof_random(int n_obs, double center_radius, uint64_t seed) {
    Scene2D scene;
    scene.name = "random";
    scene.link_lengths = {1.0, 1.0};
    scene.start_config = Eigen::Vector2d(0.5, 0.5);
    scene.goal_config = Eigen::Vector2d(2.0, 1.0);

    n_obs = std::max(0, n_obs);
    center_radius = std::max(0.05, center_radius);
    std::mt19937_64 rng(seed == 0 ? 0xA11CE5EEDULL : seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    // Keep obstacle half-sizes compatible with sampling radius so a
    // non-origin-crossing placement is always likely.
    const double max_h = std::min(0.20, std::max(0.02, center_radius * 0.45));
    const double min_h = std::max(0.01, std::min(0.06, max_h * 0.5));
    std::uniform_real_distribution<double> usx(min_h, max_h);
    std::uniform_real_distribution<double> usy(min_h, max_h);

    auto covers_origin_xy = [](double x0, double x1, double y0, double y1) {
        return (x0 <= 0.0 && 0.0 <= x1 && y0 <= 0.0 && 0.0 <= y1);
    };

    scene.obstacles.reserve(static_cast<size_t>(n_obs));
    for (int i = 0; i < n_obs; ++i) {
        bool placed = false;
        for (int k = 0; k < 512; ++k) {
            double r = center_radius * std::sqrt(u01(rng));
            double th = 2.0 * M_PI * u01(rng);
            double cx = r * std::cos(th);
            double cy = r * std::sin(th);
            double hx = usx(rng);
            double hy = usy(rng);
            double x0 = cx - hx;
            double x1 = cx + hx;
            double y0 = cy - hy;
            double y1 = cy + hy;
            if (covers_origin_xy(x0, x1, y0, y1)) continue;

            scene.obstacles.emplace_back(
                static_cast<float>(x0), static_cast<float>(y0), -0.25f,
                static_cast<float>(x1), static_cast<float>(y1),  0.25f);
            placed = true;
            break;
        }
        if (!placed) {
            // Fallback: force place near boundary in a deterministic angle,
            // shifted away from origin so the box cannot cover (0,0).
            double th = 2.0 * M_PI * (static_cast<double>(i) + 0.5)
                        / std::max(1, n_obs);
            double hx = min_h;
            double hy = min_h;
            double safe = std::max(center_radius, 0.05) + std::max(hx, hy);
            double cx = safe * std::cos(th);
            double cy = safe * std::sin(th);
            scene.obstacles.emplace_back(
                static_cast<float>(cx - hx), static_cast<float>(cy - hy), -0.25f,
                static_cast<float>(cx + hx), static_cast<float>(cy + hy),  0.25f);
        }
    }
    return scene;
}

// ─── main ───────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    // ── Logging setup ──
    sbf::set_log_level(sbf::LogLevel::TRACE);

    auto print_usage = []() {
        std::fprintf(stderr,
            "Usage: exp_2d_trace [--scene simple|narrow|cluttered|scattered|random] [--max-boxes N] [--threads N]\n"
            "                    [--endpoint ifk|critsample] [--envelope linkiaabb|linkiaabb_grid|hull16_grid]\\n"
            "                    [--max-depth N] [--vol-bonus-alpha A] [--ffb-fail-limit N]\n"
            "                    [--random-obstacles N] [--obs-center-max-radius R] [--scene-seed S] [--random-start-goal]\n"
            "\\n"
            "Notes:\\n"
            "  - Grower mode is fixed to RRT for replay consistency.\\n"
            "  - Endpoint source is restricted to IFK/CritSample.\\n"
            "  - Envelope type is restricted to LinkIAABB/LinkIAABB_Grid/Hull16_Grid.\\n");
    };

    // Parse CLI args first to determine log file
    std::string scene_name = "simple";
    int max_boxes = 40;
    int n_threads = 1;
    int max_depth = 100;
    int ffb_fail_limit = 200;  // consecutive FFB-failure abort threshold
    double vol_bonus_alpha = 0.05;  // RRT nearest-box volume bonus coefficient
    int random_obstacles = 6;
    double obs_center_max_radius = 2.0;
    uint64_t scene_seed = 0;
    bool random_start_goal = false;
    std::string endpoint_name = "ifk";
    std::string envelope_name = "linkiaabb";
    
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            print_usage();
            return 0;
        }
        if (a == "--scene" && i + 1 < argc) {
            scene_name = argv[++i];
        } else if (a == "--max-boxes" && i + 1 < argc) {
            max_boxes = std::atoi(argv[++i]);
        } else if (a == "--threads" && i + 1 < argc) {
            n_threads = std::atoi(argv[++i]);
        } else if (a == "--max-depth" && i + 1 < argc) {
            max_depth = std::atoi(argv[++i]);
        } else if (a == "--ffb-fail-limit" && i + 1 < argc) {
            ffb_fail_limit = std::atoi(argv[++i]);
        } else if (a == "--vol-bonus-alpha" && i + 1 < argc) {
            vol_bonus_alpha = std::atof(argv[++i]);
        } else if (a == "--random-obstacles" && i + 1 < argc) {
            random_obstacles = std::atoi(argv[++i]);
        } else if (a == "--obs-center-max-radius" && i + 1 < argc) {
            obs_center_max_radius = std::atof(argv[++i]);
        } else if (a == "--scene-seed" && i + 1 < argc) {
            scene_seed = static_cast<uint64_t>(std::strtoull(argv[++i], nullptr, 10));
        } else if (a == "--random-start-goal") {
            random_start_goal = true;
        } else if (a == "--endpoint" && i + 1 < argc) {
            endpoint_name = argv[++i];
        } else if (a == "--envelope" && i + 1 < argc) {
            envelope_name = argv[++i];
        }
    }

    auto to_lower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return s;
    };
    endpoint_name = to_lower(endpoint_name);
    envelope_name = to_lower(envelope_name);

    if (max_boxes <= 0) {
        std::fprintf(stderr, "[exp_2d_trace] ERROR: --max-boxes must be > 0\\n");
        return 2;
    }
    if (n_threads <= 0) {
        std::fprintf(stderr, "[exp_2d_trace] ERROR: --threads must be > 0\\n");
        return 2;
    }
    if (max_depth <= 0) {
        std::fprintf(stderr, "[exp_2d_trace] ERROR: --max-depth must be > 0\\n");
        return 2;
    }
    if (obs_center_max_radius <= 0.0) {
        std::fprintf(stderr, "[exp_2d_trace] ERROR: --obs-center-max-radius must be > 0\\n");
        return 2;
    }

    EndpointSource endpoint_source = EndpointSource::IFK;
    if (endpoint_name == "ifk") {
        endpoint_source = EndpointSource::IFK;
    } else if (endpoint_name == "critsample" || endpoint_name == "crit") {
        endpoint_source = EndpointSource::CritSample;
        endpoint_name = "critsample";
    } else {
        std::fprintf(stderr,
                     "[exp_2d_trace] WARNING: unknown --endpoint=%s, fallback to ifk\n",
                     endpoint_name.c_str());
        endpoint_source = EndpointSource::IFK;
        endpoint_name = "ifk";
    }

    EnvelopeType envelope_type = EnvelopeType::LinkIAABB;
    if (envelope_name == "linkiaabb") {
        envelope_type = EnvelopeType::LinkIAABB;
    } else if (envelope_name == "linkiaabb_grid" || envelope_name == "grid") {
        envelope_type = EnvelopeType::LinkIAABB_Grid;
        envelope_name = "linkiaabb_grid";
    } else if (envelope_name == "hull16_grid" || envelope_name == "hull16") {
        envelope_type = EnvelopeType::Hull16_Grid;
        envelope_name = "hull16_grid";
    } else {
        std::fprintf(stderr,
                     "[exp_2d_trace] WARNING: unknown --envelope=%s, fallback to linkiaabb\n",
                     envelope_name.c_str());
        envelope_type = EnvelopeType::LinkIAABB;
        envelope_name = "linkiaabb";
    }

    // Set log file (check env var, or use default with timestamp)
    if (const char* f = std::getenv("SBF_LOG_FILE"); f && *f) {
        sbf::set_log_file(f);
    } else {
        std::time_t t = std::time(nullptr);
        std::tm tm = {};
        localtime_r(&t, &tm);
        char ts[32], path[1024];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
        
        // Ensure log dir exists
        const char* log_dir = SBF_LOG_DIR;
        ::mkdir(log_dir, 0755);
        
        std::snprintf(path, sizeof(path), "%s/2d_trace_%s_%s_%s_%s.log",
                      log_dir, scene_name.c_str(), endpoint_name.c_str(),
                      envelope_name.c_str(), ts);
        sbf::set_log_file(path);
        std::fprintf(stderr, "[exp_2d_trace] log -> %s\n", path);
    }

    // ── Load robot ──
    std::string robot_path = std::string(SBF_DATA_DIR) + "/2dof_planar.json";
    Robot robot;
    try {
        robot = Robot::from_json(robot_path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[exp_2d_trace] ERROR loading robot: %s\n", e.what());
        return 1;
    }

    // ── Select scene ──
    Scene2D scene;
    if (scene_name == "narrow") {
        scene = make_2dof_narrow();
    } else if (scene_name == "cluttered") {
        scene = make_2dof_cluttered();
    } else if (scene_name == "scattered") {
        scene = make_2dof_scattered();
    } else if (scene_name == "random") {
        scene = make_2dof_random(random_obstacles, obs_center_max_radius,
                                 scene_seed);
    } else {
        scene = make_2dof_simple();
    }

    if (random_start_goal) {
        std::mt19937_64 seed_rng(scene_seed == 0 ? 0xBADC0DEULL : (scene_seed ^ 0x9E3779B97F4A7C15ULL));
        std::uniform_real_distribution<double> uq(-M_PI, M_PI);
        scene.start_config = Eigen::Vector2d(uq(seed_rng), uq(seed_rng));
        scene.goal_config = Eigen::Vector2d(uq(seed_rng), uq(seed_rng));
    }

    // ── Emit metadata header ──
    std::string obstacles_str = "[";
    for (size_t k = 0; k < scene.obstacles.size(); ++k) {
        const auto& obs = scene.obstacles[k];
        if (k > 0) obstacles_str += ",";
        char buf[256];
        std::snprintf(buf, sizeof(buf), "(%.3f,%.3f,%.3f,%.3f)",
                      obs.bounds[0], obs.bounds[1], obs.bounds[3], obs.bounds[4]);
        obstacles_str += buf;
    }
    obstacles_str += "]";

    char limits_str[256];
    std::snprintf(limits_str, sizeof(limits_str),
                  "[(-%.4f,%.4f),(-%.4f,%.4f)]",
                  M_PI, M_PI, M_PI, M_PI);

    SBF_INFO("[2D-META] scene=%s dof=%d robot=%s n_threads=%d max_boxes=%d max_depth=%d "
             "grower=RRT endpoint=%s envelope=%s "
             "limits=%s obstacles=%s start=[%.3f,%.3f] goal=[%.3f,%.3f] "
             "scene_seed=%llu random_obs=%d obs_center_max_radius=%.3f random_start_goal=%d "
             "link_lengths=[%.3f,%.3f]",
             scene_name.c_str(), 2, robot.name().c_str(), n_threads, max_boxes, max_depth,
             endpoint_source_name(endpoint_source), envelope_type_name(envelope_type),
             limits_str, obstacles_str.c_str(),
             scene.start_config[0], scene.start_config[1],
             scene.goal_config[0], scene.goal_config[1],
             static_cast<unsigned long long>(scene_seed), random_obstacles,
             obs_center_max_radius, random_start_goal ? 1 : 0,
             scene.link_lengths[0], scene.link_lengths[1]);

    // ── Configure planner ──
    SBFPlannerConfig cfg;
    cfg.z4_enabled = true;
    cfg.split_order = SplitOrder::BEST_TIGHTEN;
    cfg.lect_no_cache = true;  // probe FFB uses a fresh LECT; force planner to match.

    cfg.grower.mode = GrowerConfig::Mode::RRT;
    cfg.endpoint_source.source = endpoint_source;
    cfg.envelope_type.type = envelope_type;
    cfg.grower.max_boxes = max_boxes;
    cfg.grower.timeout_ms = 30000.0;
    cfg.grower.n_threads = n_threads;
    cfg.grower.bridge_n_threads = 1;
    cfg.grower.rng_seed = 0;
    cfg.grower.max_consecutive_miss = ffb_fail_limit;
    cfg.grower.vol_bonus_alpha = vol_bonus_alpha;
    cfg.grower.rrt_goal_bias = 0.1;
    cfg.grower.rrt_step_ratio = 0.05;
    cfg.grower.connect_mode = true;  // Enable bridge
    cfg.grower.enable_promotion = false;
    cfg.grower.ffb_config.max_depth = max_depth;

    cfg.coarsen.target_boxes = max_boxes;
    cfg.coarsen.max_rounds = 0;

    CollisionChecker checker(robot, {});
    checker.set_obstacles(scene.obstacles.data(),
                          static_cast<int>(scene.obstacles.size()));

    // ── Validate start/goal: must be collision-free AND FFB-succeed at max_depth.
    // If the hardcoded value fails, randomly resample within joint limits until
    // both predicates hold. This prevents the run from spending its whole budget
    // on dead-end seeds that FFB cannot expand at the configured depth.
    auto ffb_succeeds = [&](const Eigen::VectorXd& seed) {
        std::vector<Interval> root_iv;
        root_iv.reserve(robot.joint_limits().limits.size());
        for (const auto& lim : robot.joint_limits().limits) root_iv.push_back(lim);
        LECT probe_lect(robot, root_iv,
                        cfg.endpoint_source, cfg.envelope_type,
                        /*initial_cap=*/127);
        FFBConfig probe_cfg = cfg.grower.ffb_config;
        probe_cfg.max_depth = max_depth;
        probe_cfg.deadline_ms = 0.0;
        auto res = find_free_box(probe_lect, seed,
                                 scene.obstacles.data(),
                                 static_cast<int>(scene.obstacles.size()),
                                 probe_cfg);
        return res.success();
    };

    auto ensure_seed_valid = [&](Eigen::VectorXd& seed, const char* label) -> bool {
        bool collide = checker.check_config(seed);
        bool ffb_ok = !collide && ffb_succeeds(seed);
        if (!collide && ffb_ok) {
            return true;
        }
        std::fprintf(stderr,
                     "[exp_2d_trace] %s [%.3f, %.3f] invalid (collide=%d, ffb_ok=%d) "
                     "at max_depth=%d; resampling...\n",
                     label, seed[0], seed[1], collide ? 1 : 0, ffb_ok ? 1 : 0,
                     max_depth);
        const auto& limits = robot.joint_limits().limits;
        const int nd = static_cast<int>(limits.size());
        std::mt19937_64 rng(0xC0FFEEu ^ static_cast<uint64_t>(
            std::hash<std::string>{}(std::string(label) + scene_name)));
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        const int kMaxAttempts = (scene_name == "random") ? 20000 : 2000;
        Eigen::VectorXd cand(nd);
        for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
            for (int d = 0; d < nd; ++d) {
                cand[d] = limits[d].lo + u01(rng) * limits[d].width();
            }
            if (checker.check_config(cand)) continue;
            if (!ffb_succeeds(cand)) continue;
            seed = cand;
            std::fprintf(stderr,
                         "[exp_2d_trace] %s regenerated to [%.3f, %.3f] after %d attempts.\n",
                         label, seed[0], seed[1], attempt);
            SBF_INFO("[2D-META-REGEN] %s [%.3f,%.3f] attempts=%d max_depth=%d",
                     label, seed[0], seed[1], attempt, max_depth);
            return true;
        }
        std::fprintf(stderr,
                     "[exp_2d_trace] ERROR: failed to find a valid %s after %d attempts "
                     "(collision-free + FFB success at max_depth=%d).\n",
                     label, kMaxAttempts, max_depth);
        return false;
    };

    if (!ensure_seed_valid(scene.start_config, "start")) return 2;
    if (!ensure_seed_valid(scene.goal_config, "goal")) return 2;

    // ── Build ──
    SBFPlanner planner(robot, cfg);
    std::vector<Eigen::VectorXd> seed_points = {
        scene.start_config, scene.goal_config
    };

    auto t0 = std::chrono::steady_clock::now();
    planner.build_coverage(scene.obstacles.data(), scene.obstacles.size(),
                           cfg.grower.timeout_ms, seed_points);
    double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

    // ── Collect results ──
    int n_boxes = planner.n_boxes();
    const auto& adj = planner.adjacency();
    int n_edges = 0;
    for (const auto& kv : adj) {
        n_edges += static_cast<int>(kv.second.size());
    }
    n_edges /= 2;
    auto islands = find_islands(adj);

    SBF_INFO("[2D-DONE] elapsed=%.2fs boxes=%d edges=%d islands=%zu",
             elapsed_s, n_boxes, n_edges, islands.size());

    std::fprintf(stderr, "[exp_2d_trace] Build complete: %d boxes, %.2f seconds.\n",
                 n_boxes, elapsed_s);
    std::fprintf(stderr, "[exp_2d_trace] Check log for trace replay.\n");

    return 0;
}
