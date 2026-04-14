// SafeBoxForest v6 — SBF Planner: query
#include <sbf/planner/sbf_planner.h>
#include <sbf/planner/path_extract.h>
#include <sbf/planner/path_smoother.h>
#include <sbf/forest/connectivity.h>
#include <sbf/core/ray_aabb.h>
#include <sbf/ffb/ffb.h>
#include <sbf/lect/lect_io.h>

#include <chrono>
#include <atomic>
#include <future>
#include <limits>
#include <random>
#include <unordered_set>
#include <sbf/core/log.h>

namespace sbf {

PlanResult SBFPlanner::query(const Eigen::VectorXd& start,
                              const Eigen::VectorXd& goal,
                              const Obstacle* obs, int n_obs)
{
    PlanResult result;
    if (!built_ || boxes_.empty()) return result;

    auto t0 = std::chrono::steady_clock::now();
    auto t_stage = t0;  // D7: sub-stage timing
    auto ms_since = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    double dt_findbox=0, dt_bridge=0, dt_island=0, dt_dijext=0, dt_presimp=0;
    double dt_rrtcomp=0, dt_validate=0, dt_greedy=0, dt_shortcut=0;
    double dt_densify=0, dt_eb=0, dt_finalsc=0, dt_pass2=0;

    // Use provided obstacles or fall back to stored ones
    const Obstacle* use_obs = obs;
    int use_n_obs = n_obs;
    if (!use_obs && !stored_obs_.empty()) {
        use_obs = stored_obs_.data();
        use_n_obs = static_cast<int>(stored_obs_.size());
    }

    CollisionChecker checker(robot_, {});
    if (use_obs && use_n_obs > 0)
        checker.set_obstacles(use_obs, use_n_obs);

    // Find containing boxes (with nearest-box fallback)
    int start_id = -1, goal_id = -1;
    double best_s_d = std::numeric_limits<double>::max();
    double best_g_d = std::numeric_limits<double>::max();
    for (const auto& b : boxes_) {
        if (b.contains(start)) { start_id = b.id; best_s_d = -1.0; }
        if (b.contains(goal))  { goal_id  = b.id; best_g_d = -1.0; }
        if (best_s_d >= 0.0) {
            double d = (b.center() - start).squaredNorm();
            if (d < best_s_d) { best_s_d = d; start_id = b.id; }
        }
        if (best_g_d >= 0.0) {
            double d = (b.center() - goal).squaredNorm();
            if (d < best_g_d) { best_g_d = d; goal_id = b.id; }
        }
    }
    if (start_id < 0 || goal_id < 0) return result;

    { auto now = std::chrono::steady_clock::now(); dt_findbox = ms_since(t_stage, now); t_stage = now; }

    SBF_INFO("[QRY] query: start_id=%d goal_id=%d n_boxes=%d", start_id, goal_id, (int)boxes_.size());

    // ── Bridge: try to connect S/G boxes to the main graph ──
    if (use_obs && lect_) {
        int next_id = 0;
        for (const auto& b : boxes_) next_id = std::max(next_id, b.id + 1);

        auto ffb_cfg = config_.grower.ffb_config;
        ffb_cfg.max_depth = std::max(ffb_cfg.max_depth, 60);

        int created = bridge_s_t(
            start_id, goal_id, boxes_, *lect_, use_obs, use_n_obs,
            adj_, ffb_cfg, next_id,
            robot_, checker,
            /*per_pair_timeout_ms=*/3000.0, /*max_pairs=*/5,
            std::chrono::steady_clock::time_point::max());
        if (created > 0) {
            SBF_INFO("[QRY] bridge: added %d boxes", created);
            // Re-find S/G boxes (bridge may have added better ones)
            for (const auto& b : boxes_) {
                if (b.contains(start)) { start_id = b.id; }
                if (b.contains(goal))  { goal_id  = b.id; }
            }
        }
    }

    { auto now = std::chrono::steady_clock::now(); dt_bridge = ms_since(t_stage, now); t_stage = now; }

    // ── Island check + Proxy mechanism ──
    bool start_isolated = false, goal_isolated = false;
    int proxy_start_id = start_id, proxy_goal_id = goal_id;
    std::vector<Eigen::VectorXd> start_link_path, goal_link_path;

    {
        auto islands = find_islands(adj_);
        int largest_idx = 0;
        for (int i = 1; i < static_cast<int>(islands.size()); ++i) {
            if (islands[i].size() > islands[largest_idx].size())
                largest_idx = i;
        }
        std::unordered_set<int> largest_set(islands[largest_idx].begin(),
                                             islands[largest_idx].end());
        start_isolated = (largest_set.find(start_id) == largest_set.end());
        goal_isolated  = (largest_set.find(goal_id)  == largest_set.end());

        SBF_INFO("[QRY] islands=%d largest=%d start_in=%d goal_in=%d", (int)islands.size(), (int)largest_set.size(), !start_isolated, !goal_isolated);

        if (start_isolated || goal_isolated) {
            // Build box map for proxy search
            std::unordered_map<int, const BoxNode*> box_map_iso;
            for (const auto& b : boxes_) box_map_iso[b.id] = &b;

            const auto& pcfg = config_.proxy;

            auto find_proxy = [&](const Eigen::VectorXd& q, int orig_id,
                                  std::vector<Eigen::VectorXd>& link_path,
                                  const char* label) -> int {
                struct Cand { int id; double dist; };
                std::vector<Cand> candidates;
                for (int lid : islands[largest_idx]) {
                    auto it = box_map_iso.find(lid);
                    if (it == box_map_iso.end()) continue;
                    double d = (q - it->second->center()).squaredNorm();
                    candidates.push_back({lid, d});
                }
                std::sort(candidates.begin(), candidates.end(),
                    [](const Cand& a, const Cand& b) { return a.dist < b.dist; });
                if ((int)candidates.size() > pcfg.max_candidates)
                    candidates.resize(pcfg.max_candidates);

                auto proxy_t0 = std::chrono::steady_clock::now();

                for (int ci = 0; ci < (int)candidates.size(); ++ci) {
                    // Check total budget
                    auto now = std::chrono::steady_clock::now();
                    double elapsed_ms = std::chrono::duration<double, std::milli>(now - proxy_t0).count();
                    if (elapsed_ms > pcfg.total_budget_ms) {
                        SBF_INFO("[QRY] proxy: %s budget exhausted after %d candidates (%.0fms)", label, ci, elapsed_ms);
                        break;
                    }

                    // Tiered timeout
                    double timeout_ms;
                    if (ci < pcfg.tier1_count)
                        timeout_ms = pcfg.tier1_timeout_ms;   // Tier 1: fast probe
                    else if (ci < pcfg.tier1_count + pcfg.tier2_count)
                        timeout_ms = pcfg.tier2_timeout_ms;   // Tier 2: medium
                    else
                        timeout_ms = pcfg.tier3_timeout_ms;   // Tier 3: full

                    const auto& c = candidates[ci];
                    auto it = box_map_iso.find(c.id);
                    if (it == box_map_iso.end()) continue;
                    Eigen::VectorXd target = it->second->center();

                    RRTConnectConfig rrt_cfg;
                    rrt_cfg.timeout_ms = timeout_ms;
                    rrt_cfg.max_iters = pcfg.rrt_max_iters;
                    rrt_cfg.segment_resolution = pcfg.rrt_segment_res;

                    auto rrt_path = rrt_connect(q, target, checker, robot_, rrt_cfg, c.id);
                    if (!rrt_path.empty()) {
                        link_path = std::move(rrt_path);
                        auto done = std::chrono::steady_clock::now();
                        double total_ms = std::chrono::duration<double, std::milli>(done - proxy_t0).count();
                        SBF_INFO("[QRY] proxy: %s box %d -> proxy %d (dist=%.3f, rrt=%dwp, try=%d, %.0fms)", label, orig_id, c.id, std::sqrt(c.dist), (int)link_path.size(), ci+1, total_ms);
                        return c.id;
                    }
                }
                return orig_id;  // failed
            };

            if (start_isolated)
                proxy_start_id = find_proxy(start, start_id, start_link_path, "start");
            if (goal_isolated)
                proxy_goal_id = find_proxy(goal, goal_id, goal_link_path, "goal");
        }
    }

    { auto now = std::chrono::steady_clock::now(); dt_island = ms_since(t_stage, now); t_stage = now; }

    // ── Search (using proxy IDs if isolated) ──
    std::vector<Eigen::VectorXd> path;
    std::vector<int> box_seq;

    if (config_.use_gcs) {
        // Use proxy endpoints when start/goal are isolated
        Eigen::VectorXd gcs_start = start;
        Eigen::VectorXd gcs_goal = goal;
        if (start_isolated && !start_link_path.empty())
            gcs_start = start_link_path.back();
        if (goal_isolated && !goal_link_path.empty())
            gcs_goal = goal_link_path.back();

#ifdef SBF_HAS_DRAKE
        auto gcs_res = gcs_plan(adj_, boxes_, gcs_start, gcs_goal, config_.gcs, &checker, lect_.get());
#else
        auto gcs_res = gcs_plan_fallback(adj_, boxes_, gcs_start, gcs_goal);
#endif

        if (gcs_res.found) {
            path = std::move(gcs_res.path);
            box_seq = std::move(gcs_res.box_sequence);
        } else {
            return result;
        }

        // Prepend/append link paths for isolated start/goal
        if (!start_link_path.empty()) {
            std::vector<Eigen::VectorXd> merged;
            for (size_t i = 0; i < start_link_path.size() - 1; ++i)
                merged.push_back(start_link_path[i]);
            for (auto& p : path) merged.push_back(std::move(p));
            path = std::move(merged);
        }
        if (!goal_link_path.empty()) {
            for (int i = static_cast<int>(goal_link_path.size()) - 2; i >= 0; --i)
                path.push_back(goal_link_path[i]);
        }
        if (!path.empty()) {
            path.front() = start;
            path.back() = goal;
        }
    } else {
        auto dij = dijkstra_search(adj_, boxes_, proxy_start_id, proxy_goal_id);
        if (!dij.found) {
            SBF_WARN("[QRY] dijkstra FAIL: proxy_s=%d proxy_g=%d — trying RRT direct", proxy_start_id, proxy_goal_id);
            // Direct RRT fallback
            RRTConnectConfig rrt_dij;
            rrt_dij.timeout_ms = 3000.0;
            rrt_dij.max_iters  = 200000;
            rrt_dij.segment_resolution = 20;
            auto direct = rrt_connect(start, goal, checker, robot_, rrt_dij);
            if (!direct.empty()) {
                path = std::move(direct);
                start_link_path.clear();
                goal_link_path.clear();
                SBF_WARN("[QRY] RRT direct (dij-fail): OK (%d pts)", (int)path.size());
            } else {
                return result;
            }
        }

        // Box-chain extraction if dijkstra succeeded
        if (path.empty() && dij.found) {
            box_seq = shortcut_box_sequence(dij.box_sequence, adj_);

            auto it_ps = std::find_if(boxes_.begin(), boxes_.end(),
                [&](const BoxNode& b) { return b.id == proxy_start_id; });
            auto it_pg = std::find_if(boxes_.begin(), boxes_.end(),
                [&](const BoxNode& b) { return b.id == proxy_goal_id; });
            Eigen::VectorXd ws = (start_isolated && it_ps != boxes_.end())
                                    ? it_ps->center() : start;
            Eigen::VectorXd wg = (goal_isolated && it_pg != boxes_.end())
                                    ? it_pg->center() : goal;
            path = extract_waypoints(box_seq, boxes_, ws, wg);

            if (path.empty()) {
                SBF_WARN("[QRY] extract_waypoints FAIL — trying RRT fallback");
                // RRT fallback for extract failure
                RRTConnectConfig rrt_fb;
                rrt_fb.timeout_ms = 3000.0;
                rrt_fb.max_iters  = 200000;
                rrt_fb.segment_resolution = 20;

                if (start_isolated && goal_isolated) {
                    auto mid = rrt_connect(ws, wg, checker, robot_, rrt_fb);
                    if (!mid.empty()) {
                        path.clear();
                        for (auto& p : start_link_path) path.push_back(p);
                        for (size_t i = 1; i + 1 < mid.size(); ++i) path.push_back(mid[i]);
                        for (int i = (int)goal_link_path.size() - 1; i >= 0; --i)
                            path.push_back(goal_link_path[i]);
                        path.front() = start;
                        path.back()  = goal;
                        start_link_path.clear();
                        goal_link_path.clear();
                        SBF_INFO("[QRY] RRT fallback (both proxy): OK (%d pts)", (int)path.size());
                    }
                } else if (start_isolated) {
                    auto mid = rrt_connect(ws, goal, checker, robot_, rrt_fb);
                    if (!mid.empty()) {
                        path.clear();
                        for (auto& p : start_link_path) path.push_back(p);
                        for (size_t i = 1; i < mid.size(); ++i) path.push_back(mid[i]);
                        path.front() = start;
                        start_link_path.clear();
                        SBF_INFO("[QRY] RRT fallback (start proxy): OK (%d pts)", (int)path.size());
                    }
                } else if (goal_isolated) {
                    auto mid = rrt_connect(start, wg, checker, robot_, rrt_fb);
                    if (!mid.empty()) {
                        path.clear();
                        for (auto& p : mid) path.push_back(p);
                        for (int i = (int)goal_link_path.size() - 1; i >= 0; --i)
                            path.push_back(goal_link_path[i]);
                        path.back() = goal;
                        goal_link_path.clear();
                        SBF_INFO("[QRY] RRT fallback (goal proxy): OK (%d pts)", (int)path.size());
                    }
                } else {
                    auto direct = rrt_connect(start, goal, checker, robot_, rrt_fb);
                    if (!direct.empty()) {
                        path = std::move(direct);
                        SBF_INFO("[QRY] RRT direct fallback: OK (%d pts)", (int)path.size());
                    }
                }

                if (path.empty()) {
                    // Last resort: direct start→goal RRT
                    auto direct = rrt_connect(start, goal, checker, robot_, rrt_fb);
                    if (!direct.empty()) {
                        path = std::move(direct);
                        start_link_path.clear();
                        goal_link_path.clear();
                        SBF_INFO("[QRY] RRT direct (last resort): OK (%d pts)", (int)path.size());
                    } else {
                        SBF_WARN("[QRY] all fallbacks FAIL");
                        return result;
                    }
                }
            }
        }

        // Prepend start_link_path
        if (!start_link_path.empty()) {
            std::vector<Eigen::VectorXd> merged;
            for (size_t i = 0; i < start_link_path.size() - 1; ++i)
                merged.push_back(start_link_path[i]);
            for (auto& p : path)
                merged.push_back(std::move(p));
            path = std::move(merged);
        }

        // Append goal_link_path
        if (!goal_link_path.empty()) {
            for (int i = static_cast<int>(goal_link_path.size()) - 2; i >= 0; --i)
                path.push_back(goal_link_path[i]);
        }

        // Ensure path starts at start and ends at goal
        if (!path.empty()) {
            path.front() = start;
            path.back() = goal;
        }
    }

    if (path.empty()) return result;

    { auto now = std::chrono::steady_clock::now(); dt_dijext = ms_since(t_stage, now); t_stage = now; }

    // ── Path composition diagnostics ──
    {
        double raw_len = sbf::path_length(path);
        int raw_pts = (int)path.size();
        int n_start_link = (int)start_link_path.size();
        int n_goal_link  = (int)goal_link_path.size();
        int n_box_chain  = (int)box_seq.size();
        SBF_INFO("[QRY] raw path: %d pts, len=%.3f " "(start_link=%d, box_chain=%d boxes, goal_link=%d)", raw_pts, raw_len, n_start_link, n_box_chain, n_goal_link);
    }

    // ── Build expanded corridor for ray-AABB fast-path ──
    // Collect box_seq boxes + 1-hop adjacency neighbors to form corridor.
    // segment_in_box_union() can then skip expensive FK collision checks
    // when a shortcut segment lies entirely within the corridor union.
    std::vector<BoxNode> corridor_boxes;
    std::vector<int> corridor_ids;
    std::unordered_map<int, int> corridor_id_to_idx;
    {
        std::unordered_set<int> cset(box_seq.begin(), box_seq.end());
        // Expand by 1-hop adjacency
        for (int bid : box_seq) {
            auto it = adj_.find(bid);
            if (it != adj_.end())
                for (int nbr : it->second) cset.insert(nbr);
        }
        // Build lookup: box id -> index in boxes_
        std::unordered_map<int, int> global_id_to_idx;
        for (int i = 0; i < (int)boxes_.size(); ++i)
            global_id_to_idx[boxes_[i].id] = i;
        // Build corridor vectors
        corridor_boxes.reserve(cset.size());
        corridor_ids.reserve(cset.size());
        for (int bid : cset) {
            auto it = global_id_to_idx.find(bid);
            if (it != global_id_to_idx.end()) {
                corridor_ids.push_back(bid);
                corridor_boxes.push_back(boxes_[it->second]);
            }
        }
        // Build id_to_idx for corridor
        for (int i = 0; i < (int)corridor_boxes.size(); ++i)
            corridor_id_to_idx[corridor_boxes[i].id] = i;
        SBF_INFO("[QRY] corridor: %d boxes (seq=%d + neighbors)", (int)corridor_boxes.size(), (int)box_seq.size());
    }

    // Lambda: fast-path check — segment fully inside corridor box union?
    auto corridor_ok = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> bool {
        if (corridor_boxes.empty()) return false;
        return segment_in_box_union(a, b, corridor_boxes, corridor_ids, corridor_id_to_idx);
    };

    // ── RRT direct competition ──
    // Box-chain paths through bridge corridors can be very long and winding.
    // First simplify the chain path, then try a direct RRT start→goal path
    // and keep whichever is shorter.
    if (use_obs && path.size() > 3) {
        // Quick greedy forward simplification of box-chain path
        // (collision-free shortcutting removes redundant intermediate waypoints)
        auto ares_fn = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> int {
            double len = (b - a).norm();
            return std::max(20, static_cast<int>(std::ceil(len / 0.01)));  // C2: relaxed from 0.005
        };
        {
            auto chain_backup = path;  // save original chain in case simplify is invalid
            std::vector<Eigen::VectorXd> simplified;
            // S5-A: Inline config check during simplify — bail out immediately
            // if any chosen waypoint is in collision, avoiding the expensive
            // greedy segment checks for the remaining chain.
            bool simplify_valid = true;
            simplified.push_back(path[0]);
            if (checker.check_config(path[0])) simplify_valid = false;

            size_t cur = 0;
            while (simplify_valid && cur < path.size() - 1) {
                size_t best = cur + 1;
                for (size_t j = path.size() - 1; j > cur + 1; --j) {
                    if (corridor_ok(path[cur], path[j]) ||
                        !checker.check_segment(path[cur], path[j],
                                               ares_fn(path[cur], path[j]))) {
                        best = j;
                        break;
                    }
                }
                simplified.push_back(path[best]);
                // Check config of each selected waypoint immediately
                if (checker.check_config(path[best])) {
                    simplify_valid = false;
                    break;
                }
                cur = best;
            }
            double len_before = sbf::path_length(path);
            double len_after  = simplify_valid ? sbf::path_length(simplified) : len_before;

            if (simplify_valid && len_after < len_before) {
                SBF_INFO("[QRY] pre-compete simplify: %d -> %d pts, len %.3f -> %.3f", (int)chain_backup.size(), (int)simplified.size(), len_before, len_after);
                path = std::move(simplified);
            } else {
                SBF_INFO("[QRY] pre-compete simplify: %d -> %d pts, REVERTED (config invalid or no improvement)", (int)chain_backup.size(), (int)simplified.size());
                path = std::move(chain_backup);
            }
        }

        { auto now = std::chrono::steady_clock::now(); dt_presimp = ms_since(t_stage, now); t_stage = now; }

        double chain_len = sbf::path_length(path);

        // Adaptive RRT budget: shorter chain → less RRT time needed
        double euclid = (goal - start).norm();
        double ratio = chain_len / std::max(euclid, 0.01);

        double rrt_timeout = (ratio < 2.0) ? 1000.0 : (ratio < 3.0) ? 3000.0 : 5000.0;

        SBF_INFO("[QRY] RRT budget: chain=%.3f euclid=%.3f ratio=%.2f → timeout=%.0fms", chain_len, euclid, ratio, rrt_timeout);

        // Helper: greedy simplify an RRT path
        auto simplify_rrt = [&](std::vector<Eigen::VectorXd>& p) {
            std::vector<Eigen::VectorXd> simp;
            simp.push_back(p[0]);
            size_t cur = 0;
            while (cur < p.size() - 1) {
                size_t best = cur + 1;
                for (size_t j = p.size() - 1; j > cur + 1; --j) {
                    if (corridor_ok(p[cur], p[j]) ||
                        !checker.check_segment(p[cur], p[j],
                                               ares_fn(p[cur], p[j]))) {
                        best = j; break;
                    }
                }
                simp.push_back(p[best]);
                cur = best;
            }
            p = std::move(simp);
        };

        // P3: Launch all 3 RRT trials in parallel via std::async
        // CollisionChecker is thread-safe (const methods, no mutable state).
        struct TrialResult {
            std::vector<Eigen::VectorXd> path;
            double length = std::numeric_limits<double>::max();
        };

        auto run_trial = [&](double timeout, int max_iters, int seed) -> TrialResult {
            RRTConnectConfig cfg;
            cfg.timeout_ms = timeout;
            cfg.max_iters  = max_iters;
            cfg.step_size  = 0.2;
            cfg.goal_bias  = 0.15;
            cfg.segment_resolution = 20;
            auto p = rrt_connect(start, goal, checker, robot_, cfg, seed);
            if (p.empty()) return {};
            simplify_rrt(p);
            double len = sbf::path_length(p);
            return {std::move(p), len};
        };

        double bonus_timeout = std::min(rrt_timeout, 1000.0);

        auto f1 = std::async(std::launch::async, run_trial, rrt_timeout, 200000, 42);
        auto f2 = std::async(std::launch::async, run_trial, bonus_timeout, 80000, 179);
        auto f3 = std::async(std::launch::async, run_trial, bonus_timeout, 80000, 316);

        TrialResult r1 = f1.get();
        TrialResult r2 = f2.get();
        TrialResult r3 = f3.get();

        std::vector<Eigen::VectorXd> best_direct;
        double best_direct_len = std::numeric_limits<double>::max();
        int n_success = 0;
        for (auto* r : {&r1, &r2, &r3}) {
            if (!r->path.empty()) {
                ++n_success;
                if (r->length < best_direct_len) {
                    best_direct = std::move(r->path);
                    best_direct_len = r->length;
                }
            }
        }

        if (!best_direct.empty() && best_direct_len < chain_len) {
            SBF_INFO("[QRY] RRT compete: direct=%.3f < chain=%.3f → using direct (%d/3 ok)", best_direct_len, chain_len, n_success);
            path = std::move(best_direct);
            // Keep box_seq for GCS post-optimization (Python script needs corridor backbone)
        } else if (!best_direct.empty()) {
            SBF_INFO("[QRY] RRT compete: direct=%.3f >= chain=%.3f → keeping chain (%d/3 ok)", best_direct_len, chain_len, n_success);
        } else {
            SBF_WARN("[QRY] RRT compete: all FAIL (budget=%.0fms) → keeping chain", rrt_timeout);
        }
    }

    { auto now = std::chrono::steady_clock::now(); dt_rrtcomp = ms_since(t_stage, now); t_stage = now; }

    // Adaptive resolution helper (defined early for validation)
    auto ares = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> int {
        double len = (b - a).norm();
        return std::max(20, static_cast<int>(std::ceil(len / 0.005)));
    };

    // ── Final segment validation + RRT repair ──
    // Detect contiguous "bad" regions (colliding waypoints or segments)
    // and repair each region as a whole via RRT-connect between the
    // nearest clean endpoints.
    //
    // OP-3: pre_validate snapshot moved to AFTER validation so the
    // fallback is always collision-free.

    if (use_obs) {
        std::vector<Eigen::VectorXd> safe_path;
        safe_path.push_back(path[0]);
        size_t i = 0;
        while (i + 1 < path.size()) {
            bool seg_bad = checker.check_segment(path[i], path[i+1],
                                                  ares(path[i], path[i+1]));
            bool next_bad = checker.check_config(path[i+1]);
            if (!seg_bad && !next_bad) {
                // Clean segment — keep it
                safe_path.push_back(path[i+1]);
                ++i;
            } else {
                // Start of a bad region at segment i→i+1
                // Advance until we find a clean waypoint AND the segment
                // from that waypoint onward is viable.
                size_t repair_end = i + 1;
                while (repair_end < path.size()) {
                    if (!checker.check_config(path[repair_end])) break;
                    ++repair_end;
                }
                // repair_end is the first non-colliding waypoint (or path.size())
                if (repair_end >= path.size()) {
                    // All remaining waypoints collide — keep originals
                    for (size_t j = i + 1; j < path.size(); ++j)
                        safe_path.push_back(path[j]);
                    i = path.size();
                    break;
                }
                // RRT from last known-good point to path[repair_end]
                RRTConnectConfig repair_cfg;
                repair_cfg.timeout_ms = 2000.0;
                repair_cfg.max_iters = 50000;
                repair_cfg.segment_resolution = 20;
                auto repair = rrt_connect(safe_path.back(), path[repair_end],
                                          checker, robot_, repair_cfg);
                if (!repair.empty()) {
                    for (size_t j = 1; j < repair.size(); ++j)
                        safe_path.push_back(repair[j]);
                } else {
                    // Fallback — keep the bad waypoints
                    for (size_t j = i + 1; j <= repair_end; ++j)
                        safe_path.push_back(path[j]);
                }
                i = repair_end;
            }
        }
        path = std::move(safe_path);
    }

    { auto now = std::chrono::steady_clock::now(); dt_validate = ms_since(t_stage, now); t_stage = now; }

    // OP-3: Snapshot the post-validation path as fallback.
    // This is collision-repaired, so reverting to it is always safe.
    std::vector<Eigen::VectorXd> pre_validate_path = path;
    double pre_validate_len = sbf::path_length(path);

    // ── Path quality optimization (collision-safe) ──
    // (ares already defined above)

    // Step 1: Greedy forward simplification — connect to the farthest
    //         reachable point, removing all redundant intermediate waypoints.
    if (use_obs && path.size() > 2) {
        std::vector<Eigen::VectorXd> simplified;
        simplified.push_back(path[0]);
        size_t cur = 0;
        while (cur < path.size() - 1) {
            size_t best = cur + 1;
            for (size_t j = path.size() - 1; j > cur + 1; --j) {
                if (corridor_ok(path[cur], path[j]) ||
                    !checker.check_segment(path[cur], path[j],
                                           ares(path[cur], path[j]))) {
                    best = j;
                    break;
                }
            }
            simplified.push_back(path[best]);
            cur = best;
        }
        double len_before = sbf::path_length(path);
        double len_after  = sbf::path_length(simplified);
        SBF_INFO("[OPT] greedy simplify: %d -> %d pts, len %.3f -> %.3f", (int)path.size(), (int)simplified.size(), len_before, len_after);
        path = std::move(simplified);
    }

    { auto now = std::chrono::steady_clock::now(); dt_greedy = ms_since(t_stage, now); t_stage = now; }

    // Step 2: Random shortcutting with adaptive resolution.
    // S4-A: Skip for short paths (≤8 pts) — after greedy simplify
    // these are already near-optimal and random shortcut never improves them.
    if (use_obs && path.size() > 8) {
        int max_iters = std::max(300, 50 * static_cast<int>(path.size()));
        double len_before = sbf::path_length(path);

        if (path.size() > 2) {
            std::mt19937_64 rng(42);
            auto current = path;
            for (int iter = 0; iter < max_iters; ++iter) {
                if (current.size() <= 2) break;
                int n = static_cast<int>(current.size());
                std::uniform_int_distribution<int> di(0, n - 3);
                int i = di(rng);
                std::uniform_int_distribution<int> dj(i + 2, n - 1);
                int j = dj(rng);
                if (corridor_ok(current[i], current[j]) ||
                    !checker.check_segment(current[i], current[j],
                                           ares(current[i], current[j]))) {
                    std::vector<Eigen::VectorXd> shortened;
                    for (int k = 0; k <= i; ++k) shortened.push_back(current[k]);
                    for (int k = j; k < n; ++k) shortened.push_back(current[k]);
                    current = std::move(shortened);
                }
            }
            path = std::move(current);
        }
        double len_after = sbf::path_length(path);
        SBF_INFO("[OPT] random shortcut: len %.3f -> %.3f", len_before, len_after);
    }

    { auto now = std::chrono::steady_clock::now(); dt_shortcut = ms_since(t_stage, now); t_stage = now; }

    // Step 2.5: Densify — subdivide long segments for finer optimization.
    //           Skip any interpolated point that is in collision.
    if (path.size() > 2) {
        const double max_seg = 0.3;
        std::vector<Eigen::VectorXd> dense;
        dense.push_back(path[0]);
        int skipped = 0;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            double seg_len = (path[i+1] - path[i]).norm();
            int n_sub = std::max(1, static_cast<int>(std::ceil(seg_len / max_seg)));
            for (int k = 1; k < n_sub; ++k) {  // interior points only
                double t = static_cast<double>(k) / n_sub;
                Eigen::VectorXd pt = (1.0 - t) * path[i] + t * path[i+1];
                if (use_obs && checker.check_config(pt)) {
                    ++skipped;  // skip collision point
                } else {
                    dense.push_back(pt);
                }
            }
            dense.push_back(path[i+1]);  // always keep original waypoint
        }
        if (dense.size() > path.size()) {
            SBF_INFO("[OPT] densify: %d -> %d pts%s", (int)path.size(), (int)dense.size(), skipped > 0 ? (std::string(" (") + std::to_string(skipped) + " collision pts skipped)").c_str() : "");
            path = std::move(dense);
        }
    }

    { auto now = std::chrono::steady_clock::now(); dt_densify = ms_since(t_stage, now); t_stage = now; }

    // Step 3: Elastic-band optimization — pull each waypoint toward the line
    //         connecting its neighbours, with adaptive-resolution collision check.
    double eb_improvement_pct = 0.0;  // S7-B: track EB improvement for pass2 gating
    if (path.size() > 3 && use_obs) {
        auto pre_eb = path;  // save for collision fallback
        double len_before = sbf::path_length(path);
        int total_moves = 0;
        double eb_len_track = len_before;  // D3: track per-iter improvement
        int stagnant_count = 0;
        int eb_iters_done = 0;  // D3: for logging
        for (int iter = 0; iter < 60; ++iter) {
            bool any_move = false;
            for (size_t i = 1; i + 1 < path.size(); ++i) {
                const Eigen::VectorXd& prev = path[i - 1];
                const Eigen::VectorXd& next = path[i + 1];
                Eigen::VectorXd dir = next - prev;
                double d2 = dir.squaredNorm();
                if (d2 < 1e-12) continue;
                double t = dir.dot(path[i] - prev) / d2;
                t = std::max(0.0, std::min(1.0, t));
                Eigen::VectorXd target = prev + t * dir;

                static const double alphas[] = {0.5, 0.3, 0.15, 0.05};
                for (double alpha : alphas) {
                    Eigen::VectorXd cand = (1.0 - alpha) * path[i] + alpha * target;
                    int rp = ares(prev, cand);
                    int rn = ares(cand, next);
                    if (!checker.check_config(cand) &&
                        !checker.check_segment(prev, cand, rp) &&
                        !checker.check_segment(cand, next, rn)) {
                        double old_cost = (path[i] - prev).norm() + (next - path[i]).norm();
                        double new_cost = (cand - prev).norm() + (next - cand).norm();
                        if (new_cost < old_cost - 1e-8) {
                            path[i] = cand;
                            any_move = true;
                            ++total_moves;
                        }
                        break;
                    }
                }
            }
            eb_iters_done = iter + 1;
            if (!any_move) break;

            // D3: Early termination when improvement stagnates
            if (iter >= 8) {
                double eb_len_cur = sbf::path_length(path);
                double improvement = (eb_len_track - eb_len_cur) / std::max(eb_len_track, 1e-6);
                if (improvement < 0.001) {  // < 0.1% improvement this iteration
                    ++stagnant_count;
                    if (stagnant_count >= 2) {
                        eb_iters_done = iter + 1;
                        break;
                    }
                } else {
                    stagnant_count = 0;
                }
                eb_len_track = eb_len_cur;
            }
        }
        double len_after = sbf::path_length(path);
        eb_improvement_pct = (len_before > 1e-6) ? (len_before - len_after) / len_before : 0.0;
        SBF_INFO("[OPT] elastic band: %d moves (%d iters), len %.3f -> %.3f", total_moves, eb_iters_done, len_before, len_after);

        // Validate elastic band output — per-waypoint revert for collision segments.
        // Only revert individual waypoints that cause collisions with their neighbors,
        // preserving the optimization of non-colliding segments.
        int eb_reverts = 0;
        for (int pass = 0; pass < 3; ++pass) {  // multi-pass: reverting one can fix neighbor
            bool any_revert = false;
            for (size_t i = 1; i + 1 < path.size(); ++i) {
                bool left_ok  = !checker.check_segment(path[i-1], path[i],
                                                       ares(path[i-1], path[i]));
                bool right_ok = !checker.check_segment(path[i], path[i+1],
                                                       ares(path[i], path[i+1]));
                bool point_ok = !checker.check_config(path[i]);
                if (!left_ok || !right_ok || !point_ok) {
                    path[i] = pre_eb[i];  // revert only this waypoint
                    any_revert = true;
                    ++eb_reverts;
                }
            }
            if (!any_revert) break;
        }
        if (eb_reverts > 0) {
            SBF_INFO("[OPT] elastic band: %d waypoints reverted (per-segment)", eb_reverts);
        }

        // Final safety net: if per-segment revert still left collisions, full revert
        bool eb_clean = true;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            if (checker.check_config(path[i]) ||
                checker.check_segment(path[i], path[i+1],
                                      ares(path[i], path[i+1]))) {
                eb_clean = false;
                break;
            }
        }
        if (!eb_clean) {
            SBF_INFO("[OPT] elastic band: FULL REVERT (per-segment fix insufficient)");
            path = std::move(pre_eb);
        }
    }

    { auto now = std::chrono::steady_clock::now(); dt_eb = ms_since(t_stage, now); t_stage = now; }

    // Step 4: Final shortcut pass with adaptive resolution.
    // S7-A: Skip for very short paths (≤6 pts) — shortcut hit rate is near zero.
    if (use_obs && path.size() > 6) {
        int max_iters = std::max(300, 30 * static_cast<int>(path.size()));
        double len_before = sbf::path_length(path);
        auto pre_shortcut = path;  // save for fallback
        {
            std::mt19937_64 rng(123);
            auto current = path;
            for (int iter = 0; iter < max_iters; ++iter) {
                if (current.size() <= 2) break;
                int n = static_cast<int>(current.size());
                std::uniform_int_distribution<int> di(0, n - 3);
                int i = di(rng);
                std::uniform_int_distribution<int> dj(i + 2, n - 1);
                int j = dj(rng);
                if (corridor_ok(current[i], current[j]) ||
                    !checker.check_segment(current[i], current[j],
                                           ares(current[i], current[j]))) {
                    std::vector<Eigen::VectorXd> shortened;
                    for (int k = 0; k <= i; ++k) shortened.push_back(current[k]);
                    for (int k = j; k < n; ++k) shortened.push_back(current[k]);
                    current = std::move(shortened);
                }
            }
            path = std::move(current);
        }
        // Validate full path — revert if shortcut introduced collision
        bool valid = true;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            if (checker.check_segment(path[i], path[i+1],
                                      ares(path[i], path[i+1]))) {
                valid = false;
                break;
            }
        }
        if (!valid) {
            path = std::move(pre_shortcut);
            SBF_INFO("[OPT] final shortcut: REVERTED (collision)");
        } else {
            double len_after = sbf::path_length(path);
            if (len_after < len_before - 1e-6)
                SBF_INFO("[OPT] final shortcut: len %.3f -> %.3f", len_before, len_after);
        }
    }

    { auto now = std::chrono::steady_clock::now(); dt_finalsc = ms_since(t_stage, now); t_stage = now; }

    // Step 5: Second optimization pass — light densify + elastic band on
    //         the already-short path for extra smoothing (cheap: few points).
    // S7-B: Skip when Step 3 EB improvement was < 5% — further smoothing
    // yields negligible benefit for paths already near-optimal.
    if (use_obs && path.size() >= 3 && eb_improvement_pct >= 0.05) {
        auto pre_pass2 = path;  // save for collision revert
        // Light densify: only add midpoints for segments > 0.4 rad
        {
            std::vector<Eigen::VectorXd> dense2;
            dense2.push_back(path[0]);
            for (size_t i = 1; i < path.size(); ++i) {
                double seg = (path[i] - path[i-1]).norm();
                if (seg > 0.4) {
                    int nsub = std::min(3, static_cast<int>(std::ceil(seg / 0.3)));
                    for (int s = 1; s < nsub; ++s) {
                        double t = static_cast<double>(s) / nsub;
                        dense2.push_back(path[i-1] + t * (path[i] - path[i-1]));
                    }
                }
                dense2.push_back(path[i]);
            }
            if (dense2.size() > path.size()) path = std::move(dense2);
        }
        // Mini elastic band (30 iterations)
        {
            double len_before2 = sbf::path_length(path);
            int moves2 = 0;
            for (int iter = 0; iter < 30; ++iter) {
                bool any_move = false;
                for (size_t i = 1; i + 1 < path.size(); ++i) {
                    const auto& prev = path[i-1];
                    const auto& next = path[i+1];
                    Eigen::VectorXd dir = next - prev;
                    double d2 = dir.squaredNorm();
                    if (d2 < 1e-12) continue;
                    double t = dir.dot(path[i] - prev) / d2;
                    t = std::max(0.0, std::min(1.0, t));
                    Eigen::VectorXd target = prev + t * dir;

                    static const double alphas2[] = {0.4, 0.2, 0.08};
                    for (double alpha : alphas2) {
                        Eigen::VectorXd cand = (1.0 - alpha) * path[i] + alpha * target;
                        int rp = ares(prev, cand);
                        int rn = ares(cand, next);
                        if (!checker.check_config(cand) &&
                            !checker.check_segment(prev, cand, rp) &&
                            !checker.check_segment(cand, next, rn)) {
                            double old_cost = (path[i] - prev).norm() + (next - path[i]).norm();
                            double new_cost = (cand - prev).norm() + (next - cand).norm();
                            if (new_cost < old_cost - 1e-8) {
                                path[i] = cand;
                                any_move = true;
                                ++moves2;
                            }
                            break;
                        }
                    }
                }
                if (!any_move) break;
            }
            double len_after2 = sbf::path_length(path);
            if (moves2 > 0)
                SBF_INFO("[OPT] pass2 EB: %d moves, len %.3f -> %.3f", moves2, len_before2, len_after2);
        }
        // Final greedy simplify to remove redundant points
        {
            auto pre_simplify2 = path;
            std::vector<Eigen::VectorXd> simplified;
            simplified.push_back(path.front());
            size_t anchor = 0;
            while (anchor < path.size() - 1) {
                size_t farthest = anchor + 1;
                for (size_t j = anchor + 2; j < path.size(); ++j) {
                    if (corridor_ok(path[anchor], path[j]) ||
                        !checker.check_segment(path[anchor], path[j],
                                               ares(path[anchor], path[j])))
                        farthest = j;
                    else break;
                }
                simplified.push_back(path[farthest]);
                anchor = farthest;
            }
            // Validate simplified path
            bool simplify2_ok = true;
            for (size_t i = 0; i + 1 < simplified.size(); ++i) {
                if (checker.check_config(simplified[i]) ||
                    checker.check_segment(simplified[i], simplified[i+1],
                                          ares(simplified[i], simplified[i+1]))) {
                    simplify2_ok = false;
                    break;
                }
            }
            if (simplify2_ok)
                path = std::move(simplified);
            else
                path = std::move(pre_simplify2);
        }
        // pass2 collision revert: if pass2 introduced collision, revert entirely
        {
            bool pass2_ok = true;
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                if (checker.check_config(path[i]) ||
                    checker.check_segment(path[i], path[i+1],
                                          ares(path[i], path[i+1]))) {
                    pass2_ok = false;
                    break;
                }
            }
            if (!pass2_ok) {
                SBF_INFO("[OPT] pass2: REVERTED (collision)");
                path = std::move(pre_pass2);
            }
        }
    }

    { auto now = std::chrono::steady_clock::now(); dt_pass2 = ms_since(t_stage, now); t_stage = now; }

    // ── Final safety-net: whole-path collision validation ──────────────
    if (use_obs && path.size() > 1) {
        bool final_ok = true;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            if (checker.check_config(path[i]) ||
                checker.check_segment(path[i], path[i+1],
                                      ares(path[i], path[i+1]))) {
                final_ok = false;
                break;
            }
        }
        if (!final_ok) {
            // Fall back to start-goal straight line via RRT (emergency)
            SBF_INFO("[OPT] FINAL SAFETY NET: path still has collision, " "attempting emergency RRT");
            RRTConnectConfig emrrt;
            emrrt.timeout_ms = 5000.0;
            emrrt.max_iters = 200000;
            emrrt.step_size = 0.15;
            emrrt.goal_bias = 0.15;
            emrrt.segment_resolution = 20;
            auto empath = rrt_connect(path.front(), path.back(),
                                      checker, robot_, emrrt, 9999);
            if (!empath.empty()) {
                // Quick optimization of emergency RRT path
                // 1. Greedy simplify
                {
                    std::vector<Eigen::VectorXd> em_simp;
                    em_simp.push_back(empath[0]);
                    size_t anchor = 0;
                    while (anchor < empath.size() - 1) {
                        size_t farthest = anchor + 1;
                        for (size_t j = empath.size() - 1; j > anchor + 1; --j) {
                            if (corridor_ok(empath[anchor], empath[j]) ||
                                !checker.check_segment(empath[anchor], empath[j],
                                                       ares(empath[anchor], empath[j]))) {
                                farthest = j;
                                break;
                            }
                        }
                        em_simp.push_back(empath[farthest]);
                        anchor = farthest;
                    }
                    empath = std::move(em_simp);
                }
                // 2. Densify
                {
                    std::vector<Eigen::VectorXd> em_dense;
                    em_dense.push_back(empath[0]);
                    for (size_t i = 0; i + 1 < empath.size(); ++i) {
                        double sl = (empath[i+1] - empath[i]).norm();
                        int ns = std::max(1, static_cast<int>(std::ceil(sl / 0.3)));
                        for (int k = 1; k < ns; ++k) {
                            double t = static_cast<double>(k) / ns;
                            Eigen::VectorXd pt = (1.0 - t) * empath[i] + t * empath[i+1];
                            if (!checker.check_config(pt))
                                em_dense.push_back(pt);
                        }
                        em_dense.push_back(empath[i+1]);
                    }
                    empath = std::move(em_dense);
                }
                // 3. Mini elastic band (30 iter)
                for (int iter = 0; iter < 30; ++iter) {
                    bool any = false;
                    for (size_t i = 1; i + 1 < empath.size(); ++i) {
                        const auto& prev = empath[i-1];
                        const auto& next = empath[i+1];
                        Eigen::VectorXd dir = next - prev;
                        double d2 = dir.squaredNorm();
                        if (d2 < 1e-12) continue;
                        double t = dir.dot(empath[i] - prev) / d2;
                        t = std::max(0.0, std::min(1.0, t));
                        Eigen::VectorXd target = prev + t * dir;
                        static const double aa[] = {0.5, 0.3, 0.15};
                        for (double alpha : aa) {
                            Eigen::VectorXd cand = (1.0 - alpha) * empath[i] + alpha * target;
                            if (!checker.check_config(cand) &&
                                !checker.check_segment(prev, cand, ares(prev, cand)) &&
                                !checker.check_segment(cand, next, ares(cand, next))) {
                                double oc = (empath[i]-prev).norm() + (next-empath[i]).norm();
                                double nc = (cand-prev).norm() + (next-cand).norm();
                                if (nc < oc - 1e-8) { empath[i] = cand; any = true; }
                                break;
                            }
                        }
                    }
                    if (!any) break;
                }
                // 4. Final greedy simplify
                {
                    std::vector<Eigen::VectorXd> em_final;
                    em_final.push_back(empath[0]);
                    size_t anchor = 0;
                    while (anchor < empath.size() - 1) {
                        size_t far = anchor + 1;
                        for (size_t j = empath.size()-1; j > anchor+1; --j) {
                            if (!checker.check_segment(empath[anchor], empath[j],
                                                       ares(empath[anchor], empath[j]))) {
                                far = j; break;
                            }
                        }
                        em_final.push_back(empath[far]);
                        anchor = far;
                    }
                    empath = std::move(em_final);
                }
                path = std::move(empath);
                SBF_INFO("[OPT] emergency RRT: %d pts, len=%.3f", (int)path.size(), sbf::path_length(path));
            }
        }
    }

    // OP-3: If optimization inflated the path beyond 1.5× the pre-validate
    // length, fall back to the pre-validate version (which was either the
    // chain path or direct RRT, both known-shorter).
    {
        double final_len = sbf::path_length(path);
        if (final_len > pre_validate_len * 1.5 && pre_validate_len > 1e-6) {
            SBF_INFO("[OPT] path regression (%.3f > 1.5×%.3f) → reverting to pre-validate", final_len, pre_validate_len);
            path = std::move(pre_validate_path);
        }
    }

    result.success = true;
    result.path = std::move(path);
    result.box_sequence = std::move(box_seq);
    result.path_length = sbf::path_length(result.path);
    result.n_boxes = static_cast<int>(boxes_.size());

    auto t1 = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    // D7: Sub-stage timing summary
    double dt_safety = ms_since(t_stage, t1);
    SBF_INFO("[QRY-T] find=%.1fms bridge=%.1fms island=%.1fms dij+ext=%.1fms pre_simp=%.1fms" " rrt_compete=%.1fms validate=%.1fms greedy=%.1fms shortcut=%.1fms\n" " densify=%.1fms eb=%.1fms final_sc=%.1fms pass2=%.1fms safety=%.1fms total=%.1fms\n", dt_findbox, dt_bridge, dt_island, dt_dijext, dt_presimp, dt_rrtcomp, dt_validate, dt_greedy, dt_shortcut, dt_densify, dt_eb, dt_finalsc, dt_pass2, dt_safety, ms_since(t0, t1));

    return result;
}

}  // namespace sbf
