// SafeBoxForest v6 — Connectivity (Phase F7 + RRT-Connect bridge)
#include <sbf/forest/connectivity.h>

#include <algorithm>
#include <atomic>
#include <future>
#include <limits>
#include <random>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Dense>

#include <sbf/forest/thread_pool.h>
#include <sbf/core/log.h>

#ifdef SBF_HAS_OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/util/Console.h>
#include <sbf/core/log.h>
#endif

namespace sbf {

// ─── find_islands ───────────────────────────────────────────────────────────
std::vector<std::vector<int>> find_islands(const AdjacencyGraph& adj) {
    // Collect all box ids
    std::vector<int> all_ids;
    all_ids.reserve(adj.size());
    for (const auto& kv : adj)
        all_ids.push_back(kv.first);

    if (all_ids.empty()) return {};

    // Map id → compact index
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(all_ids.size()); ++i)
        id_to_idx[all_ids[i]] = i;

    int n = static_cast<int>(all_ids.size());
    UnionFind uf(n);

    for (const auto& kv : adj) {
        int u = id_to_idx[kv.first];
        for (int neighbor_id : kv.second) {
            auto it = id_to_idx.find(neighbor_id);
            if (it != id_to_idx.end())
                uf.unite(u, it->second);
        }
    }

    // Group by root
    std::unordered_map<int, std::vector<int>> groups;
    for (int i = 0; i < n; ++i)
        groups[uf.find(i)].push_back(all_ids[i]);

    std::vector<std::vector<int>> islands;
    islands.reserve(groups.size());
    for (auto& kv : groups)
        islands.push_back(std::move(kv.second));

    return islands;
}

// ─── bridge_islands ─────────────────────────────────────────────────────────

// ─── bridge_s_t ─────────────────────────────────────────────────────────────

int bridge_s_t(
        int start_box_id,
        int goal_box_id,
        std::vector<BoxNode>& boxes,
        LECT& lect,
        const Obstacle* obs, int n_obs,
        AdjacencyGraph& adj,
        const FFBConfig& ffb_config,
        int& next_box_id,
        const Robot& robot,
        const CollisionChecker& checker,
        double per_pair_timeout_ms,
        int max_pairs,
        std::chrono::steady_clock::time_point deadline) {

    auto deadline_reached = [&]() {
        return deadline != std::chrono::steady_clock::time_point::max()
               && std::chrono::steady_clock::now() >= deadline;
    };

    // Helper: check if start_box and goal_box are in the same island
    auto s_t_connected = [&]() -> bool {
        auto islands = find_islands(adj);
        for (const auto& isl : islands) {
            bool has_s = false, has_g = false;
            for (int id : isl) {
                if (id == start_box_id) has_s = true;
                if (id == goal_box_id)  has_g = true;
            }
            if (has_s && has_g) return true;
        }
        return false;
    };

    if (s_t_connected()) return 0;

    // Build id → idx map
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
        id_to_idx[boxes[i].id] = i;

    // Collect boxes belonging to S-island and G-island by connectivity
    // (not root_id — start/goal may reside in diversity-root boxes)
    auto islands = find_islands(adj);
    std::vector<int> s_boxes, g_boxes;
    for (const auto& isl : islands) {
        bool has_s = false, has_g = false;
        for (int id : isl) {
            if (id == start_box_id) has_s = true;
            if (id == goal_box_id)  has_g = true;
        }
        if (has_s) s_boxes = isl;
        if (has_g) g_boxes = isl;
    }
    if (s_boxes.empty() || g_boxes.empty()) return 0;

    // Build candidate pairs sorted by distance (best-first)
    struct CandPair {
        int s_id, g_id;
        double dist;
        bool operator<(const CandPair& o) const { return dist < o.dist; }
    };
    std::vector<CandPair> candidates;
    candidates.reserve(std::min((int)s_boxes.size(), 200) *
                       std::min((int)g_boxes.size(), 200));

    for (int sid : s_boxes) {
        auto its = id_to_idx.find(sid);
        if (its == id_to_idx.end()) continue;
        Eigen::VectorXd cs = boxes[its->second].center();
        for (int gid : g_boxes) {
            auto itg = id_to_idx.find(gid);
            if (itg == id_to_idx.end()) continue;
            double d = (cs - boxes[itg->second].center()).squaredNorm();
            candidates.push_back({sid, gid, d});
        }
    }
    std::sort(candidates.begin(), candidates.end());

    // Limit to max_pairs
    if (static_cast<int>(candidates.size()) > max_pairs)
        candidates.resize(max_pairs);

    SBF_INFO("[BRG] bridge_s_t: s_tree=%d boxes, g_tree=%d boxes, " "candidates=%d (max_pairs=%d)", (int)s_boxes.size(), (int)g_boxes.size(), (int)candidates.size(), max_pairs);

    int bridges_created = 0;
    int rrt_seed = 42;

    for (int ci = 0; ci < static_cast<int>(candidates.size()); ++ci) {
        if (deadline_reached()) break;
        if (s_t_connected()) break;

        const auto& cp = candidates[ci];

        // Re-lookup box indices (may have shifted due to new boxes)
        id_to_idx.clear();
        for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
            id_to_idx[boxes[i].id] = i;

        auto its = id_to_idx.find(cp.s_id);
        auto itg = id_to_idx.find(cp.g_id);
        if (its == id_to_idx.end() || itg == id_to_idx.end()) continue;

        Eigen::VectorXd ca = boxes[its->second].center();
        Eigen::VectorXd cb = boxes[itg->second].center();

        RRTConnectConfig pair_rrt;
        pair_rrt.max_iters = 100000;
        pair_rrt.step_size = 0.1;
        pair_rrt.goal_bias = 0.20;
        pair_rrt.timeout_ms = per_pair_timeout_ms;
        pair_rrt.segment_resolution = 10;

        auto path = rrt_connect(ca, cb, checker, robot, pair_rrt, rrt_seed++);
        if (path.empty()) continue;

        // Chain-pave from the S-side box along the RRT path
        int added = chain_pave_along_path(
            path, cp.s_id, boxes, lect, obs, n_obs,
            ffb_config, adj, next_box_id, robot,
            /*max_chain=*/500, /*max_steps_per_wp=*/15);
        bridges_created += added;

        SBF_INFO("[BRG] pair %d/%d: box %d->%d, dist=%.3f, " "rrt=%dwp, paved=%d, connected=%s", ci + 1, (int)candidates.size(), cp.s_id, cp.g_id, std::sqrt(cp.dist), (int)path.size(), added, s_t_connected() ? "YES" : "no");
    }
    return bridges_created;
}

// ─── bridge_all_islands (merge ALL disconnected components) ──────────────────
int bridge_all_islands(
        std::vector<BoxNode>& boxes,
        LECT& lect,
        const Obstacle* obs, int n_obs,
        AdjacencyGraph& adj,
        const FFBConfig& ffb_config,
        int& next_box_id,
        const Robot& robot,
        const CollisionChecker& checker,
        double per_pair_timeout_ms,
        int max_pairs_per_gap,
        int max_total_bridges,
        int n_threads,
        std::chrono::steady_clock::time_point deadline) {

    auto deadline_reached = [&]() {
        return deadline != std::chrono::steady_clock::time_point::max()
               && std::chrono::steady_clock::now() >= deadline;
    };

    int total_bridges = 0;
    int rrt_seed = 1000;

    // ── Thread pool for parallel RRT ───────────────────────────────────────
    const int n_rrt_threads = (n_threads > 0)
        ? n_threads
        : std::max(1, std::min((int)std::thread::hardware_concurrency() - 1, 8));
    ThreadPool rrt_pool(n_rrt_threads);
    SBF_INFO("[BRG-ALL] parallel RRT: %d threads", n_rrt_threads);
    // ── Persistent id_to_idx ───────────────────────────────────────────────
    std::unordered_map<int, int> id_to_idx;
    id_to_idx.reserve(boxes.size() * 2);
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
        id_to_idx[boxes[i].id] = i;

    // ── Persistent UnionFind (initialised from adj) ────────────────────────
    int n = static_cast<int>(boxes.size());
    UnionFind uf(n);
    for (const auto& kv : adj) {
        auto it_u = id_to_idx.find(kv.first);
        if (it_u == id_to_idx.end()) continue;
        for (int nid : kv.second) {
            auto it_v = id_to_idx.find(nid);
            if (it_v != id_to_idx.end())
                uf.unite(it_u->second, it_v->second);
        }
    }

    // Helper: extract islands from UF (O(n), no adj traversal)
    auto get_islands = [&]() {
        std::unordered_map<int, std::vector<int>> groups;
        int sz = uf.size();
        for (int i = 0; i < sz; ++i)
            groups[uf.find(i)].push_back(boxes[i].id);
        std::vector<std::vector<int>> islands;
        islands.reserve(groups.size());
        for (auto& kv : groups)
            islands.push_back(std::move(kv.second));
        return islands;
    };

    // Helper: incrementally absorb newly added boxes into UF + id_to_idx
    auto absorb_new_boxes = [&](int old_n) {
        int new_n = static_cast<int>(boxes.size());
        if (new_n <= old_n) return;
        uf.resize(new_n);
        for (int i = old_n; i < new_n; ++i) {
            id_to_idx[boxes[i].id] = i;
            auto it = adj.find(boxes[i].id);
            if (it != adj.end()) {
                for (int nid : it->second) {
                    auto jt = id_to_idx.find(nid);
                    if (jt != id_to_idx.end())
                        uf.unite(i, jt->second);
                }
            }
        }
    };

    // Outer loop: keep merging until 1 island or budget exhausted
    for (int round = 0; round < 50; ++round) {
        if (deadline_reached()) break;
        if (total_bridges >= max_total_bridges) break;

        auto islands = get_islands();
        if (islands.size() <= 1) {
            SBF_INFO("[BRG-ALL] round %d: single component (%d boxes) — done", round, islands.empty() ? 0 : (int)islands[0].size());
            break;
        }

        // Sort islands by size descending (largest = main component)
        std::sort(islands.begin(), islands.end(),
                  [](const std::vector<int>& a, const std::vector<int>& b) {
                      return a.size() > b.size();
                  });

        SBF_INFO("[BRG-ALL] round %d: %d islands (main=%d", round, (int)islands.size(), (int)islands[0].size()); for (size_t k = 1; k < std::min(islands.size(), (size_t)6); ++k) SBF_INFO(", #%d=%d", (int)k, (int)islands[k].size()); SBF_INFO(")");
        const auto& main_island = islands[0];

        // Pre-compute main island box centers for candidate generation
        // Sample at most 300 boxes from main island to limit O(n*m)
        std::vector<std::pair<int, Eigen::VectorXd>> main_centers;
        {
            int stride = std::max(1, (int)main_island.size() / 300);
            for (int k = 0; k < (int)main_island.size(); k += stride) {
                int mid = main_island[k];
                auto it = id_to_idx.find(mid);
                if (it != id_to_idx.end())
                    main_centers.push_back({mid, boxes[it->second].center()});
            }
        }

        bool merged_any = false;

        // Try to merge each small island into the main component
        for (size_t ii = 1; ii < islands.size(); ++ii) {
            if (deadline_reached()) break;
            if (total_bridges >= max_total_bridges) break;

            const auto& small_island = islands[ii];

            // Build candidate pairs: small × main, sorted by distance
            struct CandPair {
                int small_id, main_id;
                double dist;
                bool operator<(const CandPair& o) const { return dist < o.dist; }
            };

            // Sample at most 200 boxes from small island
            std::vector<CandPair> candidates;
            int s_stride = std::max(1, (int)small_island.size() / 200);
            for (int k = 0; k < (int)small_island.size(); k += s_stride) {
                int sid = small_island[k];
                auto its = id_to_idx.find(sid);
                if (its == id_to_idx.end()) continue;
                Eigen::VectorXd sc = boxes[its->second].center();
                for (const auto& [mid, mc] : main_centers) {
                    double d = (sc - mc).squaredNorm();
                    candidates.push_back({sid, mid, d});
                }
            }
            std::sort(candidates.begin(), candidates.end());

            if ((int)candidates.size() > max_pairs_per_gap)
                candidates.resize(max_pairs_per_gap);

            // ── Phase 1: parallel RRT for all candidates ────────────────────
            struct RRTJob {
                CandPair cp;
                Eigen::VectorXd ca, cb;
                int seed;
            };
            std::vector<RRTJob> jobs;
            jobs.reserve(candidates.size());

            for (int ci = 0; ci < (int)candidates.size(); ++ci) {
                const auto& cp = candidates[ci];
                auto its = id_to_idx.find(cp.small_id);
                auto itm = id_to_idx.find(cp.main_id);
                if (its == id_to_idx.end() || itm == id_to_idx.end()) continue;
                RRTJob job;
                job.cp = cp;
                job.ca = boxes[its->second].center();
                job.cb = boxes[itm->second].center();
                job.seed = rrt_seed++;
                jobs.push_back(std::move(job));
            }

            // Cancellation flag: set when island is merged → remaining
            // queued RRT tasks return immediately, freeing pool workers.
            auto cancel = std::make_shared<std::atomic<bool>>(false);

            // Launch all RRT calls in parallel
            using PathVec = std::vector<Eigen::VectorXd>;
            std::vector<std::future<PathVec>> futures;
            futures.reserve(jobs.size());

            RRTConnectConfig pair_rrt;
            // B6: progressive timeout — lower in early rounds, full later.
            // Easy pairs merge in round 0 quickly; hard pairs get more time later.
            // C4: Reduced timeouts — tree_b saturation at ~26K nodes indicates
            // full exploration; extra time yields diminishing returns.
            double round_timeout;
            int round_max_iters;
            if (round == 0) {
                round_timeout = std::min(per_pair_timeout_ms, 2500.0);
                round_max_iters = 50000;
            } else {
                round_timeout = per_pair_timeout_ms;
                round_max_iters = 80000;
            }
            pair_rrt.timeout_ms = round_timeout;
            pair_rrt.max_iters = round_max_iters;
            pair_rrt.step_size = 0.1;
            pair_rrt.goal_bias = 0.20;
            pair_rrt.segment_resolution = 10;

            for (const auto& job : jobs) {
                futures.push_back(rrt_pool.submit(
                    [&checker, &robot, pair_rrt, cancel,
                     ca = job.ca, cb = job.cb, seed = job.seed]() -> PathVec {
                        if (cancel->load(std::memory_order_relaxed))
                            return {};  // island already merged
                        return rrt_connect(ca, cb, checker, robot,
                                           pair_rrt, seed, cancel);
                    }));
            }

            // ── Phase 2: serial chain_pave for first successful path ────────
            // P1-A: early termination on consecutive RRT failures.
            // When only 2 islands remain, try all pairs (the last gap matters most —
            // even paved=0 RRT connections can provide adjacency updates).
            // With ≥3 islands, skip after 2 consecutive fails and move to next island.
            int consecutive_fails = 0;
            const int max_consecutive_fails =
                (islands.size() <= 2) ? (int)futures.size() : 5;

            for (size_t fi = 0; fi < futures.size(); ++fi) {
                if (deadline_reached()) break;
                if (total_bridges >= max_total_bridges) break;

                auto path = futures[fi].get();
                const auto& cp = jobs[fi].cp;

                if (path.empty()) {
                    ++consecutive_fails;
                    SBF_WARN("[BRG-ALL] island #%d pair %d/%d: box %d->%d, " "dist=%.3f, rrt=FAIL (consec=%d/%d)", (int)ii, (int)fi + 1, (int)futures.size(), cp.small_id, cp.main_id, std::sqrt(cp.dist), consecutive_fails, max_consecutive_fails);
                    if (consecutive_fails >= max_consecutive_fails) {
                        SBF_WARN("[BRG-ALL] island #%d: %d consecutive fails, " "skipping remaining pairs", (int)ii, consecutive_fails);
                        cancel->store(true, std::memory_order_relaxed);
                        break;
                    }
                    continue;
                }

                consecutive_fails = 0;  // reset on success

                // Chain-pave from small-island box along the RRT path
                int old_n = static_cast<int>(boxes.size());
                int added = chain_pave_along_path(
                    path, cp.small_id, boxes, lect, obs, n_obs,
                    ffb_config, adj, next_box_id, robot,
                    /*max_chain=*/500, /*max_steps_per_wp=*/15);

                total_bridges += added;

                // Incrementally absorb new boxes into UF
                absorb_new_boxes(old_n);

                // Check merge via O(1) UF query
                auto it_s = id_to_idx.find(small_island[0]);
                auto it_m = id_to_idx.find(main_island[0]);
                bool merged = (it_s != id_to_idx.end() && it_m != id_to_idx.end()
                               && uf.connected(it_s->second, it_m->second));

                SBF_INFO("[BRG-ALL] island #%d pair %d/%d: box %d->%d, " "dist=%.3f, rrt=%dwp, paved=%d, merged=%s", (int)ii, (int)fi + 1, (int)futures.size(), cp.small_id, cp.main_id, std::sqrt(cp.dist), (int)path.size(), added, merged ? "YES" : "no");

                if (merged) {
                    merged_any = true;
                    cancel->store(true, std::memory_order_relaxed);
                    break;  // Move to next small island
                }
            }
        }

        // If no island was merged in this round, stop
        if (!merged_any) {
            SBF_INFO("[BRG-ALL] no island merged in round %d — stopping", round);
            break;
        }
    }

    // Final report (O(n) via UF, no full adj traversal)
    auto final_islands = get_islands();
    SBF_INFO("[BRG-ALL] done: %d islands remain, %d bridge boxes created", (int)final_islands.size(), total_bridges);

    return total_bridges;
}

}  // namespace sbf
