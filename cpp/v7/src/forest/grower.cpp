/// @file grower.cpp
#include "sbf/forest/grower.h"

#include "sbf/forest/bridge.h"
#include "sbf/forest/union_find.h"
#include "sbf/util/thread_pool.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <random>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace sbf::forest {

using Clock = std::chrono::steady_clock;

ForestGrower::ForestGrower(const sbf::core::Robot& robot,
                           sbf::lect::LECT& lect,
                           GrowerConfig cfg)
    : robot_(robot), lect_(lect), cfg_(std::move(cfg)) {}

void ForestGrower::set_endpoints(const Eigen::VectorXd& q_start,
                                 const Eigen::VectorXd& q_goal) {
    q_start_ = q_start;
    q_goal_  = q_goal;
    has_endpoints_ = true;
}

namespace {

sbf::scene::BoxNode build_box_from_leaf(
    sbf::lect::LECT& lect, int leaf_idx, int box_id, int parent_box_id) {
    sbf::scene::BoxNode nb;
    nb.id = box_id;
    nb.joint_intervals = lect.node_intervals(leaf_idx);
    nb.parent_box_id = parent_box_id;
    nb.compute_volume();
    int n_slots = lect.n_slots();
    const float* env = lect.get_link_iaabbs(leaf_idx);
    nb.link_iaabbs.assign(env, env + n_slots * 6);
    nb.seed_config = nb.center();
    return nb;
}

}  // namespace

GrowerResult ForestGrower::grow(const float* obs_compact, int n_obs) {
    int n = cfg_.n_threads;
    if (n <= 0) n = static_cast<int>(std::thread::hardware_concurrency());
    if (n <= 0) n = 1;
    if (n == 1) return grow_serial(obs_compact, n_obs);
    return grow_parallel(obs_compact, n_obs);
}

GrowerResult ForestGrower::grow_serial(const float* obs_compact, int n_obs) {
    GrowerResult res;
    auto t0 = Clock::now();
    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(
                   Clock::now() - t0).count();
    };

    UnionFind box_uf(0);
    std::vector<sbf::scene::BoxNode>& boxes = res.boxes;
    boxes.reserve(cfg_.max_boxes + 32);

    // Returns the new box id, or -1 if the candidate was rejected (not face-adjacent
    // to its parent). On rejection, no LECT mutation persists.
    auto add_box = [&](int leaf, int parent_box_id, int tree_id) -> int {
        int id = static_cast<int>(boxes.size());
        sbf::scene::BoxNode nb = build_box_from_leaf(lect_, leaf, id, parent_box_id);
        nb.tree_id = tree_id;
        nb.root_id = (parent_box_id < 0) ? id : boxes[parent_box_id].root_id;
        if (parent_box_id >= 0) {
            bool ok = enforce_parent_adjacency(nb, boxes[parent_box_id],
                                               lect_, obs_compact, n_obs);
            if (!ok) return -1;     // strict adjacency: reject
        }
        boxes.push_back(nb);
        lect_.mark_occupied(leaf, id);
        box_uf.push();
        for (int i = 0; i < id; ++i)
            if (boxes_adjacent(boxes[id], boxes[i])) box_uf.unite(id, i);
        return id;
    };

    // Seed start/goal boxes.
    if (has_endpoints_) {
        FFBResult sr = find_free_box(lect_, q_start_, obs_compact, n_obs, cfg_.ffb);
        if (!sr.success()) {
            res.build_time_ms = elapsed_ms();
            return res;     // INFEASIBLE: q_start in collision/occupied
        }
        res.start_box = add_box(sr.node_idx, /*parent=*/-1, /*tree=*/0);

        FFBResult gr = find_free_box(lect_, q_goal_, obs_compact, n_obs, cfg_.ffb);
        if (!gr.success()) {
            res.build_time_ms = elapsed_ms();
            return res;     // INFEASIBLE: q_goal in collision/occupied
        }
        res.goal_box = add_box(gr.node_idx, /*parent=*/-1, /*tree=*/1);
    }

    // RRT main loop.
    std::mt19937_64 rng(cfg_.rng_seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    const int nd = lect_.n_dims();
    const auto& root_iv = lect_.root_intervals();

    // Domain restriction: when set_domain_root is active, all samples are
    // clamped inside the subtree's intervals (with a hairline margin), and
    // FFB results outside the subtree are rejected.
    std::vector<sbf::core::Interval> dom_iv = root_iv;
    if (domain_root_ >= 0) {
        dom_iv = lect_.node_intervals(domain_root_);
        for (auto& iv : dom_iv) {
            double w = iv.hi - iv.lo;
            double m = std::max(1e-9, 1e-6 * w);
            iv.lo += m;
            iv.hi -= m;
            if (iv.hi <= iv.lo) {
                double mid = 0.5 * (iv.lo + iv.hi);
                iv.lo = mid - 1e-12; iv.hi = mid + 1e-12;
            }
        }
    }
    const auto& sample_iv = (domain_root_ >= 0) ? dom_iv : root_iv;

    int miss = 0;
    while ((int)boxes.size() < cfg_.max_boxes &&
           miss < cfg_.max_consecutive_miss) {
        if (elapsed_ms() > cfg_.timeout_ms) break;

        bool sg_connected =
            (res.start_box >= 0 && res.goal_box >= 0 &&
             box_uf.connected(res.start_box, res.goal_box));
        if (cfg_.connect_mode && cfg_.stop_after_connect && sg_connected) break;

        // Sample q.
        Eigen::VectorXd q(nd);
        if (has_endpoints_ && u01(rng) < cfg_.rrt_goal_bias) {
            const auto& target = (u01(rng) < 0.5) ? q_start_ : q_goal_;
            q = target;
        } else {
            for (int d = 0; d < nd; ++d) {
                double lo = sample_iv[d].lo, hi = sample_iv[d].hi;
                q[d] = lo + u01(rng) * (hi - lo);
            }
        }

        // Find nearest box (O(N) brute force).
        int parent_id = -1;
        double best_d2 = 1e300;
        for (int i = 0; i < (int)boxes.size(); ++i) {
            Eigen::VectorXd c = boxes[i].center();
            double d2 = 0.0;
            for (int d = 0; d < nd; ++d) {
                double dx = q[d] - c[d];
                d2 += dx * dx;
            }
            if (d2 < best_d2) { best_d2 = d2; parent_id = i; }
        }
        if (parent_id < 0) {
            // No boxes yet (no endpoints) — seed at q itself.
            parent_id = -1;
        }

        // Step from nearest box's center toward q via snap_to_face: seed
        // lands just past parent's chosen face, with non-exit dims biased.
        Eigen::VectorXd seed(nd);
        if (parent_id >= 0) {
            Eigen::VectorXd c = boxes[parent_id].center();
            Eigen::VectorXd dir = q - c;
            if (dir.norm() < 1e-15) {
                // Degenerate: q == center. Pick a small random direction.
                for (int d = 0; d < nd; ++d) dir[d] = u01(rng) - 0.5;
            }
            SnapResult sn = snap_to_face(boxes[parent_id], dir,
                                         root_iv, cfg_.rrt_step_ratio, rng);
            seed = sn.seed;
        } else {
            seed = q;
        }

        FFBResult fr = find_free_box(lect_, seed, obs_compact, n_obs, cfg_.ffb);
        if (!fr.success()) { ++miss; ++res.n_ffb_fail; continue; }
        if (lect_.is_occupied(fr.node_idx)) { ++miss; ++res.n_ffb_fail; continue; }
        if (domain_root_ >= 0 &&
            !lect_.is_descendant_of(fr.node_idx, domain_root_)) {
            // FFB descended outside this worker's subtree — reject.
            ++miss; continue;
        }

        int parent_for_box = parent_id;
        int tree_id = (parent_for_box >= 0) ? boxes[parent_for_box].tree_id : 0;
        int new_id = add_box(fr.node_idx, parent_for_box, tree_id);
        if (new_id < 0) {
            // Strict adjacency rejected — do NOT mark lect occupied; just retry.
            ++miss;
            continue;
        }
        ++res.n_ffb_success;
        miss = 0;
    }

    // Update connectivity flag from DSU.
    if (res.start_box >= 0 && res.goal_box >= 0)
        res.start_goal_connected =
            box_uf.connected(res.start_box, res.goal_box);

    // Endpoint auto-bridge.
    if (cfg_.endpoint_auto_bridge && !res.start_goal_connected &&
        res.start_box >= 0 && res.goal_box >= 0) {
        int budget = cfg_.endpoint_bridge_max_boxes;
        if (budget <= 0)
            budget = std::clamp(cfg_.max_boxes / 20, 50, 500);
        auto br = endpoint_auto_bridge(
            lect_, boxes, res.start_box, res.goal_box,
            q_start_, q_goal_, obs_compact, n_obs,
            budget, cfg_.rng_seed + 1, cfg_.ffb);
        res.n_bridge_boxes = br.n_bridges_added;
        res.start_goal_connected = br.start_goal_connected;
    }

    // Final adjacency graph.
    res.adjacency = compute_adjacency_graph(boxes);
    auto isl = find_islands(res.adjacency);
    res.adjacency_islands = isl.n_components;
    res.adjacency_largest_island = isl.largest_size;
    res.adjacency_all_connected = (isl.n_components <= 1);
    if (res.start_box >= 0 && res.goal_box >= 0)
        res.start_goal_connected =
            isl.component_id[res.start_box] == isl.component_id[res.goal_box];

    res.build_time_ms = elapsed_ms();
    return res;
}

// ─────────────────────────── grow_parallel ───────────────────────────

GrowerResult ForestGrower::grow_parallel(const float* obs, int n_obs) {
    GrowerResult res;
    auto t0 = Clock::now();
    auto ms = [&](){ return std::chrono::duration<double, std::milli>(
                              Clock::now() - t0).count(); };

    int n = cfg_.n_threads;
    if (n <= 0) n = static_cast<int>(std::thread::hardware_concurrency());
    if (n <= 0) n = 1;

    if (!has_endpoints_) {
        // Without endpoints there's nothing meaningful to partition by.
        return grow_serial(obs, n_obs);
    }

    // 1) Seed start/goal in the master LECT (so workers' snapshots inherit
    //    the occupations).
    auto build_box_at = [&](int leaf, int box_id, int parent_id, int tree_id) {
        sbf::scene::BoxNode nb = build_box_from_leaf(lect_, leaf, box_id, parent_id);
        nb.tree_id = tree_id;
        nb.root_id = (parent_id < 0) ? box_id : 0;
        return nb;
    };

    FFBResult sr = find_free_box(lect_, q_start_, obs, n_obs, cfg_.ffb);
    if (!sr.success()) { res.build_time_ms = ms(); return res; }
    res.start_box = static_cast<int>(res.boxes.size());
    res.boxes.push_back(build_box_at(sr.node_idx, res.start_box, -1, 0));
    lect_.mark_occupied(sr.node_idx, res.start_box);

    FFBResult gr = find_free_box(lect_, q_goal_, obs, n_obs, cfg_.ffb);
    if (!gr.success()) { res.build_time_ms = ms(); return res; }
    res.goal_box = static_cast<int>(res.boxes.size());
    res.boxes.push_back(build_box_at(gr.node_idx, res.goal_box, -1, 1));
    lect_.mark_occupied(gr.node_idx, res.goal_box);

    // 2) Build seed list = {q_start, q_goal} + random fillers up to n.
    std::mt19937_64 rng(cfg_.rng_seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<Eigen::VectorXd> seeds;
    seeds.push_back(q_start_);
    seeds.push_back(q_goal_);
    int nd = lect_.n_dims();
    const auto& iv = lect_.root_intervals();
    for (int k = 2; k < n; ++k) {
        Eigen::VectorXd q(nd);
        for (int d = 0; d < nd; ++d)
            q[d] = iv[d].lo + u01(rng) * (iv[d].hi - iv[d].lo);
        seeds.push_back(q);
    }

    // 3) Partition the master LECT.
    std::vector<int> domain_roots = lect_.partition_for_seeds(
        seeds, cfg_.partition_max_splits);

    // Deduplicate: distinct domain roots → workers.
    std::vector<int> unique_roots;
    {
        std::unordered_map<int,int> root_to_wkr;
        for (int dr : domain_roots) {
            if (root_to_wkr.find(dr) == root_to_wkr.end()) {
                root_to_wkr[dr] = static_cast<int>(unique_roots.size());
                unique_roots.push_back(dr);
            }
        }
    }
    int n_workers = static_cast<int>(unique_roots.size());
    res.n_workers_used = n_workers;
    if (n_workers <= 1) {
        // Fall back to serial — partitioning gave us nothing useful.
        // Note: start/goal already in master.boxes; reuse the rest of the
        // serial loop by clearing res and rerunning. Simpler: just call
        // grow_serial (which re-seeds endpoints), but start/goal leaves
        // are already marked. Undo and dispatch.
        lect_.unmark_occupied(sr.node_idx);
        lect_.unmark_occupied(gr.node_idx);
        return grow_serial(obs, n_obs);
    }

    // 4) Per-worker LECT snapshots.
    std::vector<sbf::lect::LECT> wlects;
    wlects.reserve(n_workers);
    for (int w = 0; w < n_workers; ++w) wlects.push_back(lect_.snapshot());

    // 5) Spawn workers.
    sbf::util::ThreadPool pool(n_workers);
    std::vector<std::future<GrowerResult>> futs;
    futs.reserve(n_workers);
    int per_worker_budget = std::max(1, cfg_.max_boxes / n_workers);
    for (int w = 0; w < n_workers; ++w) {
        int dr = unique_roots[w];
        sbf::lect::LECT* wl = &wlects[w];
        futs.push_back(pool.submit([this, w, dr, wl, obs, n_obs,
                                    per_worker_budget]() -> GrowerResult {
            GrowerConfig wcfg = cfg_;
            wcfg.n_threads             = 1;
            wcfg.max_boxes             = per_worker_budget;
            wcfg.endpoint_auto_bridge  = false;
            wcfg.rng_seed              = cfg_.rng_seed + 1000ull + (uint64_t)w;
            ForestGrower wg(robot_, *wl, wcfg);
            wg.set_domain_root(dr);
            // Workers do NOT seed endpoints — start/goal already in master.
            return wg.grow(obs, n_obs);
        }));
    }

    // 6) Sequentially merge each worker's result.
    for (int w = 0; w < n_workers; ++w) {
        GrowerResult wr = futs[w].get();
        // 6a) id_map: worker box id → global id.
        std::unordered_map<int,int> id_map;
        id_map.reserve(wr.boxes.size());
        int base = static_cast<int>(res.boxes.size());
        for (int i = 0; i < (int)wr.boxes.size(); ++i)
            id_map[i] = base + i;
        // 6b) Transplant LECT subtree into master.
        std::unordered_map<int,int> node_remap;
        lect_.transplant_domain(wlects[w], unique_roots[w],
                                id_map, node_remap);
        // 6c) Append boxes with remapped ids.
        for (auto& b : wr.boxes) {
            sbf::scene::BoxNode nb = b;
            nb.id            = id_map[b.id];
            nb.parent_box_id = (b.parent_box_id >= 0)
                                   ? id_map[b.parent_box_id] : -1;
            nb.root_id       = (b.root_id >= 0)
                                   ? id_map[b.root_id] : nb.id;
            res.boxes.push_back(nb);
        }
        res.n_ffb_success += wr.n_ffb_success;
        res.n_ffb_fail    += wr.n_ffb_fail;
    }

    // 7) Global adjacency & connectivity.
    res.adjacency = compute_adjacency_graph(res.boxes);
    auto isl0 = find_islands(res.adjacency);
    if (res.start_box >= 0 && res.goal_box >= 0)
        res.start_goal_connected =
            isl0.component_id[res.start_box] == isl0.component_id[res.goal_box];

    // 8) Endpoint auto-bridge in master if still disconnected.
    if (cfg_.endpoint_auto_bridge && !res.start_goal_connected &&
        res.start_box >= 0 && res.goal_box >= 0) {
        int budget = cfg_.endpoint_bridge_max_boxes;
        if (budget <= 0) budget = std::clamp(cfg_.max_boxes / 20, 50, 500);
        auto br = endpoint_auto_bridge(
            lect_, res.boxes, res.start_box, res.goal_box,
            q_start_, q_goal_, obs, n_obs,
            budget, cfg_.rng_seed + 7, cfg_.ffb);
        res.n_bridge_boxes = br.n_bridges_added;
        res.start_goal_connected = br.start_goal_connected;
        res.adjacency = compute_adjacency_graph(res.boxes);
    }

    auto isl = find_islands(res.adjacency);
    res.adjacency_islands         = isl.n_components;
    res.adjacency_largest_island  = isl.largest_size;
    res.adjacency_all_connected   = (isl.n_components <= 1);
    if (res.start_box >= 0 && res.goal_box >= 0)
        res.start_goal_connected =
            isl.component_id[res.start_box] == isl.component_id[res.goal_box];

    res.build_time_ms = ms();
    return res;
}

}  // namespace sbf::forest
