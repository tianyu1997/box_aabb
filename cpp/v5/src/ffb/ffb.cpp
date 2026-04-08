// SafeBoxForest v5 — FFB (Phase E)
#include <sbf/ffb/ffb.h>
#include <sbf/core/fk_state.h>

#include <cassert>
#include <chrono>
#include <limits>

namespace sbf {

FFBResult find_free_box(
    LECT& lect,
    const Eigen::VectorXd& seed,
    const Obstacle* obs,
    int n_obs,
    const FFBConfig& config)
{
    assert(seed.size() == lect.n_dims());

    FFBResult result;
    const Robot& robot = lect.robot();

    // Initialize running FK state and intervals from root
    FKState fk = lect.root_fk();
    auto intervals = lect.root_intervals();

    int current = 0;  // start at root
    result.path.push_back(current);

    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();

    // Helper lambda: descend from current to child, updating fk+intervals
    auto descend = [&](int parent, int child) {
        int sd = lect.get_split_dim(parent);
        double sv = lect.split_val(parent);
        if (child == lect.left(parent))
            intervals[sd].hi = sv;
        else
            intervals[sd].lo = sv;
        fk = compute_fk_incremental(fk, robot, intervals, sd);
        current = child;
        result.path.push_back(current);
    };

    while (true) {
        result.n_steps++;

        // Count cache hit/miss for every visited node
        if (lect.has_data(current))
            result.n_cache_hits++;
        else
            result.n_cache_misses++;

        // 1. Check deadline
        if (config.deadline_ms > 0.0) {
            auto elapsed = std::chrono::duration<double, std::milli>(
                Clock::now() - t0).count();
            if (elapsed > config.deadline_ms) {
                result.fail_code = 4;
                result.total_ms = elapsed;
                return result;
            }
        }

        // 2. Check occupation
        if (lect.is_occupied(current)) {
            // Occupied leaf → expand it so we can explore sub-regions
            if (lect.is_leaf(current)) {
                int dp = lect.depth(current);
                if (dp >= config.max_depth) {
                    result.fail_code = 1;
                    result.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
                    return result;
                }
                double max_w = 0.0;
                for (int dim = 0; dim < lect.n_dims(); ++dim)
                    max_w = std::max(max_w, intervals[dim].width());
                if (max_w / 2.0 < config.min_edge) {
                    result.fail_code = 1;
                    result.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
                    return result;
                }
                {
                    auto t_exp = Clock::now();
                    lect.expand_leaf(current, fk, intervals);
                    result.expand_ms += std::chrono::duration<double, std::milli>(
                        Clock::now() - t_exp).count();
                    result.n_expand_calls++;
                }
                result.n_new_nodes += 2;
            }
            // Descend past occupied internal node
            int sd = lect.get_split_dim(current);
            int child = (seed[sd] <= lect.split_val(current))
                        ? lect.left(current) : lect.right(current);
            descend(current, child);
            continue;
        }

        // 3. Compute envelope if not cached (use running intervals+fk)
        int parent_idx = (result.path.size() >= 2)
                             ? result.path[result.path.size() - 2]
                             : -1;
        int changed_dim = (parent_idx >= 0) ? lect.get_split_dim(parent_idx) : -1;

        if (!lect.has_data(current)) {
            auto t_env = Clock::now();
            lect.compute_envelope(current, fk, intervals, changed_dim, parent_idx);
            result.envelope_ms += std::chrono::duration<double, std::milli>(
                Clock::now() - t_env).count();
            result.n_fk_calls++;
        }

        // 4. Collision detection
        {
            auto t_col = Clock::now();
            bool collides = lect.collides_scene(current, obs, n_obs);
            result.collide_ms += std::chrono::duration<double, std::milli>(
                Clock::now() - t_col).count();
            result.n_collide_calls++;

            if (!collides) {
                result.node_idx = current;
                result.fail_code = 0;
                result.total_ms = std::chrono::duration<double, std::milli>(
                    Clock::now() - t0).count();
                return result;
            }
        }

        // 5. Need to split/descend
        int d = lect.depth(current);
        if (d >= config.max_depth) {
            result.fail_code = 2;
            result.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            return result;
        }

        {
            double max_width = 0.0;
            for (int dim = 0; dim < lect.n_dims(); ++dim)
                max_width = std::max(max_width, intervals[dim].width());
            if (max_width / 2.0 < config.min_edge) {
                result.fail_code = 3;
                result.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
                return result;
            }
        }

        // 6. Expand leaf if needed
        if (lect.is_leaf(current)) {
            auto t_exp = Clock::now();
            lect.expand_leaf(current, fk, intervals);
            result.expand_ms += std::chrono::duration<double, std::milli>(
                Clock::now() - t_exp).count();
            result.n_expand_calls++;
            result.n_new_nodes += 2;
        }

        // 7. Select child containing seed, descend with incremental FK
        int sd = lect.get_split_dim(current);
        int child = (seed[sd] <= lect.split_val(current))
                    ? lect.left(current) : lect.right(current);
        descend(current, child);
    }
}

}  // namespace sbf
