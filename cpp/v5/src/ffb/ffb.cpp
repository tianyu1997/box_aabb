// SafeBoxForest v5 — FFB (Phase E)
// Optimized: in-place FK, sampled deadline, no per-step timing overhead.
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
    int prev_node = -1;  // parent of current (replaces result.path)

    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();

    // Deadline sampling: check Clock::now() only every N steps to avoid
    // syscall overhead (~100–200ns each × 492K calls = 50–100ms waste).
    constexpr int DEADLINE_SAMPLE_INTERVAL = 64;
    const bool has_deadline = (config.deadline_ms > 0.0);

    while (true) {
        result.n_steps++;

        // Count cache hit/miss for every visited node
        if (lect.has_data(current))
            result.n_cache_hits++;
        else
            result.n_cache_misses++;

        // 1. Sampled deadline check (every N steps instead of every step)
        if (has_deadline && (result.n_steps & (DEADLINE_SAMPLE_INTERVAL - 1)) == 0) {
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
                lect.expand_leaf(current, fk, intervals);
                result.n_expand_calls++;
                result.n_new_nodes += 2;
            }
            // Descend past occupied internal node — in-place FK update
            int sd = lect.get_split_dim(current);
            double sv = lect.split_val(current);
            int child = (seed[sd] <= sv) ? lect.left(current) : lect.right(current);
            if (child == lect.left(current))
                intervals[sd].hi = sv;
            else
                intervals[sd].lo = sv;
            update_fk_inplace(fk, robot, intervals, sd);
            prev_node = current;
            current = child;
            continue;
        }

        // 3. Compute envelope if not cached (use running intervals+fk)
        int changed_dim = (prev_node >= 0) ? lect.get_split_dim(prev_node) : -1;

        if (!lect.has_data(current)) {
            lect.compute_envelope(current, fk, intervals, changed_dim, prev_node);
            result.n_fk_calls++;
        }

        // 4. Collision detection
        {
            bool collides = lect.collides_scene(current, obs, n_obs);
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

        // 6. Expand leaf if needed
        if (lect.is_leaf(current)) {
            lect.expand_leaf(current, fk, intervals);
            result.n_expand_calls++;
            result.n_new_nodes += 2;
        }

        // 7. Select child containing seed, descend with in-place FK
        {
            int sd = lect.get_split_dim(current);
            double sv = lect.split_val(current);
            int child = (seed[sd] <= sv) ? lect.left(current) : lect.right(current);
            if (child == lect.left(current))
                intervals[sd].hi = sv;
            else
                intervals[sd].lo = sv;
            update_fk_inplace(fk, robot, intervals, sd);
            prev_node = current;
            current = child;
        }
    }
}

}  // namespace sbf
