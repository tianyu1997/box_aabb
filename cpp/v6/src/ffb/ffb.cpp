// SafeBoxForest v6 — FFB (Phase E)
// Optimized: in-place FK, sampled deadline, no per-step timing overhead.
#include <sbf/ffb/ffb.h>
#include <sbf/core/fk_state.h>
#include <sbf/core/log.h>
#include <sbf/core/log_format.h>
#include <sbf/scene/collision_checker.h>

#include <cassert>
#include <chrono>
#include <limits>

namespace sbf {

namespace {

FFBResult find_free_box_from_node(
    LECT& lect,
    int start_node,
    const Eigen::VectorXd& seed,
    const Obstacle* obs,
    int n_obs,
    const FFBConfig& config)
{
    assert(seed.size() == lect.n_dims());

    FFBResult result;
    const Robot& robot = lect.robot();

    // Initialize running FK state and intervals from the requested root.
    int current = (start_node >= 0) ? start_node : 0;
    FKState fk;
    std::vector<Interval> intervals;
    if (current == 0) {
        fk = lect.root_fk();
        intervals = lect.root_intervals();
    } else {
        intervals = lect.node_intervals(current);
        fk = compute_fk_full(robot, intervals);
    }
    int prev_node = -1;  // parent of current (replaces result.path)

    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();

    SBF_TRACE("[FFB] begin seed=%s start=%d max_depth=%d",
              fmt_vec(seed).c_str(), current, config.max_depth);

    // Hard guard: if seed configuration is in collision, do not enter FFB descent.
    if (!config.seed_known_free) {
        CollisionChecker seed_checker(robot, {});
        seed_checker.set_obstacles(obs, n_obs);
        if (seed_checker.check_config(seed)) {
            SBF_TRACE("[FFB] reject(seed_collision) seed=%s", fmt_vec(seed).c_str());
            result.fail_code = 1;
            result.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            return result;
        }
    }

    // Deadline sampling: check Clock::now() only every N steps to avoid
    // syscall overhead (~100–200ns each × 492K calls = 50–100ms waste).
    constexpr int DEADLINE_SAMPLE_INTERVAL = 64;
    const bool has_deadline = (config.deadline_ms > 0.0);

    // Per-branch timers (FFB sub-phase profiling).
    // Use rdtsc-equivalent steady_clock and only accumulate at the end of each
    // branch to keep loop overhead minimal.
    auto t_env_acc = Clock::duration::zero();
    auto t_col_acc = Clock::duration::zero();
    auto t_exp_acc = Clock::duration::zero();

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
                SBF_TRACE("[FFB]  step=%d node=%d depth=%d -> DEADLINE (%.2fms)",
                          result.n_steps, current, lect.depth(current), elapsed);
                result.fail_code = 4;
                result.envelope_ms = std::chrono::duration<double,std::milli>(t_env_acc).count();
                result.collide_ms  = std::chrono::duration<double,std::milli>(t_col_acc).count();
                result.expand_ms   = std::chrono::duration<double,std::milli>(t_exp_acc).count();
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
                    SBF_TRACE("[FFB]  step=%d node=%d depth=%d -> FAIL "
                              "(occupied-leaf at max_depth)",
                              result.n_steps, current, dp);
                    result.fail_code = 1;
                    result.envelope_ms = std::chrono::duration<double,std::milli>(t_env_acc).count();
                    result.collide_ms  = std::chrono::duration<double,std::milli>(t_col_acc).count();
                    result.expand_ms   = std::chrono::duration<double,std::milli>(t_exp_acc).count();
                    result.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
                    return result;
                }
                auto t_e0 = Clock::now();
                lect.expand_leaf(current, fk, intervals);
                t_exp_acc += Clock::now() - t_e0;
                result.n_expand_calls++;
                result.n_new_nodes += 2;
                SBF_TRACE("[FFB]  step=%d node=%d depth=%d EXPAND occupied-leaf",
                          result.n_steps, current, dp);
            }
            // Descend past occupied internal node — in-place FK update
            int sd = lect.get_split_dim(current);
            double sv = lect.split_val(current);
            int child = (seed[sd] <= sv) ? lect.left(current) : lect.right(current);
            const char* side = (child == lect.left(current)) ? "L" : "R";
            SBF_TRACE("[FFB]  step=%d node=%d depth=%d OCCUPIED descend "
                      "split_dim=%d split_val=%.4f seed[d]=%.4f -> %s child=%d",
                      result.n_steps, current, lect.depth(current),
                      sd, sv, seed[sd], side, child);
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

        bool envelope_was_cached = lect.has_data(current);
        if (!envelope_was_cached) {
            auto t_v0 = Clock::now();
            lect.compute_envelope(current, fk, intervals, changed_dim, prev_node);
            t_env_acc += Clock::now() - t_v0;
            result.n_fk_calls++;
        }

        // 4. Collision detection (with Phase A collide-verified cache)
        {
            bool collides;
            if (lect.collide_cache_hit_free(current)) {
                // Cached free against current obstacle generation → skip call
                collides = false;
            } else if (lect.collide_cache_hit_collide(current)) {
                // Cached collide → skip call, fall through to expand+descend
                collides = true;
            } else {
                auto t_c0 = Clock::now();
                collides = (config.obs_grid && config.grid_margin_threshold > 0.0f)
                    ? lect.collides_scene(current, obs, n_obs,
                                          config.obs_grid,
                                          config.grid_margin_threshold)
                    : lect.collides_scene(current, obs, n_obs);
                t_col_acc += Clock::now() - t_c0;
                result.n_collide_calls++;
                if (collides) lect.mark_collide_hit(current);
                else          lect.mark_collide_free(current);
            }

            if (!collides) {
                SBF_TRACE("[FFB]  step=%d node=%d depth=%d FREE -> RETURN "
                          "(envelope_cached=%d)",
                          result.n_steps, current, lect.depth(current),
                          envelope_was_cached ? 1 : 0);
                result.node_idx = current;
                result.fail_code = 0;
                result.envelope_ms = std::chrono::duration<double,std::milli>(t_env_acc).count();
                result.collide_ms  = std::chrono::duration<double,std::milli>(t_col_acc).count();
                result.expand_ms   = std::chrono::duration<double,std::milli>(t_exp_acc).count();
                result.total_ms = std::chrono::duration<double, std::milli>(
                    Clock::now() - t0).count();
                return result;
            }
        }

        // 5. Need to split/descend
        int d = lect.depth(current);
        if (d >= config.max_depth) {
            SBF_TRACE("[FFB]  step=%d node=%d depth=%d -> FAIL (collide at max_depth)",
                      result.n_steps, current, d);
            result.fail_code = 2;
            result.envelope_ms = std::chrono::duration<double,std::milli>(t_env_acc).count();
            result.collide_ms  = std::chrono::duration<double,std::milli>(t_col_acc).count();
            result.expand_ms   = std::chrono::duration<double,std::milli>(t_exp_acc).count();
            result.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            return result;
        }

        // 6. Expand leaf if needed
        if (lect.is_leaf(current)) {
            auto t_e0 = Clock::now();
            lect.expand_leaf(current, fk, intervals);
            t_exp_acc += Clock::now() - t_e0;
            result.n_expand_calls++;
            result.n_new_nodes += 2;
            SBF_TRACE("[FFB]  step=%d node=%d depth=%d EXPAND collide-leaf",
                      result.n_steps, current, d);
        }

        // 7. Select child containing seed, descend with in-place FK
        {
            int sd = lect.get_split_dim(current);
            double sv = lect.split_val(current);
            int child = (seed[sd] <= sv) ? lect.left(current) : lect.right(current);
            const char* side = (child == lect.left(current)) ? "L" : "R";
            SBF_TRACE("[FFB]  step=%d node=%d depth=%d COLLIDE descend "
                      "split_dim=%d split_val=%.4f seed[d]=%.4f -> %s child=%d",
                      result.n_steps, current, d,
                      sd, sv, seed[sd], side, child);
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

}  // namespace

FFBResult find_free_box(
    LECT& lect,
    const Eigen::VectorXd& seed,
    const Obstacle* obs,
    int n_obs,
    const FFBConfig& config)
{
    return find_free_box_from_node(lect, 0, seed, obs, n_obs, config);
}

FFBResult find_free_box_in_domain(
    LECT& lect,
    int domain_root,
    const Eigen::VectorXd& seed,
    const Obstacle* obs,
    int n_obs,
    const FFBConfig& config)
{
    return find_free_box_from_node(lect, domain_root, seed, obs, n_obs, config);
}

}  // namespace sbf
