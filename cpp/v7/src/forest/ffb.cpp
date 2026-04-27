/// @file ffb.cpp
#include "sbf/forest/ffb.h"

#include <chrono>

namespace sbf::forest {

using Clock = std::chrono::steady_clock;

FFBResult find_free_box(
    sbf::lect::LECT& lect,
    const Eigen::VectorXd& seed,
    const float* obs_compact, int n_obs,
    const FFBConfig& cfg) {

    FFBResult r;
    if (seed.size() != lect.n_dims()) {
        r.fail_code = 5;       // outside / dim mismatch
        return r;
    }

    auto t0 = Clock::now();
    auto deadline_hit = [&]() {
        if (cfg.deadline_ms <= 0.0) return false;
        double ms = std::chrono::duration<double, std::milli>(
                        Clock::now() - t0).count();
        return ms >= cfg.deadline_ms;
    };

    int cur = 0;     // root
    r.path.push_back(cur);

    // (No early root collision check. The root envelope spans the full
    // joint range, so for any non-trivial obstacle layout the union of
    // link AABBs at the root will collide; that does NOT mean the seed
    // is infeasible.  Per-child collision checks during descent give the
    // real verdict.)

    // Verify seed is in root's joint intervals.
    auto root_iv = lect.node_intervals(0);
    for (int d = 0; d < lect.n_dims(); ++d) {
        if (!root_iv[d].contains(seed[d])) {
            r.fail_code = 5;
            return r;
        }
    }

    while (true) {
        ++r.n_steps;
        if (deadline_hit()) { r.fail_code = 4; return r; }
        if (lect.depth(cur) >= cfg.max_depth) {
            // Reached depth cap. If the leaf-as-is is free + unoccupied, accept.
            if (lect.is_occupied(cur)) { r.fail_code = 1; return r; }
            ++r.n_collide_calls;
            if (lect.collides_scene(cur, obs_compact, n_obs)) {
                r.fail_code = 3; return r;
            }
            r.node_idx = cur;
            r.fail_code = 0;
            return r;
        }

        if (lect.is_leaf(cur)) {
            if (lect.is_occupied(cur)) { r.fail_code = 1; return r; }
            // Try to expand (split) so envelope tightens. If split fails
            // (e.g. all dims too narrow), accept current node only if its
            // envelope is collision-free.
            int n_alloc = lect.expand_leaf(cur);
            if (n_alloc != 2) {
                ++r.n_collide_calls;
                if (lect.collides_scene(cur, obs_compact, n_obs)) {
                    r.fail_code = 3; return r;
                }
                r.node_idx = cur;
                r.fail_code = 0;
                return r;
            }
            ++r.n_new_nodes;
            // Fall through: descend into the child containing seed.
        }

        // Descend by split plane.
        int d = lect.split_dim(cur);
        double v = lect.split_val(cur);
        int next = (seed[d] <= v) ? lect.left(cur) : lect.right(cur);
        if (next < 0) {
            r.fail_code = 5;
            return r;
        }

        // Prefer sibling only if seed sits exactly on the split boundary
        // and the sibling's envelope is provably free.  Otherwise descend
        // into seed's child unconditionally — its envelope is conservative
        // and will tighten with further splits.
        ++r.n_collide_calls;
        bool next_collide = lect.collides_scene(next, obs_compact, n_obs);
        if (next_collide) {
            int sib = (next == lect.left(cur)) ? lect.right(cur) : lect.left(cur);
            if (sib >= 0) {
                ++r.n_collide_calls;
                bool sib_collide = lect.collides_scene(sib, obs_compact, n_obs);
                if (!sib_collide) {
                    auto sib_iv = lect.node_intervals(sib);
                    bool ok = true;
                    for (int dd = 0; dd < lect.n_dims(); ++dd)
                        if (!sib_iv[dd].contains(seed[dd])) { ok = false; break; }
                    if (ok) { cur = sib; r.path.push_back(cur); continue; }
                }
            }
            // Both colliding (or sibling doesn't contain seed): descend into
            // seed's child anyway. Final collision verdict is at depth cap.
        }
        cur = next;
        r.path.push_back(cur);
    }
}

}  // namespace sbf::forest
