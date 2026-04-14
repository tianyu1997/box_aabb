// SafeBoxForest v6 — Critical Sampling Source (Phase B2)
#include <sbf/envelope/crit_source.h>
#include <sbf/envelope/dh_enumerate.h>

#include <vector>

namespace sbf {

EndpointIAABBResult compute_endpoint_iaabb_crit(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_samples,
    uint64_t seed,
    int changed_dim)
{
    (void)n_samples;
    (void)seed;
    (void)changed_dim;

    EndpointIAABBResult result;
    result.source = EndpointSource::CritSample;
    result.is_safe = false;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(result.endpoint_iaabb_len());

    const int n = robot.n_joints();
    const int n_act = result.n_active_links;
    const int* alm = robot.active_link_map();
    const auto& dh = robot.dh_params();

    init_endpoints_inf(result.endpoint_iaabbs.data(), n_act);

    // Collect candidates (narrow intervals → single midpoint)
    std::vector<std::vector<double>> candidates(n);
    for (int j = 0; j < n; ++j) {
        collect_kpi2(intervals[j].lo, intervals[j].hi, candidates[j]);
    }

    // Cap combinatorial explosion: if total combos > MAX_COMBOS,
    // reduce joints with the most candidates to {lo, mid, hi} (3 each).
    // For 7-DOF KUKA with v4 planning limits, root interval typically
    // produces ~5 × 3^6 = 3645 combos — well under budget.
    // This guard handles edge cases where intervals are still wide.
    static constexpr int64_t MAX_COMBOS = 8192;
    {
        int64_t total = 1;
        for (int j = 0; j < n; ++j) {
            total *= static_cast<int64_t>(candidates[j].size());
            if (total > MAX_COMBOS * 16) break;  // early exit for overflow
        }
        while (total > MAX_COMBOS) {
            // Find joint with most candidates
            int worst = 0;
            for (int j = 1; j < n; ++j)
                if (candidates[j].size() > candidates[worst].size()) worst = j;
            if (candidates[worst].size() <= 3) break;  // can't reduce further
            // Replace with {lo, mid, hi}
            double lo = intervals[worst].lo, hi = intervals[worst].hi;
            candidates[worst] = {lo, 0.5*(lo+hi), hi};
            // Recompute total
            total = 1;
            for (int j = 0; j < n; ++j)
                total *= static_cast<int64_t>(candidates[j].size());
        }
    }

    // Precompute DH matrices for all (joint, candidate) pairs
    std::vector<std::vector<PreDH>> pre_dh(n);
    std::vector<int> n_cands(n);
    for (int j = 0; j < n; ++j) {
        n_cands[j] = static_cast<int>(candidates[j].size());
        pre_dh[j].resize(n_cands[j]);
        for (int k = 0; k < n_cands[j]; ++k) {
            build_dh_matrix(dh[j], candidates[j][k], pre_dh[j][k].A);
        }
    }

    // Iterative enumeration with precomputed matrices
    enumerate_critical_iterative(robot, pre_dh, n_cands,
                                 alm, n_act,
                                 result.endpoint_iaabbs.data());

    return result;
}

}  // namespace sbf
