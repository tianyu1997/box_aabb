// SafeBoxForest v5 — Critical Sampling Source (Phase B2)
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
