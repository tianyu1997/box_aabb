// SafeBoxForest v6 — Pure Monte Carlo endpoint source
#include <sbf/envelope/mc_source.h>

#include <sbf/core/fk_state.h>
#include <sbf/envelope/dh_enumerate.h>

#include <algorithm>
#include <random>
#include <vector>

namespace sbf {

EndpointIAABBResult compute_endpoint_iaabb_mc(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_samples,
    uint64_t seed,
    int changed_dim)
{
    (void)changed_dim;

    EndpointIAABBResult result;
    result.source = EndpointSource::MC;
    result.is_safe = false;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(result.endpoint_iaabb_len());

    const int n = robot.n_joints();
    const int n_act = result.n_active_links;
    const int* alm = robot.active_link_map();

    init_endpoints_inf(result.endpoint_iaabbs.data(), n_act);

    if (n_samples <= 0) n_samples = 1;

    std::mt19937_64 rng(seed);
    std::vector<std::uniform_real_distribution<double>> dists;
    dists.reserve(n);
    for (int j = 0; j < n; ++j) {
        const double lo = intervals[j].lo;
        const double hi = intervals[j].hi;
        dists.emplace_back(std::min(lo, hi), std::max(lo, hi));
    }

    std::vector<Interval> sample_ivs(n);
    std::vector<float> sample_ep(static_cast<size_t>(n_act) * 2 * 6);

    for (int s = 0; s < n_samples; ++s) {
        for (int j = 0; j < n; ++j) {
            const double q = dists[j](rng);
            sample_ivs[j] = Interval(q, q);
        }

        FKState fk = compute_fk_full(robot, sample_ivs);
        extract_endpoint_iaabbs(fk, alm, n_act, sample_ep.data());
        hull_endpoint_iaabbs(result.endpoint_iaabbs.data(),
                             sample_ep.data(), n_act * 2);
    }

    return result;
}

}  // namespace sbf
