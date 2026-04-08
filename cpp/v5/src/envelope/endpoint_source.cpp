// SafeBoxForest v5 — Endpoint Source unified dispatch (Phase B5)
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/ifk_source.h>
#include <sbf/envelope/crit_source.h>
#include <sbf/envelope/analytical_source.h>
#include <sbf/envelope/gcpc_source.h>

#include <cassert>

namespace sbf {

EndpointIAABBResult compute_endpoint_iaabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config,
    FKState* fk,
    int changed_dim)
{
    switch (config.source) {
    case EndpointSource::IFK:
        return compute_endpoint_iaabb_ifk(robot, intervals, fk, changed_dim);

    case EndpointSource::CritSample:
        return compute_endpoint_iaabb_crit(robot, intervals, config.n_samples_crit,
                                           42, changed_dim);

    case EndpointSource::Analytical:
        return compute_endpoint_iaabb_analytical(robot, intervals,
                                                 config.max_phase_analytical);

    case EndpointSource::GCPC: {
        assert(config.gcpc_cache != nullptr);
        return compute_endpoint_iaabb_gcpc(robot, intervals, *config.gcpc_cache);
    }

    default:
        // Fallback to IFK
        return compute_endpoint_iaabb_ifk(robot, intervals, fk, changed_dim);
    }
}

void hull_endpoint_iaabbs(float* dst, const float* src, int n_endpoints) {
    for (int i = 0; i < n_endpoints; ++i) {
        const float* s = src + i * 6;
        float*       d = dst + i * 6;
        // lo = min(dst_lo, src_lo)
        for (int k = 0; k < 3; ++k)
            if (s[k] < d[k]) d[k] = s[k];
        // hi = max(dst_hi, src_hi)
        for (int k = 3; k < 6; ++k)
            if (s[k] > d[k]) d[k] = s[k];
    }
}

}  // namespace sbf
