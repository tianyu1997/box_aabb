// SafeBoxForest v5 — IFK Source (Phase B1)
#include <sbf/envelope/ifk_source.h>

namespace sbf {

EndpointIAABBResult compute_endpoint_iaabb_ifk(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    FKState* fk,
    int changed_dim)
{
    EndpointIAABBResult result;
    result.source = EndpointSource::IFK;
    result.is_safe = true;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(result.endpoint_iaabb_len());

    if (changed_dim >= 0 && fk && fk->valid) {
        result.fk_state = compute_fk_incremental(*fk, robot, intervals, changed_dim);
    } else {
        result.fk_state = compute_fk_full(robot, intervals);
    }

    extract_endpoint_iaabbs(result.fk_state,
                            robot.active_link_map(),
                            robot.n_active_links(),
                            result.endpoint_iaabbs.data());

    if (fk) {
        *fk = result.fk_state;
    }

    return result;
}

}  // namespace sbf
