// SafeBoxForest — Envelope Computer implementation
#include "sbf/envelope/envelope_computer.h"
#include "sbf/scene/aabb_collision_checker.h"

namespace sbf {

IntervalFKEnvelopeComputer::IntervalFKEnvelopeComputer(const Robot& robot)
    : robot_(&robot) {}

EnvelopeResult IntervalFKEnvelopeComputer::compute_envelope(
    const std::vector<Interval>& intervals) const
{
    EnvelopeResult result;
    result.fk_state = compute_fk_full(*robot_, intervals);
    extract_aabbs(result.fk_state, result);
    result.valid = true;
    return result;
}

EnvelopeResult IntervalFKEnvelopeComputer::compute_envelope_incremental(
    const FKState& parent_fk,
    const std::vector<Interval>& intervals,
    int changed_dim) const
{
    EnvelopeResult result;
    result.fk_state = compute_fk_incremental(parent_fk, *robot_, intervals, changed_dim);
    extract_aabbs(result.fk_state, result);
    result.valid = true;
    return result;
}

int IntervalFKEnvelopeComputer::n_total_aabb_slots() const {
    return robot_->n_active_links() + robot_->n_ee_aabb_slots();
}

bool IntervalFKEnvelopeComputer::envelope_collides(
    const EnvelopeResult& env,
    const float* obs_compact,
    int n_obs) const
{
    int total_slots = env.n_link_slots + env.n_ee_slots;
    // Combine link and EE AABBs into contiguous array for SAT test
    std::vector<float> combined(total_slots * 6);
    for (int i = 0; i < env.n_link_slots * 6; ++i)
        combined[i] = env.link_aabbs[i];
    for (int i = 0; i < env.n_ee_slots * 6; ++i)
        combined[env.n_link_slots * 6 + i] = env.ee_aabbs[i];
    return aabbs_collide_obs(combined.data(), total_slots, obs_compact, n_obs);
}

void IntervalFKEnvelopeComputer::extract_aabbs(
    const FKState& fk, EnvelopeResult& result) const
{
    result.n_link_slots = robot_->n_active_links();
    result.link_aabbs.resize(result.n_link_slots * 6);
    extract_link_aabbs(fk, robot_->active_link_map(), robot_->n_active_links(),
                       result.link_aabbs.data(), robot_->active_link_radii());

    result.n_ee_slots = robot_->n_ee_aabb_slots();
    if (result.n_ee_slots > 0) {
        result.ee_aabbs.resize(result.n_ee_slots * 6);
        if (robot_->has_ee_groups()) {
            extract_ee_group_aabbs(fk, robot_->ee_groups().data(),
                                   robot_->n_ee_groups(),
                                   robot_->ee_spheres_frame(),
                                   result.ee_aabbs.data());
        } else if (robot_->has_ee_spheres()) {
            extract_ee_sphere_aabbs(fk, robot_->ee_spheres().data(),
                                    robot_->n_ee_spheres(),
                                    robot_->ee_spheres_frame(),
                                    result.ee_aabbs.data());
        }
    }
}

} // namespace sbf
