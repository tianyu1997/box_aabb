// SafeBoxForest — Envelope Computer Interface and Implementation
// Module: sbf::envelope
// Given a robot + C-space box, computes the full workspace envelope (link AABBs)
#pragma once

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/common/types.h"
#include <vector>

namespace sbf {

// ─── Envelope result ────────────────────────────────────────────────────────
struct EnvelopeResult {
    std::vector<float> link_aabbs;    // [n_active_links * 6]
    std::vector<float> ee_aabbs;      // [n_ee_aabb_slots * 6]
    int n_link_slots = 0;
    int n_ee_slots = 0;
    bool valid = false;
    FKState fk_state;                 // FK state for further use
};

// ─── Abstract envelope computer interface ───────────────────────────────────
class IEnvelopeComputer {
public:
    virtual ~IEnvelopeComputer() = default;

    // Compute full envelope for the given box intervals
    virtual EnvelopeResult compute_envelope(
        const std::vector<Interval>& intervals) const = 0;

    // Compute incremental envelope (after changing one dimension)
    virtual EnvelopeResult compute_envelope_incremental(
        const FKState& parent_fk,
        const std::vector<Interval>& intervals,
        int changed_dim) const = 0;

    // Number of AABB slots (links + EE)
    virtual int n_total_aabb_slots() const = 0;

    // Check if envelope collides with compact obstacle array
    virtual bool envelope_collides(const EnvelopeResult& env,
                                    const float* obs_compact,
                                    int n_obs) const = 0;
};

// ─── Interval FK Envelope Computer ──────────────────────────────────────────
// Concrete implementation using interval FK + AABB extraction
class IntervalFKEnvelopeComputer : public IEnvelopeComputer {
public:
    IntervalFKEnvelopeComputer() = default;
    explicit IntervalFKEnvelopeComputer(const Robot& robot);

    EnvelopeResult compute_envelope(
        const std::vector<Interval>& intervals) const override;

    EnvelopeResult compute_envelope_incremental(
        const FKState& parent_fk,
        const std::vector<Interval>& intervals,
        int changed_dim) const override;

    int n_total_aabb_slots() const override;

    bool envelope_collides(const EnvelopeResult& env,
                            const float* obs_compact,
                            int n_obs) const override;

    const Robot& robot() const { return *robot_; }

private:
    const Robot* robot_ = nullptr;

    void extract_aabbs(const FKState& fk, EnvelopeResult& result) const;
};

} // namespace sbf
