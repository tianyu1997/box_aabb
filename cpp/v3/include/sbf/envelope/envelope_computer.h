// SafeBoxForest v2 — Legacy Envelope Computer Interface
// Module: sbf::envelope
//
// *** DEPRECATED — prefer sbf::envelope::IEnvelopeComputer (envelope.h) ***
//
// This header is kept for backward compatibility with HierAABBTree and
// other v1-era consumers that use the flat EnvelopeResult struct.
// New code should use the abstract IEnvelope / IEnvelopeComputer from
// sbf/envelope/envelope.h and sbf/envelope/aabb_envelope.h.
//
#pragma once

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/common/types.h"
#include <vector>

namespace sbf {

// ─── Legacy envelope result (flat AABB arrays) ──────────────────────────────
// Prefer AabbEnvelope (sbf::envelope) for new code.
struct EnvelopeResult {
    std::vector<float> link_aabbs;    // [n_active_links * 6]
    std::vector<float> ee_aabbs;      // [n_ee_aabb_slots * 6]
    int n_link_slots = 0;
    int n_ee_slots = 0;
    bool valid = false;
    FKState fk_state;
};

// ─── Legacy abstract envelope computer ──────────────────────────────────────
// Prefer sbf::envelope::IEnvelopeComputer for new code.
class IEnvelopeComputer {
public:
    virtual ~IEnvelopeComputer() = default;

    virtual EnvelopeResult compute_envelope(
        const std::vector<Interval>& intervals) const = 0;

    virtual EnvelopeResult compute_envelope_incremental(
        const FKState& parent_fk,
        const std::vector<Interval>& intervals,
        int changed_dim) const = 0;

    virtual int n_total_aabb_slots() const = 0;

    virtual bool envelope_collides(const EnvelopeResult& env,
                                    const float* obs_compact,
                                    int n_obs) const = 0;
};

// ─── Interval FK Envelope Computer (legacy API) ─────────────────────────────
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
