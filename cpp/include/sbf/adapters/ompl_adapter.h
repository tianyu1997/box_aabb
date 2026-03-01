// SafeBoxForest — OMPL Adapter
// Provides SBF-accelerated StateValidityChecker, StateSampler, MotionValidator
// for OMPL planners (RRTConnect, RRT*, BIT*, PRM*, etc.)
#pragma once

#include "sbf/core/types.h"
#include "sbf/forest/collision.h"
#include "sbf/forest/safe_box_forest.h"

#ifdef SBF_WITH_OMPL
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSampler.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#endif

#include <Eigen/Core>
#include <string>
#include <vector>

namespace sbf {

#ifdef SBF_WITH_OMPL

// ─── SBF-accelerated validity checker ───────────────────────────────────────
// First checks if state is inside a known safe box (O(1) cache hit).
// Only falls back to full collision check on cache miss.
class SBFStateValidityChecker : public ompl::base::StateValidityChecker {
public:
    SBFStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
                             const SafeBoxForest& forest,
                             const CollisionChecker& checker);

    bool isValid(const ompl::base::State* state) const override;

    // Statistics
    int cache_hits() const { return cache_hits_; }
    int cache_misses() const { return cache_misses_; }
    int total_checks() const { return cache_hits_ + cache_misses_; }

private:
    const SafeBoxForest* forest_;
    const CollisionChecker* checker_;
    mutable int cache_hits_ = 0;
    mutable int cache_misses_ = 0;
};

// ─── SBF-accelerated state sampler ──────────────────────────────────────────
// Samples from inside known safe boxes (volume-weighted),
// significantly boosting valid sample ratio.
class SBFStateSampler : public ompl::base::StateSampler {
public:
    SBFStateSampler(const ompl::base::StateSpace* space,
                     const SafeBoxForest& forest,
                     double box_sample_prob = 0.7);

    void sampleUniform(ompl::base::State* state) override;
    void sampleUniformNear(ompl::base::State* state,
                            const ompl::base::State* near,
                            double distance) override;
    void sampleGaussian(ompl::base::State* state,
                         const ompl::base::State* mean,
                         double stdDev) override;

private:
    const SafeBoxForest* forest_;
    double box_sample_prob_;
    mutable std::mt19937 rng_;

    void sample_from_box(ompl::base::State* state) const;
    void sample_uniform_default(ompl::base::State* state) const;
};

// ─── SBF-accelerated motion validator ───────────────────────────────────────
// If both states are in the same box or adjacent boxes, motion is trivially valid.
class SBFMotionValidator : public ompl::base::MotionValidator {
public:
    SBFMotionValidator(const ompl::base::SpaceInformationPtr& si,
                        const SafeBoxForest& forest,
                        const CollisionChecker& checker,
                        double resolution = 0.05);

    bool checkMotion(const ompl::base::State* s1,
                     const ompl::base::State* s2) const override;
    bool checkMotion(const ompl::base::State* s1,
                     const ompl::base::State* s2,
                     std::pair<ompl::base::State*, double>& lastValid) const override;

private:
    const SafeBoxForest* forest_;
    const CollisionChecker* checker_;
    double resolution_;
};

#endif // SBF_WITH_OMPL

// ─── High-level OMPL adapter ────────────────────────────────────────────────
// Works whether or not OMPL is available (errors at runtime if not).
class OMPLPlannerAdapter {
public:
    OMPLPlannerAdapter() = default;

    // Supported algorithms
    enum class Algorithm {
        RRTConnect,
        RRTstar,
        InformedRRTstar,
        BITstar,
        ABITstar,
        PRMstar
    };

    // Setup the planner with SBF acceleration
    void setup(const Robot& robot,
               const std::vector<Obstacle>& obstacles,
               const SafeBoxForest& forest,
               Algorithm algo = Algorithm::RRTConnect,
               double timeout = 30.0);

    // Plan a path
    PlanningResult plan(const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal);

    // Check if OMPL is available at compile time
    static bool ompl_available();

    static std::string algorithm_name(Algorithm algo);

private:
    const Robot* robot_ = nullptr;
    const SafeBoxForest* forest_ = nullptr;
    CollisionChecker checker_;
    Algorithm algo_ = Algorithm::RRTConnect;
    double timeout_ = 30.0;
    bool setup_done_ = false;
};

} // namespace sbf
