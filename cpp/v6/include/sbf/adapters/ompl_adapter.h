// SafeBoxForest v6 — OMPL Adapter
// Provides SBF-accelerated StateValidityChecker, StateSampler, MotionValidator
// for OMPL planners (RRTConnect, RRT*, BIT*, PRM*, etc.)
//
// Ported from v1/include/sbf/adapters/ompl_adapter.h with v5 API changes.
#pragma once

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/scene/collision_checker.h>
#include <sbf/planner/sbf_planner.h>

#ifdef SBF_HAS_OMPL
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

#include <random>
#endif

#include <Eigen/Core>
#include <string>
#include <vector>

namespace sbf {

#ifdef SBF_HAS_OMPL

// ─── SBF-accelerated validity checker ───────────────────────────────────────
// First checks if state is inside a known safe box (O(1) cache hit).
// Only falls back to full collision check on cache miss.
class SBFStateValidityChecker : public ompl::base::StateValidityChecker {
public:
    SBFStateValidityChecker(
        const ompl::base::SpaceInformationPtr& si,
        const std::vector<BoxNode>& boxes,
        const CollisionChecker& checker)
        : ompl::base::StateValidityChecker(si)
        , boxes_(&boxes)
        , checker_(&checker)
    {}

    bool isValid(const ompl::base::State* state) const override {
        auto* rv = state->as<ompl::base::RealVectorStateSpace::StateType>();
        const int ndim = static_cast<int>(si_->getStateSpace()->getDimension());

        Eigen::VectorXd q(ndim);
        for (int i = 0; i < ndim; ++i)
            q[i] = rv->values[i];

        // Fast path: check if inside any known box
        for (const auto& box : *boxes_) {
            bool inside = true;
            for (int d = 0; d < ndim; ++d) {
                if (q[d] < box.joint_intervals[d].lo || q[d] > box.joint_intervals[d].hi) {
                    inside = false;
                    break;
                }
            }
            if (inside) {
                ++cache_hits_;
                return true;
            }
        }

        // Slow path: full collision check
        ++cache_misses_;
        return !checker_->check_config(q);
    }

    int cache_hits() const { return cache_hits_; }
    int cache_misses() const { return cache_misses_; }
    int total_checks() const { return cache_hits_ + cache_misses_; }

private:
    const std::vector<BoxNode>* boxes_;
    const CollisionChecker* checker_;
    mutable int cache_hits_ = 0;
    mutable int cache_misses_ = 0;
};

// ─── SBF-accelerated state sampler ──────────────────────────────────────────
// Samples from inside known safe boxes (volume-weighted),
// significantly boosting valid sample ratio.
class SBFStateSampler : public ompl::base::StateSampler {
public:
    SBFStateSampler(
        const ompl::base::StateSpace* space,
        const std::vector<BoxNode>& boxes,
        double box_sample_prob = 0.7)
        : ompl::base::StateSampler(space)
        , boxes_(&boxes)
        , box_sample_prob_(box_sample_prob)
        , rng_(std::random_device{}())
    {}

    void sampleUniform(ompl::base::State* state) override {
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        if (!boxes_->empty() && u01(rng_) < box_sample_prob_) {
            sample_from_box(state);
        } else {
            sample_uniform_default(state);
        }
    }

    void sampleUniformNear(ompl::base::State* state,
                           const ompl::base::State* near,
                           double distance) override {
        // Default: Gaussian around near with radius = distance
        sampleGaussian(state, near, distance / 3.0);
    }

    void sampleGaussian(ompl::base::State* state,
                        const ompl::base::State* mean,
                        double stdDev) override {
        auto* rv = state->as<ompl::base::RealVectorStateSpace::StateType>();
        auto* rv_mean = mean->as<ompl::base::RealVectorStateSpace::StateType>();
        const int ndim = static_cast<int>(space_->getDimension());
        auto* bounds = space_->as<ompl::base::RealVectorStateSpace>();

        std::normal_distribution<double> gauss(0.0, stdDev);
        for (int i = 0; i < ndim; ++i) {
            double v = rv_mean->values[i] + gauss(rng_);
            v = std::clamp(v, bounds->getBounds().low[i],
                           bounds->getBounds().high[i]);
            rv->values[i] = v;
        }
    }

private:
    const std::vector<BoxNode>* boxes_;
    double box_sample_prob_;
    mutable std::mt19937 rng_;

    void sample_from_box(ompl::base::State* state) const {
        auto* rv = state->as<ompl::base::RealVectorStateSpace::StateType>();
        const int ndim = static_cast<int>(space_->getDimension());
        std::uniform_int_distribution<int> box_dist(
            0, static_cast<int>(boxes_->size()) - 1);
        const auto& box = (*boxes_)[box_dist(rng_)];
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        for (int i = 0; i < ndim; ++i)
            rv->values[i] = box.joint_intervals[i].lo + u01(rng_) * (box.joint_intervals[i].hi - box.joint_intervals[i].lo);
    }

    void sample_uniform_default(ompl::base::State* state) const {
        auto* rv = state->as<ompl::base::RealVectorStateSpace::StateType>();
        const int ndim = static_cast<int>(space_->getDimension());
        auto* bounds = space_->as<ompl::base::RealVectorStateSpace>();
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        for (int i = 0; i < ndim; ++i) {
            double lo = bounds->getBounds().low[i];
            double hi = bounds->getBounds().high[i];
            rv->values[i] = lo + u01(rng_) * (hi - lo);
        }
    }
};

// ─── SBF-accelerated motion validator ───────────────────────────────────────
// If both states are in the same box, motion is trivially valid.
class SBFMotionValidator : public ompl::base::MotionValidator {
public:
    SBFMotionValidator(
        const ompl::base::SpaceInformationPtr& si,
        const std::vector<BoxNode>& boxes,
        const CollisionChecker& checker,
        double resolution = 0.05)
        : ompl::base::MotionValidator(si)
        , boxes_(&boxes)
        , checker_(&checker)
        , resolution_(resolution)
    {}

    bool checkMotion(const ompl::base::State* s1,
                     const ompl::base::State* s2) const override {
        const int ndim = static_cast<int>(si_->getStateSpace()->getDimension());

        Eigen::VectorXd q1(ndim), q2(ndim);
        auto* r1 = s1->as<ompl::base::RealVectorStateSpace::StateType>();
        auto* r2 = s2->as<ompl::base::RealVectorStateSpace::StateType>();
        for (int i = 0; i < ndim; ++i) {
            q1[i] = r1->values[i];
            q2[i] = r2->values[i];
        }

        // Fast path: both in same box → trivially valid
        for (const auto& box : *boxes_) {
            bool in1 = true, in2 = true;
            for (int d = 0; d < ndim; ++d) {
                if (q1[d] < box.joint_intervals[d].lo || q1[d] > box.joint_intervals[d].hi) in1 = false;
                if (q2[d] < box.joint_intervals[d].lo || q2[d] > box.joint_intervals[d].hi) in2 = false;
                if (!in1 && !in2) break;
            }
            if (in1 && in2) return true;
        }

        // Slow path: discrete collision check
        return !checker_->check_segment(q1, q2,
            std::max(2, static_cast<int>((q2 - q1).norm() / resolution_)));
    }

    bool checkMotion(const ompl::base::State* s1,
                     const ompl::base::State* s2,
                     std::pair<ompl::base::State*, double>& lastValid) const override {
        // Simple fallback: return full validity or nothing
        bool valid = checkMotion(s1, s2);
        if (valid) {
            lastValid.second = 1.0;
        } else {
            lastValid.second = 0.0;
            if (lastValid.first != nullptr)
                si_->copyState(lastValid.first, s1);
        }
        return valid;
    }

private:
    const std::vector<BoxNode>* boxes_;
    const CollisionChecker* checker_;
    double resolution_;
};

#endif  // SBF_HAS_OMPL

// ─── High-level OMPL adapter ────────────────────────────────────────────────
// Works whether or not OMPL is available (errors at runtime if not).
class OMPLPlannerAdapter {
public:
    OMPLPlannerAdapter() = default;

    enum class Algorithm {
        RRTConnect,
        RRTstar,
        InformedRRTstar,
        BITstar,
        ABITstar,
        PRMstar
    };

    /// Setup the planner with SBF acceleration
    void setup(const Robot& robot,
               const std::vector<Obstacle>& obstacles,
               const std::vector<BoxNode>& boxes,
               Algorithm algo = Algorithm::RRTConnect,
               double timeout = 30.0);

    /// Plan a path
    PlanResult plan(const Eigen::VectorXd& start,
                    const Eigen::VectorXd& goal);

    /// Check if OMPL is available at compile time
    static bool ompl_available() {
#ifdef SBF_HAS_OMPL
        return true;
#else
        return false;
#endif
    }

    static std::string algorithm_name(Algorithm algo) {
        switch (algo) {
            case Algorithm::RRTConnect:      return "RRTConnect";
            case Algorithm::RRTstar:         return "RRT*";
            case Algorithm::InformedRRTstar: return "Informed-RRT*";
            case Algorithm::BITstar:         return "BIT*";
            case Algorithm::ABITstar:        return "ABIT*";
            case Algorithm::PRMstar:         return "PRM*";
        }
        return "Unknown";
    }

    int cache_hits() const { return cache_hits_; }
    int cache_misses() const { return cache_misses_; }

private:
    const Robot* robot_ = nullptr;
    const std::vector<BoxNode>* boxes_ = nullptr;
    CollisionChecker checker_;
    Algorithm algo_ = Algorithm::RRTConnect;
    double timeout_ = 30.0;
    bool setup_done_ = false;
    int cache_hits_ = 0;
    int cache_misses_ = 0;
};

}  // namespace sbf
