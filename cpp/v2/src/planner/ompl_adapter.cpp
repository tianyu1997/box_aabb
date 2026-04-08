// SafeBoxForest v2 — OMPL adapter implementation
// Module: sbf (planner)
#include "sbf/adapters/ompl_adapter.h"
#include <stdexcept>
#include <random>
#include <algorithm>

#ifdef SBF_WITH_OMPL

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace sbf {

namespace og = ompl::geometric;
namespace ob = ompl::base;

// ─── State Validity Checker ──────────────────────────────────────────────────
SBFStateValidityChecker::SBFStateValidityChecker(
    const ob::SpaceInformationPtr& si,
    const SafeBoxForest& forest,
    const CollisionChecker& checker)
    : ob::StateValidityChecker(si),
      forest_(&forest),
      checker_(&checker) {}

bool SBFStateValidityChecker::isValid(const ob::State* state) const {
    auto* rvs = state->as<ob::RealVectorStateSpace::StateType>();
    int n = static_cast<int>(si_->getStateDimension());
    Eigen::VectorXd q(n);
    for (int i = 0; i < n; ++i) q[i] = rvs->values[i];

    // Fast path: check if inside a known safe box
    if (forest_ && forest_->find_containing(q)) {
        ++cache_hits_;
        return true;
    }

    ++cache_misses_;
    return !checker_->check_config(q);
}

// ─── State Sampler ───────────────────────────────────────────────────────────
SBFStateSampler::SBFStateSampler(const ob::StateSpace* space,
                                   const SafeBoxForest& forest,
                                   double box_sample_prob)
    : ob::StateSampler(space), forest_(&forest),
      box_sample_prob_(box_sample_prob),
      rng_(std::random_device{}()) {}

void SBFStateSampler::sample_from_box(ob::State* state) const {
    auto* rvs = state->as<ob::RealVectorStateSpace::StateType>();
    int n = static_cast<int>(space_->getDimension());

    // Build volume-weighted selection on the fly
    const auto& boxes = forest_->boxes();
    if (boxes.empty()) {
        sample_uniform_default(state);
        return;
    }

    // Collect cumulative volumes
    std::vector<int> ids;
    std::vector<double> cum;
    for (auto& [id, box] : boxes) {
        ids.push_back(id);
        cum.push_back((cum.empty() ? 0.0 : cum.back()) + box.volume);
    }

    std::uniform_real_distribution<double> unif(0.0, 1.0);
    double r = unif(rng_) * cum.back();
    auto it = std::lower_bound(cum.begin(), cum.end(), r);
    int idx = std::min(static_cast<int>(std::distance(cum.begin(), it)),
                       static_cast<int>(ids.size()) - 1);

    auto box_it = boxes.find(ids[idx]);
    if (box_it != boxes.end()) {
        const BoxNode& box = box_it->second;
        for (int d = 0; d < n; ++d) {
            rvs->values[d] = box.joint_intervals[d].lo +
                              unif(rng_) * box.joint_intervals[d].width();
        }
    } else {
        sample_uniform_default(state);
    }
}

void SBFStateSampler::sample_uniform_default(ob::State* state) const {
    auto* rvs = state->as<ob::RealVectorStateSpace::StateType>();
    int n = static_cast<int>(space_->getDimension());
    auto bounds = space_->as<ob::RealVectorStateSpace>()->getBounds();
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    for (int d = 0; d < n; ++d) {
        rvs->values[d] = bounds.low[d] +
                          unif(rng_) * (bounds.high[d] - bounds.low[d]);
    }
}

void SBFStateSampler::sampleUniform(ob::State* state) {
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    if (unif(rng_) < box_sample_prob_) {
        sample_from_box(state);
    } else {
        sample_uniform_default(state);
    }
}

void SBFStateSampler::sampleUniformNear(ob::State* state,
                                          const ob::State* near,
                                          double distance) {
    auto* rvs = state->as<ob::RealVectorStateSpace::StateType>();
    auto* nrvs = near->as<ob::RealVectorStateSpace::StateType>();
    int n = static_cast<int>(space_->getDimension());
    auto bounds = space_->as<ob::RealVectorStateSpace>()->getBounds();

    std::uniform_real_distribution<double> unif(-1.0, 1.0);
    for (int d = 0; d < n; ++d) {
        rvs->values[d] = nrvs->values[d] + unif(rng_) * distance;
        rvs->values[d] = std::clamp(rvs->values[d],
                                      bounds.low[d], bounds.high[d]);
    }
}

void SBFStateSampler::sampleGaussian(ob::State* state,
                                       const ob::State* mean,
                                       double stdDev) {
    auto* rvs = state->as<ob::RealVectorStateSpace::StateType>();
    auto* mrvs = mean->as<ob::RealVectorStateSpace::StateType>();
    int n = static_cast<int>(space_->getDimension());
    auto bounds = space_->as<ob::RealVectorStateSpace>()->getBounds();

    std::normal_distribution<double> norm(0.0, stdDev);
    for (int d = 0; d < n; ++d) {
        rvs->values[d] = mrvs->values[d] + norm(rng_);
        rvs->values[d] = std::clamp(rvs->values[d],
                                      bounds.low[d], bounds.high[d]);
    }
}

// ─── Motion Validator ────────────────────────────────────────────────────────
SBFMotionValidator::SBFMotionValidator(const ob::SpaceInformationPtr& si,
                                         const SafeBoxForest& forest,
                                         const CollisionChecker& checker,
                                         double resolution)
    : ob::MotionValidator(si), forest_(&forest), checker_(&checker),
      resolution_(resolution) {}

bool SBFMotionValidator::checkMotion(const ob::State* s1,
                                       const ob::State* s2) const {
    int n = static_cast<int>(si_->getStateDimension());
    auto* r1 = s1->as<ob::RealVectorStateSpace::StateType>();
    auto* r2 = s2->as<ob::RealVectorStateSpace::StateType>();

    Eigen::VectorXd q1(n), q2(n);
    for (int i = 0; i < n; ++i) {
        q1[i] = r1->values[i];
        q2[i] = r2->values[i];
    }

    // Fast path: if both in same box, motion is safe
    if (forest_) {
        auto box1 = forest_->find_containing(q1);
        auto box2 = forest_->find_containing(q2);
        if (box1 && box2 && box1->id == box2->id)
            return true;
    }

    return !checker_->check_segment(q1, q2, resolution_);
}

bool SBFMotionValidator::checkMotion(const ob::State* s1,
                                       const ob::State* s2,
                                       std::pair<ob::State*, double>& lastValid) const {
    bool valid = checkMotion(s1, s2);
    if (valid) {
        lastValid.second = 1.0;
    } else {
        lastValid.second = 0.0;
        if (lastValid.first) si_->copyState(lastValid.first, s1);
    }
    return valid;
}

} // namespace sbf

#endif // SBF_WITH_OMPL

namespace sbf {

// ─── OMPL Planner Adapter (always compiled) ─────────────────────────────────

void OMPLPlannerAdapter::setup(const Robot& robot,
                                const std::vector<Obstacle>& obstacles,
                                const SafeBoxForest& forest,
                                Algorithm algo,
                                double timeout) {
    robot_ = &robot;
    forest_ = &forest;
    checker_ = CollisionChecker(*robot_, obstacles);
    algo_ = algo;
    timeout_ = timeout;
    setup_done_ = true;
}

PlanningResult OMPLPlannerAdapter::plan(const Eigen::VectorXd& start,
                                          const Eigen::VectorXd& goal) {
#ifdef SBF_WITH_OMPL
    namespace ob = ompl::base;
    namespace og = ompl::geometric;

    if (!setup_done_)
        throw std::runtime_error("OMPLPlannerAdapter::plan() called before setup()");

    int n = robot_->n_joints();
    auto space = std::make_shared<ob::RealVectorStateSpace>(n);
    ob::RealVectorBounds bounds(n);
    for (int d = 0; d < n; ++d) {
        bounds.setLow(d, robot_->joint_limits().limits[d].lo);
        bounds.setHigh(d, robot_->joint_limits().limits[d].hi);
    }
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(
        std::make_shared<SBFStateValidityChecker>(si, *forest_, checker_));
    si->setMotionValidator(
        std::make_shared<SBFMotionValidator>(si, *forest_, checker_));
    si->setup();

    ob::ScopedState<ob::RealVectorStateSpace> s_start(space);
    ob::ScopedState<ob::RealVectorStateSpace> s_goal(space);
    for (int d = 0; d < n; ++d) {
        s_start[d] = start[d];
        s_goal[d] = goal[d];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(s_start, s_goal);
    pdef->setOptimizationObjective(
        std::make_shared<ob::PathLengthOptimizationObjective>(si));

    ob::PlannerPtr planner;
    switch (algo_) {
        case Algorithm::RRTConnect:
            planner = std::make_shared<og::RRTConnect>(si);
            break;
        case Algorithm::RRTstar:
            planner = std::make_shared<og::RRTstar>(si);
            break;
        case Algorithm::InformedRRTstar:
            planner = std::make_shared<og::InformedRRTstar>(si);
            break;
        case Algorithm::BITstar:
            planner = std::make_shared<og::BITstar>(si);
            break;
        case Algorithm::ABITstar:
            planner = std::make_shared<og::ABITstar>(si);
            break;
        case Algorithm::PRMstar:
            planner = std::make_shared<og::PRMstar>(si);
            break;
    }

    planner->setProblemDefinition(pdef);
    planner->setup();

    PlanningResult result;
    auto status = planner->solve(timeout_);

    if (status == ob::PlannerStatus::EXACT_SOLUTION ||
        status == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
        auto omplPath = std::dynamic_pointer_cast<og::PathGeometric>(
            pdef->getSolutionPath());
        if (omplPath) {
            int n_states = static_cast<int>(omplPath->getStateCount());
            result.path.resize(n_states, n);
            for (int i = 0; i < n_states; ++i) {
                auto* rvs = omplPath->getState(i)->as<
                    ob::RealVectorStateSpace::StateType>();
                for (int d = 0; d < n; ++d)
                    result.path(i, d) = rvs->values[d];
            }
            result.success = true;
            result.cost = omplPath->length();
        }
    }

    return result;
#else
    (void)start; (void)goal;
    throw std::runtime_error(
        "OMPL not available — rebuild with -DSBF_WITH_OMPL=ON");
#endif
}

bool OMPLPlannerAdapter::ompl_available() {
#ifdef SBF_WITH_OMPL
    return true;
#else
    return false;
#endif
}

std::string OMPLPlannerAdapter::algorithm_name(Algorithm algo) {
    switch (algo) {
        case Algorithm::RRTConnect:      return "RRTConnect";
        case Algorithm::RRTstar:         return "RRT*";
        case Algorithm::InformedRRTstar: return "InformedRRT*";
        case Algorithm::BITstar:         return "BIT*";
        case Algorithm::ABITstar:        return "ABIT*";
        case Algorithm::PRMstar:         return "PRM*";
    }
    return "Unknown";
}

} // namespace sbf
