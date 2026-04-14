// SafeBoxForest v6 — OMPL Adapter implementation
#include <sbf/adapters/ompl_adapter.h>
#include <chrono>
#include <stdexcept>

namespace sbf {

void OMPLPlannerAdapter::setup(
    const Robot& robot,
    const std::vector<Obstacle>& obstacles,
    const std::vector<BoxNode>& boxes,
    Algorithm algo,
    double timeout)
{
    robot_ = &robot;
    boxes_ = &boxes;
    checker_ = CollisionChecker(robot, obstacles);
    algo_ = algo;
    timeout_ = timeout;
    setup_done_ = true;
    cache_hits_ = 0;
    cache_misses_ = 0;
}

PlanResult OMPLPlannerAdapter::plan(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal)
{
    PlanResult result;
    result.success = false;

    if (!setup_done_) {
        return result;
    }

#ifdef SBF_HAS_OMPL
    namespace ob = ompl::base;
    namespace og = ompl::geometric;

    const int ndim = robot_->n_joints();
    auto space = std::make_shared<ob::RealVectorStateSpace>(ndim);

    // Set joint limits as bounds
    ob::RealVectorBounds bounds(ndim);
    for (int i = 0; i < ndim; ++i) {
        bounds.setLow(i, robot_->joint_limits().limits[i].lo);
        bounds.setHigh(i, robot_->joint_limits().limits[i].hi);
    }
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);

    // SBF-accelerated components
    auto checker = std::make_shared<SBFStateValidityChecker>(si, *boxes_, checker_);
    si->setStateValidityChecker(checker);

    auto mv = std::make_shared<SBFMotionValidator>(si, *boxes_, checker_, 0.05);
    si->setMotionValidator(mv);

    // Custom sampler via allocator
    space->setStateSamplerAllocator(
        [this](const ob::StateSpace* s) -> ob::StateSamplerPtr {
            return std::make_shared<SBFStateSampler>(s, *boxes_, 0.7);
        });

    si->setup();

    // Create start/goal states
    ob::ScopedState<ob::RealVectorStateSpace> s_start(space);
    ob::ScopedState<ob::RealVectorStateSpace> s_goal(space);
    for (int i = 0; i < ndim; ++i) {
        s_start[i] = start[i];
        s_goal[i] = goal[i];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(s_start, s_goal);

    // Create planner
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

    auto t0 = std::chrono::high_resolution_clock::now();
    ob::PlannerStatus status = planner->ob::Planner::solve(timeout_);
    auto t1 = std::chrono::high_resolution_clock::now();
    double dt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    result.planning_time_ms = dt_ms;

    if (status == ob::PlannerStatus::EXACT_SOLUTION ||
        status == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
        auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
        result.success = true;
        result.path.reserve(path->getStateCount());
        double total_len = 0.0;
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
            auto* rv = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
            Eigen::VectorXd q(ndim);
            for (int d = 0; d < ndim; ++d) q[d] = rv->values[d];
            if (i > 0)
                total_len += (q - result.path.back()).norm();
            result.path.push_back(std::move(q));
        }
        result.path_length = total_len;
    }

    cache_hits_ = checker->cache_hits();
    cache_misses_ = checker->cache_misses();
#else
    (void)start; (void)goal;
    throw std::runtime_error(
        "OMPLPlannerAdapter::plan() called but OMPL not available. "
        "Rebuild with -DSBF_HAS_OMPL=ON");
#endif

    return result;
}

}  // namespace sbf
