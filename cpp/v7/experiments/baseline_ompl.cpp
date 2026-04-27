/// @file baseline_ompl.cpp
/// @brief OMPL baseline (RRT-Connect) using the same SBF collision model.
///
/// Per-configuration validity uses `QFreeChecker` (zero-width interval
/// FK + LinkIAABB), so this baseline is apples-to-apples with the v7
/// SBF planner. Path interpolation step matches the SBF segment check
/// at 0.05 rad.
#include "experiments/common.h"
#include "experiments/baseline_collision.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/util/RandomNumbers.h>

#include <atomic>
#include <chrono>
#include <limits>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace {

double path_length(const og::PathGeometric& p) {
    return p.length();
}

bool is_prm_planner(const std::string& name) {
    return name == "prm" || name == "prm_star";
}

ob::PlannerPtr make_planner(const std::string& name,
                            const ob::SpaceInformationPtr& si) {
    if (name == "rrt_connect") {
        auto p = std::make_shared<og::RRTConnect>(si);
        p->setRange(0.30);
        return p;
    }
    if (name == "rrt_star") {
        auto p = std::make_shared<og::RRTstar>(si);
        p->setRange(0.30);
        return p;
    }
    if (name == "informed_rrt_star") {
        auto p = std::make_shared<og::InformedRRTstar>(si);
        p->setRange(0.30);
        return p;
    }
    if (name == "bit_star") {
        auto p = std::make_shared<og::BITstar>(si);
        return p;
    }
    if (name == "prm") {
        return std::make_shared<og::PRM>(si);
    }
    if (name == "prm_star") {
        return std::make_shared<og::PRMstar>(si);
    }
    std::cerr << "unknown planner: " << name << "\n";
    std::exit(2);
}

}  // namespace

int main(int argc, char** argv) {
    auto a  = sbf::exp::CliArgs::parse(argc, argv);
    auto sc = sbf::scene::load_scene_json(a.scene_path);

    auto robot = sbf::core::Robot::from_json(sbf::exp::robot_json_path(sc.robot));
    auto packed = sc.packed_obstacles();
    const int dof = robot.n_joints();

    nlohmann::json out = {
        {"experiment", "main"},
        {"baseline",   std::string("ompl_") + a.planner},
        {"scene",      sc.name},
        {"robot",      sc.robot},
        {"quick",      a.quick},
        {"seeds",      a.seeds},
        {"trials",     nlohmann::json::array()},
    };

    int    n_success = 0;
    double sum_total = 0.0, sum_len = 0.0;
    for (int s = 0; s < a.seeds; ++s) {
        // OMPL 1.5 uses a global RNG; seed it before any OMPL objects are constructed.
        ompl::RNG::setSeed(static_cast<uint32_t>(a.seed_base + static_cast<uint64_t>(s)));

        // Per-trial OMPL setup (clean planner state per seed).
        auto space = std::make_shared<ob::RealVectorStateSpace>(dof);
        ob::RealVectorBounds bounds(dof);
        for (int i = 0; i < dof; ++i) {
            bounds.setLow (i, robot.joint_limits().limits[i].lo);
            bounds.setHigh(i, robot.joint_limits().limits[i].hi);
        }
        space->setBounds(bounds);
        og::SimpleSetup ss(space);

        // Validity check: capture-by-shared-ptr so the lambda stays alive.
        auto checker = std::make_shared<sbf::exp::QFreeChecker>(
            robot, packed.empty() ? nullptr : packed.data(),
            static_cast<int>(sc.obstacles.size()));
        ss.setStateValidityChecker([checker, dof](const ob::State* st) {
            const auto* rv = st->as<ob::RealVectorStateSpace::StateType>();
            Eigen::VectorXd q(dof);
            for (int i = 0; i < dof; ++i) q[i] = rv->values[i];
            return checker->is_free(q);
        });
        ss.getSpaceInformation()->setStateValidityCheckingResolution(
            0.05 / space->getMaximumExtent());   // ~0.05 rad

        ob::ScopedState<ob::RealVectorStateSpace> qs(space), qg(space);
        for (int i = 0; i < dof; ++i) {
            qs->values[i] = sc.q_start[i];
            qg->values[i] = sc.q_goal[i];
        }
        ss.setStartAndGoalStates(qs, qg);

        auto planner = make_planner(a.planner, ss.getSpaceInformation());
        ss.setPlanner(planner);
        og::PRM* prm_planner = is_prm_planner(a.planner)
            ? dynamic_cast<og::PRM*>(planner.get())
            : nullptr;
        const bool optimizing_planner =
            a.planner == "rrt_star" ||
            a.planner == "informed_rrt_star" ||
            a.planner == "bit_star" ||
            a.planner == "prm_star";

        std::shared_ptr<ob::PathLengthOptimizationObjective> objective;
        std::atomic<bool> target_hit{false};
        double first_target_ms = std::numeric_limits<double>::quiet_NaN();
        double best_cost = std::numeric_limits<double>::infinity();
        auto t0 = std::chrono::steady_clock::now();

        if (optimizing_planner) {
            objective = std::make_shared<ob::PathLengthOptimizationObjective>(
                ss.getSpaceInformation());
            if (a.cost_threshold >= 0.0) {
                objective->setCostThreshold(ob::Cost(a.cost_threshold));
                ss.getProblemDefinition()->setIntermediateSolutionCallback(
                    [&](const ob::Planner*,
                        const std::vector<const ob::State*>&,
                        const ob::Cost cost) {
                        const double value = cost.value();
                        if (value < best_cost) {
                            best_cost = value;
                        }
                        if (!target_hit.load() && value <= a.cost_threshold) {
                            first_target_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - t0).count();
                            target_hit.store(true);
                        }
                    });
            }
            ss.setOptimizationObjective(objective);
        }

        const double timeout_s = a.quick ? 5.0 : static_cast<double>(a.timeout_s);
        const double ptc_interval_s = std::max(1e-3, std::min(0.05, timeout_s / 20.0));
        ob::PlannerTerminationCondition ptc =
            ob::timedPlannerTerminationCondition(timeout_s, ptc_interval_s);
        if (a.cost_threshold >= 0.0) {
            ob::PlannerTerminationCondition hit_target_ptc(
                [&target_hit]() { return target_hit.load(); });
            ptc = ob::plannerOrTerminationCondition(ptc, hit_target_ptc);
        }
        ob::PlannerStatus status = ss.solve(ptc);
        double total_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count();
        nlohmann::json build_time_json = nullptr;
        nlohmann::json query_time_json = nullptr;

        bool   ok   = static_cast<bool>(status);
        double len  = 0.0;
        int    nwp  = 0;
        if (ok) {
            if (!a.no_simplify) {
                ss.simplifySolution(0.5);
            }
            auto& p = ss.getSolutionPath();
            len = path_length(p);
            nwp = static_cast<int>(p.getStateCount());
            if (len < best_cost) {
                best_cost = len;
            }
            if (a.cost_threshold >= 0.0 && !target_hit.load() && len <= a.cost_threshold) {
                first_target_ms = total_ms;
                target_hit.store(true);
            }
            ++n_success;
            sum_total += total_ms;
            sum_len   += len;
        }
        if (prm_planner != nullptr) {
            const double build_ms = total_ms;
            if (ok) {
                prm_planner->clearQuery();
                const double query_timeout_s = std::min(timeout_s, 0.1);
                const double query_interval_s = std::max(1e-4, std::min(0.01, query_timeout_s / 10.0));
                const auto query_t0 = std::chrono::steady_clock::now();
                const ob::PlannerStatus query_status = prm_planner->solve(
                    ob::timedPlannerTerminationCondition(query_timeout_s, query_interval_s));
                const double query_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - query_t0).count();
                if (static_cast<bool>(query_status)) {
                    query_time_json = query_ms;
                }
            }
            build_time_json = build_ms;
        }
        out["trials"].push_back({
            {"seed",          s},
            {"success",       ok},
            {"total_time_ms", total_ms},
            {"build_time_ms", build_time_json},
            {"query_time_ms", query_time_json},
            {"path_length",   len},
            {"n_waypoints",   nwp},
            {"status",        status.asString()},
            {"cost_threshold", a.cost_threshold >= 0.0 ? nlohmann::json(a.cost_threshold) : nlohmann::json(nullptr)},
            {"target_hit",    target_hit.load()},
            {"target_time_ms", target_hit.load() ? nlohmann::json(first_target_ms) : nlohmann::json(nullptr)},
            {"best_cost",     std::isfinite(best_cost) ? nlohmann::json(best_cost) : nlohmann::json(nullptr)},
        });
        std::cout << "[ompl] seed=" << s
                  << " success=" << ok
                  << " len=" << len
                  << " t_ms=" << total_ms
                                    << " build_ms=" << (build_time_json.is_null() ? -1.0 : build_time_json.get<double>())
                                    << " query_ms=" << (query_time_json.is_null() ? -1.0 : query_time_json.get<double>())
                << " target_hit=" << target_hit.load()
                  << " (" << status.asString() << ")\n";
    }
    double sr = static_cast<double>(n_success) / std::max(1, a.seeds);
    out["summary"] = {
        {"success_rate",      sr},
        {"n_success",         n_success},
        {"avg_total_time_ms", n_success ? sum_total / n_success : 0.0},
        {"avg_path_length",   n_success ? sum_len   / n_success : 0.0},
    };
    if (!a.out_path.empty()) sbf::exp::write_json(a.out_path, out);
    std::cout << "[ompl] SR=" << sr << " (" << n_success << "/" << a.seeds
              << ")\n";
    return 0;
}
