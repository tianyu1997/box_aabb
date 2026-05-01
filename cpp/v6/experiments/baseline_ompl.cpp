/// @file baseline_ompl.cpp
/// @brief v6-native OMPL baselines using the v6 collision model.

#include <sbf/core/robot.h>
#include <sbf/core/types.h>
#include <sbf/scene/collision_checker.h>

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace {

struct SceneConfig {
    std::string name;
    std::string robot;
    std::vector<double> q_start;
    std::vector<double> q_goal;
    std::vector<sbf::Obstacle> obstacles;
};

struct CliArgs {
    bool quick = false;
    int seeds = 3;
    double timeout_s = 30.0;
    std::string scene_path;
    std::string out_path;
    std::string planner = "rrt_connect";
    double cost_threshold = -1.0;
    double prm_build_s = 2.0;
    double prm_query_s = 0.1;
    bool no_simplify = false;
    std::uint64_t seed_base = 42;
    int bitstar_samples_per_batch = -1;
    double bitstar_rewire_factor = -1.0;
    int bitstar_use_k_nearest = -1;
    int bitstar_pruning = -1;
    int bitstar_delay_rewiring = -1;
    int bitstar_jit_sampling = -1;
    int bitstar_drop_samples_on_prune = -1;
    int bitstar_consider_approximate = -1;

    static CliArgs parse(int argc, char** argv) {
        CliArgs args;
        for (int index = 1; index < argc; ++index) {
            std::string token = argv[index];
            auto eat = [&](const char* key) -> std::string {
                const std::string prefix = std::string("--") + key + "=";
                if (token.rfind(prefix, 0) == 0) {
                    return token.substr(prefix.size());
                }
                return {};
            };
            auto parse_bool = [](const std::string& value) -> int {
                if (value == "1" || value == "true" || value == "True" || value == "on" || value == "yes") {
                    return 1;
                }
                if (value == "0" || value == "false" || value == "False" || value == "off" || value == "no") {
                    return 0;
                }
                std::cerr << "expected boolean value, got: " << value << "\n";
                std::exit(2);
            };
            if (token == "--quick") {
                args.quick = true;
                args.seeds = 3;
                args.timeout_s = 30.0;
            } else if (token == "--full") {
                args.quick = false;
                args.seeds = 20;
                args.timeout_s = 120.0;
            } else if (auto value = eat("scene"); !value.empty()) {
                args.scene_path = value;
            } else if (auto value = eat("out"); !value.empty()) {
                args.out_path = value;
            } else if (auto value = eat("seeds"); !value.empty()) {
                args.seeds = std::stoi(value);
            } else if (auto value = eat("timeout"); !value.empty()) {
                args.timeout_s = std::stod(value);
            } else if (auto value = eat("planner"); !value.empty()) {
                args.planner = value;
            } else if (auto value = eat("cost-threshold"); !value.empty()) {
                args.cost_threshold = std::stod(value);
            } else if (auto value = eat("prm-build"); !value.empty()) {
                args.prm_build_s = std::stod(value);
            } else if (auto value = eat("prm-query"); !value.empty()) {
                args.prm_query_s = std::stod(value);
            } else if (auto value = eat("seed-base"); !value.empty()) {
                args.seed_base = static_cast<std::uint64_t>(std::stoull(value));
            } else if (auto value = eat("bitstar-samples-per-batch"); !value.empty()) {
                args.bitstar_samples_per_batch = std::stoi(value);
            } else if (auto value = eat("bitstar-rewire-factor"); !value.empty()) {
                args.bitstar_rewire_factor = std::stod(value);
            } else if (auto value = eat("bitstar-use-k-nearest"); !value.empty()) {
                args.bitstar_use_k_nearest = parse_bool(value);
            } else if (auto value = eat("bitstar-pruning"); !value.empty()) {
                args.bitstar_pruning = parse_bool(value);
            } else if (auto value = eat("bitstar-delay-rewiring"); !value.empty()) {
                args.bitstar_delay_rewiring = parse_bool(value);
            } else if (auto value = eat("bitstar-jit-sampling"); !value.empty()) {
                args.bitstar_jit_sampling = parse_bool(value);
            } else if (auto value = eat("bitstar-drop-samples-on-prune"); !value.empty()) {
                args.bitstar_drop_samples_on_prune = parse_bool(value);
            } else if (auto value = eat("bitstar-consider-approximate"); !value.empty()) {
                args.bitstar_consider_approximate = parse_bool(value);
            } else if (token == "--no-simplify") {
                args.no_simplify = true;
            } else {
                std::cerr << "unknown arg: " << token << "\n";
                std::exit(2);
            }
        }
        if (args.scene_path.empty()) {
            std::cerr << "usage: baseline_ompl --scene=<path.json> --out=<path.json> "
                         "[--planner=prm|bit_star] [--seeds=N] [--timeout=SEC]\n";
            std::exit(2);
        }
        return args;
    }
};

std::filesystem::path robot_json_path(const std::string& robot_id) {
    return std::filesystem::path(SBF_DATA_DIR) / (robot_id + ".json");
}

SceneConfig load_scene_json(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open scene json: " + path.string());
    }
    nlohmann::json raw;
    input >> raw;

    SceneConfig scene;
    scene.name = raw.value("name", path.stem().string());
    scene.robot = raw.at("robot").get<std::string>();
    scene.q_start = raw.at("q_start").get<std::vector<double>>();
    scene.q_goal = raw.at("q_goal").get<std::vector<double>>();
    for (const auto& item : raw.at("obstacles")) {
        const auto lo = item.at("lo").get<std::vector<float>>();
        const auto hi = item.at("hi").get<std::vector<float>>();
        if (lo.size() != 3 || hi.size() != 3) {
            throw std::runtime_error("obstacle bounds must be 3D in " + path.string());
        }
        scene.obstacles.emplace_back(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
    }
    return scene;
}

void write_json(const std::filesystem::path& path, const nlohmann::json& payload) {
    if (!path.empty()) {
        std::filesystem::create_directories(path.parent_path());
        std::ofstream output(path);
        if (!output) {
            throw std::runtime_error("cannot write json: " + path.string());
        }
        output << payload.dump(2) << "\n";
    }
}

double path_length(const og::PathGeometric& path) {
    return path.length();
}

bool is_prm_planner(const std::string& name) {
    return name == "prm" || name == "prm_star";
}

void configure_bitstar(const std::shared_ptr<og::BITstar>& planner, const CliArgs& args) {
    if (args.bitstar_samples_per_batch > 0) {
        planner->setSamplesPerBatch(static_cast<unsigned int>(args.bitstar_samples_per_batch));
    }
    if (args.bitstar_rewire_factor > 0.0) {
        planner->setRewireFactor(args.bitstar_rewire_factor);
    }
    if (args.bitstar_use_k_nearest >= 0) {
        planner->setUseKNearest(args.bitstar_use_k_nearest != 0);
    }
    if (args.bitstar_pruning >= 0) {
        planner->setPruning(args.bitstar_pruning != 0);
    }
    if (args.bitstar_delay_rewiring >= 0) {
        planner->setDelayRewiringUntilInitialSolution(args.bitstar_delay_rewiring != 0);
    }
    if (args.bitstar_jit_sampling >= 0) {
        planner->setJustInTimeSampling(args.bitstar_jit_sampling != 0);
    }
    if (args.bitstar_drop_samples_on_prune >= 0) {
        planner->setDropSamplesOnPrune(args.bitstar_drop_samples_on_prune != 0);
    }
    if (args.bitstar_consider_approximate >= 0) {
        planner->setConsiderApproximateSolutions(args.bitstar_consider_approximate != 0);
    }
}

ob::PlannerPtr make_planner(const std::string& name, const ob::SpaceInformationPtr& si, const CliArgs& args) {
    if (name == "rrt_connect") {
        auto planner = std::make_shared<og::RRTConnect>(si);
        planner->setRange(0.30);
        return planner;
    }
    if (name == "rrt_star") {
        auto planner = std::make_shared<og::RRTstar>(si);
        planner->setRange(0.30);
        return planner;
    }
    if (name == "informed_rrt_star") {
        auto planner = std::make_shared<og::InformedRRTstar>(si);
        planner->setRange(0.30);
        return planner;
    }
    if (name == "bit_star") {
        auto planner = std::make_shared<og::BITstar>(si);
        configure_bitstar(planner, args);
        return planner;
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
    const auto args = CliArgs::parse(argc, argv);
    const auto scene = load_scene_json(args.scene_path);
    const auto robot = sbf::Robot::from_json(robot_json_path(scene.robot).string());
    const int dof = robot.n_joints();

    nlohmann::json output = {
        {"experiment", "marcucci_ompl_v6"},
        {"baseline", std::string("ompl_") + args.planner},
        {"scene", scene.name},
        {"robot", scene.robot},
        {"quick", args.quick},
        {"seeds", args.seeds},
        {"collision_model", "v6_sbf_collision_checker"},
        {"state_validity_resolution_rad", 0.05},
        {"prm_build_budget_s", args.prm_build_s},
        {"prm_query_budget_s", args.prm_query_s},
        {"bitstar_params", {
            {"samples_per_batch", args.bitstar_samples_per_batch > 0 ? nlohmann::json(args.bitstar_samples_per_batch) : nlohmann::json(nullptr)},
            {"rewire_factor", args.bitstar_rewire_factor > 0.0 ? nlohmann::json(args.bitstar_rewire_factor) : nlohmann::json(nullptr)},
            {"use_k_nearest", args.bitstar_use_k_nearest >= 0 ? nlohmann::json(args.bitstar_use_k_nearest != 0) : nlohmann::json(nullptr)},
            {"pruning", args.bitstar_pruning >= 0 ? nlohmann::json(args.bitstar_pruning != 0) : nlohmann::json(nullptr)},
            {"delay_rewiring_until_initial_solution", args.bitstar_delay_rewiring >= 0 ? nlohmann::json(args.bitstar_delay_rewiring != 0) : nlohmann::json(nullptr)},
            {"jit_sampling", args.bitstar_jit_sampling >= 0 ? nlohmann::json(args.bitstar_jit_sampling != 0) : nlohmann::json(nullptr)},
            {"drop_samples_on_prune", args.bitstar_drop_samples_on_prune >= 0 ? nlohmann::json(args.bitstar_drop_samples_on_prune != 0) : nlohmann::json(nullptr)},
            {"consider_approximate", args.bitstar_consider_approximate >= 0 ? nlohmann::json(args.bitstar_consider_approximate != 0) : nlohmann::json(nullptr)},
        }},
        {"trials", nlohmann::json::array()},
    };

    int n_success = 0;
    double sum_total = 0.0;
    double sum_length = 0.0;
    for (int seed_index = 0; seed_index < args.seeds; ++seed_index) {
        ompl::RNG::setSeed(static_cast<std::uint32_t>(args.seed_base + static_cast<std::uint64_t>(seed_index)));

        auto space = std::make_shared<ob::RealVectorStateSpace>(dof);
        ob::RealVectorBounds bounds(dof);
        for (int joint = 0; joint < dof; ++joint) {
            bounds.setLow(joint, robot.joint_limits().limits[joint].lo);
            bounds.setHigh(joint, robot.joint_limits().limits[joint].hi);
        }
        space->setBounds(bounds);
        og::SimpleSetup setup(space);

        auto checker = std::make_shared<sbf::CollisionChecker>(robot, scene.obstacles);
        setup.setStateValidityChecker([checker, dof](const ob::State* state) {
            const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
            Eigen::VectorXd q(dof);
            for (int joint = 0; joint < dof; ++joint) {
                q[joint] = vector_state->values[joint];
            }
            return !checker->check_config(q);
        });
        setup.getSpaceInformation()->setStateValidityCheckingResolution(0.05 / space->getMaximumExtent());

        ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
        for (int joint = 0; joint < dof; ++joint) {
            q_start->values[joint] = scene.q_start[joint];
            q_goal->values[joint] = scene.q_goal[joint];
        }
        setup.setStartAndGoalStates(q_start, q_goal);

        auto planner = make_planner(args.planner, setup.getSpaceInformation(), args);
        setup.setPlanner(planner);
        auto* prm_planner = is_prm_planner(args.planner) ? dynamic_cast<og::PRM*>(planner.get()) : nullptr;
        auto* bitstar_planner = args.planner == "bit_star" ? dynamic_cast<og::BITstar*>(planner.get()) : nullptr;
        const bool optimizing_planner =
            args.planner == "rrt_star" ||
            args.planner == "informed_rrt_star" ||
            args.planner == "bit_star" ||
            args.planner == "prm_star";

        std::shared_ptr<ob::PathLengthOptimizationObjective> objective;
        std::atomic<bool> target_hit{false};
        double first_target_ms = std::numeric_limits<double>::quiet_NaN();
        double best_cost = std::numeric_limits<double>::infinity();
        const auto solve_start = std::chrono::steady_clock::now();

        if (optimizing_planner) {
            objective = std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation());
            if (args.cost_threshold >= 0.0) {
                objective->setCostThreshold(ob::Cost(args.cost_threshold));
                setup.getProblemDefinition()->setIntermediateSolutionCallback(
                    [&](const ob::Planner*, const std::vector<const ob::State*>&, const ob::Cost cost) {
                        const double value = cost.value();
                        if (value < best_cost) {
                            best_cost = value;
                        }
                        if (!target_hit.load() && value <= args.cost_threshold) {
                            first_target_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - solve_start).count();
                            target_hit.store(true);
                        }
                    });
            }
            setup.setOptimizationObjective(objective);
        }

        const double timeout_s = args.quick ? 5.0 : args.timeout_s;
        const double ptc_interval_s = std::max(1e-3, std::min(0.05, timeout_s / 20.0));
        ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(timeout_s, ptc_interval_s);
        if (args.cost_threshold >= 0.0) {
            ob::PlannerTerminationCondition hit_target_ptc([&target_hit]() { return target_hit.load(); });
            ptc = ob::plannerOrTerminationCondition(ptc, hit_target_ptc);
        }

        ob::PlannerStatus status = ob::PlannerStatus::UNKNOWN;
        double solve_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - solve_start).count();
        double total_ms = solve_ms;
        double simplify_ms = 0.0;
        double query_simplify_ms = 0.0;
        nlohmann::json build_time_json = nullptr;
        nlohmann::json query_time_json = nullptr;

        bool ok = false;
        double length = 0.0;
        int n_waypoints = 0;

        if (prm_planner != nullptr) {
            setup.setup();
            const auto build_start = std::chrono::steady_clock::now();
            prm_planner->constructRoadmap(
                ob::timedPlannerTerminationCondition(args.prm_build_s));
            solve_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - build_start).count();
            total_ms = solve_ms;
            build_time_json = solve_ms;

            prm_planner->clearQuery();
            const auto query_start = std::chrono::steady_clock::now();
            status = prm_planner->solve(
                ob::timedPlannerTerminationCondition(args.prm_query_s));
            double query_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - query_start).count();
            ok = static_cast<bool>(status);
            if (ok) {
                if (!args.no_simplify) {
                    const auto simplify_start = std::chrono::steady_clock::now();
                    setup.simplifySolution(0.5);
                    query_simplify_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - simplify_start).count();
                    query_ms += query_simplify_ms;
                }
                auto& path = setup.getSolutionPath();
                length = path_length(path);
                n_waypoints = static_cast<int>(path.getStateCount());
                if (length < best_cost) {
                    best_cost = length;
                }
                ++n_success;
                sum_total += query_ms;
                sum_length += length;
                query_time_json = query_ms;
            }
        } else {
            status = setup.solve(ptc);
            solve_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - solve_start).count();
            total_ms = solve_ms;
            ok = static_cast<bool>(status);
        }

        if (ok && prm_planner == nullptr) {
            if (!args.no_simplify && prm_planner == nullptr) {
                const auto simplify_start = std::chrono::steady_clock::now();
                setup.simplifySolution(0.5);
                simplify_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - simplify_start).count();
                total_ms += simplify_ms;
            }
            auto& path = setup.getSolutionPath();
            length = path_length(path);
            n_waypoints = static_cast<int>(path.getStateCount());
            if (length < best_cost) {
                best_cost = length;
            }
            if (args.cost_threshold >= 0.0 && !target_hit.load() && length <= args.cost_threshold) {
                first_target_ms = total_ms;
                target_hit.store(true);
            }
            ++n_success;
            sum_total += total_ms;
            sum_length += length;
        }

        const double bitstar_best_cost_value =
            bitstar_planner != nullptr ? bitstar_planner->bestCost().value() : std::numeric_limits<double>::quiet_NaN();
        output["trials"].push_back({
            {"seed", seed_index},
            {"success", ok},
            {"total_time_ms", total_ms},
            {"solve_time_ms", solve_ms},
            {"simplify_time_ms", simplify_ms},
            {"build_time_ms", build_time_json},
            {"query_time_ms", query_time_json},
            {"query_simplify_time_ms", query_simplify_ms},
            {"path_length", length},
            {"n_waypoints", n_waypoints},
            {"status", status.asString()},
            {"cost_threshold", args.cost_threshold >= 0.0 ? nlohmann::json(args.cost_threshold) : nlohmann::json(nullptr)},
            {"target_hit", target_hit.load()},
            {"target_time_ms", target_hit.load() ? nlohmann::json(first_target_ms) : nlohmann::json(nullptr)},
            {"best_cost", std::isfinite(best_cost) ? nlohmann::json(best_cost) : nlohmann::json(nullptr)},
            {"bitstar_iterations", bitstar_planner != nullptr ? nlohmann::json(bitstar_planner->numIterations()) : nlohmann::json(nullptr)},
            {"bitstar_batches", bitstar_planner != nullptr ? nlohmann::json(bitstar_planner->numBatches()) : nlohmann::json(nullptr)},
            {"bitstar_best_cost", std::isfinite(bitstar_best_cost_value) ? nlohmann::json(bitstar_best_cost_value) : nlohmann::json(nullptr)},
        });
        std::cout << "[ompl] seed=" << seed_index
                  << " success=" << ok
                  << " len=" << length
                  << " t_ms=" << total_ms
                  << " build_ms=" << (build_time_json.is_null() ? -1.0 : build_time_json.get<double>())
                  << " query_ms=" << (query_time_json.is_null() ? -1.0 : query_time_json.get<double>())
                  << " target_hit=" << target_hit.load()
                  << " (" << status.asString() << ")\n";
    }

    const double success_rate = static_cast<double>(n_success) / std::max(1, args.seeds);
    output["summary"] = {
        {"success_rate", success_rate},
        {"n_success", n_success},
        {"avg_total_time_ms", n_success ? sum_total / n_success : 0.0},
        {"avg_path_length", n_success ? sum_length / n_success : 0.0},
    };
    if (!args.out_path.empty()) {
        write_json(args.out_path, output);
    }
    std::cout << "[ompl] SR=" << success_rate << " (" << n_success << "/" << args.seeds << ")\n";
    return 0;
}