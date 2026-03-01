// SafeBoxForest — JSON I/O implementation
#include "sbf/io/json_io.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace sbf {

Robot load_robot_json(const std::string& path) {
    return Robot::from_json(path);
}

std::vector<Obstacle> load_obstacles_json(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open())
        throw std::runtime_error("Cannot open obstacle file: " + path);

    nlohmann::json j;
    ifs >> j;

    std::vector<Obstacle> obs;
    auto& arr = j.is_array() ? j : j.at("obstacles");
    for (auto& item : arr) {
        Obstacle o;
        o.name = item.value("name", "obs_" + std::to_string(obs.size()));

        if (item.contains("center")) {
            auto& c = item["center"];
            o.center = Eigen::Vector3d(c[0].get<double>(),
                                        c[1].get<double>(),
                                        c[2].get<double>());
        }
        if (item.contains("half_sizes")) {
            auto& h = item["half_sizes"];
            o.half_sizes = Eigen::Vector3d(h[0].get<double>(),
                                            h[1].get<double>(),
                                            h[2].get<double>());
        } else if (item.contains("size")) {
            auto& s = item["size"];
            o.half_sizes = Eigen::Vector3d(s[0].get<double>() * 0.5,
                                            s[1].get<double>() * 0.5,
                                            s[2].get<double>() * 0.5);
        }
        obs.push_back(o);
    }
    return obs;
}

void save_obstacles_json(const std::string& path,
                          const std::vector<Obstacle>& obstacles) {
    nlohmann::json j = nlohmann::json::array();
    for (auto& o : obstacles) {
        nlohmann::json item;
        item["name"] = o.name;
        item["center"] = {o.center.x(), o.center.y(), o.center.z()};
        item["half_sizes"] = {o.half_sizes.x(), o.half_sizes.y(), o.half_sizes.z()};
        j.push_back(item);
    }

    std::ofstream ofs(path);
    if (!ofs.is_open())
        throw std::runtime_error("Cannot write obstacle file: " + path);
    ofs << j.dump(2);
}

void save_result_json(const std::string& path, const PlanningResult& result) {
    nlohmann::json j;
    j["success"] = result.success;
    j["cost"] = result.cost;
    j["planning_time"] = result.planning_time;
    j["first_solution_time"] = result.first_solution_time;
    j["collision_checks"] = result.collision_checks;
    j["nodes_explored"] = result.nodes_explored;

    // Path as array of arrays
    nlohmann::json path_j = nlohmann::json::array();
    for (int i = 0; i < result.path.rows(); ++i) {
        nlohmann::json row = nlohmann::json::array();
        for (int d = 0; d < result.path.cols(); ++d)
            row.push_back(result.path(i, d));
        path_j.push_back(row);
    }
    j["path"] = path_j;

    // Phase times
    nlohmann::json pt;
    for (auto& [key, val] : result.phase_times)
        pt[key] = val;
    j["phase_times"] = pt;

    std::ofstream ofs(path);
    if (!ofs.is_open())
        throw std::runtime_error("Cannot write result file: " + path);
    ofs << j.dump(2);
}

PlanningResult load_result_json(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open())
        throw std::runtime_error("Cannot open result file: " + path);

    nlohmann::json j;
    ifs >> j;

    PlanningResult result;
    result.success = j.value("success", false);
    result.cost = j.value("cost", 0.0);
    result.planning_time = j.value("planning_time", 0.0);
    result.first_solution_time = j.value("first_solution_time", 0.0);
    result.collision_checks = j.value("collision_checks", 0);
    result.nodes_explored = j.value("nodes_explored", 0);

    if (j.contains("path") && j["path"].is_array()) {
        auto& path_j = j["path"];
        int rows = static_cast<int>(path_j.size());
        if (rows > 0) {
            int cols = static_cast<int>(path_j[0].size());
            result.path.resize(rows, cols);
            for (int i = 0; i < rows; ++i)
                for (int d = 0; d < cols; ++d)
                    result.path(i, d) = path_j[i][d].get<double>();
        }
    }

    if (j.contains("phase_times") && j["phase_times"].is_object()) {
        for (auto& [key, val] : j["phase_times"].items())
            result.phase_times[key] = val.get<double>();
    }

    return result;
}

} // namespace sbf
