/// @file scene_config.cpp
#include "sbf/scene/scene_config.h"
#include "sbf/scene/collision.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>

namespace sbf::scene {

std::vector<float> SceneConfig::packed_obstacles() const {
    std::vector<float> out(6 * obstacles.size());
    pack_obstacles(obstacles.data(), static_cast<int>(obstacles.size()),
                   out.data());
    return out;
}

SceneConfig load_scene_json(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("scene_config: cannot open " + path);
    nlohmann::json j;
    f >> j;

    SceneConfig sc;
    sc.name  = j.value("name", std::string{});
    sc.robot = j.at("robot").get<std::string>();

    auto qs = j.at("q_start").get<std::vector<double>>();
    auto qg = j.at("q_goal").get<std::vector<double>>();
    if (qs.size() != qg.size())
        throw std::runtime_error("scene_config: q_start/q_goal dim mismatch");
    sc.q_start = Eigen::Map<Eigen::VectorXd>(qs.data(), qs.size());
    sc.q_goal  = Eigen::Map<Eigen::VectorXd>(qg.data(), qg.size());

    if (j.contains("obstacles")) {
        for (const auto& o : j["obstacles"]) {
            auto lo = o.at("lo").get<std::vector<double>>();
            auto hi = o.at("hi").get<std::vector<double>>();
            if (lo.size() != 3 || hi.size() != 3)
                throw std::runtime_error("scene_config: obstacle lo/hi must be 3-D");
            sc.obstacles.emplace_back(
                static_cast<float>(lo[0]), static_cast<float>(lo[1]),
                static_cast<float>(lo[2]), static_cast<float>(hi[0]),
                static_cast<float>(hi[1]), static_cast<float>(hi[2]));
        }
    }
    return sc;
}

}  // namespace sbf::scene
