// SafeBoxForest v2 — Visualization Exporter implementation
#include "sbf/viz/viz_exporter.h"
#include "sbf/robot/fk.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace sbf {
namespace viz {

void export_forest_json(const SafeBoxForest& forest,
                         const std::string& filepath)
{
    export_forest_json(forest, {}, Eigen::MatrixXd(), filepath);
}

void export_forest_json(const SafeBoxForest& forest,
                         const std::vector<Obstacle>& obstacles,
                         const Eigen::MatrixXd& path,
                         const std::string& filepath)
{
    nlohmann::json j;

    // Boxes
    nlohmann::json boxes_arr = nlohmann::json::array();
    for (const auto& [id, bx] : forest.boxes()) {
        nlohmann::json bj;
        bj["id"] = id;
        nlohmann::json intervals = nlohmann::json::array();
        for (const auto& iv : bx.joint_intervals)
            intervals.push_back({iv.lo, iv.hi});
        bj["intervals"] = intervals;
        nlohmann::json seed = nlohmann::json::array();
        for (int d = 0; d < bx.seed_config.size(); ++d)
            seed.push_back(bx.seed_config[d]);
        bj["seed"] = seed;
        boxes_arr.push_back(bj);
    }
    j["boxes"] = boxes_arr;

    // Adjacency
    nlohmann::json adj_arr = nlohmann::json::array();
    for (const auto& [a, neighbors] : forest.adjacency()) {
        for (int b : neighbors) {
            if (a < b)
                adj_arr.push_back({a, b});
        }
    }
    j["adjacency"] = adj_arr;

    // Path
    if (path.rows() > 0) {
        nlohmann::json path_arr = nlohmann::json::array();
        for (int i = 0; i < path.rows(); ++i) {
            nlohmann::json row = nlohmann::json::array();
            for (int d = 0; d < path.cols(); ++d)
                row.push_back(path(i, d));
            path_arr.push_back(row);
        }
        j["path"] = path_arr;
    }

    // Obstacles
    if (!obstacles.empty()) {
        nlohmann::json obs_arr = nlohmann::json::array();
        for (const auto& obs : obstacles) {
            nlohmann::json oj;
            oj["name"] = obs.name;
            oj["center"] = {obs.center.x(), obs.center.y(), obs.center.z()};
            oj["half"] = {obs.half_sizes.x(), obs.half_sizes.y(), obs.half_sizes.z()};
            obs_arr.push_back(oj);
        }
        j["obstacles"] = obs_arr;
    }

    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        std::cerr << "[viz] failed to open " << filepath << "\n";
        return;
    }
    ofs << j.dump(2);
}

void export_path_frames(const Robot& robot,
                         const Eigen::MatrixXd& path,
                         const std::string& filepath)
{
    nlohmann::json j;
    nlohmann::json frames = nlohmann::json::array();

    for (int i = 0; i < path.rows(); ++i) {
        Eigen::VectorXd q = path.row(i).transpose();
        auto positions = fk_link_positions(robot, q);

        nlohmann::json frame;
        nlohmann::json q_arr = nlohmann::json::array();
        for (int d = 0; d < q.size(); ++d) q_arr.push_back(q[d]);
        frame["q"] = q_arr;

        nlohmann::json pos_arr = nlohmann::json::array();
        for (const auto& p : positions)
            pos_arr.push_back({p.x(), p.y(), p.z()});
        frame["link_positions"] = pos_arr;

        frames.push_back(frame);
    }

    j["frames"] = frames;

    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        std::cerr << "[viz] failed to open " << filepath << "\n";
        return;
    }
    ofs << j.dump(2);
}

void export_boxes_csv(const SafeBoxForest& forest,
                      const std::string& filepath)
{
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        std::cerr << "[viz] failed to open " << filepath << "\n";
        return;
    }

    int n_dims = 0;
    if (!forest.boxes().empty())
        n_dims = static_cast<int>(forest.boxes().begin()->second.joint_intervals.size());

    ofs << "id";
    for (int d = 0; d < n_dims; ++d)
        ofs << ",lo" << d << ",hi" << d;
    ofs << "\n";

    for (const auto& [id, bx] : forest.boxes()) {
        ofs << id;
        for (const auto& iv : bx.joint_intervals)
            ofs << "," << iv.lo << "," << iv.hi;
        ofs << "\n";
    }
}

} // namespace viz
} // namespace sbf
