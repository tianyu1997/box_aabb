/// @file exp_compare_crit_boxes.cpp
/// @brief Dump v7 CritSample endpoint AABBs, zero-radius link IAABBs, and
///        downstream link-envelope volumes on a shared box set.

#include "experiments/common.h"
#include "experiments/endpoint_sources_profile.h"

#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/link_iaabb.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

using sbf::core::Interval;

struct Args {
    std::string robot = "iiwa14";
    std::string robot_json;
    int n_sub = 4;
    std::filesystem::path in_path;
    std::filesystem::path out_path;
};

std::string eat_value(const std::string& s, const char* key) {
    const std::string prefix = std::string("--") + key + "=";
    if (s.rfind(prefix, 0) == 0) return s.substr(prefix.size());
    return {};
}

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string s = argv[i];
        if (auto v = eat_value(s, "robot"); !v.empty()) a.robot = v;
        else if (auto v = eat_value(s, "robot-json"); !v.empty()) a.robot_json = v;
        else if (auto v = eat_value(s, "n-sub"); !v.empty()) a.n_sub = std::stoi(v);
        else if (auto v = eat_value(s, "in"); !v.empty()) a.in_path = v;
        else if (auto v = eat_value(s, "out"); !v.empty()) a.out_path = v;
        else {
            std::cerr << "unknown arg: " << s << "\n";
            std::exit(2);
        }
    }
    if (a.in_path.empty() || a.out_path.empty()) {
        std::cerr << "usage: exp_compare_crit_boxes --in=<boxes.json> --out=<dump.json> "
                     "[--robot=iiwa14] [--robot-json=/abs/path.json] [--n-sub=4]\n";
        std::exit(2);
    }
    if (a.n_sub < 1) {
        std::cerr << "--n-sub must be >= 1\n";
        std::exit(2);
    }
    return a;
}

nlohmann::json intervals_to_json(const std::vector<Interval>& intervals) {
    nlohmann::json out = nlohmann::json::array();
    for (const auto& iv : intervals) out.push_back({iv.lo, iv.hi});
    return out;
}

double exact_union_volume_3d(const std::vector<std::array<double, 6>>& boxes) {
    if (boxes.empty()) return 0.0;

    std::vector<double> xs;
    xs.reserve(boxes.size() * 2);
    for (const auto& box : boxes) {
        xs.push_back(box[0]);
        xs.push_back(box[3]);
    }
    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    if (xs.size() < 2) return 0.0;

    double total = 0.0;
    for (std::size_t xi = 0; xi + 1 < xs.size(); ++xi) {
        const double x0 = xs[xi];
        const double x1 = xs[xi + 1];
        if (!(x1 > x0)) continue;

        std::vector<std::array<double, 4>> rects;
        rects.reserve(boxes.size());
        for (const auto& box : boxes) {
            if (box[0] < x1 && box[3] > x0) {
                rects.push_back({box[1], box[4], box[2], box[5]});
            }
        }
        if (rects.empty()) continue;

        std::vector<double> ys;
        std::vector<double> zs;
        ys.reserve(rects.size() * 2);
        zs.reserve(rects.size() * 2);
        for (const auto& rect : rects) {
            ys.push_back(rect[0]);
            ys.push_back(rect[1]);
            zs.push_back(rect[2]);
            zs.push_back(rect[3]);
        }
        std::sort(ys.begin(), ys.end());
        ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
        std::sort(zs.begin(), zs.end());
        zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
        if (ys.size() < 2 || zs.size() < 2) continue;

        const int ny = static_cast<int>(ys.size() - 1);
        const int nz = static_cast<int>(zs.size() - 1);
        std::vector<unsigned char> covered(static_cast<std::size_t>(ny * nz), 0);

        for (const auto& rect : rects) {
            const int y0 = static_cast<int>(std::lower_bound(ys.begin(), ys.end(), rect[0]) - ys.begin());
            const int y1 = static_cast<int>(std::lower_bound(ys.begin(), ys.end(), rect[1]) - ys.begin());
            const int z0 = static_cast<int>(std::lower_bound(zs.begin(), zs.end(), rect[2]) - zs.begin());
            const int z1 = static_cast<int>(std::lower_bound(zs.begin(), zs.end(), rect[3]) - zs.begin());
            for (int yi = y0; yi < y1; ++yi) {
                for (int zi = z0; zi < z1; ++zi) {
                    covered[static_cast<std::size_t>(yi * nz + zi)] = 1;
                }
            }
        }

        double area = 0.0;
        for (int yi = 0; yi < ny; ++yi) {
            const double dy = ys[yi + 1] - ys[yi];
            if (!(dy > 0.0)) continue;
            for (int zi = 0; zi < nz; ++zi) {
                if (!covered[static_cast<std::size_t>(yi * nz + zi)]) continue;
                const double dz = zs[zi + 1] - zs[zi];
                if (dz > 0.0) area += dy * dz;
            }
        }
        total += (x1 - x0) * area;
    }
    return total;
}

double exact_inflated_union_volume(const std::vector<float>& aabbs,
                                   const double* radii,
                                   int n_sub,
                                   double delta) {
    const double pad = std::sqrt(3.0) * delta * 0.5;
    std::vector<std::array<double, 6>> boxes;
    boxes.reserve(aabbs.size() / 6);
    for (std::size_t i = 0; i < aabbs.size(); i += 6) {
        const int slot = static_cast<int>(i / 6);
        const int link_index = n_sub > 0 ? slot / n_sub : slot;
        const double radius = (radii ? radii[link_index] : 0.0) + pad;
        boxes.push_back({
            static_cast<double>(aabbs[i + 0]) - radius,
            static_cast<double>(aabbs[i + 1]) - radius,
            static_cast<double>(aabbs[i + 2]) - radius,
            static_cast<double>(aabbs[i + 3]) + radius,
            static_cast<double>(aabbs[i + 4]) + radius,
            static_cast<double>(aabbs[i + 5]) + radius,
        });
    }
    return exact_union_volume_3d(boxes);
}

struct VolumeVariant {
    const char* key;
    const char* stage;
    const char* name;
    sbf::envelope::EnvelopeType type;
    int n_sub;
    double delta;
};

const std::vector<VolumeVariant>& volume_variants() {
    static const std::vector<VolumeVariant> kVariants = {
        {"LinkIAABB",         "subdivision", "LinkIAABB",   sbf::envelope::EnvelopeType::LinkIAABB,      1, 0.0},
        {"LinkIAABB_S2",      "subdivision", "LinkIAABB_S2",sbf::envelope::EnvelopeType::LinkIAABB,      2, 0.0},
        {"LinkIAABB_S4",      "subdivision", "LinkIAABB_S4",sbf::envelope::EnvelopeType::LinkIAABB,      4, 0.0},
        {"LinkIAABB_S8",      "subdivision", "LinkIAABB_S8",sbf::envelope::EnvelopeType::LinkIAABB,      8, 0.0},
        {"LinkGrid_S4_d0.02", "grid",        "LinkGrid_S4", sbf::envelope::EnvelopeType::LinkIAABB_Grid, 4, 0.02},
        {"LinkGrid_S4_d0.04", "grid",        "LinkGrid_S4", sbf::envelope::EnvelopeType::LinkIAABB_Grid, 4, 0.04},
        {"LinkGrid_S4_d0.06", "grid",        "LinkGrid_S4", sbf::envelope::EnvelopeType::LinkIAABB_Grid, 4, 0.06},
        {"LinkGrid_S4_d0.08", "grid",        "LinkGrid_S4", sbf::envelope::EnvelopeType::LinkIAABB_Grid, 4, 0.08},
        {"Hull16Grid_d0.02",  "grid",        "Hull16Grid",  sbf::envelope::EnvelopeType::Hull16_Grid,    1, 0.02},
        {"Hull16Grid_d0.04",  "grid",        "Hull16Grid",  sbf::envelope::EnvelopeType::Hull16_Grid,    1, 0.04},
        {"Hull16Grid_d0.06",  "grid",        "Hull16Grid",  sbf::envelope::EnvelopeType::Hull16_Grid,    1, 0.06},
        {"Hull16Grid_d0.08",  "grid",        "Hull16Grid",  sbf::envelope::EnvelopeType::Hull16_Grid,    1, 0.08},
    };
    return kVariants;
}

std::vector<std::vector<Interval>> load_boxes(const std::filesystem::path& in_path) {
    std::ifstream f(in_path);
    if (!f) throw std::runtime_error("cannot read " + in_path.string());
    nlohmann::json j;
    f >> j;

    const nlohmann::json* boxes_json = &j;
    if (j.is_object()) {
        if (!j.contains("boxes")) {
            throw std::runtime_error("input json object must contain 'boxes'");
        }
        boxes_json = &j["boxes"];
    }
    if (!boxes_json->is_array()) {
        throw std::runtime_error("boxes payload must be an array");
    }

    std::vector<std::vector<Interval>> boxes;
    boxes.reserve(boxes_json->size());
    for (const auto& box_j : *boxes_json) {
        if (!box_j.is_array()) {
            throw std::runtime_error("each box must be an array of [lo, hi] pairs");
        }
        std::vector<Interval> ivs;
        ivs.reserve(box_j.size());
        for (const auto& iv_j : box_j) {
            if (!iv_j.is_array() || iv_j.size() != 2) {
                throw std::runtime_error("interval must be [lo, hi]");
            }
            ivs.push_back(Interval{iv_j[0].get<double>(), iv_j[1].get<double>()});
        }
        boxes.push_back(std::move(ivs));
    }
    return boxes;
}

}  // namespace

int main(int argc, char** argv) {
    const Args a = parse_args(argc, argv);
    const std::string robot_json = a.robot_json.empty()
        ? sbf::exp::robot_json_path(a.robot)
        : a.robot_json;
    auto robot = sbf::core::Robot::from_json(robot_json);
    const auto boxes = load_boxes(a.in_path);

    nlohmann::json out = {
        {"robot", a.robot},
        {"robot_json", robot_json},
        {"endpoint_source", "CritSample"},
        {"n_boxes", static_cast<int>(boxes.size())},
        {"n_sub", a.n_sub},
        {"volume_variants", nlohmann::json::array()},
        {"rows", nlohmann::json::array()},
    };

    for (const auto& var : volume_variants()) {
        const double effective_delta = var.delta > 0.0 ? var.delta : 0.05;
        out["volume_variants"].push_back({
            {"key", var.key},
            {"stage", var.stage},
            {"envelope", var.name},
            {"type", sbf::envelope::envelope_type_name(var.type)},
            {"n_subdivisions", var.n_sub},
            {"voxel_delta", effective_delta},
        });
    }

    for (std::size_t i = 0; i < boxes.size(); ++i) {
        const auto& iv = boxes[i];
        auto ep = sbf::exp::endpoint_profile::endpoint_crit(robot, iv);

        std::vector<float> paired_zero(static_cast<std::size_t>(ep.n_active_links) * 6, 0.0f);
        sbf::envelope::derive_link_iaabb_paired_zero(
            ep.endpoint_iaabbs.data(), ep.n_active_links, paired_zero.data());

        std::vector<float> subdiv_zero(
            static_cast<std::size_t>(ep.n_active_links) * static_cast<std::size_t>(a.n_sub) * 6,
            0.0f);
        sbf::envelope::derive_link_iaabb_subdivided_zero(
            ep.endpoint_iaabbs.data(), ep.n_active_links, a.n_sub, subdiv_zero.data());

        nlohmann::json downstream_volumes = nlohmann::json::object();
        for (const auto& var : volume_variants()) {
            sbf::envelope::EnvelopeTypeConfig cfg;
            cfg.type = var.type;
            cfg.n_subdivisions = var.n_sub;
            if (var.delta > 0.0) {
                cfg.grid_config.voxel_delta = var.delta;
            }
            const double effective_delta = cfg.grid_config.voxel_delta;
            auto env = sbf::envelope::compute_link_envelope(
                ep.endpoint_iaabbs.data(), ep.n_active_links,
                robot.active_link_radii(), cfg);

            const double volume = env.sparse_grid
                ? env.sparse_grid->occupied_volume()
                : exact_inflated_union_volume(
                    env.link_iaabbs, robot.active_link_radii(), var.n_sub,
                    effective_delta);
            const double voxel_count = env.sparse_grid
                ? static_cast<double>(env.sparse_grid->count_occupied())
                : 0.0;

            downstream_volumes[var.key] = {
                {"stage", var.stage},
                {"envelope", var.name},
                {"type", sbf::envelope::envelope_type_name(var.type)},
                {"n_subdivisions", var.n_sub},
                {"voxel_delta", effective_delta},
                {"volume", volume},
                {"voxel_count", voxel_count},
            };
        }

        out["rows"].push_back({
            {"index", static_cast<int>(i)},
            {"intervals", intervals_to_json(iv)},
            {"n_active_links", ep.n_active_links},
            {"endpoint_iaabbs", ep.endpoint_iaabbs},
            {"link_iaabbs_zero_paired", paired_zero},
            {"link_iaabbs_zero_subdivided", subdiv_zero},
            {"downstream_volumes", downstream_volumes},
        });
    }

    sbf::exp::write_json(a.out_path, out);
    std::cout << "wrote " << a.out_path << " with " << boxes.size() << " boxes\n";
    return 0;
}