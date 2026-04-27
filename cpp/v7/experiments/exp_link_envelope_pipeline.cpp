/// @file exp_link_envelope_pipeline.cpp
/// @brief v7 paper Exp. 2: link-envelope pipeline comparison.
///
/// Exp. 1-aligned protocol:
///   - reuse the four width bins from the endpoint-source experiment;
///   - sample one shared IIWA14 box set per width bin;
///   - keep the endpoint source fixed to CritSample;
///   - report subdivision and grid sweeps against the aggregated stratified
///     box set;
///   - time the full endpoint + envelope pipeline, not envelope-only;
///   - use exact inflated-AABB union volume for non-grid rows.
#include "experiments/common.h"
#include "experiments/endpoint_sources.h"

#include "sbf/core/endpoint_iaabb.h"
#include "sbf/envelope/envelope_type.h"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using sbf::core::Interval;

struct Args {
    bool quick = false;
    int n_boxes = 400;
    int n_repeats = 20;
    std::string robot = "iiwa14";
    std::filesystem::path out_path;
};

std::string eat_value(const std::string& s, const char* key) {
    std::string p = std::string("--") + key + "=";
    if (s.rfind(p, 0) == 0) return s.substr(p.size());
    return {};
}

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--quick") {
            a.quick = true;
            a.n_boxes = 20;
            a.n_repeats = 5;
        }
        else if (s == "--full") { a.quick = false; }
        else if (auto v = eat_value(s, "out"); !v.empty()) a.out_path = v;
        else if (auto v = eat_value(s, "robot"); !v.empty()) a.robot = v;
        else if (auto v = eat_value(s, "n-boxes"); !v.empty()) a.n_boxes = std::stoi(v);
        else if (auto v = eat_value(s, "repeats"); !v.empty()) a.n_repeats = std::stoi(v);
        else {
            std::cerr << "unknown arg: " << s << "\n";
            std::exit(2);
        }
    }
    if (a.out_path.empty()) {
        std::cerr << "usage: exp_link_envelope_pipeline --out=<json> "
                     "[--quick|--full] [--n-boxes=N] [--repeats=N] "
                     "[--robot=iiwa14]\n";
        std::exit(2);
    }
    return a;
}

std::vector<Interval> sample_box(const sbf::core::Robot& robot,
                                 double w_lo, double w_hi,
                                 std::mt19937_64& rng) {
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<Interval> iv(robot.n_joints());
    for (int d = 0; d < robot.n_joints(); ++d) {
        const auto lim = robot.joint_limits().limits[d];
        const double range = lim.hi - lim.lo;
        const double w = std::min(range, w_lo + u01(rng) * (w_hi - w_lo));
        const double lo = lim.lo + u01(rng) * std::max(1e-12, range - w);
        iv[d] = Interval{lo, lo + w};
    }
    return iv;
}

double aabb_volume(const std::vector<float>& aabbs) {
    double v = 0.0;
    for (std::size_t i = 0; i < aabbs.size(); i += 6) {
        const double dx = std::max(0.0f, aabbs[i + 3] - aabbs[i + 0]);
        const double dy = std::max(0.0f, aabbs[i + 4] - aabbs[i + 1]);
        const double dz = std::max(0.0f, aabbs[i + 5] - aabbs[i + 2]);
        v += dx * dy * dz;
    }
    return v;
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

double voxel_union_volume(const std::vector<float>& aabbs,
                          const double* radii,
                          int n_sub,
                          double delta) {
    sbf::voxel::SparseVoxelGrid grid(delta);
    const float pad = static_cast<float>(grid.safety_pad());
    for (std::size_t i = 0; i < aabbs.size(); i += 6) {
        const int slot = static_cast<int>(i / 6);
        const int ci = n_sub > 0 ? slot / n_sub : slot;
        const float r = (radii ? static_cast<float>(radii[ci]) : 0.0f) + pad;
        const float inflated[6] = {
            aabbs[i + 0] - r, aabbs[i + 1] - r, aabbs[i + 2] - r,
            aabbs[i + 3] + r, aabbs[i + 4] + r, aabbs[i + 5] + r,
        };
        grid.fill_aabb(inflated);
    }
    return grid.occupied_volume();
}

double mean(const std::vector<double>& xs) {
    return xs.empty() ? 0.0 : std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
}

double median(std::vector<double> xs) {
    if (xs.empty()) return 0.0;
    std::sort(xs.begin(), xs.end());
    return xs[xs.size() / 2];
}

}  // namespace

int main(int argc, char** argv) {
    const Args a = parse_args(argc, argv);
    auto robot = sbf::core::Robot::from_json(sbf::exp::robot_json_path(a.robot));

    struct Bin {
        const char* name;
        double lo;
        double hi;
    };
    const std::vector<Bin> bins = {
        {"0.001-0.05", 0.001, 0.05},
        {"0.05-0.10", 0.05, 0.10},
        {"0.10-0.20", 0.10, 0.20},
        {"0.20-0.50", 0.20, 0.50},
    };

    struct Variant {
        const char* stage;
        const char* name;
        sbf::envelope::EnvelopeType type;
        int n_sub;
        double delta;
    };
    const std::vector<Variant> variants = {
        {"subdivision", "LinkIAABB",      sbf::envelope::EnvelopeType::LinkIAABB,      1, 0.0},
        {"subdivision", "LinkIAABB_S2",   sbf::envelope::EnvelopeType::LinkIAABB,      2, 0.0},
        {"subdivision", "LinkIAABB_S4",   sbf::envelope::EnvelopeType::LinkIAABB,      4, 0.0},
        {"subdivision", "LinkIAABB_S8",   sbf::envelope::EnvelopeType::LinkIAABB,      8, 0.0},
        {"grid",        "LinkGrid_S4",    sbf::envelope::EnvelopeType::LinkIAABB_Grid, 4, 0.02},
        {"grid",        "LinkGrid_S4",    sbf::envelope::EnvelopeType::LinkIAABB_Grid, 4, 0.04},
        {"grid",        "LinkGrid_S4",    sbf::envelope::EnvelopeType::LinkIAABB_Grid, 4, 0.06},
        {"grid",        "LinkGrid_S4",    sbf::envelope::EnvelopeType::LinkIAABB_Grid, 4, 0.08},
        {"grid",        "Hull16Grid",     sbf::envelope::EnvelopeType::Hull16_Grid,    1, 0.02},
        {"grid",        "Hull16Grid",     sbf::envelope::EnvelopeType::Hull16_Grid,    1, 0.04},
        {"grid",        "Hull16Grid",     sbf::envelope::EnvelopeType::Hull16_Grid,    1, 0.06},
        {"grid",        "Hull16Grid",     sbf::envelope::EnvelopeType::Hull16_Grid,    1, 0.08},
    };

    nlohmann::json out = {
        {"experiment", "link_envelope_pipeline"},
        {"robot", a.robot},
        {"endpoint_source", "CritSample"},
        {"n_boxes_per_bin", a.n_boxes},
        {"n_bins", static_cast<int>(bins.size())},
        {"n_boxes_total", a.n_boxes * static_cast<int>(bins.size())},
        {"n_repeats", a.n_repeats},
        {"width_bins", nlohmann::json::array()},
        {"width_sampling_mode", "exp1_stratified_aggregate"},
        {"rows", nlohmann::json::array()},
    };

    for (const auto& bin : bins) {
        out["width_bins"].push_back({
            {"width_bin", bin.name},
            {"width_lo", bin.lo},
            {"width_hi", bin.hi},
            {"n_boxes", a.n_boxes},
        });
    }

    std::vector<std::vector<Interval>> boxes;
    boxes.reserve(static_cast<std::size_t>(a.n_boxes * static_cast<int>(bins.size())));
    std::mt19937_64 rng(0xE91AABB);
    for (const auto& bin : bins) {
        for (int i = 0; i < a.n_boxes; ++i) {
            boxes.push_back(sample_box(robot, bin.lo, bin.hi, rng));
        }
    }

    double reference_volume = 0.0;
    for (const auto& var : variants) {
        std::vector<double> vols;
        std::vector<double> times_us;
        std::vector<double> voxels;
        vols.reserve(boxes.size());
        voxels.reserve(boxes.size());
        times_us.reserve(static_cast<std::size_t>(boxes.size()) * std::max(1, a.n_repeats));
        for (const auto& iv : boxes) {
            sbf::envelope::EnvelopeTypeConfig cfg;
            cfg.type = var.type;
            cfg.n_subdivisions = var.n_sub;
            if (var.delta > 0.0) {
                cfg.grid_config.voxel_delta = var.delta;
            }
            const double effective_delta = cfg.grid_config.voxel_delta;
            for (int rep = 0; rep < a.n_repeats; ++rep) {
                auto t0 = Clock::now();
                auto ep = sbf::exp::endpoint_profile::endpoint_crit(robot, iv);
                auto env = sbf::envelope::compute_link_envelope(
                    ep.endpoint_iaabbs.data(), ep.n_active_links,
                    robot.active_link_radii(), cfg);
                auto t1 = Clock::now();
                times_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
                if (rep != 0) continue;

                double vol = 0.0;
                if (env.sparse_grid) {
                    vol = env.sparse_grid->occupied_volume();
                    voxels.push_back(static_cast<double>(env.sparse_grid->count_occupied()));
                } else {
                    vol = exact_inflated_union_volume(
                        env.link_iaabbs, robot.active_link_radii(), var.n_sub,
                        effective_delta);
                }
                vols.push_back(vol);
            }
        }
        if (std::string(var.name) == "LinkIAABB" && var.n_sub == 1) {
            reference_volume = mean(vols);
        }
        out["rows"].push_back({
            {"stage", var.stage},
            {"envelope", var.name},
            {"type", sbf::envelope::envelope_type_name(var.type)},
            {"n_subdivisions", var.n_sub},
            {"voxel_delta", var.delta > 0.0 ? var.delta : 0.05},
            {"volume_mean", mean(vols)},
            {"volume_median", median(vols)},
            {"time_us_mean", mean(times_us)},
            {"time_us_median", median(times_us)},
            {"voxel_count_mean", mean(voxels)},
            {"ratio_to_linkiaabb", reference_volume > 0.0 ? mean(vols) / reference_volume : 0.0},
        });
        std::cout << "[link-env] stage=" << var.stage
                  << " env=" << var.name
                << " delta=" << (var.delta > 0.0 ? var.delta : 0.05)
                  << " vol=" << mean(vols)
                  << " total_us=" << mean(times_us) << "\n";
    }

    sbf::exp::write_json(a.out_path, out);
    std::cout << "[link-env] wrote " << a.out_path << "\n";
    return 0;
}
