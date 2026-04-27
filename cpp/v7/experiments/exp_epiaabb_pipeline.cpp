/// @file exp_epiaabb_pipeline.cpp
/// @brief v7 paper Exp. 1: endpoint iAABB source-pipeline comparison.
///
/// v7 currently exposes IFK as the certified endpoint source.  For the paper
/// comparison we benchmark IFK against two non-certified scalar-FK reference
/// sources (corner enumeration and Monte-Carlo sampling) using the same output
/// format, so the table separates certified hot-path behaviour from tightness
/// references.
#include "experiments/common.h"
#include "experiments/endpoint_sources.h"

#include "sbf/core/endpoint_iaabb.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
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
    int mc_samples = 0;
    bool rho_provided = false;
    double mc_density_rho = 0.0;
    double mc_reference_width = 0.35;
    int mc_ref_samples = 2'000'000;
    int mc_min_samples = 1'000;
    int mc_max_samples = 10'000'000;
    int analytical_phase = 3;
    bool bypass_narrow_skip = true;
    std::string robot = "iiwa14";
    std::string gcpc_path;
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
            a.mc_ref_samples = 50'000;
        }
        else if (s == "--full") { a.quick = false; }
        else if (auto v = eat_value(s, "out"); !v.empty()) a.out_path = v;
        else if (auto v = eat_value(s, "robot"); !v.empty()) a.robot = v;
        else if (auto v = eat_value(s, "n-boxes"); !v.empty()) a.n_boxes = std::stoi(v);
        else if (auto v = eat_value(s, "mc-samples"); !v.empty()) a.mc_samples = std::stoi(v);
        else if (auto v = eat_value(s, "rho"); !v.empty()) {
            a.rho_provided = true;
            a.mc_density_rho = std::stod(v);
        }
        else if (auto v = eat_value(s, "ref-samples"); !v.empty()) a.mc_ref_samples = std::stoi(v);
        else if (auto v = eat_value(s, "min-samples"); !v.empty()) a.mc_min_samples = std::stoi(v);
        else if (auto v = eat_value(s, "max-samples"); !v.empty()) a.mc_max_samples = std::stoi(v);
        else if (auto v = eat_value(s, "analytical-phase"); !v.empty()) a.analytical_phase = std::stoi(v);
        else if (auto v = eat_value(s, "gcpc"); !v.empty()) a.gcpc_path = v;
        else if (s == "--no-bypass-narrow-skip") a.bypass_narrow_skip = false;
        else {
            std::cerr << "unknown arg: " << s << "\n";
            std::exit(2);
        }
    }
    if (a.out_path.empty()) {
        std::cerr << "usage: exp_epiaabb_pipeline --out=<json> [--quick|--full] "
                     "[--n-boxes=N] [--mc-samples=N] [--rho=R] "
                     "[--ref-samples=N] [--min-samples=N] [--max-samples=N] "
                     "[--robot=iiwa14] "
                     "[--gcpc=<path>] [--analytical-phase=0..3]\n";
        std::exit(2);
    }
    return a;
}

double joint_box_volume(const std::vector<Interval>& intervals) {
    double vol = 1.0;
    for (const auto& iv : intervals) vol *= std::max(0.0, iv.width());
    return vol;
}

double geometric_mean_width(const std::vector<Interval>& intervals) {
    if (intervals.empty()) return 0.0;
    const double vol = joint_box_volume(intervals);
    return std::pow(std::max(0.0, vol), 1.0 / static_cast<double>(intervals.size()));
}

double mc_density_rho(const Args& a) {
    if (a.rho_provided) return a.mc_density_rho;
    return static_cast<double>(a.mc_ref_samples) / a.mc_reference_width;
}

int mc_samples_for_box(const Args& a,
                       const std::vector<Interval>& intervals) {
    if (a.mc_samples > 0) return a.mc_samples;
    const double raw = mc_density_rho(a) * geometric_mean_width(intervals);
    const long long rounded = static_cast<long long>(std::llround(raw));
    const long long clipped = std::clamp(
        rounded,
        static_cast<long long>(a.mc_min_samples),
        static_cast<long long>(a.mc_max_samples));
    return static_cast<int>(clipped);
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

void init_union(std::vector<float>& out) {
    for (std::size_t i = 0; i < out.size(); i += 6) {
        out[i + 0] = out[i + 1] = out[i + 2] =  std::numeric_limits<float>::infinity();
        out[i + 3] = out[i + 4] = out[i + 5] = -std::numeric_limits<float>::infinity();
    }
}

void union_endpoint(std::vector<float>& acc, const std::vector<float>& one) {
    for (std::size_t i = 0; i < acc.size(); i += 6) {
        acc[i + 0] = std::min(acc[i + 0], one[i + 0]);
        acc[i + 1] = std::min(acc[i + 1], one[i + 1]);
        acc[i + 2] = std::min(acc[i + 2], one[i + 2]);
        acc[i + 3] = std::max(acc[i + 3], one[i + 3]);
        acc[i + 4] = std::max(acc[i + 4], one[i + 4]);
        acc[i + 5] = std::max(acc[i + 5], one[i + 5]);
    }
}

double endpoint_volume(const std::vector<float>& ep) {
    double v = 0.0;
    for (std::size_t i = 0; i < ep.size(); i += 6) {
        const double dx = std::max(0.0f, ep[i + 3] - ep[i + 0]);
        const double dy = std::max(0.0f, ep[i + 4] - ep[i + 1]);
        const double dz = std::max(0.0f, ep[i + 5] - ep[i + 2]);
        v += dx * dy * dz;
    }
    return v;
}

std::vector<float> endpoint_ifk(const sbf::core::Robot& robot,
                                const std::vector<Interval>& iv) {
    return sbf::core::compute_endpoint_iaabb_ifk(robot, iv).endpoint_iaabbs;
}

std::vector<float> endpoint_corners(const sbf::core::Robot& robot,
                                    const std::vector<Interval>& iv) {
    const int dof = robot.n_joints();
    std::vector<float> acc(robot.n_active_links() * 2 * 6);
    init_union(acc);
    std::vector<Interval> q(dof);
    const std::uint64_t n = 1ULL << std::min(dof, 20);
    for (std::uint64_t mask = 0; mask < n; ++mask) {
        for (int d = 0; d < dof; ++d) {
            const double x = (mask & (1ULL << d)) ? iv[d].hi : iv[d].lo;
            q[d] = Interval{x, x};
        }
        union_endpoint(acc, endpoint_ifk(robot, q));
    }
    return acc;
}

std::vector<float> endpoint_mc(const sbf::core::Robot& robot,
                               const std::vector<Interval>& iv,
                               int n_samples,
                               std::mt19937_64& rng) {
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const int dof = robot.n_joints();
    std::vector<float> acc(robot.n_active_links() * 2 * 6);
    init_union(acc);
    std::vector<Interval> q(dof);
    for (int s = 0; s < n_samples; ++s) {
        for (int d = 0; d < dof; ++d) {
            const double x = iv[d].lo + u01(rng) * (iv[d].hi - iv[d].lo);
            q[d] = Interval{x, x};
        }
        union_endpoint(acc, endpoint_ifk(robot, q));
    }
    return acc;
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
    auto gcpc = sbf::exp::endpoint_profile::GcpcCache::load(a.gcpc_path);
    const bool include_gcpc = !gcpc.empty();

    struct Bin { const char* name; double lo; double hi; };
    const std::vector<Bin> bins = {
        {"0.001-0.05", 0.001, 0.05}, {"0.05-0.10", 0.05, 0.10},
        {"0.10-0.20", 0.10, 0.20}, {"0.20-0.50", 0.20, 0.50},
    };

    nlohmann::json out = {
        {"experiment", "epiaabb_pipeline"},
        {"robot", a.robot},
        {"n_boxes_per_bin", a.n_boxes},
        {"mc_samples", a.mc_samples > 0 ? a.mc_samples : a.mc_ref_samples},
        {"mc_sampling_mode", a.mc_samples > 0 ? "fixed" : "width_proportional"},
        {"mc_density_rho", mc_density_rho(a)},
        {"mc_reference_width", a.mc_reference_width},
        {"mc_reference_samples", a.mc_ref_samples},
        {"mc_min_samples", a.mc_min_samples},
        {"mc_max_samples", a.mc_max_samples},
        {"analytical_phase", a.analytical_phase},
        {"bypass_narrow_skip", a.bypass_narrow_skip},
        {"gcpc_cache", a.gcpc_path},
        {"gcpc_loaded", !gcpc.empty()},
        {"bin_stats", nlohmann::json::array()},
        {"rows", nlohmann::json::array()},
    };

    std::mt19937_64 rng(0xE91AABB);
    std::uint64_t box_counter = 0;
    for (const auto& bin : bins) {
        std::vector<std::vector<Interval>> boxes;
        boxes.reserve(a.n_boxes);
        for (int i = 0; i < a.n_boxes; ++i) boxes.push_back(sample_box(robot, bin.lo, bin.hi, rng));

        std::vector<int> mc_samples_used;
        mc_samples_used.reserve(boxes.size());
        std::vector<double> geo_mean_widths;
        geo_mean_widths.reserve(boxes.size());

        struct Acc {
            const char* source;
            bool certified;
            std::vector<double> vols;
            std::vector<double> times_us;
            std::vector<double> gaps;
            std::vector<double> gcpc_hits;
        };
        std::vector<Acc> accs = {
            {"IFK", true, {}, {}, {}, {}},
            {"CritSample", false, {}, {}, {}, {}},
            {"Analytical", true, {}, {}, {}, {}},
            {"MC", false, {}, {}, {}, {}},
        };
        if (include_gcpc) {
            accs.insert(accs.begin() + 3,
                        {"GCPC", true, {}, {}, {}, {}});
        }

        for (const auto& iv : boxes) {
            const std::uint64_t seed = 0xC0FFEEULL + box_counter++;
            const double box_geo_mean_width = geometric_mean_width(iv);
            const int n_mc = mc_samples_for_box(a, iv);
            geo_mean_widths.push_back(box_geo_mean_width);
            mc_samples_used.push_back(n_mc);
            auto t0_mc = Clock::now();
            auto mc_ref = sbf::exp::endpoint_profile::endpoint_mc(robot, iv, n_mc, seed);
            auto t1_mc = Clock::now();
            const double mc_time_us = std::chrono::duration<double, std::micro>(t1_mc - t0_mc).count();

            for (auto& acc : accs) {
                sbf::exp::endpoint_profile::EndpointResult ep;
                double t_us = 0.0;
                if (std::string(acc.source) == "MC") {
                    ep = mc_ref;
                    t_us = mc_time_us;
                } else {
                    auto t0 = Clock::now();
                    if (std::string(acc.source) == "IFK") {
                        ep = sbf::exp::endpoint_profile::endpoint_ifk(robot, iv);
                    } else if (std::string(acc.source) == "CritSample") {
                        ep = sbf::exp::endpoint_profile::endpoint_crit(robot, iv);
                    } else if (std::string(acc.source) == "Analytical") {
                        ep = sbf::exp::endpoint_profile::endpoint_analytical(
                            robot, iv, a.analytical_phase, a.bypass_narrow_skip);
                    } else {
                        ep = sbf::exp::endpoint_profile::endpoint_gcpc(
                            robot, iv, gcpc, a.analytical_phase, a.bypass_narrow_skip);
                    }
                    auto t1 = Clock::now();
                    t_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
                }
                acc.vols.push_back(sbf::exp::endpoint_profile::endpoint_volume(ep.endpoint_iaabbs));
                acc.times_us.push_back(t_us);
                acc.gaps.push_back(std::string(acc.source) == "MC"
                    ? 0.0
                    : sbf::exp::endpoint_profile::max_negative_gap_to(
                        ep.endpoint_iaabbs, mc_ref.endpoint_iaabbs));
                acc.gcpc_hits.push_back(static_cast<double>(ep.gcpc_hits));
            }
        }

        const auto [min_mc_it, max_mc_it] = std::minmax_element(
            mc_samples_used.begin(), mc_samples_used.end());
        out["bin_stats"].push_back({
            {"width_bin", bin.name},
            {"width_lo", bin.lo},
            {"width_hi", bin.hi},
            {"n_boxes", static_cast<int>(boxes.size())},
            {"geo_mean_width_mean", mean(geo_mean_widths)},
            {"geo_mean_width_median", median(geo_mean_widths)},
            {"mc_samples_mean", mc_samples_used.empty() ? 0.0 : mean(std::vector<double>(mc_samples_used.begin(), mc_samples_used.end()))},
            {"mc_samples_median", mc_samples_used.empty() ? 0.0 : median(std::vector<double>(mc_samples_used.begin(), mc_samples_used.end()))},
            {"mc_samples_min", mc_samples_used.empty() ? 0 : *min_mc_it},
            {"mc_samples_max", mc_samples_used.empty() ? 0 : *max_mc_it},
        });

        for (auto& acc : accs) {
            const double worst_gap = acc.gaps.empty()
                ? 0.0
                : *std::min_element(acc.gaps.begin(), acc.gaps.end());
            out["rows"].push_back({
                {"width_bin", bin.name},
                {"source", acc.source},
                {"certified", acc.certified},
                {"volume_mean", mean(acc.vols)},
                {"volume_median", median(acc.vols)},
                {"time_us_mean", mean(acc.times_us)},
                {"time_us_median", median(acc.times_us)},
                {"max_negative_gap_vs_mc", worst_gap},
                {"gcpc_hits_mean", mean(acc.gcpc_hits)},
            });
            std::cout << "[epiaabb] bin=" << bin.name
                      << " source=" << acc.source
                      << " vol=" << mean(acc.vols)
                      << " t_us=" << mean(acc.times_us)
                      << " gap=" << worst_gap << "\n";
        }
    }

    sbf::exp::write_json(a.out_path, out);
    std::cout << "[epiaabb] wrote " << a.out_path << "\n";
    return 0;
}
