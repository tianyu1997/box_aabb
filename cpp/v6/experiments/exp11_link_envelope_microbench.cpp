#include <sbf/core/fk_state.h>
#include <sbf/core/robot.h>
#include <sbf/core/types.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace sbf;
using Clock = std::chrono::steady_clock;

namespace {

struct VariantSpec {
    std::string key;
    std::string label;
    std::string endpoint_source;
    std::string type;
    int n_subdivisions;
    double voxel_delta;
    EnvelopeType envelope_type;
    EndpointSource endpoint_type;
};

struct WorkItem {
    std::vector<Interval> intervals;
    FKState fk;
};

struct RepeatStats {
    double fk_us_total = 0.0;
    double extract_us_total = 0.0;
    double envelope_us_total = 0.0;
    double total_us_total = 0.0;
    std::uint64_t nodes = 0;
    double sink = 0.0;
    std::vector<double> fk_us_total_by_depth;
    std::vector<double> extract_us_total_by_depth;
    std::vector<double> envelope_us_total_by_depth;
    std::vector<double> total_us_total_by_depth;
    std::vector<std::uint64_t> nodes_by_depth;
};

struct AggregateStats {
    std::vector<double> fk_us_per_node;
    std::vector<double> extract_us_per_node;
    std::vector<double> envelope_us_per_node;
    std::vector<double> total_us_per_node;
    std::uint64_t nodes_per_repeat = 0;
    std::vector<std::vector<double>> total_us_per_node_by_depth;
    std::vector<std::vector<double>> envelope_us_per_node_by_depth;
};

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

double mean(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    double sum = 0.0;
    for (double value : values) sum += value;
    return sum / static_cast<double>(values.size());
}

std::vector<Interval> planning_intervals() {
    const double limits[7][2] = {
        {-1.865488,  1.865691},
        {-0.100000,  1.086648},
        {-0.662656,  0.662338},
        {-2.094400, -0.371673},
        {-0.619251,  0.619534},
        {-1.095222,  1.257951},
        { 1.050209,  2.091190},
    };
    std::vector<Interval> out;
    out.reserve(7);
    for (const auto& limit : limits) out.push_back(Interval(limit[0], limit[1]));
    return out;
}

int widest_dim(const std::vector<Interval>& intervals) {
    int best = 0;
    double best_width = intervals[0].width();
    for (int idx = 1; idx < static_cast<int>(intervals.size()); ++idx) {
        if (intervals[idx].width() > best_width) {
            best = idx;
            best_width = intervals[idx].width();
        }
    }
    return best;
}

std::pair<std::vector<Interval>, std::vector<Interval>> split_midpoint(const std::vector<Interval>& parent, int dim) {
    auto left = parent;
    auto right = parent;
    const double mid = 0.5 * (parent[dim].lo + parent[dim].hi);
    left[dim].hi = mid;
    right[dim].lo = mid;
    return {std::move(left), std::move(right)};
}

RepeatStats run_repeat(const Robot& robot,
                       const VariantSpec& variant,
                       int benchmark_depth,
                       int n_threads) {
    RepeatStats stats;
    stats.fk_us_total_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), 0.0);
    stats.extract_us_total_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), 0.0);
    stats.envelope_us_total_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), 0.0);
    stats.total_us_total_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), 0.0);
    stats.nodes_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), 0);
    EnvelopeTypeConfig env_cfg;
    env_cfg.type = variant.envelope_type;
    env_cfg.n_subdivisions = variant.n_subdivisions;
    env_cfg.grid_config.voxel_delta = static_cast<float>(variant.voxel_delta);
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = variant.endpoint_type;
    if (variant.endpoint_type == EndpointSource::CritSample) {
        ep_cfg.n_samples_crit = 0;
    }

    std::vector<WorkItem> frontier;
    frontier.push_back({planning_intervals(), compute_fk_full(robot, planning_intervals())});

    const int n_active = robot.n_active_links();
    const int ep_len = n_active * 12;
    const double* radii = robot.active_link_radii();

    for (int depth = 0; depth < benchmark_depth; ++depth) {
        const int worker_count = std::max(1, std::min<int>(n_threads, frontier.size()));
        std::vector<std::vector<WorkItem>> next_frontiers(worker_count);
        std::vector<RepeatStats> locals(worker_count);
        for (auto& local : locals) {
            local.fk_us_total_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), 0.0);
            local.extract_us_total_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), 0.0);
            local.envelope_us_total_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), 0.0);
            local.total_us_total_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), 0.0);
            local.nodes_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), 0);
        }
        std::vector<std::thread> workers;
        workers.reserve(worker_count);
        const auto depth_t0 = Clock::now();

        auto worker = [&](int worker_id, int begin, int end) {
            auto& next_items = next_frontiers[worker_id];
            auto& local = locals[worker_id];
            next_items.reserve(static_cast<std::size_t>(std::max(0, end - begin)) * 2);
            std::vector<float> endpoints(static_cast<std::size_t>(ep_len));

            for (int idx = begin; idx < end; ++idx) {
                const auto& parent = frontier[idx];
                const int changed_dim = widest_dim(parent.intervals);
                auto [left_intervals, right_intervals] = split_midpoint(parent.intervals, changed_dim);

                const int child_depth = depth + 1;
                auto process_child = [&](std::vector<Interval>& child_intervals) {
                    auto t0 = Clock::now();
                    FKState child_fk = parent.fk;
                    if (variant.endpoint_type == EndpointSource::IFK) {
                        child_fk = compute_fk_incremental(parent.fk, robot, child_intervals, changed_dim);
                        auto t1 = Clock::now();
                        extract_endpoint_iaabbs(child_fk, robot.active_link_map(), n_active, endpoints.data());
                        auto t2 = Clock::now();
                        LinkEnvelope env = compute_link_envelope(endpoints.data(), n_active, radii, env_cfg);
                        auto t3 = Clock::now();

                        local.fk_us_total += std::chrono::duration<double, std::micro>(t1 - t0).count();
                        local.extract_us_total += std::chrono::duration<double, std::micro>(t2 - t1).count();
                        local.envelope_us_total += std::chrono::duration<double, std::micro>(t3 - t2).count();
                        local.nodes += 1;
                        local.fk_us_total_by_depth[child_depth] += std::chrono::duration<double, std::micro>(t1 - t0).count();
                        local.extract_us_total_by_depth[child_depth] += std::chrono::duration<double, std::micro>(t2 - t1).count();
                        local.envelope_us_total_by_depth[child_depth] += std::chrono::duration<double, std::micro>(t3 - t2).count();
                        local.nodes_by_depth[child_depth] += 1;
                        if (!env.link_iaabbs.empty()) {
                            local.sink += env.link_iaabbs[0];
                        }
                        if (env.sparse_grid) {
                            local.sink += static_cast<double>(env.sparse_grid->count_occupied());
                        }
                    } else {
                        EndpointIAABBResult ep = compute_endpoint_iaabb(
                            robot, child_intervals, ep_cfg, &child_fk, changed_dim);
                        auto t1 = Clock::now();
                        std::copy(ep.endpoint_iaabbs.begin(), ep.endpoint_iaabbs.end(), endpoints.begin());
                        LinkEnvelope env = compute_link_envelope(endpoints.data(), n_active, radii, env_cfg);
                        auto t2 = Clock::now();

                        local.fk_us_total += std::chrono::duration<double, std::micro>(t1 - t0).count();
                        local.extract_us_total += 0.0;
                        local.envelope_us_total += std::chrono::duration<double, std::micro>(t2 - t1).count();
                        local.nodes += 1;
                        local.fk_us_total_by_depth[child_depth] += std::chrono::duration<double, std::micro>(t1 - t0).count();
                        local.extract_us_total_by_depth[child_depth] += 0.0;
                        local.envelope_us_total_by_depth[child_depth] += std::chrono::duration<double, std::micro>(t2 - t1).count();
                        local.nodes_by_depth[child_depth] += 1;
                        if (!env.link_iaabbs.empty()) {
                            local.sink += env.link_iaabbs[0];
                        }
                        if (env.sparse_grid) {
                            local.sink += static_cast<double>(env.sparse_grid->count_occupied());
                        }
                    }

                    next_items.push_back({std::move(child_intervals), std::move(child_fk)});
                };

                process_child(left_intervals);
                process_child(right_intervals);
            }
        };

        const int chunk = static_cast<int>((frontier.size() + worker_count - 1) / worker_count);
        for (int worker_id = 0; worker_id < worker_count; ++worker_id) {
            const int begin = worker_id * chunk;
            const int end = std::min<int>(begin + chunk, frontier.size());
            if (begin >= end) break;
            workers.emplace_back(worker, worker_id, begin, end);
        }
        for (auto& thread : workers) thread.join();
        const auto depth_t1 = Clock::now();
        const double depth_wall_us = std::chrono::duration<double, std::micro>(depth_t1 - depth_t0).count();
        const int child_depth = depth + 1;

        frontier.clear();
        for (std::size_t worker_id = 0; worker_id < next_frontiers.size(); ++worker_id) {
            frontier.insert(frontier.end(),
                            std::make_move_iterator(next_frontiers[worker_id].begin()),
                            std::make_move_iterator(next_frontiers[worker_id].end()));
            stats.fk_us_total += locals[worker_id].fk_us_total;
            stats.extract_us_total += locals[worker_id].extract_us_total;
            stats.envelope_us_total += locals[worker_id].envelope_us_total;
            stats.nodes += locals[worker_id].nodes;
            stats.sink += locals[worker_id].sink;
            for (int local_depth = 0; local_depth <= benchmark_depth; ++local_depth) {
                stats.fk_us_total_by_depth[local_depth] += locals[worker_id].fk_us_total_by_depth[local_depth];
                stats.extract_us_total_by_depth[local_depth] += locals[worker_id].extract_us_total_by_depth[local_depth];
                stats.envelope_us_total_by_depth[local_depth] += locals[worker_id].envelope_us_total_by_depth[local_depth];
                stats.nodes_by_depth[local_depth] += locals[worker_id].nodes_by_depth[local_depth];
            }
        }
        stats.total_us_total += depth_wall_us;
        stats.total_us_total_by_depth[child_depth] += depth_wall_us;
    }

    return stats;
}

nlohmann::json benchmark_variant(const Robot& robot,
                                const VariantSpec& variant,
                                int benchmark_depth,
                                int repeats,
                                int n_threads) {
    AggregateStats agg;
    agg.total_us_per_node_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), {});
    agg.envelope_us_per_node_by_depth.assign(static_cast<std::size_t>(benchmark_depth + 1), {});
    for (int repeat = 0; repeat < repeats; ++repeat) {
        const RepeatStats stats = run_repeat(robot, variant, benchmark_depth, n_threads);
        agg.nodes_per_repeat = stats.nodes;
        const double denom = stats.nodes > 0 ? static_cast<double>(stats.nodes) : 1.0;
        agg.fk_us_per_node.push_back(stats.fk_us_total / denom);
        agg.extract_us_per_node.push_back(stats.extract_us_total / denom);
        agg.envelope_us_per_node.push_back(stats.envelope_us_total / denom);
        agg.total_us_per_node.push_back(stats.total_us_total / denom);
        for (int depth = 1; depth <= benchmark_depth; ++depth) {
            const double depth_nodes = stats.nodes_by_depth[depth] > 0 ? static_cast<double>(stats.nodes_by_depth[depth]) : 1.0;
            agg.total_us_per_node_by_depth[depth].push_back(stats.total_us_total_by_depth[depth] / depth_nodes);
            agg.envelope_us_per_node_by_depth[depth].push_back(stats.envelope_us_total_by_depth[depth] / depth_nodes);
        }
    }

    nlohmann::json out;
    out["variant_key"] = variant.key;
    out["envelope_label"] = variant.label;
    out["endpoint_source"] = variant.endpoint_source;
    out["type"] = variant.type;
    out["n_subdivisions"] = variant.n_subdivisions;
    out["voxel_delta"] = variant.voxel_delta;
    out["threads"] = n_threads;
    out["benchmark_depth"] = benchmark_depth;
    out["nodes_per_repeat"] = agg.nodes_per_repeat;
    out["fk_us_per_node_mean"] = mean(agg.fk_us_per_node);
    out["fk_us_per_node_median"] = median(agg.fk_us_per_node);
    out["extract_us_per_node_mean"] = mean(agg.extract_us_per_node);
    out["extract_us_per_node_median"] = median(agg.extract_us_per_node);
    out["envelope_us_per_node_mean"] = mean(agg.envelope_us_per_node);
    out["envelope_us_per_node_median"] = median(agg.envelope_us_per_node);
    out["microbench_us_per_node_mean"] = mean(agg.total_us_per_node);
    out["microbench_us_per_node_median"] = median(agg.total_us_per_node);
    nlohmann::json depth_profile = nlohmann::json::array();
    for (int depth = 1; depth <= benchmark_depth; ++depth) {
        if (agg.total_us_per_node_by_depth[depth].empty()) continue;
        depth_profile.push_back({
            {"depth", depth},
            {"microbench_us_per_node_mean", mean(agg.total_us_per_node_by_depth[depth])},
            {"microbench_us_per_node_median", median(agg.total_us_per_node_by_depth[depth])},
            {"envelope_us_per_node_mean", mean(agg.envelope_us_per_node_by_depth[depth])},
            {"envelope_us_per_node_median", median(agg.envelope_us_per_node_by_depth[depth])},
        });
    }
    out["depth_profile"] = depth_profile;
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    std::string out_path;
    int benchmark_depth = 12;
    int repeats = 3;
    int n_threads = 16;

    for (int idx = 1; idx < argc; ++idx) {
        const std::string arg = argv[idx];
        if (arg == "--output" && idx + 1 < argc) out_path = argv[++idx];
        else if (arg == "--benchmark-depth" && idx + 1 < argc) benchmark_depth = std::max(1, std::atoi(argv[++idx]));
        else if (arg == "--repeats" && idx + 1 < argc) repeats = std::max(1, std::atoi(argv[++idx]));
        else if (arg == "--threads" && idx + 1 < argc) n_threads = std::max(1, std::atoi(argv[++idx]));
    }

    if (out_path.empty()) {
        std::cerr << "--output is required\n";
        return 1;
    }

    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/iiwa14.json");
    if (robot.n_joints() == 0) {
        std::cerr << "Failed to load iiwa14 robot data\n";
        return 1;
    }

    const std::vector<VariantSpec> variants = {
        {"crit_aabb_s1", "Crit+AABB S=1", "CritSample", "LinkIAABB", 1, 0.05, EnvelopeType::LinkIAABB, EndpointSource::CritSample},
        {"crit_aabb_s2", "Crit+AABB S=2", "CritSample", "LinkIAABB", 2, 0.05, EnvelopeType::LinkIAABB, EndpointSource::CritSample},
        {"crit_aabb_s4", "Crit+AABB S=4", "CritSample", "LinkIAABB", 4, 0.05, EnvelopeType::LinkIAABB, EndpointSource::CritSample},
        {"crit_aabb_s8", "Crit+AABB S=8", "CritSample", "LinkIAABB", 8, 0.05, EnvelopeType::LinkIAABB, EndpointSource::CritSample},
        {"crit_hull16_grid_d002", "Crit+Hull16-grid d=0.02", "CritSample", "Hull16_Grid", 1, 0.02, EnvelopeType::Hull16_Grid, EndpointSource::CritSample},
        {"crit_hull16_grid_d004", "Crit+Hull16-grid d=0.04", "CritSample", "Hull16_Grid", 1, 0.04, EnvelopeType::Hull16_Grid, EndpointSource::CritSample},
        {"crit_hull16_grid_d006", "Crit+Hull16-grid d=0.06", "CritSample", "Hull16_Grid", 1, 0.06, EnvelopeType::Hull16_Grid, EndpointSource::CritSample},
        {"crit_hull16_grid_d008", "Crit+Hull16-grid d=0.08", "CritSample", "Hull16_Grid", 1, 0.08, EnvelopeType::Hull16_Grid, EndpointSource::CritSample},
        {"ifk_aabb_s4", "IFK+AABB S=4", "IFK", "LinkIAABB", 4, 0.05, EnvelopeType::LinkIAABB, EndpointSource::IFK},
        {"ifk_hull16_grid_d004", "IFK+Hull16-grid d=0.04", "IFK", "Hull16_Grid", 1, 0.04, EnvelopeType::Hull16_Grid, EndpointSource::IFK},
    };

    nlohmann::json payload;
    payload["experiment"] = "link_envelope_microbench";
    payload["method"] = "no_tree_incremental_fk_plus_envelope";
    payload["endpoint_source"] = "mixed_Crit_main_plus_IFK_controls";
    payload["threads"] = n_threads;
    payload["benchmark_depth"] = benchmark_depth;
    payload["repeats"] = repeats;
    payload["rows"] = nlohmann::json::array();

    for (const auto& variant : variants) {
        payload["rows"].push_back(benchmark_variant(robot, variant, benchmark_depth, repeats, n_threads));
    }

    std::ofstream out(out_path);
    out << payload.dump(2) << '\n';
    return 0;
}