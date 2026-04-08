// SafeBoxForest v5 — VizExporter implementation (Phase I, P)
#include <sbf/viz/viz_exporter.h>

#include <sbf/core/fk_state.h>
#include <sbf/envelope/link_iaabb.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/voxel/voxel_grid.h>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace sbf::viz {

using json = nlohmann::json;

// ─── detail helpers (Step I5) ───────────────────────────────────────────────

static void write_json_file(const std::string& path, const json& j)
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        std::cerr << "[viz] cannot open " << path << "\n";
        return;
    }
    ofs << j.dump(2);
}

Eigen::VectorXd detail::box_center(const BoxNode& box)
{
    return box.center();
}

// ─── Robot FK Export (Step I1) ──────────────────────────────────────────────

void export_robot_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<Eigen::VectorXd>& configs)
{
    json j;
    j["type"]     = "robot";
    j["name"]     = robot.name();
    j["n_joints"] = robot.n_joints();

    // link_radii
    json radii = json::array();
    for (double r : robot.link_radii())
        radii.push_back(r);
    j["link_radii"] = radii;

    // configs + FK
    json cfgs = json::array();
    for (const auto& q : configs) {
        json c;
        // q vector
        json qa = json::array();
        for (int d = 0; d < static_cast<int>(q.size()); ++d)
            qa.push_back(q[d]);
        c["q"] = qa;

        // FK: build intervals from point config, extract link positions
        std::vector<Interval> intervals(robot.n_joints());
        for (int d = 0; d < robot.n_joints(); ++d)
            intervals[d] = Interval(q[d], q[d]);

        FKState fk = compute_fk_full(robot, intervals);

        // Extract link endpoint positions (proximal of each active link)
        int n_active = robot.n_active_links();
        const int* alm = robot.active_link_map();
        // We extract link AABBs in point config �?lo==hi, take lo as position
        std::vector<float> aabbs(n_active * 6);
        extract_link_aabbs(fk, alm, n_active, aabbs.data(),
                           robot.has_link_radii() ? nullptr : nullptr);

        json pos = json::array();
        for (int a = 0; a < n_active; ++a) {
            // For point config, aabb lo == hi; midpoint is the position
            double x = 0.5 * (aabbs[a * 6 + 0] + aabbs[a * 6 + 3]);
            double y = 0.5 * (aabbs[a * 6 + 1] + aabbs[a * 6 + 4]);
            double z = 0.5 * (aabbs[a * 6 + 2] + aabbs[a * 6 + 5]);
            pos.push_back({x, y, z});
        }
        c["link_positions"] = pos;

        cfgs.push_back(c);
    }
    j["configs"] = cfgs;

    write_json_file(path, j);
}

// ─── Envelope iAABB Export (Step I2) ────────────────────────────────────────

void export_envelope_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<BoxNode>& boxes,
    const LECT& lect)
{
    (void)lect;  // reserved for future LECT-cached envelope readback
    json j;
    j["type"]   = "envelope";
    j["method"] = "link_iaabb";

    json jboxes = json::array();
    for (const auto& box : boxes) {
        json bj;
        bj["box_id"] = box.id;

        // intervals
        json ivs = json::array();
        for (const auto& iv : box.joint_intervals)
            ivs.push_back({iv.lo, iv.hi});
        bj["intervals"] = ivs;

        // Compute FK for this box and derive link iAABBs
        FKState fk = compute_fk_full(robot, box.joint_intervals);
        int n_active = robot.n_active_links();
        const int* alm = robot.active_link_map();

        std::vector<float> link_aabbs(n_active * 6);
        extract_link_aabbs(fk, alm, n_active, link_aabbs.data(),
                           robot.has_link_radii() ? robot.active_link_radii() : nullptr);

        json links = json::array();
        for (int a = 0; a < n_active; ++a) {
            json lj;
            lj["link_idx"] = alm[a];
            lj["aabb"] = {
                {link_aabbs[a * 6 + 0], link_aabbs[a * 6 + 3]},
                {link_aabbs[a * 6 + 1], link_aabbs[a * 6 + 4]},
                {link_aabbs[a * 6 + 2], link_aabbs[a * 6 + 5]}
            };
            links.push_back(lj);
        }
        bj["links"] = links;

        jboxes.push_back(bj);
    }
    j["boxes"] = jboxes;

    write_json_file(path, j);
}

void export_envelope_from_boxes_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<BoxNode>& boxes,
    int samples_per_box)
{
    json j;
    j["type"]   = "envelope";
    j["method"] = "link_iaabb_sampled";
    j["samples_per_box"] = samples_per_box;

    int n_active = robot.n_active_links();
    const int* alm = robot.active_link_map();

    json jboxes = json::array();
    for (const auto& box : boxes) {
        json bj;
        bj["box_id"] = box.id;

        json ivs = json::array();
        for (const auto& iv : box.joint_intervals)
            ivs.push_back({iv.lo, iv.hi});
        bj["intervals"] = ivs;

        // Compute FK with interval to get conservative bound
        FKState fk = compute_fk_full(robot, box.joint_intervals);
        std::vector<float> link_aabbs(n_active * 6);
        extract_link_aabbs(fk, alm, n_active, link_aabbs.data(),
                           robot.has_link_radii() ? robot.active_link_radii() : nullptr);

        json links = json::array();
        for (int a = 0; a < n_active; ++a) {
            json lj;
            lj["link_idx"] = alm[a];
            lj["aabb"] = {
                {link_aabbs[a * 6 + 0], link_aabbs[a * 6 + 3]},
                {link_aabbs[a * 6 + 1], link_aabbs[a * 6 + 4]},
                {link_aabbs[a * 6 + 2], link_aabbs[a * 6 + 5]}
            };
            links.push_back(lj);
        }
        bj["links"] = links;

        jboxes.push_back(bj);
    }
    j["boxes"] = jboxes;

    write_json_file(path, j);
}

// ─── Scene / Obstacle Export (Step I3) ──────────────────────────────────────

void export_scene_json(
    const std::string& path,
    const Obstacle* obs, int n_obs)
{
    json j;
    j["type"] = "scene";

    json obstacles = json::array();
    for (int i = 0; i < n_obs; ++i) {
        const auto& o = obs[i];
        // Obstacle.bounds = [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
        double cx = 0.5 * (o.bounds[0] + o.bounds[3]);
        double cy = 0.5 * (o.bounds[1] + o.bounds[4]);
        double cz = 0.5 * (o.bounds[2] + o.bounds[5]);
        double hx = 0.5 * (o.bounds[3] - o.bounds[0]);
        double hy = 0.5 * (o.bounds[4] - o.bounds[1]);
        double hz = 0.5 * (o.bounds[5] - o.bounds[2]);
        obstacles.push_back({
            {"center", {cx, cy, cz}},
            {"half_sizes", {hx, hy, hz}}
        });
    }
    j["obstacles"] = obstacles;

    write_json_file(path, j);
}

// ─── Snapshot Unified Export (Step I4) ──────────────────────────────────────

void export_snapshot_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<BoxNode>& boxes,
    const LECT& lect,
    const Obstacle* obs, int n_obs,
    const std::vector<Eigen::VectorXd>& sample_configs)
{
    (void)lect;  // reserved for future LECT-cached envelope readback
    json j;
    j["type"] = "snapshot";

    // ── Robot section ──
    {
        json rj;
        rj["name"]     = robot.name();
        rj["n_joints"] = robot.n_joints();
        json radii = json::array();
        for (double r : robot.link_radii())
            radii.push_back(r);
        rj["link_radii"] = radii;

        if (!sample_configs.empty()) {
            json cfgs = json::array();
            for (const auto& q : sample_configs) {
                json c;
                json qa = json::array();
                for (int d = 0; d < static_cast<int>(q.size()); ++d)
                    qa.push_back(q[d]);
                c["q"] = qa;

                std::vector<Interval> intervals(robot.n_joints());
                for (int d = 0; d < robot.n_joints(); ++d)
                    intervals[d] = Interval(q[d], q[d]);
                FKState fk = compute_fk_full(robot, intervals);
                int n_active = robot.n_active_links();
                const int* alm = robot.active_link_map();
                std::vector<float> aabbs(n_active * 6);
                extract_link_aabbs(fk, alm, n_active, aabbs.data());

                json pos = json::array();
                for (int a = 0; a < n_active; ++a) {
                    double x = 0.5 * (aabbs[a*6+0] + aabbs[a*6+3]);
                    double y = 0.5 * (aabbs[a*6+1] + aabbs[a*6+4]);
                    double z = 0.5 * (aabbs[a*6+2] + aabbs[a*6+5]);
                    pos.push_back({x, y, z});
                }
                c["link_positions"] = pos;
                cfgs.push_back(c);
            }
            rj["configs"] = cfgs;
        }
        j["robot"] = rj;
    }

    // ── Envelope section ──
    {
        json ej;
        ej["method"] = "link_iaabb";
        int n_active = robot.n_active_links();
        const int* alm = robot.active_link_map();

        json eboxes = json::array();
        for (const auto& box : boxes) {
            json bj;
            bj["box_id"] = box.id;
            json ivs = json::array();
            for (const auto& iv : box.joint_intervals)
                ivs.push_back({iv.lo, iv.hi});
            bj["intervals"] = ivs;

            FKState fk = compute_fk_full(robot, box.joint_intervals);
            std::vector<float> link_aabbs(n_active * 6);
            extract_link_aabbs(fk, alm, n_active, link_aabbs.data(),
                               robot.has_link_radii() ? robot.active_link_radii() : nullptr);

            json links = json::array();
            for (int a = 0; a < n_active; ++a) {
                json lj;
                lj["link_idx"] = alm[a];
                lj["aabb"] = {
                    {link_aabbs[a*6+0], link_aabbs[a*6+3]},
                    {link_aabbs[a*6+1], link_aabbs[a*6+4]},
                    {link_aabbs[a*6+2], link_aabbs[a*6+5]}
                };
                links.push_back(lj);
            }
            bj["links"] = links;
            eboxes.push_back(bj);
        }
        ej["boxes"] = eboxes;
        j["envelope"] = ej;
    }

    // ── Scene section ──
    {
        json sj;
        json obstacles = json::array();
        for (int i = 0; i < n_obs; ++i) {
            const auto& o = obs[i];
            double cx = 0.5 * (o.bounds[0] + o.bounds[3]);
            double cy = 0.5 * (o.bounds[1] + o.bounds[4]);
            double cz = 0.5 * (o.bounds[2] + o.bounds[5]);
            double hx = 0.5 * (o.bounds[3] - o.bounds[0]);
            double hy = 0.5 * (o.bounds[4] - o.bounds[1]);
            double hz = 0.5 * (o.bounds[5] - o.bounds[2]);
            obstacles.push_back({
                {"center", {cx, cy, cz}},
                {"half_sizes", {hx, hy, hz}}
            });
        }
        sj["obstacles"] = obstacles;
        j["scene"] = sj;
    }

    // ── Forest section ──
    {
        json fj;
        fj["n_boxes"] = static_cast<int>(boxes.size());
        double total_vol = 0.0;
        json fboxes = json::array();
        for (const auto& box : boxes) {
            json bj;
            bj["id"] = box.id;
            json ivs = json::array();
            for (const auto& iv : box.joint_intervals)
                ivs.push_back({iv.lo, iv.hi});
            bj["intervals"] = ivs;
            bj["volume"] = box.volume;
            total_vol += box.volume;
            fboxes.push_back(bj);
        }
        fj["boxes"] = fboxes;
        fj["total_volume"] = total_vol;
        j["forest"] = fj;
    }

    write_json_file(path, j);
}

// ─── Envelope Comparison Export (Step P1) ───────────────────────────────────

void export_envelope_comparison_json(
    const std::string& path,
    const Robot& robot,
    const BoxNode& box,
    const LECT& lect,
    double voxel_delta)
{
    (void)lect;  // reserved for future LECT-cached readback

    json j;
    j["type"]   = "envelope_comparison";
    j["box_id"] = box.id;

    // Serialize intervals
    json ivs = json::array();
    for (const auto& iv : box.joint_intervals)
        ivs.push_back({iv.lo, iv.hi});
    j["intervals"] = ivs;

    // Compute endpoint iAABBs via IFK
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    auto ep_result = compute_endpoint_iaabb(robot, box.joint_intervals, ep_cfg);
    const float* ep_data = ep_result.endpoint_iaabbs.data();
    int n_active = ep_result.n_active_links;
    const double* radii = robot.active_link_radii();

    json methods = json::object();

    // Method 1: LinkIAABB
    {
        EnvelopeTypeConfig cfg;
        cfg.type = EnvelopeType::LinkIAABB;
        cfg.n_subdivisions = 1;
        auto env = compute_link_envelope(ep_data, n_active, radii, cfg);

        json links = json::array();
        for (int a = 0; a < n_active; ++a) {
            json lj;
            lj["link_idx"] = robot.active_link_map()[a];
            const float* lb = env.link_iaabbs.data() + a * 6;
            lj["aabb"] = {
                {lb[0], lb[3]}, {lb[1], lb[4]}, {lb[2], lb[5]}
            };
            links.push_back(lj);
        }
        methods["link_iaabb"] = {{"links", links}};
    }

    // Method 2: LinkIAABB_Grid
    {
        EnvelopeTypeConfig cfg;
        cfg.type = EnvelopeType::LinkIAABB_Grid;
        cfg.n_subdivisions = 1;
        cfg.grid_config.voxel_delta = voxel_delta;
        auto env = compute_link_envelope(ep_data, n_active, radii, cfg);

        json links = json::array();
        for (int a = 0; a < n_active; ++a) {
            json lj;
            lj["link_idx"] = robot.active_link_map()[a];
            const float* lb = env.link_iaabbs.data() + a * 6;
            lj["aabb"] = {
                {lb[0], lb[3]}, {lb[1], lb[4]}, {lb[2], lb[5]}
            };
            links.push_back(lj);
        }
        methods["link_iaabb_grid"] = {{"links", links}};
    }

    // Method 3: Hull16_Grid (voxel)
    {
        EnvelopeTypeConfig cfg;
        cfg.type = EnvelopeType::Hull16_Grid;
        cfg.n_subdivisions = 1;
        cfg.grid_config.voxel_delta = voxel_delta;
        auto env = compute_link_envelope(ep_data, n_active, radii, cfg);

        json hull_j;
        hull_j["delta"] = voxel_delta;
        if (env.sparse_grid) {
            int n_occ = env.sparse_grid->count_occupied();
            hull_j["n_occupied"] = n_occ;
            json centres = json::array();
            for (auto e : env.sparse_grid->bricks()) {
                for (int z = 0; z < 8; ++z)
                    for (int y = 0; y < 8; ++y)
                        for (int x = 0; x < 8; ++x)
                            if (e.value.test(x, y, z)) {
                                int gx = e.key.bx * 8 + x;
                                int gy = e.key.by * 8 + y;
                                int gz = e.key.bz * 8 + z;
                                double cx = env.sparse_grid->cell_center(gx, 0);
                                double cy = env.sparse_grid->cell_center(gy, 1);
                                double cz = env.sparse_grid->cell_center(gz, 2);
                                centres.push_back({cx, cy, cz});
                            }
            }
            hull_j["centres"] = centres;
        } else {
            hull_j["n_occupied"] = 0;
            hull_j["centres"] = json::array();
        }
        methods["hull16_grid"] = hull_j;
    }

    j["methods"] = methods;
    write_json_file(path, j);
}

// ─── Voxel Export (Step P2) ─────────────────────────────────────────────────

static std::string hex64(uint64_t v)
{
    static const char digits[] = "0123456789abcdef";
    std::string s(16, '0');
    for (int i = 15; i >= 0; --i) {
        s[i] = digits[v & 0xfU];
        v >>= 4;
    }
    return s;
}

void export_voxel_json(
    const std::string& path,
    const voxel::SparseVoxelGrid& grid)
{
    json j;
    j["type"]  = "voxel";
    j["delta"] = grid.delta();

    int total = 0;
    json bricks_arr = json::array();
    for (auto e : grid.bricks()) {
        json b;
        b["coord"] = {e.key.bx, e.key.by, e.key.bz};
        int pc = e.value.popcount();
        b["popcount"] = pc;
        total += pc;
        json words = json::array();
        for (int w = 0; w < 8; ++w)
            words.push_back(hex64(e.value.words[w]));
        b["words"] = words;
        bricks_arr.push_back(b);
    }
    j["n_bricks"]       = grid.num_bricks();
    j["total_occupied"] = total;
    j["bricks"]         = bricks_arr;

    write_json_file(path, j);
}

void export_voxel_centres_json(
    const std::string& path,
    const voxel::SparseVoxelGrid& grid)
{
    json j;
    j["type"]  = "voxel_centres";
    j["delta"] = grid.delta();

    json pts = json::array();
    for (auto e : grid.bricks()) {
        for (int z = 0; z < 8; ++z)
            for (int y = 0; y < 8; ++y)
                for (int x = 0; x < 8; ++x)
                    if (e.value.test(x, y, z)) {
                        int gx = e.key.bx * 8 + x;
                        int gy = e.key.by * 8 + y;
                        int gz = e.key.bz * 8 + z;
                        double cx = grid.cell_center(gx, 0);
                        double cy = grid.cell_center(gy, 1);
                        double cz = grid.cell_center(gz, 2);
                        pts.push_back({cx, cy, cz});
                    }
    }
    j["n_points"] = pts.size();
    j["centres"]  = pts;

    write_json_file(path, j);
}

}  // namespace sbf::viz
