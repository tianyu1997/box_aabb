#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/viz/viz_exporter.h>
#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/voxel/voxel_grid.h>
#include <sbf/voxel/bit_brick.h>

#include <nlohmann/json.hpp>
#include <fstream>
#include <cstdio>

using namespace sbf;
using json = nlohmann::json;

// ─── Helpers ────────────────────────────────────────────────────────────────

static Robot make_2dof() {
    return Robot::from_json("data/2dof_planar.json");
}

static BoxNode make_box(int id, std::vector<std::pair<double,double>> ivs) {
    BoxNode b;
    b.id = id;
    for (auto& [lo, hi] : ivs)
        b.joint_intervals.push_back({lo, hi});
    b.seed_config = b.center();
    b.compute_volume();
    return b;
}

static json load_json(const std::string& path) {
    std::ifstream ifs(path);
    REQUIRE(ifs.is_open());
    json j;
    ifs >> j;
    return j;
}

static void remove_file(const std::string& path) {
    std::remove(path.c_str());
}

// ═════════════════════════════════════════════════════════════════════════════
TEST_SUITE("VizExporter") {

TEST_CASE("robot_json_roundtrip") {
    auto robot = make_2dof();
    Eigen::VectorXd q(2);
    q << 0.1, 0.2;
    std::vector<Eigen::VectorXd> configs = { q };

    const std::string path = "test_robot_export.json";
    viz::export_robot_json(path, robot, configs);

    auto j = load_json(path);
    CHECK(j["type"] == "robot");
    CHECK(j["n_joints"] == 2);
    REQUIRE(j.contains("configs"));
    REQUIRE(j["configs"].size() == 1);

    auto& c = j["configs"][0];
    REQUIRE(c.contains("q"));
    CHECK(c["q"].size() == 2);
    CHECK(c["q"][0].get<double>() == doctest::Approx(0.1).epsilon(1e-6));
    CHECK(c["q"][1].get<double>() == doctest::Approx(0.2).epsilon(1e-6));

    REQUIRE(c.contains("link_positions"));
    CHECK(c["link_positions"].size() > 0);
    // Each link position should be [x, y, z]
    for (auto& lp : c["link_positions"])
        CHECK(lp.size() == 3);

    remove_file(path);
}

TEST_CASE("envelope_json_structure") {
    auto robot = make_2dof();
    std::vector<BoxNode> boxes = {
        make_box(0, {{0.0, 0.5}, {0.0, 0.5}}),
        make_box(1, {{0.5, 1.0}, {0.0, 0.5}}),
    };

    const std::string path = "test_envelope_export.json";
    viz::export_envelope_from_boxes_json(path, robot, boxes);

    auto j = load_json(path);
    CHECK(j["type"] == "envelope");
    REQUIRE(j.contains("boxes"));
    CHECK(j["boxes"].size() == 2);

    for (auto& bj : j["boxes"]) {
        REQUIRE(bj.contains("box_id"));
        REQUIRE(bj.contains("links"));
        CHECK(bj["links"].size() > 0);
        for (auto& lj : bj["links"]) {
            REQUIRE(lj.contains("link_idx"));
            REQUIRE(lj.contains("aabb"));
            CHECK(lj["aabb"].size() == 3);  // [[xlo,xhi],[ylo,yhi],[zlo,zhi]]
        }
    }

    remove_file(path);
}

TEST_CASE("scene_json_structure") {
    std::vector<Obstacle> obs = {
        Obstacle(-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f),
        Obstacle(2.0f, 0.0f, 0.0f, 3.0f, 1.0f, 1.0f),
    };

    const std::string path = "test_scene_export.json";
    viz::export_scene_json(path, obs.data(), static_cast<int>(obs.size()));

    auto j = load_json(path);
    CHECK(j["type"] == "scene");
    REQUIRE(j.contains("obstacles"));
    CHECK(j["obstacles"].size() == 2);

    // First obstacle: center=[0,0,0], half_sizes=[1,1,1]
    auto& o0 = j["obstacles"][0];
    CHECK(o0["center"][0].get<double>() == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(o0["half_sizes"][0].get<double>() == doctest::Approx(1.0).epsilon(1e-6));

    remove_file(path);
}

TEST_CASE("snapshot_has_all_layers") {
    auto robot = make_2dof();
    std::vector<BoxNode> boxes = {
        make_box(0, {{0.0, 0.3}, {0.0, 0.3}}),
    };

    Obstacle obs(5.0f, 5.0f, 5.0f, 6.0f, 6.0f, 6.0f);

    // Build a minimal LECT
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    env_cfg.type = EnvelopeType::LinkIAABB;
    env_cfg.n_subdivisions = 1;

    LECT lect(robot, robot.joint_limits().limits, ep_cfg, env_cfg);

    Eigen::VectorXd q(2);
    q << 0.15, 0.15;
    std::vector<Eigen::VectorXd> configs = { q };

    const std::string path = "test_snapshot_export.json";
    viz::export_snapshot_json(path, robot, boxes, lect, &obs, 1, configs);

    auto j = load_json(path);
    CHECK(j["type"] == "snapshot");
    REQUIRE(j.contains("robot"));
    REQUIRE(j.contains("envelope"));
    REQUIRE(j.contains("scene"));
    REQUIRE(j.contains("forest"));

    CHECK(j["robot"].contains("name"));
    CHECK(j["robot"].contains("n_joints"));
    CHECK(j["envelope"].contains("boxes"));
    CHECK(j["scene"].contains("obstacles"));
    CHECK(j["forest"].contains("n_boxes"));
    CHECK(j["forest"]["n_boxes"] == 1);

    remove_file(path);
}

TEST_CASE("envelope_comparison_json") {
    auto robot = make_2dof();
    auto box = make_box(0, {{0.0, 0.5}, {0.0, 0.5}});

    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    env_cfg.type = EnvelopeType::LinkIAABB;

    LECT lect(robot, robot.joint_limits().limits, ep_cfg, env_cfg);

    const std::string path = "test_envelope_comparison.json";
    viz::export_envelope_comparison_json(path, robot, box, lect, 0.05);

    auto j = load_json(path);
    CHECK(j["type"] == "envelope_comparison");
    CHECK(j["box_id"] == 0);
    REQUIRE(j.contains("methods"));

    auto& methods = j["methods"];
    CHECK(methods.contains("link_iaabb"));
    CHECK(methods.contains("link_iaabb_grid"));
    CHECK(methods.contains("hull16_grid"));

    // link_iaabb has links array
    REQUIRE(methods["link_iaabb"].contains("links"));
    CHECK(methods["link_iaabb"]["links"].size() > 0);

    // hull16_grid has delta, n_occupied, centres
    REQUIRE(methods["hull16_grid"].contains("delta"));
    REQUIRE(methods["hull16_grid"].contains("n_occupied"));
    REQUIRE(methods["hull16_grid"].contains("centres"));
    CHECK(methods["hull16_grid"]["n_occupied"].get<int>() > 0);

    remove_file(path);
}

TEST_CASE("voxel_json_roundtrip") {
    using namespace sbf::voxel;
    SparseVoxelGrid grid(0.1);

    // Set some voxels in a brick
    BrickCoord bc{0, 0, 0};
    BitBrick brick;
    brick.set(0, 0, 0);
    brick.set(1, 2, 3);
    brick.set(7, 7, 7);
    grid.set_brick(bc, brick);

    const std::string path = "test_voxel_export.json";
    viz::export_voxel_json(path, grid);

    auto j = load_json(path);
    CHECK(j["type"] == "voxel");
    CHECK(j["delta"].get<double>() == doctest::Approx(0.1));
    CHECK(j["n_bricks"] == 1);
    CHECK(j["total_occupied"] == 3);
    REQUIRE(j.contains("bricks"));
    CHECK(j["bricks"].size() == 1);

    auto& b = j["bricks"][0];
    CHECK(b["popcount"] == 3);
    REQUIRE(b.contains("words"));
    CHECK(b["words"].size() == 8);

    remove_file(path);
}

TEST_CASE("voxel_centres_json") {
    using namespace sbf::voxel;
    SparseVoxelGrid grid(0.1);

    BrickCoord bc{0, 0, 0};
    BitBrick brick;
    brick.set(1, 1, 1);
    brick.set(2, 2, 2);
    grid.set_brick(bc, brick);

    const std::string path = "test_voxel_centres.json";
    viz::export_voxel_centres_json(path, grid);

    auto j = load_json(path);
    CHECK(j["type"] == "voxel_centres");
    CHECK(j["delta"].get<double>() == doctest::Approx(0.1));
    CHECK(j["n_points"] == 2);
    REQUIRE(j.contains("centres"));
    CHECK(j["centres"].size() == 2);
    // Each centre should be [x, y, z]
    for (auto& c : j["centres"])
        CHECK(c.size() == 3);

    remove_file(path);
}

}  // TEST_SUITE
