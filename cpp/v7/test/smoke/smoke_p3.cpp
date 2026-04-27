// SafeBoxForest v7 — Smoke tests for P3 (LECT minimal port).
#include "sbf/core/endpoint_iaabb.h"
#include "sbf/core/robot.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/link_iaabb.h"
#include "sbf/lect/lect.h"
#include "sbf/scene/collision.h"
#include "sbf/scene/obstacle.h"

#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace {

sbf::core::Robot load_iiwa() {
    return sbf::core::Robot::from_json(
        std::string(SBF_DATA_DIR) + "/iiwa14.json");
}

std::vector<sbf::core::Interval> joint_limit_intervals(const sbf::core::Robot& r) {
    std::vector<sbf::core::Interval> iv;
    iv.reserve(r.n_joints());
    for (const auto& l : r.joint_limits().limits) iv.push_back(l);
    return iv;
}

sbf::lect::LECT make_lect(const sbf::core::Robot& robot,
                          const std::vector<sbf::core::Interval>& iv) {
    sbf::envelope::EnvelopeTypeConfig cfg;
    cfg.type = sbf::envelope::EnvelopeType::LinkIAABB;
    return sbf::lect::LECT(robot, iv, cfg);
}

}  // namespace

// ─── 1. Root envelope == direct P1+P2 pipeline (zero-radius) ────────────────
TEST(P3, RootEnvelopeIsZeroRadius) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);

    auto direct = sbf::core::compute_endpoint_iaabb_ifk(robot, iv);
    std::vector<float> direct_link(robot.n_active_links() * 6);
    sbf::envelope::derive_link_iaabb_paired_zero(
        direct.endpoint_iaabbs.data(),
        direct.n_active_links,
        direct_link.data());

    const float* tree_link = tree.get_link_iaabbs(0);
    for (int k = 0; k < tree.n_active_links() * 6; ++k) {
        EXPECT_FLOAT_EQ(tree_link[k], direct_link[k]) << "k=" << k;
    }
}

// ─── 2. expand_leaf splits one leaf into two ────────────────────────────────
TEST(P3, ExpandSplitsLeafIntoTwo) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);

    EXPECT_TRUE(tree.is_leaf(0));
    EXPECT_EQ(tree.expand_leaf(0), 2);
    EXPECT_FALSE(tree.is_leaf(0));
    EXPECT_GE(tree.left(0), 0);
    EXPECT_GE(tree.right(0), 0);
    EXPECT_TRUE(tree.is_leaf(tree.left(0)));
    EXPECT_TRUE(tree.is_leaf(tree.right(0)));
    EXPECT_EQ(tree.depth(tree.left(0)), 1);
    EXPECT_EQ(tree.parent(tree.left(0)), 0);

    // Re-expanding a non-leaf is a no-op.
    EXPECT_EQ(tree.expand_leaf(0), 0);
}

// ─── 3. Child envelopes are no looser than parent (conservativity) ──────────
TEST(P3, ChildEnvelopesNoLooserThanParent) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);
    tree.expand_leaf(0);

    auto vol = [&](const float* a, int n) {
        double v = 0.0;
        for (int i = 0; i < n; ++i) {
            const float* p = a + i * 6;
            double dx = std::max(0.0f, p[3] - p[0]);
            double dy = std::max(0.0f, p[4] - p[1]);
            double dz = std::max(0.0f, p[5] - p[2]);
            v += dx * dy * dz;
        }
        return v;
    };

    int n = tree.n_active_links();
    double vp = vol(tree.get_link_iaabbs(0), n);
    double vl = vol(tree.get_link_iaabbs(tree.left(0)), n);
    double vr = vol(tree.get_link_iaabbs(tree.right(0)), n);

    // Each child fits inside the parent envelope (≤ parent volume).
    EXPECT_LE(vl, vp + 1e-6);
    EXPECT_LE(vr, vp + 1e-6);
}

// ─── 4. collides_scene applies link_radii inline (D1) ───────────────────────
TEST(P3, CollidesSceneInflatesByRadii) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);

    // Far obstacle — no collision.
    sbf::scene::Obstacle far(20, 20, 20, 21, 21, 21);
    float far_c[6];
    sbf::scene::pack_obstacles(&far, 1, far_c);
    EXPECT_FALSE(tree.collides_scene(0, far_c, 1));

    // Near/intersecting obstacle — collision.
    sbf::scene::Obstacle near(-1, -1, -1, 1, 1, 1);
    float near_c[6];
    sbf::scene::pack_obstacles(&near, 1, near_c);
    EXPECT_TRUE(tree.collides_scene(0, near_c, 1));

    // Ground-truth equivalence: tree.collides_scene must equal a manual
    // call to aabbs_collide_obs_inflated with the same zero-radius env.
    const float* env = tree.get_link_iaabbs(0);
    std::vector<float> radii_per_slot(tree.n_active_links());
    sbf::scene::expand_radii_to_slots(
        robot.active_link_radii(), tree.n_active_links(), 1,
        radii_per_slot.data());
    EXPECT_EQ(tree.collides_scene(0, far_c, 1),
              sbf::scene::aabbs_collide_obs_inflated(
                  env, tree.n_active_links(),
                  radii_per_slot.data(), far_c, 1));
    EXPECT_EQ(tree.collides_scene(0, near_c, 1),
              sbf::scene::aabbs_collide_obs_inflated(
                  env, tree.n_active_links(),
                  radii_per_slot.data(), near_c, 1));
}

// ─── 5. mark_occupied propagates subtree_occ + free volume up parent chain ──
TEST(P3, MarkOccupiedPropagatesUp) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);
    tree.expand_leaf(0);
    tree.expand_leaf(tree.left(0));

    int leaf = tree.left(tree.left(0));   // grandchild leaf
    double v_before = tree.subtree_free_volume(0);
    double leaf_vol = tree.subtree_free_volume(leaf);
    EXPECT_GT(leaf_vol, 0.0);

    tree.mark_occupied(leaf, /*box_id=*/42);
    EXPECT_TRUE(tree.is_occupied(leaf));
    EXPECT_EQ(tree.forest_id(leaf), 42);
    EXPECT_EQ(tree.subtree_occ(0), 1);
    EXPECT_NEAR(tree.subtree_free_volume(0), v_before - leaf_vol, 1e-9);

    tree.unmark_occupied(leaf);
    EXPECT_FALSE(tree.is_occupied(leaf));
    EXPECT_EQ(tree.subtree_occ(0), 0);
    EXPECT_NEAR(tree.subtree_free_volume(0), v_before, 1e-9);
}

// ─── 6. is_point_occupied uses split-plane descent ──────────────────────────
TEST(P3, IsPointOccupiedFastPath) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);
    tree.expand_leaf(0);

    int li = tree.left(0);
    auto liv = tree.node_intervals(li);
    std::vector<double> q(tree.n_dims());
    for (int d = 0; d < tree.n_dims(); ++d) q[d] = liv[d].center();

    EXPECT_FALSE(tree.is_point_occupied(q.data()));
    EXPECT_EQ(tree.find_leaf_containing(q.data()), li);

    tree.mark_occupied(li, /*box_id=*/7);
    EXPECT_TRUE(tree.is_point_occupied(q.data()));
}

// ─── 7. snapshot is a deep copy ─────────────────────────────────────────────
TEST(P3, SnapshotIsDeepCopy) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);
    tree.expand_leaf(0);

    int n_before = tree.n_nodes();
    auto worker = tree.snapshot();
    EXPECT_EQ(worker.n_nodes(), n_before);
    EXPECT_EQ(worker.snapshot_base(), n_before);

    worker.expand_leaf(worker.left(0));
    EXPECT_GT(worker.n_nodes(), n_before);
    EXPECT_EQ(tree.n_nodes(), n_before);          // master untouched
    EXPECT_TRUE(tree.is_leaf(tree.left(0)));      // master leaf unchanged
}

// ─── 8. partition_for_seeds: two seeds in distinct domains ──────────────────
TEST(P3, PartitionForSeedsTwoSeeds) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);

    int nd = robot.n_joints();
    Eigen::VectorXd s1(nd), s2(nd);
    for (int d = 0; d < nd; ++d) {
        s1[d] = iv[d].lo + 0.05 * iv[d].width();
        s2[d] = iv[d].hi - 0.05 * iv[d].width();
    }
    std::vector<Eigen::VectorXd> seeds = {s1, s2};
    auto domains = tree.partition_for_seeds(seeds, 32);
    ASSERT_EQ(domains.size(), 2u);
    EXPECT_NE(domains[0], domains[1]);
    // Each domain must contain its seed.
    EXPECT_EQ(tree.find_leaf_containing(s1.data()),
              [&](){
                  // walk down domain[0] to leaf containing s1
                  int n = domains[0];
                  while (!tree.is_leaf(n)) {
                      int d = tree.split_dim(n);
                      n = (s1[d] <= tree.split_val(n)) ? tree.left(n) : tree.right(n);
                  }
                  return n;
              }());
    EXPECT_TRUE(tree.is_descendant_of(
        tree.find_leaf_containing(s1.data()), domains[0]));
    EXPECT_TRUE(tree.is_descendant_of(
        tree.find_leaf_containing(s2.data()), domains[1]));
    // Domains themselves are disjoint.
    EXPECT_FALSE(tree.is_descendant_of(domains[0], domains[1]));
    EXPECT_FALSE(tree.is_descendant_of(domains[1], domains[0]));
}

// ─── 9. partition_for_seeds: LCA promotion to non-leaf ──────────────────────
TEST(P3, PartitionForSeedsLcaPromote) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);
    // Pre-split a few times so we have non-trivial tree.
    tree.expand_leaf(0);
    tree.expand_leaf(tree.left(0));

    int nd = robot.n_joints();
    Eigen::VectorXd s1(nd), s2(nd);
    for (int d = 0; d < nd; ++d) s1[d] = iv[d].lo + 0.05 * iv[d].width();
    for (int d = 0; d < nd; ++d) s2[d] = iv[d].hi - 0.05 * iv[d].width();

    auto domains = tree.partition_for_seeds({s1, s2}, 64);
    // At least one domain should be a non-leaf (LCA-promoted), since both
    // seeds were initially in different deep leaves.
    int n_nonleaf = 0;
    for (int d : domains) if (!tree.is_leaf(d)) ++n_nonleaf;
    EXPECT_GE(n_nonleaf, 1);
}

// ─── 10. transplant_domain: round-trip preserves new structure ──────────────
TEST(P3, TransplantDomainRoundTrip) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto master = make_lect(robot, iv);
    master.expand_leaf(0);
    int domain_root = master.left(0);

    auto worker = master.snapshot();
    int n_master_before = master.n_nodes();
    int worker_base = worker.snapshot_base();
    EXPECT_EQ(worker_base, n_master_before);

    // Worker grows: split the domain root and one child.
    worker.expand_leaf(domain_root);
    int wl = worker.left(domain_root);
    worker.expand_leaf(wl);
    int worker_added = worker.n_nodes() - n_master_before;
    EXPECT_GE(worker_added, 4);    // 2 splits = 4 new nodes

    // Worker also marks one new leaf occupied with local box id 7.
    int wll = worker.left(wl);
    worker.mark_occupied(wll, 7);

    // Transplant into master with id remap 7 → 100.
    std::unordered_map<int,int> id_map = {{7, 100}};
    std::unordered_map<int,int> remap;
    int n_shipped = master.transplant_domain(worker, domain_root, id_map, remap);
    EXPECT_EQ(n_shipped, worker_added);
    EXPECT_EQ(master.n_nodes(), n_master_before + worker_added);

    // Domain root remained at same index; its new children must be in remap.
    EXPECT_EQ(remap.at(domain_root), domain_root);
    EXPECT_GE(master.left(domain_root), 0);
    EXPECT_GE(master.right(domain_root), 0);
    EXPECT_FALSE(master.is_leaf(domain_root));

    // The marked leaf in worker should be remapped and carry global id 100.
    int master_wll = remap.at(wll);
    EXPECT_EQ(master.forest_id(master_wll), 100);
}

// ─── 11. binary persistence: save/load preserves structure and occupancy ────
TEST(P3, BinarySaveLoadRoundTrip) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);
    tree.expand_leaf(0);
    tree.expand_leaf(tree.left(0));

    int marked_leaf = tree.left(tree.left(0));
    tree.mark_occupied(marked_leaf, /*box_id=*/17);

    auto tmp = std::filesystem::temp_directory_path()
               / "sbf_v7_lect_roundtrip.bin";
    std::filesystem::remove(tmp);

    tree.save_binary(tmp.string());
    auto loaded = sbf::lect::LECT::load_binary(tmp.string(), robot);

    EXPECT_EQ(loaded.n_nodes(), tree.n_nodes());
    EXPECT_EQ(loaded.n_dims(), tree.n_dims());
    EXPECT_EQ(loaded.n_active_links(), tree.n_active_links());
    EXPECT_EQ(loaded.n_subdivisions(), tree.n_subdivisions());
    EXPECT_EQ(loaded.snapshot_base(), 0);

    for (int i = 0; i < tree.n_nodes(); ++i) {
        EXPECT_EQ(loaded.left(i), tree.left(i)) << "node=" << i;
        EXPECT_EQ(loaded.right(i), tree.right(i)) << "node=" << i;
        EXPECT_EQ(loaded.parent(i), tree.parent(i)) << "node=" << i;
        EXPECT_EQ(loaded.depth(i), tree.depth(i)) << "node=" << i;
        EXPECT_EQ(loaded.split_dim(i), tree.split_dim(i)) << "node=" << i;
        EXPECT_DOUBLE_EQ(loaded.split_val(i), tree.split_val(i)) << "node=" << i;
        EXPECT_EQ(loaded.forest_id(i), tree.forest_id(i)) << "node=" << i;
        EXPECT_EQ(loaded.subtree_occ(i), tree.subtree_occ(i)) << "node=" << i;
        EXPECT_NEAR(loaded.subtree_free_volume(i), tree.subtree_free_volume(i), 1e-12)
            << "node=" << i;

        const float* a = tree.get_link_iaabbs(i);
        const float* b = loaded.get_link_iaabbs(i);
        for (int k = 0; k < tree.n_slots() * 6; ++k) {
            EXPECT_FLOAT_EQ(a[k], b[k]) << "node=" << i << " k=" << k;
        }
    }

    EXPECT_TRUE(loaded.is_occupied(marked_leaf));
    std::filesystem::remove(tmp);
}

// ─── 12. binary persistence rejects incompatible robot fingerprints ─────────
TEST(P3, BinaryLoadRejectsFingerprintMismatch) {
    auto robot = load_iiwa();
    auto iv = joint_limit_intervals(robot);
    auto tree = make_lect(robot, iv);

    auto tmp = std::filesystem::temp_directory_path()
               / "sbf_v7_lect_fingerprint_mismatch.bin";
    std::filesystem::remove(tmp);
    tree.save_binary(tmp.string());

    auto wrong_robot = sbf::core::Robot::from_json(
        std::string(SBF_DATA_DIR) + "/iiwa14_far_dof2.json");
    EXPECT_THROW(sbf::lect::LECT::load_binary(tmp.string(), wrong_robot),
                 std::runtime_error);

    std::filesystem::remove(tmp);
}
