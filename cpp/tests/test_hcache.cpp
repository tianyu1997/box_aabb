// Tests for HCACHE02 persistence (HierAABBTree save/load/incremental/mmap + HCacheFile)
#include <gtest/gtest.h>
#include "sbf/forest/hier_aabb_tree.h"
#include "sbf/io/hcache.h"
#include "sbf/core/types.h"
#include <Eigen/Core>
#include <cstdio>
#include <filesystem>
#include <string>

using namespace sbf;

// Helper: create a simple 2-DOF robot for testing
static Robot make_test_robot() {
    std::vector<DHParam> dhs = {
        {0.0, 0.5, 0.0, 0.0, 0},
        {0.0, 0.5, 0.0, 0.0, 0}
    };
    JointLimits jl;
    jl.limits = {{-2.0, 2.0}, {-2.0, 2.0}};
    return Robot("test_2dof", dhs, jl);
}

// Helper: create a packed obstacle flat array
// Format per obstacle: [link_idx_f, lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
// Place a small box near link 0's workspace to force splitting
static std::vector<float> make_test_obstacles() {
    // Obstacle on link 0: box at x=[0.1, 0.3], y=[-0.05, 0.05], z=[-0.05, 0.05]
    std::vector<float> obs = {
        0.0f,          // link_idx = 0
        0.1f, 0.3f,   // lo_x, hi_x
        -0.05f, 0.05f, // lo_y, hi_y
        -0.05f, 0.05f  // lo_z, hi_z
    };
    return obs;
}

class HCacheTest : public ::testing::Test {
protected:
    void SetUp() override {
        robot_ = make_test_robot();
        tmp_path_ = std::filesystem::temp_directory_path() / "sbf_test_hcache.hcache";
    }

    void TearDown() override {
        std::filesystem::remove(tmp_path_);
    }

    Robot robot_;
    std::filesystem::path tmp_path_;
};

// ─── HierAABBTree save/load ───────────────────────────────────────────────

TEST_F(HCacheTest, SaveLoadEmpty) {
    // Brand new tree (only root node)
    HierAABBTree tree(robot_);
    EXPECT_EQ(tree.store().next_idx(), 1);  // root only

    tree.save(tmp_path_.string());

    // Load it back
    HierAABBTree loaded = HierAABBTree::load(tmp_path_.string(), robot_);
    EXPECT_EQ(loaded.store().next_idx(), 1);
    EXPECT_TRUE(loaded.store().has_aabb(0));  // root has AABB
}

TEST_F(HCacheTest, SaveLoadAfterFFB) {
    auto obs = make_test_obstacles();
    HierAABBTree tree(robot_);

    // Run a few FFB calls to grow the tree
    Eigen::VectorXd seed(2);
    seed << 0.5, 0.5;
    auto res1 = tree.find_free_box(seed, obs.data(), 1, 20, 0.1);
    EXPECT_TRUE(res1.success());

    seed << -1.0, -1.0;
    auto res2 = tree.find_free_box(seed, obs.data(), 1, 20, 0.1);
    EXPECT_TRUE(res2.success());

    int n_nodes_before = tree.store().next_idx();
    int fk_before = tree.total_fk_calls();
    EXPECT_GT(n_nodes_before, 1);

    // Save
    tree.save(tmp_path_.string());

    // Load
    HierAABBTree loaded = HierAABBTree::load(tmp_path_.string(), robot_);

    // Verify node count preserved
    EXPECT_EQ(loaded.store().next_idx(), n_nodes_before);
    EXPECT_EQ(loaded.total_fk_calls(), fk_before);

    // Verify topology preserved: check a few nodes
    for (int i = 0; i < n_nodes_before; ++i) {
        EXPECT_EQ(loaded.store().left(i), tree.store().left(i))
            << "left mismatch at node " << i;
        EXPECT_EQ(loaded.store().right(i), tree.store().right(i))
            << "right mismatch at node " << i;
        EXPECT_EQ(loaded.store().parent(i), tree.store().parent(i))
            << "parent mismatch at node " << i;
        EXPECT_EQ(loaded.store().depth(i), tree.store().depth(i))
            << "depth mismatch at node " << i;
        EXPECT_NEAR(loaded.store().split_val(i), tree.store().split_val(i), 1e-12)
            << "split_val mismatch at node " << i;
        EXPECT_EQ(loaded.store().has_aabb(i), tree.store().has_aabb(i))
            << "has_aabb mismatch at node " << i;
    }

    // Verify AABB data preserved for nodes that have AABB
    int aabb_floats = loaded.store().aabb_floats();
    for (int i = 0; i < n_nodes_before; ++i) {
        if (loaded.store().has_aabb(i)) {
            const float* a = loaded.store().aabb(i);
            const float* b = tree.store().aabb(i);
            for (int f = 0; f < aabb_floats; ++f) {
                EXPECT_FLOAT_EQ(a[f], b[f])
                    << "AABB mismatch at node " << i << " float " << f;
            }
        }
    }
}

TEST_F(HCacheTest, SaveLoadCanContinueFFB) {
    auto obs = make_test_obstacles();
    // Build a tree, save, load, then continue using it for FFB
    HierAABBTree tree(robot_);

    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;
    tree.find_free_box(seed, obs.data(), 1, 15, 0.1);

    tree.save(tmp_path_.string());
    HierAABBTree loaded = HierAABBTree::load(tmp_path_.string(), robot_);

    // Continue FFB on loaded tree — should work and produce free boxes
    seed << 1.5, -1.5;
    auto res = loaded.find_free_box(seed, obs.data(), 1, 15, 0.1);
    EXPECT_TRUE(res.success());
    EXPECT_GT(loaded.store().next_idx(), tree.store().next_idx());
}

TEST_F(HCacheTest, LoadWrongRobotFails) {
    HierAABBTree tree(robot_);
    tree.save(tmp_path_.string());

    // Try to load with a different robot (3-DOF)
    std::vector<DHParam> dhs3 = {
        {0.0, 0.3, 0.0, 0.0, 0},
        {0.0, 0.3, 0.0, 0.0, 0},
        {0.0, 0.3, 0.0, 0.0, 0}
    };
    JointLimits jl3;
    jl3.limits = {{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}};
    Robot robot3("test_3dof", dhs3, jl3);

    EXPECT_THROW(HierAABBTree::load(tmp_path_.string(), robot3), std::runtime_error);
}

// ─── HCacheFile (mmap-backed) ─────────────────────────────────────────────

TEST_F(HCacheTest, HCacheFileCreateOpen) {
    JointLimits jl;
    jl.limits = {{-2.0, 2.0}, {-2.0, 2.0}};

    // Create
    {
        auto hf = HCacheFile::create(tmp_path_.string(), 2, 2, jl, "test_fp", 16);
        EXPECT_TRUE(hf.is_open());
        EXPECT_EQ(hf.n_dims(), 2);
        EXPECT_EQ(hf.n_links(), 2);

        // Allocate some nodes
        auto& store = hf.node_store();
        store.alloc_node(-1, 0);  // node 0 (root already there? depends on ctor)
        store.alloc_node(0, 1);
        store.alloc_node(0, 1);

        hf.flush();
        hf.close();
    }

    // Re-open
    {
        auto hf = HCacheFile::open(tmp_path_.string());
        EXPECT_TRUE(hf.is_open());
        EXPECT_EQ(hf.n_dims(), 2);
        EXPECT_EQ(hf.n_links(), 2);
        EXPECT_GT(hf.n_nodes(), 0);
        hf.close();
    }
}

TEST_F(HCacheTest, HCacheFileGrow) {
    JointLimits jl;
    jl.limits = {{-1.0, 1.0}, {-1.0, 1.0}};

    auto hf = HCacheFile::create(tmp_path_.string(), 2, 2, jl, "", 4);
    auto& store = hf.node_store();

    // Allocate more nodes than initial capacity (4) to trigger grow
    for (int i = 0; i < 20; ++i) {
        store.alloc_node(0, 1);
    }
    EXPECT_GE(store.capacity(), 20);
    EXPECT_EQ(store.next_idx(), 20);

    hf.flush();
    hf.close();

    // Verify data survives re-open
    auto hf2 = HCacheFile::open(tmp_path_.string());
    EXPECT_EQ(hf2.n_nodes(), 20);
    hf2.close();
}

TEST_F(HCacheTest, FileNotFoundThrows) {
    EXPECT_THROW(HierAABBTree::load("/nonexistent/path.hcache", robot_),
                 std::runtime_error);
}

// ─── Incremental save ─────────────────────────────────────────────────────

TEST_F(HCacheTest, SaveIncrementalWithoutPriorSaveThrows) {
    HierAABBTree tree(robot_);
    EXPECT_THROW(tree.save_incremental(tmp_path_.string()), std::runtime_error);
}

TEST_F(HCacheTest, SaveIncrementalBasic) {
    auto obs = make_test_obstacles();
    HierAABBTree tree(robot_);

    // Initial FFB
    Eigen::VectorXd seed(2);
    seed << 0.5, 0.5;
    tree.find_free_box(seed, obs.data(), 1, 15, 0.1);

    int n1 = tree.store().next_idx();
    tree.save(tmp_path_.string());

    // More FFB to create new nodes
    seed << -1.0, -1.0;
    tree.find_free_box(seed, obs.data(), 1, 15, 0.1);

    int n2 = tree.store().next_idx();
    EXPECT_GT(n2, n1);

    // Incremental save
    tree.save_incremental(tmp_path_.string());

    // Load and verify
    HierAABBTree loaded = HierAABBTree::load(tmp_path_.string(), robot_);
    EXPECT_EQ(loaded.store().next_idx(), n2);
    EXPECT_EQ(loaded.total_fk_calls(), tree.total_fk_calls());

    // Verify all node data
    for (int i = 0; i < n2; ++i) {
        EXPECT_EQ(loaded.store().left(i),  tree.store().left(i));
        EXPECT_EQ(loaded.store().right(i), tree.store().right(i));
        EXPECT_EQ(loaded.store().parent(i), tree.store().parent(i));
        EXPECT_EQ(loaded.store().depth(i), tree.store().depth(i));
    }
}

TEST_F(HCacheTest, SaveIncrementalMultipleRounds) {
    auto obs = make_test_obstacles();
    HierAABBTree tree(robot_);

    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;
    tree.find_free_box(seed, obs.data(), 1, 10, 0.1);
    tree.save(tmp_path_.string());

    // Round 1: incremental
    seed << 1.0, 1.0;
    tree.find_free_box(seed, obs.data(), 1, 10, 0.1);
    tree.save_incremental(tmp_path_.string());
    int n_after_r1 = tree.store().next_idx();

    // Round 2: incremental again
    seed << -1.0, 1.0;
    tree.find_free_box(seed, obs.data(), 1, 10, 0.1);
    tree.save_incremental(tmp_path_.string());
    int n_after_r2 = tree.store().next_idx();

    EXPECT_GE(n_after_r2, n_after_r1);

    // Final verification
    HierAABBTree loaded = HierAABBTree::load(tmp_path_.string(), robot_);
    EXPECT_EQ(loaded.store().next_idx(), n_after_r2);

    // Loaded tree can continue FFB
    seed << -1.5, -1.5;
    auto res = loaded.find_free_box(seed, obs.data(), 1, 10, 0.1);
    EXPECT_TRUE(res.success());
}

// ─── mmap-backed lazy load ────────────────────────────────────────────────

TEST_F(HCacheTest, LoadMmapBasic) {
    auto obs = make_test_obstacles();
    // Build and save a tree
    HierAABBTree tree(robot_);
    Eigen::VectorXd seed(2);
    seed << 0.3, -0.3;
    tree.find_free_box(seed, obs.data(), 1, 15, 0.1);

    int n_nodes = tree.store().next_idx();
    int fk = tree.total_fk_calls();
    tree.save(tmp_path_.string());

    // Load via mmap
    HierAABBTree mmapped = HierAABBTree::load_mmap(tmp_path_.string(), robot_);
    EXPECT_EQ(mmapped.store().next_idx(), n_nodes);
    EXPECT_EQ(mmapped.total_fk_calls(), fk);

    // Verify data
    for (int i = 0; i < n_nodes; ++i) {
        EXPECT_EQ(mmapped.store().left(i),  tree.store().left(i));
        EXPECT_EQ(mmapped.store().right(i), tree.store().right(i));
        EXPECT_EQ(mmapped.store().depth(i), tree.store().depth(i));
        if (tree.store().has_aabb(i)) {
            const float* a = mmapped.store().aabb(i);
            const float* b = tree.store().aabb(i);
            int af = tree.store().aabb_floats();
            for (int f = 0; f < af; ++f)
                EXPECT_FLOAT_EQ(a[f], b[f]);
        }
    }

    mmapped.close_mmap();
}

TEST_F(HCacheTest, LoadMmapContinueFFBAutoGrow) {
    auto obs = make_test_obstacles();
    // Small tree, save, load_mmap, then grow via FFB
    HierAABBTree tree(robot_);
    tree.save(tmp_path_.string());  // just root

    HierAABBTree mmapped = HierAABBTree::load_mmap(tmp_path_.string(), robot_);
    EXPECT_EQ(mmapped.store().next_idx(), 1);

    // FFB will allocate many new nodes, triggering auto-grow of mmap
    Eigen::VectorXd seed(2);
    seed << 0.5, 0.5;
    auto res = mmapped.find_free_box(seed, obs.data(), 1, 20, 0.1);
    EXPECT_TRUE(res.success());
    EXPECT_GT(mmapped.store().next_idx(), 1);

    // Flush and close
    mmapped.flush_mmap();
    mmapped.close_mmap();

    // Re-load normally and verify data was persisted
    HierAABBTree reloaded = HierAABBTree::load(tmp_path_.string(), robot_);
    // At least the new nodes should be there
    EXPECT_GT(reloaded.store().next_idx(), 1);
}

TEST_F(HCacheTest, LoadMmapFlushAndReopen) {
    auto obs = make_test_obstacles();
    // Build tree, save, load_mmap, do more FFB, flush, reopen
    HierAABBTree tree(robot_);
    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;
    tree.find_free_box(seed, obs.data(), 1, 15, 0.1);
    tree.save(tmp_path_.string());

    {
        HierAABBTree mmapped = HierAABBTree::load_mmap(tmp_path_.string(), robot_);

        // Do additional FFB work
        seed << 1.5, 1.5;
        mmapped.find_free_box(seed, obs.data(), 1, 15, 0.1);
        int n2 = mmapped.store().next_idx();
        EXPECT_GT(n2, tree.store().next_idx());

        // Destructor will call close_mmap → flush
    }

    // Re-load and verify
    HierAABBTree reloaded = HierAABBTree::load(tmp_path_.string(), robot_);
    EXPECT_GT(reloaded.store().next_idx(), tree.store().next_idx());
}

TEST_F(HCacheTest, LoadMmapWrongRobotFails) {
    HierAABBTree tree(robot_);
    tree.save(tmp_path_.string());

    std::vector<DHParam> dhs3 = {
        {0.0, 0.3, 0.0, 0.0, 0},
        {0.0, 0.3, 0.0, 0.0, 0},
        {0.0, 0.3, 0.0, 0.0, 0}
    };
    JointLimits jl3;
    jl3.limits = {{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}};
    Robot robot3("test_3dof", dhs3, jl3);

    EXPECT_THROW(HierAABBTree::load_mmap(tmp_path_.string(), robot3),
                 std::runtime_error);
}
