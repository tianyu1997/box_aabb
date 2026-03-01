// Tests for NodeStore
#include <gtest/gtest.h>
#include "sbf/forest/node_store.h"

using namespace sbf;

TEST(NodeStoreTest, Construction) {
    NodeStore store(8, 7, 1024);
    EXPECT_EQ(store.n_dims(), 7);
    EXPECT_EQ(store.n_links(), 8);
    EXPECT_GE(store.capacity(), 1024);
}

TEST(NodeStoreTest, AllocNode) {
    NodeStore store(8, 7, 16);
    // Constructor allocates root node at idx 0, so first manual alloc is 1
    int initial = store.next_idx();
    EXPECT_EQ(initial, 1);  // root already allocated
    int n1 = store.alloc_node(0, 1);
    int n2 = store.alloc_node(0, 1);
    EXPECT_EQ(n1, 1);
    EXPECT_EQ(n2, 2);
    EXPECT_EQ(store.next_idx(), 3);
}

TEST(NodeStoreTest, LeftRightParent) {
    NodeStore store(8, 7, 16);
    int root  = store.alloc_node(-1, 0);
    int left  = store.alloc_node(root, 1);
    int right = store.alloc_node(root, 1);

    store.set_left(root, left);
    store.set_right(root, right);

    EXPECT_EQ(store.left(root), left);
    EXPECT_EQ(store.right(root), right);
    EXPECT_EQ(store.parent(left), root);
    EXPECT_EQ(store.parent(right), root);
}

TEST(NodeStoreTest, DepthAndSplit) {
    NodeStore store(2, 2, 16);
    int n = store.alloc_node(-1, 5);
    store.set_split(n, 0.42);

    EXPECT_EQ(store.depth(n), 5);
    EXPECT_NEAR(store.split_val(n), 0.42, 1e-12);
}

TEST(NodeStoreTest, Occupied) {
    NodeStore store(2, 2, 16);
    store.resize_aux(16);
    int n = store.alloc_node(-1, 0);
    EXPECT_EQ(store.occupied[n], 0);
    store.occupied[n] = 1;
    store.forest_id[n] = 42;
    EXPECT_EQ(store.occupied[n], 1);
    EXPECT_EQ(store.forest_id[n], 42);
}

TEST(NodeStoreTest, EnsureCapacityGrowth) {
    NodeStore store(2, 2, 4);  // small initial capacity (root already at idx 0)
    int base = store.next_idx();  // 1 (root)
    for (int i = 0; i < 100; ++i)
        store.alloc_node(0, 1);
    EXPECT_EQ(store.next_idx(), base + 100);
    EXPECT_GE(store.capacity(), base + 100);
}
