#pragma once
#include <numeric>
#include <vector>

namespace sbf {

/// Weighted Union-Find with path halving.
/// Header-only for inlining in hot loops.
class UnionFind {
public:
    explicit UnionFind(int n = 0) : parent_(n), rank_(n, 0) {
        std::iota(parent_.begin(), parent_.end(), 0);
    }

    int find(int x) {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];  // path halving
            x = parent_[x];
        }
        return x;
    }

    /// @return true if x and y were in different components (actual merge).
    bool unite(int x, int y) {
        int rx = find(x), ry = find(y);
        if (rx == ry) return false;
        if (rank_[rx] < rank_[ry]) std::swap(rx, ry);
        parent_[ry] = rx;
        if (rank_[rx] == rank_[ry]) rank_[rx]++;
        return true;
    }

    bool connected(int x, int y) { return find(x) == find(y); }
    int  size() const { return static_cast<int>(parent_.size()); }

    void resize(int new_size) {
        int old = static_cast<int>(parent_.size());
        if (new_size <= old) return;
        parent_.resize(new_size);
        rank_.resize(new_size, 0);
        for (int i = old; i < new_size; ++i) parent_[i] = i;
    }

private:
    std::vector<int> parent_;
    std::vector<int> rank_;
};

}  // namespace sbf
