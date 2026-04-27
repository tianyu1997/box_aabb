#pragma once
/// @file union_find.h
/// @brief Disjoint-set union (path compression + union by rank).

#include <cstdint>
#include <vector>

namespace sbf::forest {

class UnionFind {
public:
    UnionFind() = default;
    explicit UnionFind(int n) { reset(n); }

    void reset(int n) {
        parent_.resize(n);
        rank_.assign(n, 0);
        n_components_ = n;
        for (int i = 0; i < n; ++i) parent_[i] = i;
    }

    int find(int x) {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];   // path halving
            x = parent_[x];
        }
        return x;
    }

    bool unite(int a, int b) {
        int ra = find(a), rb = find(b);
        if (ra == rb) return false;
        if (rank_[ra] < rank_[rb]) std::swap(ra, rb);
        parent_[rb] = ra;
        if (rank_[ra] == rank_[rb]) ++rank_[ra];
        --n_components_;
        return true;
    }

    bool connected(int a, int b) { return find(a) == find(b); }
    int  num_components() const { return n_components_; }
    int  size() const { return static_cast<int>(parent_.size()); }

    /// Append a new singleton component; returns its index.
    int  push() {
        int idx = static_cast<int>(parent_.size());
        parent_.push_back(idx);
        rank_.push_back(0);
        ++n_components_;
        return idx;
    }

private:
    std::vector<int>     parent_;
    std::vector<uint8_t> rank_;
    int                  n_components_ = 0;
};

}  // namespace sbf::forest
