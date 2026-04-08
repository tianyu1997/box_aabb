// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — KDTree: lightweight N-dimensional KD-tree for
//  nearest-box queries using point-to-box distance.
//  Module: sbf::forest
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace sbf {
namespace forest {

class KDTree {
public:
	KDTree() = default;

	void build(const std::vector<BoxNode>& boxes) {
		n_dims_ = 0;
		nodes_.clear();
		entries_.clear();
		pending_.clear();
		pending_count_ = 0;
		tree_size_ = 0;

		if (boxes.empty()) return;
		n_dims_ = boxes[0].n_dims();

		entries_.resize(boxes.size());
		for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
			entries_[i].box_id = boxes[i].id;
			entries_[i].intervals = boxes[i].joint_intervals;
			entries_[i].center.resize(n_dims_);
			for (int d = 0; d < n_dims_; ++d)
				entries_[i].center[d] = boxes[i].joint_intervals[d].center();
		}

		std::vector<int> indices(boxes.size());
		std::iota(indices.begin(), indices.end(), 0);

		nodes_.resize(boxes.size());
		build_recursive(indices.data(), static_cast<int>(indices.size()), 0, 0);
		tree_size_ = static_cast<int>(boxes.size());
	}

	void rebuild(const std::vector<BoxNode>& boxes) { build(boxes); }

	void add_box(const BoxNode& box) {
		PendingEntry pe;
		pe.box_id = box.id;
		pe.intervals = box.joint_intervals;
		pending_.push_back(std::move(pe));
		pending_count_++;
	}

	int find_nearest(const Eigen::VectorXd& q) const {
		if (tree_size_ == 0 && pending_.empty()) return -1;

		double best_dist = std::numeric_limits<double>::max();
		int best_id = -1;

		if (tree_size_ > 0)
			search_recursive(0, q, best_dist, best_id);

		for (const auto& pe : pending_) {
			double dist = distance_to_box(q, pe.intervals);
			if (dist < best_dist) {
				best_dist = dist;
				best_id = pe.box_id;
			}
		}

		return best_id;
	}

	int size() const { return tree_size_ + pending_count_; }

	void clear() {
		nodes_.clear();
		entries_.clear();
		pending_.clear();
		tree_size_ = 0;
		pending_count_ = 0;
	}

private:
	struct Node {
		int entry_idx = -1;
		int left = -1;
		int right = -1;
		int split_dim = 0;
		std::vector<double> bbox_lo;
		std::vector<double> bbox_hi;
	};

	struct Entry {
		int box_id = -1;
		std::vector<Interval> intervals;
		std::vector<double> center;
	};

	struct PendingEntry {
		int box_id = -1;
		std::vector<Interval> intervals;
	};

	int n_dims_ = 0;
	int tree_size_ = 0;
	int pending_count_ = 0;
	std::vector<Node> nodes_;
	std::vector<Entry> entries_;
	std::vector<PendingEntry> pending_;

	int build_recursive(int* idx_arr, int count, int depth, int node_idx) {
		if (count <= 0) return -1;

		int dim = depth % n_dims_;
		int mid = count / 2;
		std::nth_element(idx_arr, idx_arr + mid, idx_arr + count,
			[this, dim](int a, int b) {
				return entries_[a].center[dim] < entries_[b].center[dim];
			});

		int ni = node_idx;
		nodes_[ni].entry_idx = idx_arr[mid];
		nodes_[ni].split_dim = dim;

		const auto& iv = entries_[idx_arr[mid]].intervals;
		nodes_[ni].bbox_lo.resize(n_dims_);
		nodes_[ni].bbox_hi.resize(n_dims_);
		for (int d = 0; d < n_dims_; ++d) {
			nodes_[ni].bbox_lo[d] = iv[d].lo;
			nodes_[ni].bbox_hi[d] = iv[d].hi;
		}

		if (mid > 0) {
			int left_ni = ni + 1;
			nodes_[ni].left = left_ni;
			build_recursive(idx_arr, mid, depth + 1, left_ni);
			merge_bbox_(nodes_[ni], nodes_[left_ni]);
		} else {
			nodes_[ni].left = -1;
		}

		int right_count = count - mid - 1;
		if (right_count > 0) {
			int right_ni = ni + 1 + mid;
			nodes_[ni].right = right_ni;
			build_recursive(idx_arr + mid + 1, right_count, depth + 1, right_ni);
			merge_bbox_(nodes_[ni], nodes_[right_ni]);
		} else {
			nodes_[ni].right = -1;
		}

		return ni;
	}

	void merge_bbox_(Node& parent, const Node& child) const {
		for (int d = 0; d < n_dims_; ++d) {
			if (child.bbox_lo[d] < parent.bbox_lo[d]) parent.bbox_lo[d] = child.bbox_lo[d];
			if (child.bbox_hi[d] > parent.bbox_hi[d]) parent.bbox_hi[d] = child.bbox_hi[d];
		}
	}

	void search_recursive(int ni, const Eigen::VectorXd& q,
						  double& best_dist, int& best_id) const {
		if (ni < 0 || ni >= tree_size_) return;

		const Node& node = nodes_[ni];
		double bbox_dist = distance_to_bbox_(q, node.bbox_lo, node.bbox_hi);
		if (bbox_dist >= best_dist) return;

		const Entry& entry = entries_[node.entry_idx];
		double dist = distance_to_box(q, entry.intervals);
		if (dist < best_dist) {
			best_dist = dist;
			best_id = entry.box_id;
		}
		if (best_dist <= 0.0) return;

		int dim = node.split_dim;
		double diff = q[dim] - entry.center[dim];
		int first = (diff <= 0) ? node.left : node.right;
		int second = (diff <= 0) ? node.right : node.left;

		search_recursive(first, q, best_dist, best_id);
		search_recursive(second, q, best_dist, best_id);
	}

	static double distance_to_box(const Eigen::VectorXd& q,
								  const std::vector<Interval>& intervals) {
		double dist_sq = 0.0;
		for (int d = 0; d < static_cast<int>(intervals.size()); ++d) {
			double v = q[d];
			double lo = intervals[d].lo;
			double hi = intervals[d].hi;
			if (v < lo) dist_sq += (lo - v) * (lo - v);
			else if (v > hi) dist_sq += (v - hi) * (v - hi);
		}
		return std::sqrt(dist_sq);
	}

	double distance_to_bbox_(const Eigen::VectorXd& q,
							 const std::vector<double>& lo,
							 const std::vector<double>& hi) const {
		double dist_sq = 0.0;
		for (int d = 0; d < n_dims_; ++d) {
			if (q[d] < lo[d]) dist_sq += (lo[d] - q[d]) * (lo[d] - q[d]);
			else if (q[d] > hi[d]) dist_sq += (q[d] - hi[d]) * (q[d] - hi[d]);
		}
		return std::sqrt(dist_sq);
	}
};

} // namespace forest
} // namespace sbf
