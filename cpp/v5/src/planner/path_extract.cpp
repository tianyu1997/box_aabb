// SafeBoxForest v5 — Path extraction (Phase H2)
// Only use shared-face centers as waypoints (no box centers).
// Each segment face_AB → face_BC stays inside box B (convex AABB).
#include <sbf/planner/path_extract.h>

#include <algorithm>
#include <unordered_map>

namespace sbf {

std::vector<Eigen::VectorXd> extract_waypoints(
    const std::vector<int>& box_sequence,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal)
{
    std::vector<Eigen::VectorXd> path;
    if (box_sequence.empty()) return path;

    // Build id→BoxNode lookup
    std::unordered_map<int, const BoxNode*> box_map;
    for (const auto& b : boxes)
        box_map[b.id] = &b;

    path.push_back(start);

    for (size_t i = 0; i + 1 < box_sequence.size(); ++i) {
        auto it_a = box_map.find(box_sequence[i]);
        auto it_b = box_map.find(box_sequence[i + 1]);
        if (it_a == box_map.end() || it_b == box_map.end()) continue;
        const BoxNode& a = *it_a->second;
        const BoxNode& b = *it_b->second;
        auto face = shared_face(a, b);

        if (face.has_value()) {
            const int nd = a.n_dims();
            Eigen::VectorXd wp(nd);
            int face_idx = 0;
            for (int d = 0; d < nd; ++d) {
                if (d == face->dim) {
                    wp[d] = face->value;
                } else {
                    wp[d] = face->face_ivs[face_idx].center();
                    ++face_idx;
                }
            }
            path.push_back(wp);
        } else {
            // Fallback: midpoint of two box centers
            path.push_back((a.center() + b.center()) * 0.5);
        }
    }

    path.push_back(goal);
    return path;
}

}  // namespace sbf
