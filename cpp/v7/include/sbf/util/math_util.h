#pragma once
#include <Eigen/Geometry>

namespace sbf::util {

// Clamp val into the closed interval [lo, hi].
template <typename T>
inline T clamp(T val, T lo, T hi) {
    return val < lo ? lo : (val > hi ? hi : val);
}

// Face-overlap area between two AABBs along the given axis (0=x, 1=y, 2=z).
// Returns 0 if the boxes do not share a face along that axis.
double face_overlap_area(
    const Eigen::AlignedBox3d& a,
    const Eigen::AlignedBox3d& b,
    int axis);

// True iff the two AABBs share a positive-area face along some axis.
// Touching only at a point or along an edge returns false.
bool has_face_overlap(
    const Eigen::AlignedBox3d& a,
    const Eigen::AlignedBox3d& b,
    double epsilon = 1e-6);

}  // namespace sbf::util
