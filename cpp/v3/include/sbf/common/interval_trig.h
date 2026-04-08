// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — Interval Trigonometric Utilities
//  Module: sbf
//
//  Multi-level Cartesian AABB reconstruction from frozen-joint local frame.
//
//  For a serial-link robot where every joint has a_i = 0 and
//  α_i ∈ {0, ±π/2} (e.g., KUKA IIWA14), each joint T_i(q_i) applies a
//  2D rotation to sub-chain coordinates.  By freezing the leading k joints
//  to q=0 during endpoint AABB computation, we obtain q_0...q_{k-1}-
//  independent stored AABBs.  Actual Cartesian AABBs are reconstructed by
//  undoing T_0(0)...T_{k-1}(0) (exact, zero-cost) then re-applying
//  T_{k-1}(q_{k-1})...T_0(q_0) with interval rotation.
//
//  α-based joint transform on position (X, Y, Z):
//    α = 0     :  (x,y) = Rot(q)·(X,Y),   z = Z + d
//    α = −π/2  :  (x,−z) = Rot(q)·(X,Y),  y = Z + d
//    α = +π/2  :  (x, z) = Rot(q)·(X,Y),  y = −Z − d
//
//  Uses existing I_cos / I_sin from interval_math.h and Interval
//  arithmetic from types.h.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/common/types.h"
#include "sbf/robot/interval_math.h"

#include <vector>
#include <cmath>

namespace sbf {

// ─── DH α-type classification ───────────────────────────────────────────
// Classifies a DH α value into one of 3 canonical types.
//   0 → α ≈  0       (z-axis rotation, passthrough on z)
//   1 → α ≈ −π/2     (xz-plane rotation, passthrough on y)
//   2 → α ≈ +π/2     (xz-plane rotation, passthrough on y, negated)
inline int classify_alpha(double alpha) {
    if (std::abs(alpha) < 0.01)                return 0;
    if (std::abs(alpha + 1.5707963268) < 0.01) return 1;  // −π/2
    if (std::abs(alpha - 1.5707963268) < 0.01) return 2;  // +π/2
    return -1;  // unsupported
}

// ─── Frozen joint descriptor (for multi-level reconstruction) ───────────
struct FrozenJointDesc {
    int    alpha_type;   // 0, 1(−π/2), 2(+π/2)
    double d;            // DH d_i offset
    Interval q;          // actual joint interval for the node
};

// ─── Undo T_i(0) on (X,Y,Z) — exact, no interval widening ─────────────
//
// T_i(0) for a=0 maps (X_in, Y_in, Z_in) to:
//   α = 0:    (X, Y, Z+d)         → undo: (X, Y, Z−d)
//   α = −π/2: (X, Z, −Y)   +d·y  → undo: (X, −Z, Y−d)    [d appears in y]
//     actually: y_out = Z_in + d, so Z_in = y_out - d
//     z_out = -Y_in ⟹ Y_in = -z_out
//     so undo: (X, −Z, Y−d)
//   α = +π/2: (X, −Z−d, Y)       → undo: (X, Z, −Y−d)
//
// This function reverses one T_i(0) application.
// Since q_i=0 means cos=1, sin=0, the transform and its inverse are exact
// permutation + offset — no interval multiplication needed.
inline void undo_frozen_joint(
    Interval& X, Interval& Y, Interval& Z,
    int alpha_type, double d)
{
    switch (alpha_type) {
    case 0: // α=0: stored=(X, Y, Z+d) → local=(X, Y, Z-d)
        Z = {Z.lo - d, Z.hi - d};
        break;
    case 1: // α=−π/2: stored=(X, Z+d, −Y) → local=(X, −Z, Y−d)
    {
        Interval newY(-Z.hi, -Z.lo);
        Interval newZ(Y.lo - d, Y.hi - d);
        Y = newY;
        Z = newZ;
        break;
    }
    case 2: // α=+π/2: stored=(X, −Z−d, Y) → local=(X, Z, −Y−d)
    {
        Interval newY = Z;
        Interval newZ(-Y.hi - d, -Y.lo - d);
        Y = newY;
        Z = newZ;
        break;
    }
    }
}

// ─── Apply T_i(q_i) on (X,Y,Z) — interval rotation ────────────────────
//
// Applies the modified DH joint transform for a=0 to interval coordinates.
// The transform depends on α:
//   α = 0:    (c·X − s·Y,  s·X + c·Y,  Z + d)
//   α = −π/2: (c·X − s·Y,  Z + d,      −(s·X + c·Y))
//   α = +π/2: (c·X − s·Y,  −Z − d,     s·X + c·Y)
//
inline void apply_joint_rotation(
    Interval& X, Interval& Y, Interval& Z,
    const Interval& q, int alpha_type, double d)
{
    Interval cq = I_cos(q.lo, q.hi);
    Interval sq = I_sin(q.lo, q.hi);
    Interval rot_a = cq * X - sq * Y;   // cos·X − sin·Y
    Interval rot_b = sq * X + cq * Y;   // sin·X + cos·Y

    switch (alpha_type) {
    case 0: // α=0
        X = rot_a;
        Y = rot_b;
        Z = {Z.lo + d, Z.hi + d};
        break;
    case 1: // α=−π/2
        X = rot_a;
        Y = {Z.lo + d, Z.hi + d};
        Z = {-rot_b.hi, -rot_b.lo};
        break;
    case 2: // α=+π/2
        X = rot_a;
        Y = {-Z.hi - d, -Z.lo - d};
        Z = rot_b;
        break;
    }
}

// ─── Multi-level Cartesian reconstruction ───────────────────────────────
//
// stored:       float[n_endpoints × 6] — Cartesian at q_0=0,...,q_{k-1}=0
// frozen:       array of FrozenJointDesc, outermost (joint 0) first
// freeze_depth: number of frozen joints (= length of frozen array)
// out_cart:     float[n_endpoints × 6] — actual Cartesian AABBs
//
// Algorithm:
//   1. Undo T_0(0)...T_{k-1}(0) to recover the innermost local frame
//      (exact — only permutations, sign flips, and scalar offsets)
//   2. Apply T_{k-1}(q_{k-1})...T_0(q_0) with interval rotation
//      (introduces interval wrapping, one rotation per frozen joint)
//
inline void reconstruct_cartesian_endpoints_multilevel(
    const float* stored,
    int n_endpoints,
    const FrozenJointDesc* frozen,
    int freeze_depth,
    float* out_cart)
{
    for (int k = 0; k < n_endpoints; ++k) {
        const float* ep = stored + k * 6;
        Interval X(static_cast<double>(ep[0]), static_cast<double>(ep[3]));
        Interval Y(static_cast<double>(ep[1]), static_cast<double>(ep[4]));
        Interval Z(static_cast<double>(ep[2]), static_cast<double>(ep[5]));

        // Step 1: Undo frozen transforms — outermost first (0, 1, ..., k-1)
        for (int j = 0; j < freeze_depth; ++j) {
            undo_frozen_joint(X, Y, Z, frozen[j].alpha_type, frozen[j].d);
        }

        // Step 2: Apply actual transforms — innermost first (k-1, ..., 1, 0)
        for (int j = freeze_depth - 1; j >= 0; --j) {
            apply_joint_rotation(X, Y, Z,
                                 frozen[j].q, frozen[j].alpha_type, frozen[j].d);
        }

        float* out = out_cart + k * 6;
        out[0] = static_cast<float>(X.lo);
        out[1] = static_cast<float>(Y.lo);
        out[2] = static_cast<float>(Z.lo);
        out[3] = static_cast<float>(X.hi);
        out[4] = static_cast<float>(Y.hi);
        out[5] = static_cast<float>(Z.hi);
    }
}

// ─── Convenience overload for std::vector ───────────────────────────────
inline std::vector<float> reconstruct_cartesian_endpoints_multilevel(
    const std::vector<float>& stored,
    int n_endpoints,
    const FrozenJointDesc* frozen,
    int freeze_depth)
{
    std::vector<float> cart(stored.size());
    reconstruct_cartesian_endpoints_multilevel(
        stored.data(), n_endpoints, frozen, freeze_depth, cart.data());
    return cart;
}

// ─── Legacy 1-level convenience wrappers ────────────────────────────────
// Kept for backward compatibility.  Equivalent to freeze_depth=1 with α=0.
inline void reconstruct_cartesian_endpoints(
    const float* local_endpoints,
    int n_endpoints,
    const Interval& q0,
    float* out_cart)
{
    FrozenJointDesc desc{0, 0.0, q0};  // α=0, d irrelevant for 1-level
    reconstruct_cartesian_endpoints_multilevel(
        local_endpoints, n_endpoints, &desc, 1, out_cart);
}

inline std::vector<float> reconstruct_cartesian_endpoints(
    const std::vector<float>& local_endpoints,
    int n_endpoints,
    const Interval& q0)
{
    FrozenJointDesc desc{0, 0.0, q0};
    return reconstruct_cartesian_endpoints_multilevel(
        local_endpoints, n_endpoints, &desc, 1);
}

} // namespace sbf
