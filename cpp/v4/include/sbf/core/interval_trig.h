// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Interval Trigonometric Utilities
//  Module: sbf::core
//
//  DH α-type classification for iAABB preprocessing.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <cmath>

namespace sbf {

// ─── DH α-type classification ───────────────────────────────────────────
// Classifies a DH α value into one of 3 canonical types.
//   0 → α ≈  0       (z-axis rotation, passthrough on z)
//   1 → α ≈ −π/2     (xz-plane rotation, passthrough on y)
//   2 → α ≈ +π/2     (xz-plane rotation, passthrough on y, negated)
//  -1 → unsupported
inline int classify_alpha(double alpha) {
    if (std::abs(alpha) < 0.01)                return 0;
    if (std::abs(alpha + 1.5707963268) < 0.01) return 1;  // −π/2
    if (std::abs(alpha - 1.5707963268) < 0.01) return 2;  // +π/2
    return -1;  // unsupported
}

} // namespace sbf
