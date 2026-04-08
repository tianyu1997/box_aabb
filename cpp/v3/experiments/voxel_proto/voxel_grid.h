// ═══════════════════════════════════════════════════════════════════════════
//  Sparse Voxel BitMask — VoxelGrid container
//  SafeBoxForest v3 / Phase-1 Prototype
//
//  VoxelGrid wraps a SpatialHash<BrickCoord, BitBrick>.
//  Provides:
//    • fill_aabb()              — mark voxels overlapping a Cartesian AABB
//    • fill_capsule()           — mark voxels inside an inflated capsule
//    • fill_interval_capsule()  — capsule with interval endpoints
//    • merge()  / collides()    — bitwise OR / AND over brick maps
//    • count_occupied()         — popcount-based volume query
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "bit_brick.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace sbf {
namespace voxel {

class VoxelGrid {
public:
    using BrickMap = std::unordered_map<BrickCoord, BitBrick, BrickCoordHash>;

    VoxelGrid() = default;

    /// @param delta  voxel edge length in metres (e.g. 0.02 = 2 cm)
    /// @param ox,oy,oz  world-space origin (lower corner of cell 0,0,0)
    explicit VoxelGrid(double delta,
                       double ox = 0.0, double oy = 0.0, double oz = 0.0)
        : delta_(delta)
        , inv_delta_(1.0 / delta)
        , origin_{ox, oy, oz}
        , safety_pad_(std::sqrt(3.0) * delta * 0.5)   // √3 Δ / 2
    {}

    // =====================================================================
    //  Coordinate helpers
    // =====================================================================

    /// World position → integer cell index (signed, unbounded)
    int to_cell(double w, int axis) const noexcept {
        return static_cast<int>(std::floor((w - origin_[axis]) * inv_delta_));
    }

    /// Cell index → world position of cell centre
    double cell_center(int c, int axis) const noexcept {
        return origin_[axis] + (c + 0.5) * delta_;
    }

    // =====================================================================
    //  Rasterisation primitives
    // =====================================================================

    /// Fill all voxels that overlap an axis-aligned box.
    /// @param aabb  float[6]: [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi]
    void fill_aabb(const float* aabb) {
        int cx0 = to_cell(static_cast<double>(aabb[0]), 0);
        int cy0 = to_cell(static_cast<double>(aabb[1]), 1);
        int cz0 = to_cell(static_cast<double>(aabb[2]), 2);
        int cx1 = to_cell(static_cast<double>(aabb[3]), 0);
        int cy1 = to_cell(static_cast<double>(aabb[4]), 1);
        int cz1 = to_cell(static_cast<double>(aabb[5]), 2);

        for (int cz = cz0; cz <= cz1; ++cz)
            for (int cy = cy0; cy <= cy1; ++cy)
                for (int cx = cx0; cx <= cx1; ++cx)
                    set_cell(cx, cy, cz);
    }

    /// Fill all voxels inside an inflated capsule.
    ///
    /// The effective radius is `radius + √3 Δ / 2` so that every point
    /// inside the true capsule falls within at least one marked voxel
    /// (certified conservative coverage).
    ///
    /// @param p1,p2  capsule segment endpoints (double[3] each)
    /// @param radius capsule radius (before safety padding)
    void fill_capsule(const double* p1, const double* p2, double radius) {
        const double r_eff    = radius + safety_pad_;
        const double r_eff_sq = r_eff * r_eff;

        // Segment direction
        const double dx = p2[0] - p1[0];
        const double dy = p2[1] - p1[1];
        const double dz = p2[2] - p1[2];
        const double len_sq = dx*dx + dy*dy + dz*dz;

        // AABB of inflated capsule
        double lo[3], hi[3];
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(p1[a], p2[a]) - r_eff;
            hi[a] = std::max(p1[a], p2[a]) + r_eff;
        }

        const int cx0 = to_cell(lo[0], 0), cx1 = to_cell(hi[0], 0);
        const int cy0 = to_cell(lo[1], 1), cy1 = to_cell(hi[1], 1);
        const int cz0 = to_cell(lo[2], 2), cz1 = to_cell(hi[2], 2);

        for (int cz = cz0; cz <= cz1; ++cz) {
            const double wz = cell_center(cz, 2);
            for (int cy = cy0; cy <= cy1; ++cy) {
                const double wy = cell_center(cy, 1);
                for (int cx = cx0; cx <= cx1; ++cx) {
                    const double wx = cell_center(cx, 0);

                    // Squared distance from voxel centre to segment [p1, p2]
                    double dist_sq;
                    if (len_sq < 1e-18) {
                        // Degenerate (point-like) capsule
                        double vx = wx - p1[0], vy = wy - p1[1], vz = wz - p1[2];
                        dist_sq = vx*vx + vy*vy + vz*vz;
                    } else {
                        double vx = wx - p1[0], vy = wy - p1[1], vz = wz - p1[2];
                        double t  = (vx*dx + vy*dy + vz*dz) / len_sq;
                        t = std::clamp(t, 0.0, 1.0);
                        double cx_ = wx - (p1[0] + t*dx);
                        double cy_ = wy - (p1[1] + t*dy);
                        double cz_ = wz - (p1[2] + t*dz);
                        dist_sq = cx_*cx_ + cy_*cy_ + cz_*cz_;
                    }

                    if (dist_sq <= r_eff_sq)
                        set_cell(cx, cy, cz);
                }
            }
        }
    }

    /// Fill a capsule whose endpoints are themselves interval-valued.
    ///
    /// Conservative treatment:  the centre-to-centre segment is used with
    /// an inflated radius that accounts for endpoint uncertainty.
    ///
    /// @param prox_iv  float[6]  proximal endpoint interval [lo_xyz, hi_xyz]
    /// @param dist_iv  float[6]  distal   endpoint interval [lo_xyz, hi_xyz]
    /// @param link_radius  capsule body radius (before padding)
    void fill_interval_capsule(const float* prox_iv,
                               const float* dist_iv,
                               double link_radius)
    {
        double p1[3], p2[3];
        double hd_prox_sq = 0.0, hd_dist_sq = 0.0;
        for (int a = 0; a < 3; ++a) {
            p1[a] = 0.5 * (prox_iv[a] + prox_iv[a + 3]);
            p2[a] = 0.5 * (dist_iv[a]  + dist_iv[a + 3]);
            double hp = 0.5 * (prox_iv[a + 3] - prox_iv[a]);
            double hd = 0.5 * (dist_iv[a + 3]  - dist_iv[a]);
            hd_prox_sq += hp * hp;
            hd_dist_sq += hd * hd;
        }
        double extra = std::max(std::sqrt(hd_prox_sq), std::sqrt(hd_dist_sq));
        fill_capsule(p1, p2, link_radius + extra);
    }

    /// Tight interval-capsule rasterisation (parametric sweep).
    ///
    /// For each candidate voxel, discretises the segment parameter t and
    /// checks whether the voxel centre is within distance `r_link + √3Δ/2`
    /// of the axis-aligned box at that parameter:
    ///     B(t) = { (1-t)·p₁ + t·p₂ : p₁ ∈ P_box, p₂ ∈ D_box }
    ///
    /// This is *much* tighter than inflating capsule radius by the full
    /// endpoint half-diagonal, because B(t) is a small box that simply
    /// slides and rescales along the segment.
    ///
    /// @param t_steps  number of discretisation steps along segment (≥8)
    void fill_interval_capsule_tight(const float* prox_iv,
                                     const float* dist_iv,
                                     double link_radius,
                                     int t_steps = 16)
    {
        const double r_eff    = link_radius + safety_pad_;
        const double r_eff_sq = r_eff * r_eff;

        // Bounding box of all possible capsule surfaces
        double lo[3], hi[3];
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(static_cast<double>(prox_iv[a]),
                             static_cast<double>(dist_iv[a]))  - r_eff;
            hi[a] = std::max(static_cast<double>(prox_iv[a+3]),
                             static_cast<double>(dist_iv[a+3])) + r_eff;
        }

        const int cx0 = to_cell(lo[0], 0), cx1 = to_cell(hi[0], 0);
        const int cy0 = to_cell(lo[1], 1), cy1 = to_cell(hi[1], 1);
        const int cz0 = to_cell(lo[2], 2), cz1 = to_cell(hi[2], 2);

        const double dt = 1.0 / t_steps;

        for (int cz = cz0; cz <= cz1; ++cz) {
            const double wz = cell_center(cz, 2);
            for (int cy = cy0; cy <= cy1; ++cy) {
                const double wy = cell_center(cy, 1);
                for (int cx = cx0; cx <= cx1; ++cx) {
                    const double wx = cell_center(cx, 0);
                    const double w[3] = {wx, wy, wz};

                    double min_d2 = 1e30;
                    for (int k = 0; k <= t_steps; ++k) {
                        const double t = k * dt;
                        // Box at parameter t:
                        //   c_lo[a] = (1-t)*prox_lo[a] + t*dist_lo[a]
                        //   c_hi[a] = (1-t)*prox_hi[a] + t*dist_hi[a]
                        double d2 = 0.0;
                        for (int a = 0; a < 3; ++a) {
                            double c_lo = (1-t)*prox_iv[a]   + t*dist_iv[a];
                            double c_hi = (1-t)*prox_iv[a+3] + t*dist_iv[a+3];
                            double d;
                            if      (w[a] < c_lo) d = w[a] - c_lo;
                            else if (w[a] > c_hi) d = w[a] - c_hi;
                            else                   d = 0.0;
                            d2 += d * d;
                        }
                        if (d2 < min_d2) min_d2 = d2;
                        if (min_d2 <= r_eff_sq) break;   // early exit
                    }
                    if (min_d2 <= r_eff_sq)
                        set_cell(cx, cy, cz);
                }
            }
        }
    }

    // =====================================================================
    //  16-Point Convex Hull rasterisation (Analytical exact)
    // =====================================================================
    //
    //  Mathematical foundation:
    //
    //  Given proximal endpoint box B₁ and distal endpoint box B₂, the
    //  swept volume of all possible link axis segments is:
    //
    //      S = { (1-t)·p₁ + t·p₂ : p₁ ∈ B₁, p₂ ∈ B₂, t ∈ [0,1] }
    //        = Conv( corners(B₁) ∪ corners(B₂) )     [16 vertices]
    //
    //  At parameter t, the cross-section B(t) = (1-t)B₁ + tB₂ is an AABB:
    //      lo_a(t) = b1_lo[a] + t · Δlo[a]
    //      hi_a(t) = b1_hi[a] + t · Δhi[a]
    //
    //  For a query point q, dist²(q, B(t)) is CONVEX piecewise-quadratic
    //  in t, with at most 6 breakpoints (2 per axis: where q[a] crosses
    //  lo_a(t) or hi_a(t)). We minimise analytically over [0,1] — O(1).
    //
    //  After Minkowski inflation by (r_link + √3·Δ/2), coverage is
    //  provably conservative.
    //

    /// Analytically minimise dist²(q, B(t)) over t ∈ [0,1].
    /// B(t): lo_a(t) = b1_lo[a] + t·slo[a],  hi_a(t) = b1_hi[a] + t·shi[a].
    /// Exploits convex piecewise-quadratic structure (≤ 8 segments).
    static double min_dist_sq_to_hull(
        const double q[3],
        const double b1_lo[3], const double b1_hi[3],
        const double slo[3],   const double shi[3]) noexcept
    {
        // --- Collect breakpoints where constraint activity changes --------
        double bp[8];
        int nb = 0;
        bp[nb++] = 0.0;
        bp[nb++] = 1.0;

        for (int a = 0; a < 3; ++a) {
            // lo_a(t) = q[a] → t = (q[a] − b1_lo[a]) / slo[a]
            if (std::abs(slo[a]) > 1e-15) {
                double t = (q[a] - b1_lo[a]) / slo[a];
                if (t > 1e-12 && t < 1.0 - 1e-12) bp[nb++] = t;
            }
            // hi_a(t) = q[a] → t = (q[a] − b1_hi[a]) / shi[a]
            if (std::abs(shi[a]) > 1e-15) {
                double t = (q[a] - b1_hi[a]) / shi[a];
                if (t > 1e-12 && t < 1.0 - 1e-12) bp[nb++] = t;
            }
        }

        // Insertion sort (≤ 8 elements)
        for (int i = 1; i < nb; ++i)
            for (int j = i; j > 0 && bp[j] < bp[j - 1]; --j)
                std::swap(bp[j], bp[j - 1]);

        // Lambda: evaluate dist² at specific t
        auto eval = [&](double t) noexcept -> double {
            double d2 = 0.0;
            for (int a = 0; a < 3; ++a) {
                double lo = b1_lo[a] + t * slo[a];
                double hi = b1_hi[a] + t * shi[a];
                double d;
                if      (q[a] < lo) d = q[a] - lo;
                else if (q[a] > hi) d = q[a] - hi;
                else continue;
                d2 += d * d;
            }
            return d2;
        };

        double best = eval(0.0);
        best = std::min(best, eval(1.0));

        // --- For each piecewise interval, find quadratic optimum ----------
        for (int i = 0; i + 1 < nb; ++i) {
            double ta = bp[i], tb = bp[i + 1];
            if (tb - ta < 1e-12) continue;

            // Evaluate at interval endpoint
            best = std::min(best, eval(tb));

            // Determine which constraints are active at interval midpoint,
            // then compute D²(t) = A·t² + B·t + C for this segment.
            double tmid = 0.5 * (ta + tb);
            double A = 0.0, B = 0.0, C = 0.0;

            for (int a = 0; a < 3; ++a) {
                double lo_m = b1_lo[a] + tmid * slo[a];
                double hi_m = b1_hi[a] + tmid * shi[a];

                if (q[a] < lo_m) {
                    // f_a(t) = lo_a(t) − q[a] = (b1_lo[a]−q[a]) + t·slo[a]
                    double c = b1_lo[a] - q[a], d = slo[a];
                    A += d * d;  B += 2 * c * d;  C += c * c;
                } else if (q[a] > hi_m) {
                    // f_a(t) = q[a] − hi_a(t) = (q[a]−b1_hi[a]) − t·shi[a]
                    double c = q[a] - b1_hi[a], d = -shi[a];
                    A += d * d;  B += 2 * c * d;  C += c * c;
                }
                // else q[a] inside interval → zero contribution
            }

            if (A > 1e-15) {
                double topt = std::clamp(-B / (2.0 * A), ta, tb);
                double d2 = A * topt * topt + B * topt + C;
                best = std::min(best, d2);
            }
        }

        return best;
    }

    /// Fill voxels inside Conv(B₁ ∪ B₂) ⊕ Ball(r_link + √3·Δ/2).
    ///
    /// This is the 16-point convex hull of two endpoint interval boxes,
    /// Minkowski-summed with the link radius + discretisation safety pad.
    /// Coverage is mathematically certified conservative.
    ///
    /// @param prox_iv  float[6]  B₁ = [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
    /// @param dist_iv  float[6]  B₂ = [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
    /// @param link_radius  capsule body radius
    void fill_convex_hull_16(const float* prox_iv,
                             const float* dist_iv,
                             double link_radius)
    {
        const double r_eff    = link_radius + safety_pad_;
        const double r_eff_sq = r_eff * r_eff;

        // B₁ at t=0, slopes for linear interpolation to B₂ at t=1
        double b1_lo[3], b1_hi[3], slo[3], shi[3];
        double lo[3], hi[3];
        for (int a = 0; a < 3; ++a) {
            b1_lo[a] = prox_iv[a];
            b1_hi[a] = prox_iv[a + 3];
            slo[a]   = dist_iv[a]     - prox_iv[a];
            shi[a]   = dist_iv[a + 3] - prox_iv[a + 3];
            lo[a] = std::min(static_cast<double>(prox_iv[a]),
                             static_cast<double>(dist_iv[a]))    - r_eff;
            hi[a] = std::max(static_cast<double>(prox_iv[a + 3]),
                             static_cast<double>(dist_iv[a + 3])) + r_eff;
        }

        const int cx0 = to_cell(lo[0], 0), cx1 = to_cell(hi[0], 0);
        const int cy0 = to_cell(lo[1], 1), cy1 = to_cell(hi[1], 1);
        const int cz0 = to_cell(lo[2], 2), cz1 = to_cell(hi[2], 2);

        for (int cz = cz0; cz <= cz1; ++cz) {
            const double wz = cell_center(cz, 2);
            for (int cy = cy0; cy <= cy1; ++cy) {
                const double wy = cell_center(cy, 1);
                for (int cx = cx0; cx <= cx1; ++cx) {
                    const double wx = cell_center(cx, 0);
                    const double q[3] = {wx, wy, wz};

                    if (min_dist_sq_to_hull(q, b1_lo, b1_hi, slo, shi) <= r_eff_sq)
                        set_cell(cx, cy, cz);
                }
            }
        }
    }

    // =====================================================================
    //  16-Point Convex Hull — Scanline Rasterisation (accelerated)
    // =====================================================================
    //
    //  Exploit the CONVEXITY of Conv(B₁∪B₂)⊕Ball(r):
    //  Any axis-aligned ray intersects this body in at most ONE contiguous
    //  segment.  Therefore for each (Y,Z) scanline we only need to find
    //  the ENTRY and EXIT cells, then fill the entire range — no per-voxel
    //  distance evaluation for interior cells.
    //
    //  Additional optimisation:  a Y/Z-only distance pre-filter skips
    //  scanlines that are entirely outside the hull cross-section.
    //

    /// Minimise dist²(q, B(t)) projected onto Y-Z only (axes 1,2).
    /// Same piecewise-quadratic approach but 2D → cheaper (≤ 6 segments).
    static double min_dist_sq_to_hull_yz(
        double qy, double qz,
        const double b1_lo[3], const double b1_hi[3],
        const double slo[3],   const double shi[3]) noexcept
    {
        // Collect breakpoints from Y and Z axes only
        double bp[6];
        int nb = 0;
        bp[nb++] = 0.0;
        bp[nb++] = 1.0;

        for (int a = 1; a <= 2; ++a) {
            double qa = (a == 1) ? qy : qz;
            if (std::abs(slo[a]) > 1e-15) {
                double t = (qa - b1_lo[a]) / slo[a];
                if (t > 1e-12 && t < 1.0 - 1e-12) bp[nb++] = t;
            }
            if (std::abs(shi[a]) > 1e-15) {
                double t = (qa - b1_hi[a]) / shi[a];
                if (t > 1e-12 && t < 1.0 - 1e-12) bp[nb++] = t;
            }
        }

        // Insertion sort
        for (int i = 1; i < nb; ++i)
            for (int j = i; j > 0 && bp[j] < bp[j - 1]; --j)
                std::swap(bp[j], bp[j - 1]);

        auto eval_yz = [&](double t) noexcept -> double {
            double d2 = 0.0;
            for (int a = 1; a <= 2; ++a) {
                double qa = (a == 1) ? qy : qz;
                double lo = b1_lo[a] + t * slo[a];
                double hi = b1_hi[a] + t * shi[a];
                double d;
                if      (qa < lo) d = qa - lo;
                else if (qa > hi) d = qa - hi;
                else continue;
                d2 += d * d;
            }
            return d2;
        };

        double best = eval_yz(0.0);
        best = std::min(best, eval_yz(1.0));

        for (int i = 0; i + 1 < nb; ++i) {
            double ta = bp[i], tb = bp[i + 1];
            if (tb - ta < 1e-12) continue;
            best = std::min(best, eval_yz(tb));

            double tmid = 0.5 * (ta + tb);
            double A = 0.0, B = 0.0, C = 0.0;
            for (int a = 1; a <= 2; ++a) {
                double qa = (a == 1) ? qy : qz;
                double lo_m = b1_lo[a] + tmid * slo[a];
                double hi_m = b1_hi[a] + tmid * shi[a];
                if (qa < lo_m) {
                    double c = b1_lo[a] - qa, d = slo[a];
                    A += d*d;  B += 2*c*d;  C += c*c;
                } else if (qa > hi_m) {
                    double c = qa - b1_hi[a], d = -shi[a];
                    A += d*d;  B += 2*c*d;  C += c*c;
                }
            }
            if (A > 1e-15) {
                double topt = std::clamp(-B / (2.0 * A), ta, tb);
                best = std::min(best, A*topt*topt + B*topt + C);
            }
        }
        return best;
    }

    /// Scanline-accelerated rasterisation of Conv(B₁∪B₂)⊕Ball(r_eff).
    ///
    /// Algorithm:
    ///   1. For each (Y,Z) scanline, check Y/Z-only distance → skip if outside
    ///   2. Scan from left  → find x_enter (first hit)
    ///   3. Scan from right → find x_exit  (last hit)
    ///   4. Fill [x_enter, x_exit] without further distance tests (convexity!)
    ///
    /// Complexity: O(ny·nz · boundary_width)  instead of  O(nx·ny·nz)
    void fill_convex_hull_16_scanline(const float* prox_iv,
                                      const float* dist_iv,
                                      double link_radius)
    {
        const double r_eff    = link_radius + safety_pad_;
        const double r_eff_sq = r_eff * r_eff;

        double b1_lo[3], b1_hi[3], slo[3], shi[3];
        double lo[3], hi[3];
        for (int a = 0; a < 3; ++a) {
            b1_lo[a] = prox_iv[a];
            b1_hi[a] = prox_iv[a + 3];
            slo[a]   = dist_iv[a]     - prox_iv[a];
            shi[a]   = dist_iv[a + 3] - prox_iv[a + 3];
            lo[a] = std::min(static_cast<double>(prox_iv[a]),
                             static_cast<double>(dist_iv[a]))    - r_eff;
            hi[a] = std::max(static_cast<double>(prox_iv[a + 3]),
                             static_cast<double>(dist_iv[a + 3])) + r_eff;
        }

        const int cx0 = to_cell(lo[0], 0), cx1 = to_cell(hi[0], 0);
        const int cy0 = to_cell(lo[1], 1), cy1 = to_cell(hi[1], 1);
        const int cz0 = to_cell(lo[2], 2), cz1 = to_cell(hi[2], 2);

        for (int cz = cz0; cz <= cz1; ++cz) {
            const double wz = cell_center(cz, 2);
            for (int cy = cy0; cy <= cy1; ++cy) {
                const double wy = cell_center(cy, 1);

                // ── YZ pre-filter: skip scanlines outside hull cross-section
                if (min_dist_sq_to_hull_yz(wy, wz, b1_lo, b1_hi, slo, shi)
                    > r_eff_sq)
                    continue;

                // ── Scan from LEFT to find entry cell
                int x_enter = cx1 + 1;   // sentinel
                for (int cx = cx0; cx <= cx1; ++cx) {
                    const double q[3] = {cell_center(cx, 0), wy, wz};
                    if (min_dist_sq_to_hull(q, b1_lo, b1_hi, slo, shi) <= r_eff_sq) {
                        x_enter = cx;
                        break;
                    }
                }
                if (x_enter > cx1) continue;   // no hit on this scanline

                // ── Scan from RIGHT to find exit cell
                int x_exit = x_enter;
                for (int cx = cx1; cx > x_enter; --cx) {
                    const double q[3] = {cell_center(cx, 0), wy, wz};
                    if (min_dist_sq_to_hull(q, b1_lo, b1_hi, slo, shi) <= r_eff_sq) {
                        x_exit = cx;
                        break;
                    }
                }

                // ── Fill entire contiguous range (convexity guarantees interior)
                for (int cx = x_enter; cx <= x_exit; ++cx)
                    set_cell(cx, cy, cz);
            }
        }
    }

    // =====================================================================
    //  16-Point Convex Hull — Fast Scanline (coherence + batch fill)
    // =====================================================================
    //
    //  Two optimisations on top of the basic scanline approach:
    //
    //  (1) **Row coherence**: For a convex body, the x_enter/x_exit of
    //      row (cy) and row (cy+1) differ by at most a few cells.
    //      We track the previous row's boundaries and search outward
    //      from them.  Amortised cost = O(perimeter), not O(area).
    //
    //  (2) **Batch bit fill**: Instead of calling set_cell per voxel in
    //      [x_enter, x_exit], we compute brick-level word masks and OR
    //      them in one shot.  One uint64_t OR covers up to 8 X-cells.
    //

    /// Fill a contiguous X range [cx_start, cx_end] at fixed cy, cz.
    /// Exploits the BitBrick layout (word = z_local, bit = y*8 + x)
    /// to batch-OR entire rows within a brick instead of per-cell set.
    void set_cell_range_x(int cx_start, int cx_end, int cy, int cz) noexcept
    {
        if (cx_start > cx_end) return;

        const int by = brick_of(cy), bz = brick_of(cz);
        const int ly = local_of(cy), lz = local_of(cz);

        int cx = cx_start;
        while (cx <= cx_end) {
            int bx  = brick_of(cx);
            int lx0 = local_of(cx);
            // How far can we go within this brick?
            int lx1 = std::min(7, lx0 + (cx_end - cx));
            int count = lx1 - lx0 + 1;

            // Build a bitmask covering bits [ly*8 + lx0 .. ly*8 + lx1]
            int bit_start = ly * 8 + lx0;
            uint64_t mask = ((uint64_t(1) << count) - 1) << bit_start;

            BrickCoord bc{bx, by, bz};
            bricks_[bc].words[lz] |= mask;

            cx += count;
        }
    }

    void fill_convex_hull_16_fast(const float* prox_iv,
                                  const float* dist_iv,
                                  double link_radius)
    {
        const double r_eff    = link_radius + safety_pad_;
        const double r_eff_sq = r_eff * r_eff;

        double b1_lo[3], b1_hi[3], slo[3], shi[3];
        double lo[3], hi[3];
        for (int a = 0; a < 3; ++a) {
            b1_lo[a] = prox_iv[a];
            b1_hi[a] = prox_iv[a + 3];
            slo[a]   = dist_iv[a]     - prox_iv[a];
            shi[a]   = dist_iv[a + 3] - prox_iv[a + 3];
            lo[a] = std::min(static_cast<double>(prox_iv[a]),
                             static_cast<double>(dist_iv[a]))    - r_eff;
            hi[a] = std::max(static_cast<double>(prox_iv[a + 3]),
                             static_cast<double>(dist_iv[a + 3])) + r_eff;
        }

        const int cx0 = to_cell(lo[0], 0), cx1 = to_cell(hi[0], 0);
        const int cy0 = to_cell(lo[1], 1), cy1 = to_cell(hi[1], 1);
        const int cz0 = to_cell(lo[2], 2), cz1 = to_cell(hi[2], 2);

        // Lambda: is cell (cx, wy, wz) inside the inflated hull?
        auto test = [&](int cx, double wy, double wz) -> bool {
            const double q[3] = {cell_center(cx, 0), wy, wz};
            return min_dist_sq_to_hull(q, b1_lo, b1_hi, slo, shi) <= r_eff_sq;
        };

        for (int cz = cz0; cz <= cz1; ++cz) {
            const double wz = cell_center(cz, 2);

            // Coherence state for this Z-slice
            int p_enter = -1, p_exit = -1;

            for (int cy = cy0; cy <= cy1; ++cy) {
                const double wy = cell_center(cy, 1);

                // ── YZ pre-filter ──
                if (min_dist_sq_to_hull_yz(wy, wz, b1_lo, b1_hi, slo, shi)
                    > r_eff_sq) {
                    p_enter = p_exit = -1;   // reset coherence
                    continue;
                }

                int x_enter, x_exit;

                if (p_enter >= 0) {
                    // ── Coherent entry search ──
                    int s = std::clamp(p_enter, cx0, cx1);
                    if (test(s, wy, wz)) {
                        // Previous entry still inside → hull same or wider.
                        // Scan LEFT to find new boundary.
                        x_enter = s;
                        while (x_enter > cx0 && test(x_enter - 1, wy, wz))
                            --x_enter;
                    } else {
                        // Hull narrowed → scan RIGHT from old entry.
                        x_enter = s + 1;
                        while (x_enter <= cx1 && !test(x_enter, wy, wz))
                            ++x_enter;
                        if (x_enter > cx1) {
                            p_enter = p_exit = -1;
                            continue;
                        }
                    }

                    // ── Coherent exit search ──
                    int e = std::clamp(std::max(p_exit, x_enter), cx0, cx1);
                    if (test(e, wy, wz)) {
                        // Scan RIGHT to extend
                        x_exit = e;
                        while (x_exit < cx1 && test(x_exit + 1, wy, wz))
                            ++x_exit;
                    } else {
                        // Hull narrowed on right → scan LEFT from old exit.
                        // We know x_enter is inside, so worst case x_exit = x_enter.
                        x_exit = e - 1;
                        while (x_exit > x_enter && !test(x_exit, wy, wz))
                            --x_exit;
                        // x_exit might equal x_enter here (single-cell row)
                    }
                } else {
                    // ── No coherence: linear scan from edges ──
                    x_enter = cx0;
                    while (x_enter <= cx1 && !test(x_enter, wy, wz))
                        ++x_enter;
                    if (x_enter > cx1) continue;

                    x_exit = cx1;
                    while (x_exit > x_enter && !test(x_exit, wy, wz))
                        --x_exit;
                }

                // ── Batch fill [x_enter, x_exit] ──
                set_cell_range_x(x_enter, x_exit, cy, cz);

                p_enter = x_enter;
                p_exit  = x_exit;
            }
        }
    }

    // =====================================================================
    //  16-Point Convex Hull — Turbo Scanline (t-range + analytical X bounds)
    // =====================================================================
    //
    //  Eliminate ALL per-voxel distance evaluations.
    //
    //  For each (Y,Z) scanline, solve for the t-range [t_lo, t_hi] where
    //  dist²_yz((y,z), B(t)) ≤ r_eff².  Then the conservative X fill range
    //  is directly computed from the linear X bounds at t_lo and t_hi.
    //
    //  Correctness proof (conservation):
    //  If P=(px,py,pz) is inside the inflated hull, then ∃ t* s.t.
    //  dist(P, B(t*)) ≤ r_eff.  This implies:
    //    (a) dist_yz ≤ r_eff  ⟹  t* ∈ [t_lo, t_hi]
    //    (b) lo_x(t*) - r_eff ≤ px ≤ hi_x(t*) + r_eff
    //  Since lo_x, hi_x are linear in t and t* ∈ [t_lo,t_hi]:
    //    px ∈ [min(lo_x(t_lo), lo_x(t_hi)) − r, max(hi_x(t_lo), hi_x(t_hi)) + r]
    //  ∴ px is in our computed X range.  QED
    //
    //  Cost: O(ny·nz) with O(1) per scanline.  No per-voxel distance evals!
    //

    /// Find the t-range [t_entry, t_exit] where dist²_yz((qy,qz), B(t)) ≤ r².
    /// Uses convex piecewise-quadratic structure.
    /// Returns (2, -1) if empty.
    static std::pair<double,double> find_t_range_yz(
        double qy, double qz, double r_eff_sq,
        const double b1_lo[3], const double b1_hi[3],
        const double slo[3],   const double shi[3]) noexcept
    {
        double bp[6];
        int nb = 0;
        bp[nb++] = 0.0;
        bp[nb++] = 1.0;
        for (int a = 1; a <= 2; ++a) {
            double qa = (a == 1) ? qy : qz;
            if (std::abs(slo[a]) > 1e-15) {
                double t = (qa - b1_lo[a]) / slo[a];
                if (t > 1e-12 && t < 1.0 - 1e-12) bp[nb++] = t;
            }
            if (std::abs(shi[a]) > 1e-15) {
                double t = (qa - b1_hi[a]) / shi[a];
                if (t > 1e-12 && t < 1.0 - 1e-12) bp[nb++] = t;
            }
        }
        for (int i = 1; i < nb; ++i)
            for (int j = i; j > 0 && bp[j] < bp[j - 1]; --j)
                std::swap(bp[j], bp[j - 1]);

        double t_entry = 2.0, t_exit = -1.0;

        for (int i = 0; i + 1 < nb; ++i) {
            double ta = bp[i], tb = bp[i + 1];
            if (tb - ta < 1e-12) continue;

            double tmid = 0.5 * (ta + tb);
            double A = 0.0, B = 0.0, C = 0.0;
            for (int a = 1; a <= 2; ++a) {
                double qa = (a == 1) ? qy : qz;
                double lo_m = b1_lo[a] + tmid * slo[a];
                double hi_m = b1_hi[a] + tmid * shi[a];
                if (qa < lo_m) {
                    double c = b1_lo[a] - qa, d = slo[a];
                    A += d * d;  B += 2 * c * d;  C += c * c;
                } else if (qa > hi_m) {
                    double c = qa - b1_hi[a], d = -shi[a];
                    A += d * d;  B += 2 * c * d;  C += c * c;
                }
            }

            // Solve: A·t² + B·t + (C − r²) ≤ 0  on [ta, tb]
            double Cadj = C - r_eff_sq;

            if (A < 1e-15) {
                // Linear or constant
                if (std::abs(B) < 1e-15) {
                    // Constant: C ≤ r² means whole segment qualifies
                    if (C <= r_eff_sq + 1e-12) {
                        t_entry = std::min(t_entry, ta);
                        t_exit  = std::max(t_exit,  tb);
                    }
                } else {
                    // Linear: B·t + Cadj ≤ 0
                    double tc = -Cadj / B;
                    double slo_t, shi_t;
                    if (B > 0) { slo_t = ta; shi_t = std::min(tb, tc); }
                    else       { slo_t = std::max(ta, tc); shi_t = tb; }
                    if (slo_t <= shi_t + 1e-12) {
                        t_entry = std::min(t_entry, slo_t);
                        t_exit  = std::max(t_exit,  shi_t);
                    }
                }
            } else {
                // Quadratic: A·t² + B·t + Cadj ≤ 0
                double disc = B * B - 4.0 * A * Cadj;
                if (disc < 0) continue;
                double sqd = std::sqrt(disc);
                double r1 = (-B - sqd) / (2.0 * A);
                double r2 = (-B + sqd) / (2.0 * A);
                double slo_t = std::max(ta, std::min(r1, r2));
                double shi_t = std::min(tb, std::max(r1, r2));
                if (slo_t <= shi_t + 1e-12) {
                    t_entry = std::min(t_entry, slo_t);
                    t_exit  = std::max(t_exit,  shi_t);
                }
            }
        }

        if (t_entry > t_exit) return {2.0, -1.0};
        return {std::max(0.0, t_entry), std::min(1.0, t_exit)};
    }

    /// Turbo rasterisation: find t-range per scanline, compute X bounds directly.
    void fill_convex_hull_16_turbo(const float* prox_iv,
                                   const float* dist_iv,
                                   double link_radius)
    {
        const double r_eff    = link_radius + safety_pad_;
        const double r_eff_sq = r_eff * r_eff;

        double b1_lo[3], b1_hi[3], slo[3], shi[3];
        double lo[3], hi[3];
        for (int a = 0; a < 3; ++a) {
            b1_lo[a] = prox_iv[a];
            b1_hi[a] = prox_iv[a + 3];
            slo[a]   = dist_iv[a]     - prox_iv[a];
            shi[a]   = dist_iv[a + 3] - prox_iv[a + 3];
            lo[a] = std::min(static_cast<double>(prox_iv[a]),
                             static_cast<double>(dist_iv[a]))    - r_eff;
            hi[a] = std::max(static_cast<double>(prox_iv[a + 3]),
                             static_cast<double>(dist_iv[a + 3])) + r_eff;
        }

        const int cx0 = to_cell(lo[0], 0), cx1 = to_cell(hi[0], 0);
        const int cy0 = to_cell(lo[1], 1), cy1 = to_cell(hi[1], 1);
        const int cz0 = to_cell(lo[2], 2), cz1 = to_cell(hi[2], 2);

        for (int cz = cz0; cz <= cz1; ++cz) {
            const double wz = cell_center(cz, 2);
            for (int cy = cy0; cy <= cy1; ++cy) {
                const double wy = cell_center(cy, 1);

                // Find t-range where (wy,wz) is within r_eff of hull YZ section
                auto [tlo, thi] = find_t_range_yz(
                    wy, wz, r_eff_sq, b1_lo, b1_hi, slo, shi);
                if (tlo > thi) continue;

                // Conservative X bounds over t ∈ [tlo, thi]
                // lo_x(t) = b1_lo[0] + t·slo[0]  (linear in t)
                // hi_x(t) = b1_hi[0] + t·shi[0]  (linear in t)
                double x_lo = std::min(b1_lo[0] + tlo * slo[0],
                                       b1_lo[0] + thi * slo[0]) - r_eff;
                double x_hi = std::max(b1_hi[0] + tlo * shi[0],
                                       b1_hi[0] + thi * shi[0]) + r_eff;

                int cx_start = std::max(cx0, to_cell(x_lo, 0));
                int cx_end   = std::min(cx1, to_cell(x_hi, 0));

                if (cx_start <= cx_end)
                    set_cell_range_x(cx_start, cx_end, cy, cz);
            }
        }
    }

    // =====================================================================
    //  Volume queries
    // =====================================================================

    /// Total number of set voxels across all bricks
    int count_occupied() const noexcept {
        int n = 0;
        for (auto& [c, b] : bricks_) n += b.popcount();
        return n;
    }

    /// Occupied volume in m³
    double occupied_volume() const noexcept {
        return count_occupied() * delta_ * delta_ * delta_;
    }

    /// Number of allocated bricks
    int num_bricks() const noexcept {
        return static_cast<int>(bricks_.size());
    }

    // =====================================================================
    //  Merge / Collision
    // =====================================================================

    /// Merge another grid into this one (bitwise OR)
    void merge(const VoxelGrid& other) {
        for (auto& [c, b] : other.bricks_)
            bricks_[c] |= b;
    }

    /// Test if any voxel is shared (bitwise AND ≠ 0)
    bool collides(const VoxelGrid& other) const {
        const auto& sm = (bricks_.size() <= other.bricks_.size())
                         ? bricks_ : other.bricks_;
        const auto& lg = (bricks_.size() <= other.bricks_.size())
                         ? other.bricks_ : bricks_;
        for (auto& [c, b] : sm) {
            auto it = lg.find(c);
            if (it != lg.end() && b.intersects(it->second))
                return true;
        }
        return false;
    }

    /// Count the number of overlapping voxels
    int count_colliding(const VoxelGrid& other) const {
        int total = 0;
        const auto& sm = (bricks_.size() <= other.bricks_.size())
                         ? bricks_ : other.bricks_;
        const auto& lg = (bricks_.size() <= other.bricks_.size())
                         ? other.bricks_ : bricks_;
        for (auto& [c, b] : sm) {
            auto it = lg.find(c);
            if (it != lg.end())
                total += (b & it->second).popcount();
        }
        return total;
    }

    void clear() { bricks_.clear(); }

    // ── Accessors ───────────────────────────────────────────────────────
    double delta()       const noexcept { return delta_; }
    double safety_pad()  const noexcept { return safety_pad_; }
    const BrickMap& bricks() const noexcept { return bricks_; }

private:
    double delta_      = 0.02;
    double inv_delta_  = 50.0;
    double origin_[3]  = {};
    double safety_pad_ = 0.0;
    BrickMap bricks_;

    // ── Cell → brick / local coordinate ─────────────────────────────────
    static int brick_of(int cell) noexcept {
        // Correct negative-friendly integer division by 8
        return (cell >= 0) ? cell / 8 : (cell - 7) / 8;
    }
    static int local_of(int cell) noexcept {
        int m = cell % 8;
        return (m >= 0) ? m : m + 8;
    }

    /// Set a single voxel at cell coordinate (cx, cy, cz).
    void set_cell(int cx, int cy, int cz) {
        BrickCoord bc{brick_of(cx), brick_of(cy), brick_of(cz)};
        bricks_[bc].set(local_of(cx), local_of(cy), local_of(cz));
    }
};

} // namespace voxel
} // namespace sbf
