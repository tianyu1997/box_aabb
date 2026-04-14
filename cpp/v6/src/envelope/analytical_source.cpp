// SafeBoxForest v6 — Analytical Multi-Phase Endpoint Source (Phase B3)
//
// Multi-phase exact analytical critical-point enumeration.
// Reference: v4 analytical_solve.cpp
//
// For each endpoint (proximal/distal of each active link), the position
// along axis d is: f_d(q) = α·cos(q_j) + β·sin(q_j) + γ (fixing other joints).
// Critical: ∂f_d/∂q_j = 0  ⟹  q_j* = atan2(β, α)  (and q_j* + π).
//
// Phase 0: enumerate vertex combos (kπ/2 grid) — uses shared dh_enumerate
// Phase 1: 1D edge solve via 3-point QR → atan2
// Phase 2: 2D face solve via alternating 1D
// Phase 3: interior coordinate descent
//
// Early exit: if max interval width < kPhase123Threshold, skip Phases 1-3.
//
// AA Gap Pruning: after Phase 0, IFK bounds are computed. If a link's IFK
// bounds are already within the Phase 0 AABB on all axes, that link is
// skipped in subsequent phases.
#include <sbf/envelope/analytical_source.h>
#include <sbf/envelope/dh_enumerate.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

namespace sbf {

namespace {

// Intervals narrower than this skip Phases 1-3 entirely (all joints narrow).
static constexpr double kPhase123Threshold = 0.15;  // ~8.6°

// Scalar FK to extract positions for all prefix frames.
// Must match interval FK DH convention (see interval_math.cpp build_dh_joint).
void scalar_fk_positions(const Robot& robot, const double* q,
                         double positions[][3])
{
    const auto& dh = robot.dh_params();
    int n = robot.n_joints();

    double T[16];
    for (int i = 0; i < 16; ++i) T[i] = 0.0;
    T[0] = T[5] = T[10] = T[15] = 1.0;

    positions[0][0] = 0.0;
    positions[0][1] = 0.0;
    positions[0][2] = 0.0;

    for (int j = 0; j < n; ++j) {
        double alpha = dh[j].alpha;
        double a     = dh[j].a;
        double d_val = (dh[j].joint_type == 1) ? (q[j] + dh[j].d) : dh[j].d;
        double angle = (dh[j].joint_type == 0) ? (q[j] + dh[j].theta) : dh[j].theta;

        double ct = std::cos(angle), st = std::sin(angle);
        double ca = std::cos(alpha), sa = std::sin(alpha);

        double A[16] = {
            ct,      -st,      0.0,  a,
            st*ca,    ct*ca,  -sa,  -d_val*sa,
            st*sa,    ct*sa,   ca,   d_val*ca,
            0.0,      0.0,    0.0,   1.0
        };

        double R[16];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                R[r*4+c] = T[r*4+0]*A[c] + T[r*4+1]*A[4+c]
                          + T[r*4+2]*A[8+c] + T[r*4+3]*A[12+c];
        R[12] = 0.0; R[13] = 0.0; R[14] = 0.0; R[15] = 1.0;
        for (int i = 0; i < 16; ++i) T[i] = R[i];

        positions[j+1][0] = T[3];
        positions[j+1][1] = T[7];
        positions[j+1][2] = T[11];
    }

    // Tool frame (if present)
    if (robot.has_tool()) {
        const auto& tool = *robot.tool_frame();
        double ct = std::cos(tool.theta), st = std::sin(tool.theta);
        double ca = std::cos(tool.alpha), sa = std::sin(tool.alpha);
        double A[16] = {
            ct,      -st,      0.0,  tool.a,
            st*ca,    ct*ca,  -sa,  -tool.d*sa,
            st*sa,    ct*sa,   ca,   tool.d*ca,
            0.0,      0.0,    0.0,   1.0
        };
        double R[16];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                R[r*4+c] = T[r*4+0]*A[c] + T[r*4+1]*A[4+c]
                          + T[r*4+2]*A[8+c] + T[r*4+3]*A[12+c];
        R[12] = 0.0; R[13] = 0.0; R[14] = 0.0; R[15] = 1.0;
        for (int i = 0; i < 16; ++i) T[i] = R[i];

        positions[n+1][0] = T[3];
        positions[n+1][1] = T[7];
        positions[n+1][2] = T[11];
    }
}

// update_endpoints is still needed for Phases 1-3 which use full 4×4 FK
void update_endpoints(const double positions[][3],
                      const int* alm, int n_act, float* out)
{
    for (int ci = 0; ci < n_act; ++ci) {
        int V = alm[ci];
        // Proximal
        float* p = out + (ci * 2) * 6;
        for (int d = 0; d < 3; ++d) {
            float v = static_cast<float>(positions[V][d]);
            if (v < p[d])     p[d]     = v;
            if (v > p[d + 3]) p[d + 3] = v;
        }
        // Distal
        float* dd = out + (ci * 2 + 1) * 6;
        for (int d = 0; d < 3; ++d) {
            float v = static_cast<float>(positions[V + 1][d]);
            if (v < dd[d])     dd[d]     = v;
            if (v > dd[d + 3]) dd[d + 3] = v;
        }
    }
}

// ── Phase 0: precomputed iterative kπ/2 vertex enumeration ─────────
// Uses shared dh_enumerate infrastructure (precomputed DH + narrow collapse).
void phase0_vertices(const Robot& robot,
                     const std::vector<Interval>& intervals,
                     float* out, int n_act, const int* alm)
{
    int n = robot.n_joints();
    const auto& dh = robot.dh_params();

    std::vector<std::vector<double>> candidates(n);
    for (int j = 0; j < n; ++j)
        collect_kpi2(intervals[j].lo, intervals[j].hi, candidates[j]);

    std::vector<std::vector<PreDH>> pre_dh(n);
    std::vector<int> n_cands(n);
    for (int j = 0; j < n; ++j) {
        n_cands[j] = static_cast<int>(candidates[j].size());
        pre_dh[j].resize(n_cands[j]);
        for (int k = 0; k < n_cands[j]; ++k)
            build_dh_matrix(dh[j], candidates[j][k], pre_dh[j][k].A);
    }

    enumerate_critical_iterative(robot, pre_dh, n_cands, alm, n_act, out);
}

// ── Phase 1: 1D edge solve ──────────────────────────────────────────────
// For each joint j, for each background combo of other joints at {lo,hi}:
//   Sample 3 points of q_j to fit f_d = α·cos + β·sin + γ via QR.
//   Critical: q* = atan2(β, α), q* + π.
// Narrow joints (width < kNarrowThreshold) are skipped — already covered by Phase 0.
void phase1_edges(const Robot& robot,
                  const std::vector<Interval>& intervals,
                  float* out, int n_act, const int* alm,
                  const bool* skip_link = nullptr)
{
    int n = robot.n_joints();
    double positions[MAX_TF][3];
    std::vector<double> q(n);

    for (int j = 0; j < n; ++j) {
        double lo_j = intervals[j].lo;
        double hi_j = intervals[j].hi;
        if (hi_j - lo_j < kNarrowThreshold) continue;

        double mid_j = 0.5 * (lo_j + hi_j);
        double qvals[3] = { lo_j, mid_j, hi_j };

        // Build sampling matrix [cos(q), sin(q), 1]
        Eigen::Matrix3d A;
        for (int si = 0; si < 3; ++si) {
            A(si, 0) = std::cos(qvals[si]);
            A(si, 1) = std::sin(qvals[si]);
            A(si, 2) = 1.0;
        }
        auto qr = A.colPivHouseholderQr();

        // Number of background joints (all except j): 2^(nj-1) combos
        int nj = n;
        int n_bg = nj - 1;
        int max_bg = 1 << n_bg;
        if (max_bg > 128) max_bg = 128;  // cap for high-DOF

        for (int bg = 0; bg < max_bg; ++bg) {
            // Set background joints to lo/hi corners
            for (int jj = 0; jj < n; ++jj)
                q[jj] = intervals[jj].center();
            int bit = 0;
            for (int jj = 0; jj < nj; ++jj) {
                if (jj == j) continue;
                q[jj] = (bg & (1 << bit)) ? intervals[jj].hi : intervals[jj].lo;
                ++bit;
            }

            // Evaluate FK at 3 sample points of joint j
            // For each active link, for each endpoint (proximal V, distal V+1),
            // for each axis d: fit α·cos + β·sin + γ
            // Get positions at all 3 qvals
            double p3[3][MAX_TF][3];
            for (int si = 0; si < 3; ++si) {
                q[j] = qvals[si];
                scalar_fk_positions(robot, q.data(), p3[si]);
            }

            // For each active link endpoint, solve per axis
            for (int ci = 0; ci < n_act; ++ci) {
                if (skip_link && skip_link[ci]) continue;
                int V = alm[ci];
                // Process both proximal (V) and distal (V+1)
                for (int ep = 0; ep < 2; ++ep) {
                    int frame = (ep == 0) ? V : V + 1;
                    for (int d = 0; d < 3; ++d) {
                        Eigen::Vector3d b_vec;
                        for (int si = 0; si < 3; ++si)
                            b_vec[si] = p3[si][frame][d];
                        Eigen::Vector3d coeff = qr.solve(b_vec);
                        double alpha_c = coeff[0];
                        double beta_c  = coeff[1];
                        if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                            continue;

                        double q_star = std::atan2(beta_c, alpha_c);
                        // Two candidates: q_star (max) and q_star+π (min)
                        double cands[2] = { q_star, q_star + PI };

                        for (double qc : cands) {
                            // Normalize to [-π, π] range and check bounds
                            while (qc > PI + 1e-6) qc -= TWO_PI;
                            while (qc < -PI - 1e-6) qc += TWO_PI;
                            if (qc >= lo_j - 1e-12 && qc <= hi_j + 1e-12) {
                                qc = std::max(lo_j, std::min(hi_j, qc));
                                q[j] = qc;
                                scalar_fk_positions(robot, q.data(), positions);
                                update_endpoints(positions, alm, n_act, out);
                            }
                        }
                    }
                }
            }
        }
    }
}

// ── Phase 2: 2D face solve ──────────────────────────────────────────────
// For each pair of joints (i,j), fix others at {lo,hi} combos.
// Use alternating 1D solves: fix j → solve i, fix i → solve j, repeat.
// Pairs where either joint is narrow are skipped — Phase 1 already handles the
// single-wide-joint case sufficiently.
void phase2_faces(const Robot& robot,
                  const std::vector<Interval>& intervals,
                  float* out, int n_act, const int* alm,
                  const bool* skip_link = nullptr)
{
    int n = robot.n_joints();
    if (n < 2) return;

    double positions[MAX_TF][3];
    std::vector<double> q(n);

    for (int ji = 0; ji < n; ++ji) {
        for (int jj = ji + 1; jj < n; ++jj) {
            double lo_i = intervals[ji].lo, hi_i = intervals[ji].hi;
            double lo_j = intervals[jj].lo, hi_j = intervals[jj].hi;
            if (hi_i - lo_i < kNarrowThreshold || hi_j - lo_j < kNarrowThreshold) continue;

            int n_bg = n - 2;
            int max_bg = 1 << std::max(n_bg, 0);
            if (max_bg > 16) max_bg = 16;

            for (int bg = 0; bg < max_bg; ++bg) {
                for (int k = 0; k < n; ++k) q[k] = intervals[k].center();
                int bit = 0;
                for (int k = 0; k < n; ++k) {
                    if (k == ji || k == jj) continue;
                    q[k] = (bg & (1 << bit)) ? intervals[k].hi : intervals[k].lo;
                    ++bit;
                }

                // Initialize q_i, q_j at midpoints
                q[ji] = intervals[ji].center();
                q[jj] = intervals[jj].center();

                // Alternating 1D solve: 3 iterations
                for (int iter = 0; iter < 3; ++iter) {
                    // Fix jj, solve ji
                    {
                        double qvals[3] = { lo_i, 0.5*(lo_i+hi_i), hi_i };
                        Eigen::Matrix3d A;
                        for (int si = 0; si < 3; ++si) {
                            A(si, 0) = std::cos(qvals[si]);
                            A(si, 1) = std::sin(qvals[si]);
                            A(si, 2) = 1.0;
                        }
                        auto qr = A.colPivHouseholderQr();

                        double p3[3][MAX_TF][3];
                        for (int si = 0; si < 3; ++si) {
                            q[ji] = qvals[si];
                            scalar_fk_positions(robot, q.data(), p3[si]);
                        }

                        // Find the best qi across all endpoints/axes
                        for (int ci = 0; ci < n_act; ++ci) {
                            if (skip_link && skip_link[ci]) continue;
                            int V = alm[ci];
                            for (int ep = 0; ep < 2; ++ep) {
                                int frame = (ep == 0) ? V : V + 1;
                                for (int d = 0; d < 3; ++d) {
                                    Eigen::Vector3d b;
                                    for (int si = 0; si < 3; ++si)
                                        b[si] = p3[si][frame][d];
                                    Eigen::Vector3d coeff = qr.solve(b);
                                    if (std::abs(coeff[0]) < 1e-15 && std::abs(coeff[1]) < 1e-15)
                                        continue;
                                    double qs = std::atan2(coeff[1], coeff[0]);
                                    for (double shift : {0.0, PI, -PI}) {
                                        double qc = qs + shift;
                                        while (qc > PI + 1e-6) qc -= TWO_PI;
                                        while (qc < -PI - 1e-6) qc += TWO_PI;
                                        if (qc >= lo_i - 1e-12 && qc <= hi_i + 1e-12) {
                                            qc = std::max(lo_i, std::min(hi_i, qc));
                                            q[ji] = qc;
                                            scalar_fk_positions(robot, q.data(), positions);
                                            update_endpoints(positions, alm, n_act, out);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // Fix ji, solve jj
                    {
                        double qvals[3] = { lo_j, 0.5*(lo_j+hi_j), hi_j };
                        Eigen::Matrix3d A;
                        for (int si = 0; si < 3; ++si) {
                            A(si, 0) = std::cos(qvals[si]);
                            A(si, 1) = std::sin(qvals[si]);
                            A(si, 2) = 1.0;
                        }
                        auto qr = A.colPivHouseholderQr();

                        double p3[3][MAX_TF][3];
                        for (int si = 0; si < 3; ++si) {
                            q[jj] = qvals[si];
                            scalar_fk_positions(robot, q.data(), p3[si]);
                        }

                        for (int ci = 0; ci < n_act; ++ci) {
                            if (skip_link && skip_link[ci]) continue;
                            int V = alm[ci];
                            for (int ep = 0; ep < 2; ++ep) {
                                int frame = (ep == 0) ? V : V + 1;
                                for (int d = 0; d < 3; ++d) {
                                    Eigen::Vector3d b;
                                    for (int si = 0; si < 3; ++si)
                                        b[si] = p3[si][frame][d];
                                    Eigen::Vector3d coeff = qr.solve(b);
                                    if (std::abs(coeff[0]) < 1e-15 && std::abs(coeff[1]) < 1e-15)
                                        continue;
                                    double qs = std::atan2(coeff[1], coeff[0]);
                                    for (double shift : {0.0, PI, -PI}) {
                                        double qc = qs + shift;
                                        while (qc > PI + 1e-6) qc -= TWO_PI;
                                        while (qc < -PI - 1e-6) qc += TWO_PI;
                                        if (qc >= lo_j - 1e-12 && qc <= hi_j + 1e-12) {
                                            qc = std::max(lo_j, std::min(hi_j, qc));
                                            q[jj] = qc;
                                            scalar_fk_positions(robot, q.data(), positions);
                                            update_endpoints(positions, alm, n_act, out);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

// ── Phase 3: Interior coordinate descent ────────────────────────────────
// Multi-start: start from corner combos + midpoint, then sweep all joints
// doing 1D analytical solve per joint.
void phase3_interior(const Robot& robot,
                     const std::vector<Interval>& intervals,
                     float* out, int n_act, const int* alm,
                     const bool* skip_link = nullptr)
{
    int n = robot.n_joints();
    if (n < 2) return;

    double positions[MAX_TF][3];
    std::vector<double> q(n);

    // Generate starting points: midpoint + 2^n corners (capped)
    std::vector<std::vector<double>> starts;
    {
        std::vector<double> mid(n);
        for (int j = 0; j < n; ++j) mid[j] = intervals[j].center();
        starts.push_back(mid);

        int n_corners = 1 << n;
        if (n_corners > 16) n_corners = 16;
        for (int c = 0; c < n_corners; ++c) {
            std::vector<double> corner(n);
            for (int j = 0; j < n; ++j)
                corner[j] = (c & (1 << j)) ? intervals[j].hi : intervals[j].lo;
            starts.push_back(corner);
        }
    }

    int max_sweeps = 3;
    for (const auto& start : starts) {
        for (int j = 0; j < n; ++j) q[j] = start[j];

        for (int sweep = 0; sweep < max_sweeps; ++sweep) {
            for (int j = 0; j < n; ++j) {
                double lo_j = intervals[j].lo;
                double hi_j = intervals[j].hi;
                if (hi_j - lo_j < kNarrowThreshold) continue;

                double qvals[3] = { lo_j, 0.5*(lo_j+hi_j), hi_j };
                Eigen::Matrix3d A;
                for (int si = 0; si < 3; ++si) {
                    A(si, 0) = std::cos(qvals[si]);
                    A(si, 1) = std::sin(qvals[si]);
                    A(si, 2) = 1.0;
                }
                auto qr = A.colPivHouseholderQr();

                double p3[3][MAX_TF][3];
                for (int si = 0; si < 3; ++si) {
                    q[j] = qvals[si];
                    scalar_fk_positions(robot, q.data(), p3[si]);
                }

                // Find critical angles across all endpoints/axes, update best
                for (int ci = 0; ci < n_act; ++ci) {
                    if (skip_link && skip_link[ci]) continue;
                    int V = alm[ci];
                    for (int ep = 0; ep < 2; ++ep) {
                        int frame = (ep == 0) ? V : V + 1;
                        for (int d = 0; d < 3; ++d) {
                            Eigen::Vector3d b;
                            for (int si = 0; si < 3; ++si)
                                b[si] = p3[si][frame][d];
                            Eigen::Vector3d coeff = qr.solve(b);
                            if (std::abs(coeff[0]) < 1e-15 && std::abs(coeff[1]) < 1e-15)
                                continue;
                            double qs = std::atan2(coeff[1], coeff[0]);
                            for (double shift : {0.0, PI, -PI}) {
                                double qc = qs + shift;
                                while (qc > PI + 1e-6) qc -= TWO_PI;
                                while (qc < -PI - 1e-6) qc += TWO_PI;
                                if (qc >= lo_j - 1e-12 && qc <= hi_j + 1e-12) {
                                    qc = std::max(lo_j, std::min(hi_j, qc));
                                    q[j] = qc;
                                    scalar_fk_positions(robot, q.data(), positions);
                                    update_endpoints(positions, alm, n_act, out);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

}  // anonymous namespace

EndpointIAABBResult compute_endpoint_iaabb_analytical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int max_phase)
{
    EndpointIAABBResult result;
    result.source = EndpointSource::Analytical;
    result.is_safe = false;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(result.endpoint_iaabb_len());

    const int n_act = result.n_active_links;
    const int* alm = robot.active_link_map();

    init_endpoints_inf(result.endpoint_iaabbs.data(), n_act);

    // Also compute and store FK state for downstream use
    result.fk_state = compute_fk_full(robot, intervals);

    // Phase 0: kπ/2 vertex enumeration (uses precomputed DH + narrow collapse)
    phase0_vertices(robot, intervals, result.endpoint_iaabbs.data(), n_act, alm);

    // ── AA Gap Pruning ──────────────────────────────────────────────────
    // Compare IFK bounds vs Phase 0 AABB per link.  If IFK bounds are
    // already within the current AABB (within tolerance) on all axes for
    // both endpoints, the link cannot meaningfully improve in later phases.
    // Tolerance accounts for IFK wrapping vs. true critical-point AABB gap.
    // Note: avoid std::vector<bool> (no .data()); use unique_ptr<bool[]>.
    auto skip_link = std::make_unique<bool[]>(n_act);
    for (int ci = 0; ci < n_act; ++ci) skip_link[ci] = false;
    int n_pruned = 0;
    if (max_phase >= 1) {
        std::vector<float> ifk_ep(n_act * 2 * 6);
        extract_endpoint_iaabbs(result.fk_state, alm, n_act, ifk_ep.data());

        constexpr float prune_tol = 1e-4f;  // 0.1mm tolerance

        for (int ci = 0; ci < n_act; ++ci) {
            bool can_skip = true;
            for (int ep = 0; ep < 2 && can_skip; ++ep) {
                const float* ifk_b = ifk_ep.data() + (ci * 2 + ep) * 6;
                const float* cur_b = result.endpoint_iaabbs.data() + (ci * 2 + ep) * 6;
                for (int d = 0; d < 3; ++d) {
                    // If IFK extends beyond Phase 0 by more than tolerance
                    // on any axis, the link may still improve.
                    if (ifk_b[d] < cur_b[d] - prune_tol ||
                        ifk_b[d + 3] > cur_b[d + 3] + prune_tol) {
                        can_skip = false;
                        break;
                    }
                }
            }
            if (can_skip) {
                skip_link[ci] = true;
                ++n_pruned;
            }
        }
    }
    result.n_pruned_links = n_pruned;

    // ── Early exit: skip Phases 1-3 for narrow intervals ────────────────
    // When max interval width <= kPhase123Threshold, critical points from
    // atan2 solves barely differ from the boundary values already captured
    // by Phase 0.  This is the dominant fast path for LECT hot expansions.
    double max_width = 0.0;
    for (int j = 0; j < robot.n_joints(); ++j) {
        double w = intervals[j].width();
        if (w > max_width) max_width = w;
    }
    if (max_width <= kPhase123Threshold || max_phase < 1) {
        return result;
    }

    const bool* skip_ptr = (n_pruned > 0) ? skip_link.get() : nullptr;

    if (max_phase >= 1) {
        phase1_edges(robot, intervals, result.endpoint_iaabbs.data(), n_act, alm, skip_ptr);
    }

    if (max_phase >= 2) {
        phase2_faces(robot, intervals, result.endpoint_iaabbs.data(), n_act, alm, skip_ptr);
    }

    if (max_phase >= 3) {
        phase3_interior(robot, intervals, result.endpoint_iaabbs.data(), n_act, alm, skip_ptr);
    }

    return result;
}

}  // namespace sbf
