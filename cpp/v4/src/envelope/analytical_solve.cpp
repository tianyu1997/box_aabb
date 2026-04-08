// ══════════════════════════════════════════════════════════════════════════�?
//  SafeBoxForest v4 �?Analytical Critical Solve implementation
//  迁移�?v3 envelope_derive_critical.cpp，完�?6 阶段解析管线
//  v4 新增：AA gap 剪枝
// ══════════════════════════════════════════════════════════════════════════�?
#include "sbf/envelope/analytical_solve.h"
#include "sbf/envelope/analytical_utils.h"
#include "sbf/envelope/analytical_coeff.h"
#include "sbf/robot/affine_fk.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cmath>
#include <set>
#include <thread>
#include <utility>
#include <vector>

namespace sbf {
namespace envelope {

struct DedupMetrics {
    int64_t raw_candidates = 0;
    int64_t unique_candidates = 0;
    int64_t applied_sets = 0;
    int64_t skipped_sets = 0;
};

// (P2ProtoMetrics, merge_p2_proto_metrics, compute_p2_symbolic_coeff_from_grid
//  removed in Phase E cleanup — proto/shadow/hybrid infrastructure obsoleted
//  by direct coefficient extraction in Phases B–D)

static void merge_dedup_metrics(DedupMetrics& dst, const DedupMetrics& src) {
    dst.raw_candidates += src.raw_candidates;
    dst.unique_candidates += src.unique_candidates;
    dst.applied_sets += src.applied_sets;
    dst.skipped_sets += src.skipped_sets;
}

static int resolve_parallel_threads(const AnalyticalCriticalConfig& config, int n_act) {
    if (!config.enable_parallel_by_link) return 1;
    int threads = config.parallel_num_threads;
    if (threads <= 0) {
        const unsigned int hw = std::thread::hardware_concurrency();
        threads = (hw == 0) ? 1 : static_cast<int>(hw);
    }
    if (threads < 1) threads = 1;
    if (threads > n_act) threads = n_act;
    return threads;
}

struct DedupAdaptiveRuntime {
    const AnalyticalCriticalConfig* cfg = nullptr;
    DedupMetrics* metrics = nullptr;

    bool decide(std::size_t raw_count) const {
        if (!cfg || !cfg->enable_candidate_dedup || raw_count < 2)
            return false;

        if (!cfg->enable_adaptive_candidate_dedup)
            return true;

        if (raw_count < static_cast<std::size_t>(cfg->candidate_dedup_min_candidates))
            return false;

        const int64_t applied_sets = metrics ? metrics->applied_sets : 0;
        if (applied_sets < cfg->candidate_dedup_warmup_sets)
            return true;

        const double hit_rate = (metrics && metrics->raw_candidates > 0)
            ? (static_cast<double>(metrics->raw_candidates - metrics->unique_candidates) /
               static_cast<double>(metrics->raw_candidates))
            : 0.0;
        return hit_rate >= cfg->candidate_dedup_min_hit_rate;
    }

    void record_applied(std::size_t raw_count, std::size_t unique_count) {
        if (!metrics) return;
        metrics->raw_candidates += static_cast<int64_t>(raw_count);
        metrics->unique_candidates += static_cast<int64_t>(unique_count);
        ++metrics->applied_sets;
    }

    void record_skipped(std::size_t raw_count) {
        if (!metrics) return;
        metrics->raw_candidates += static_cast<int64_t>(raw_count);
        metrics->unique_candidates += static_cast<int64_t>(raw_count);
        ++metrics->skipped_sets;
    }
};

// ══════════════════════════════════════════════════════════════════════════�?
//  AA Pruning �?check if link's AA bounds already fit within current AABB
// ══════════════════════════════════════════════════════════════════════════�?

struct AAImproveSummary {
    bool can_improve = false;
    int pruned_segments = 0;
};

struct AAAxisImproveSummary {
    bool can_improve = false;
    bool axis_can_improve[3] = {false, false, false};
};

static AAImproveSummary aa_link_improve_summary(
    const AAMatrix4* prefix, int V,
    int n_sub, float inv_n,
    const std::vector<LinkExtremes>& seg_ext)
{
    AAImproveSummary out;
    if (V + 1 >= 16) {
        out.can_improve = true;
        out.pruned_segments = 0;
        return out;
    }

    double plo[3], phi[3], dlo[3], dhi[3];
    aa_position_bounds(prefix[V],     plo, phi);
    aa_position_bounds(prefix[V + 1], dlo, dhi);

    for (int s = 0; s < n_sub; ++s) {
        float t0 = s * inv_n;
        float t1 = (s + 1) * inv_n;
        const auto& ext = seg_ext[s];

        bool seg_can_improve = false;
        for (int d = 0; d < 3; ++d) {
            double s0_lo = plo[d] + double(t0) * (dlo[d] - plo[d]);
            double s0_hi = phi[d] + double(t0) * (dhi[d] - phi[d]);
            double s1_lo = plo[d] + double(t1) * (dlo[d] - plo[d]);
            double s1_hi = phi[d] + double(t1) * (dhi[d] - phi[d]);

            double aa_lo = std::min(s0_lo, s1_lo);
            double aa_hi = std::max(s0_hi, s1_hi);
            if (aa_lo < ext.vals[d * 2] || aa_hi > ext.vals[d * 2 + 1]) {
                seg_can_improve = true;
                break;
            }
        }

        if (seg_can_improve) out.can_improve = true;
        else ++out.pruned_segments;
    }
    return out;
}

static bool aa_link_can_improve(
    const AAMatrix4* prefix, int V,
    int n_sub, float inv_n,
    const std::vector<LinkExtremes>& seg_ext)
{
    return aa_link_improve_summary(prefix, V, n_sub, inv_n, seg_ext).can_improve;
}

static AAAxisImproveSummary aa_link_axis_improve_summary(
    const AAMatrix4* prefix, int V,
    int n_sub, float inv_n,
    const std::vector<LinkExtremes>& seg_ext)
{
    AAAxisImproveSummary out;
    if (V + 1 >= 16) {
        out.can_improve = true;
        out.axis_can_improve[0] = true;
        out.axis_can_improve[1] = true;
        out.axis_can_improve[2] = true;
        return out;
    }

    double plo[3], phi[3], dlo[3], dhi[3];
    aa_position_bounds(prefix[V],     plo, phi);
    aa_position_bounds(prefix[V + 1], dlo, dhi);

    for (int s = 0; s < n_sub; ++s) {
        float t0 = s * inv_n;
        float t1 = (s + 1) * inv_n;
        const auto& ext = seg_ext[s];

        for (int d = 0; d < 3; ++d) {
            if (out.axis_can_improve[d]) continue;
            double s0_lo = plo[d] + double(t0) * (dlo[d] - plo[d]);
            double s0_hi = phi[d] + double(t0) * (dhi[d] - phi[d]);
            double s1_lo = plo[d] + double(t1) * (dlo[d] - plo[d]);
            double s1_hi = phi[d] + double(t1) * (dhi[d] - phi[d]);
            double aa_lo = std::min(s0_lo, s1_lo);
            double aa_hi = std::max(s0_hi, s1_hi);
            if (aa_lo < ext.vals[d * 2] || aa_hi > ext.vals[d * 2 + 1]) {
                out.axis_can_improve[d] = true;
            }
        }
    }

    out.can_improve = out.axis_can_improve[0] ||
                      out.axis_can_improve[1] ||
                      out.axis_can_improve[2];
    return out;
}

static int refresh_phase_skip_links_from_aa(
    const AAMatrix4* prefix,
    const int* map,
    int n_act,
    int n_sub,
    float inv_n,
    const std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    std::vector<char>& skip_link_vec,
    int* n_pruned_segments)
{
    int pruned_links = 0;
    int pruned_segments = 0;
    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        const AAImproveSummary summary = aa_link_improve_summary(
            prefix, V, n_sub, inv_n, link_seg_ext[ci]);
        const bool can_improve = summary.can_improve;
        skip_link_vec[ci] = can_improve ? 0 : 1;
        if (!can_improve) {
            ++pruned_links;
            pruned_segments += summary.pruned_segments;
        }
    }
    if (n_pruned_segments) *n_pruned_segments = pruned_segments;
    return pruned_links;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Phase 1: 1D Edge Solver (atan2)
// ══════════════════════════════════════════════════════════════════════════�?

static void solve_edges(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    const AnalyticalCriticalConfig& config,
    DedupMetrics* dedup_metrics,
    const char* skip_link,
    const AAMatrix4* aa_prefix,
    int ci_begin,
    int ci_end,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;
    DedupAdaptiveRuntime dedup_rt{&config, dedup_metrics};

    FKWorkspace ws;
    ws.resize(n + 2);

    for (int ci = ci_begin; ci < ci_end; ++ci) {
        if (skip_link && skip_link[ci]) continue;
        int V = map[ci];
        AAAxisImproveSummary aa_axis;
        if (aa_prefix) {
            aa_axis = aa_link_axis_improve_summary(
                aa_prefix, V, n_sub, inv_n, link_seg_ext[ci]);
            if (!aa_axis.can_improve) continue;
        }
        int nj = std::min(V + 1, n);

        int n_bg = nj - 1;
        int max_bg = std::min(1 << n_bg, 128);

        for (int j = 0; j < nj; ++j) {
            double lo_j = intervals[j].lo, hi_j = intervals[j].hi;
            if (hi_j - lo_j < 1e-12) continue;

            double mid_j = 0.5 * (lo_j + hi_j);
            double qvals[3] = { lo_j, mid_j, hi_j };

            Eigen::Matrix3d A;
            for (int si = 0; si < 3; ++si) {
                A(si, 0) = std::cos(qvals[si]);
                A(si, 1) = std::sin(qvals[si]);
                A(si, 2) = 1.0;
            }
            auto qr_A = A.colPivHouseholderQr();

            Eigen::VectorXd q(n);
            // S1: suffix cache — compute_suffix_pos depends only on
            // q[j+1..nj-1] (bg bits j..nj-2). Cache and recompute
            // only when suffix bits change.
            Eigen::Vector3d cached_suf_dist, cached_suf_prox;
            int last_suf_bg = -1;
            bool has_prox = (V >= 1);

            for (int bg = 0; bg < max_bg; ++bg) {
                for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();

                int bit = 0;
                for (int jj = 0; jj < nj; ++jj) {
                    if (jj == j) continue;
                    q[jj] = (bg & (1 << bit)) ? intervals[jj].hi : intervals[jj].lo;
                    ++bit;
                }

                ws.set_identity();
                for (int k = 0; k < j; ++k)
                    ws.compute_joint(robot, q, k);

                // ── Phase B: direct coefficient extraction via DH chain ──
                // When enabled, extract alpha/beta/gamma analytically from
                // prefix * T_j * suffix, bypassing the 3-point FK sampling.
                if (config.enable_p1_direct_coeff) {
                    const Eigen::Matrix4d& prefix = ws.tf[j];
                    const auto& dh_j = robot.dh_params()[j];

                    // S1: recompute suffix only when suffix bits change
                    int suf_bg = bg >> j;
                    if (suf_bg != last_suf_bg) {
                        cached_suf_dist = compute_suffix_pos(robot, q, j, V + 1);
                        if (has_prox)
                            cached_suf_prox = compute_suffix_pos(robot, q, j, V);
                        last_suf_bg = suf_bg;
                    }

                    Coeff1D c_dist = extract_1d_coefficients(prefix, cached_suf_dist, dh_j);
                    Coeff1D c_prox{};
                    if (has_prox)
                        c_prox = extract_1d_coefficients(prefix, cached_suf_prox, dh_j);

                    std::vector<double> q_candidates;
                    q_candidates.reserve(12);

                    auto push_q_candidate = [&](double qc) {
                        for (double shift : kShifts) {
                            double qtest = qc + shift;
                            if (qtest >= lo_j - 1e-12 && qtest <= hi_j + 1e-12) {
                                qtest = std::max(lo_j, std::min(hi_j, qtest));
                                q_candidates.push_back(qtest);
                            }
                        }
                    };

                    // Process distal endpoint (V+1)
                    for (int d = 0; d < 3; ++d) {
                        if (aa_prefix && !aa_axis.axis_can_improve[d]) continue;
                        double alpha_c = c_dist.alpha[d];
                        double beta_c  = c_dist.beta[d];
                        if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                            continue;

                        double q_star = std::atan2(beta_c, alpha_c);
                        double candidates[2] = { q_star, q_star + M_PI };
                        if (candidates[1] > M_PI + 0.1) candidates[1] -= 2 * M_PI;
                        for (double qc : candidates)
                            push_q_candidate(qc);
                    }

                    // Process proximal endpoint (V)
                    if (has_prox) {
                        for (int d = 0; d < 3; ++d) {
                            if (aa_prefix && !aa_axis.axis_can_improve[d]) continue;
                            double alpha_c = c_prox.alpha[d];
                            double beta_c  = c_prox.beta[d];
                            if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                                continue;
                            double q_star = std::atan2(beta_c, alpha_c);
                            double candidates[2] = { q_star, q_star + M_PI };
                            if (candidates[1] > M_PI + 0.1) candidates[1] -= 2 * M_PI;
                            for (double qcc : candidates)
                                push_q_candidate(qcc);
                        }
                    }

                    if (!q_candidates.empty()) {
                        const std::size_t raw_count = q_candidates.size();
                        const bool do_dedup = dedup_rt.decide(raw_count);
                        if (do_dedup) {
                            std::sort(q_candidates.begin(), q_candidates.end());
                            q_candidates.erase(
                                std::unique(q_candidates.begin(), q_candidates.end(),
                                            [](double a, double b) {
                                                return std::abs(a - b) < 1e-10;
                                            }),
                                q_candidates.end());
                            dedup_rt.record_applied(raw_count, q_candidates.size());
                        } else {
                            dedup_rt.record_skipped(raw_count);
                        }

                        for (double qtest : q_candidates) {
                            q[j] = qtest;
                            eval_and_update_from(robot, q, j, n_sub, inv_n,
                                            map, n_act, link_seg_ext, ws);
                            ++fk_calls;
                            ++found;
                        }
                    }
                    continue;  // skip the normal QR path below
                }

                // ── Original QR path ──
                Eigen::Vector3d pts_dist[3];
                Eigen::Vector3d pts_prox[3];
                bool ok = true;
                for (int si = 0; si < 3; ++si) {
                    q[j] = qvals[si];
                    ws.compute_from(robot, q, j);
                    ++fk_calls;
                    if (V + 1 >= ws.np) { ok = false; break; }
                    pts_dist[si] = ws.pos(V + 1);
                    if (has_prox) pts_prox[si] = ws.pos(V);
                }
                if (!ok) continue;

                {
                    Eigen::Vector3d b_vec;
                    Eigen::Vector3d coeff;
                    std::vector<double> q_candidates;
                    q_candidates.reserve(12);

                    auto push_q_candidate = [&](double qc) {
                        for (double shift : kShifts) {
                            double qtest = qc + shift;
                            if (qtest >= lo_j - 1e-12 && qtest <= hi_j + 1e-12) {
                                qtest = std::max(lo_j, std::min(hi_j, qtest));
                                q_candidates.push_back(qtest);
                            }
                        }
                    };

                    // Process distal endpoint (V+1)
                    for (int d = 0; d < 3; ++d) {
                        if (aa_prefix && !aa_axis.axis_can_improve[d]) continue;
                        b_vec << pts_dist[0][d], pts_dist[1][d], pts_dist[2][d];
                        coeff = qr_A.solve(b_vec);
                        double alpha_c = coeff[0];
                        double beta_c  = coeff[1];
                        if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                            continue;

                        double q_star = std::atan2(beta_c, alpha_c);
                        double candidates[2] = { q_star, q_star + M_PI };
                        if (candidates[1] > M_PI + 0.1) candidates[1] -= 2 * M_PI;

                        for (double qc : candidates) {
                            push_q_candidate(qc);
                        }
                    }

                    // Process proximal endpoint (V)
                    if (has_prox) {
                        for (int d = 0; d < 3; ++d) {
                            if (aa_prefix && !aa_axis.axis_can_improve[d]) continue;
                            b_vec << pts_prox[0][d], pts_prox[1][d], pts_prox[2][d];
                            coeff = qr_A.solve(b_vec);
                            double alpha_c = coeff[0];
                            double beta_c  = coeff[1];
                            if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                                continue;
                            double q_star = std::atan2(beta_c, alpha_c);
                            double candidates[2] = { q_star, q_star + M_PI };
                            if (candidates[1] > M_PI + 0.1) candidates[1] -= 2 * M_PI;
                            for (double qcc : candidates) {
                                push_q_candidate(qcc);
                            }
                        }
                    }

                    if (!q_candidates.empty()) {
                        const std::size_t raw_count = q_candidates.size();
                        const bool do_dedup = dedup_rt.decide(raw_count);
                        if (do_dedup) {
                            std::sort(q_candidates.begin(), q_candidates.end());
                            q_candidates.erase(
                                std::unique(q_candidates.begin(), q_candidates.end(),
                                            [](double a, double b) {
                                                return std::abs(a - b) < 1e-10;
                                            }),
                                q_candidates.end());
                            dedup_rt.record_applied(raw_count, q_candidates.size());
                        } else {
                            dedup_rt.record_skipped(raw_count);
                        }

                        for (double qtest : q_candidates) {
                            q[j] = qtest;
                            eval_and_update_from(robot, q, j, n_sub, inv_n,
                                            map, n_act, link_seg_ext, ws);
                            ++fk_calls;
                            ++found;
                        }
                    }
                }
            }
        }
    }

    if (n_found) *n_found = found;
    if (n_fk_calls) *n_fk_calls = fk_calls;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Phase 2: 2D Face Solver (degree-8 polynomial)
// ══════════════════════════════════════════════════════════════════════════�?

static void solve_faces(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    const std::vector<std::pair<int,int>>& face_pairs,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    const AnalyticalCriticalConfig& config,
    DedupMetrics* dedup_metrics,
    const char* skip_link,
    const AAMatrix4* aa_prefix,
    int ci_begin,
    int ci_end,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;
    DedupAdaptiveRuntime dedup_rt{&config, dedup_metrics};

    FKWorkspace ws;
    ws.resize(n + 2);

    for (int ci = ci_begin; ci < ci_end; ++ci) {
        if (skip_link && skip_link[ci]) continue;
        int V = map[ci];
        AAAxisImproveSummary aa_axis;
        if (aa_prefix) {
            aa_axis = aa_link_axis_improve_summary(
                aa_prefix, V, n_sub, inv_n, link_seg_ext[ci]);
            if (!aa_axis.can_improve) continue;
        }
        int nj = std::min(V + 1, n);

        for (auto& [pi, pj] : face_pairs) {
            if (pi >= nj || pj >= nj) continue;
            double lo_i = intervals[pi].lo, hi_i = intervals[pi].hi;
            double lo_j = intervals[pj].lo, hi_j = intervals[pj].hi;
            if (hi_i - lo_i < 1e-12 || hi_j - lo_j < 1e-12) continue;

            double tj_lo_pre = std::tan(lo_j * 0.5);
            double tj_hi_pre = std::tan(hi_j * 0.5);

            int n_bg_joints = nj - 2;
            int max_bg = std::min(1 << std::max(n_bg_joints, 0), 16);

            Eigen::VectorXd q(n);
            for (int bg = 0; bg < max_bg; ++bg) {
                for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();

                int bit = 0;
                for (int jj = 0; jj < nj; ++jj) {
                    if (jj == pi || jj == pj) continue;
                    q[jj] = (bg & (1 << bit)) ? intervals[jj].hi : intervals[jj].lo;
                    ++bit;
                }

                double qi_vals[3] = { lo_i, 0.5*(lo_i+hi_i), hi_i };
                double qj_vals[3] = { lo_j, 0.5*(lo_j+hi_j), hi_j };

                Eigen::Vector3d pos_V_cache[9], pos_V1_cache[9];
                bool valid = true;

                {
                    int j_lo = std::min(pi, pj);
                    int j_hi = std::max(pi, pj);

                    ws.set_identity();
                    for (int k = 0; k < j_lo; ++k)
                        ws.compute_joint(robot, q, k);

                    // ── Phase C: direct 2D coefficient extraction via DH chain ──
                    if (config.enable_p2_direct_coeff) {
                        const Eigen::Matrix4d& prefix = ws.tf[j_lo];
                        const auto& dh_lo = robot.dh_params()[j_lo];
                        const auto& dh_hi = robot.dh_params()[j_hi];
                        Eigen::Matrix4d mid = compute_middle_matrix(robot, q, j_lo, j_hi);
                        const bool swap = (pi > pj);
                        const int max_eval_frame = n + (robot.has_tool() ? 1 : 0);

                        for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                            int eval_link = (eval_idx == 0) ? V + 1 : V;
                            if (eval_link < 1 || eval_link > max_eval_frame) continue;

                            Eigen::Vector3d suf = compute_suffix_pos(robot, q, j_hi, eval_link);
                            Coeff2D c2d = extract_2d_coefficients(prefix, dh_lo, mid, dh_hi, suf);

                            for (int d = 0; d < 3; ++d) {
                                if (aa_prefix && !aa_axis.axis_can_improve[d]) continue;
                                double a1, a2, a3, a4, a5, a6, a7, a8;
                                if (!swap) {
                                    a1 = c2d.a[0][d]; a2 = c2d.a[1][d];
                                    a3 = c2d.a[2][d]; a4 = c2d.a[3][d];
                                    a5 = c2d.a[4][d]; a6 = c2d.a[5][d];
                                    a7 = c2d.a[6][d]; a8 = c2d.a[7][d];
                                } else {
                                    // Remap chain-order (lo/hi) to standard (pi/pj):
                                    // ci=c_hi, si=s_hi, cj=c_lo, sj=s_lo
                                    a1 = c2d.a[0][d]; a2 = c2d.a[2][d];
                                    a3 = c2d.a[1][d]; a4 = c2d.a[3][d];
                                    a5 = c2d.a[6][d]; a6 = c2d.a[7][d];
                                    a7 = c2d.a[4][d]; a8 = c2d.a[5][d];
                                }

                                double poly_arr[9];
                                build_symbolic_poly8(a1,a2,a3,a4,a5,a6,a7,a8, poly_arr);

                                FixedRoots tj_roots;
                                solve_poly_in_interval(poly_arr, 8,
                                                       tj_lo_pre, tj_hi_pre, tj_roots);

                                std::vector<std::pair<double, double>> q_candidates;
                                q_candidates.reserve(16);

                                for (double tj_root : tj_roots) {
                                    double qj_cand;
                                    if (!half_angle_to_q(tj_root, lo_j, hi_j, qj_cand))
                                        continue;

                                    double tj2 = tj_root * tj_root;
                                    double denom_j = 1.0 + tj2;
                                    double cj_v2 = (1.0 - tj2) / denom_j;
                                    double sj_v2 = 2.0 * tj_root / denom_j;

                                    double P = a3*cj_v2 + a4*sj_v2 + a6;
                                    double Q = a1*cj_v2 + a2*sj_v2 + a5;
                                    double qi_cand = std::atan2(P, Q);

                                    double qi_tries[3] = {qi_cand, qi_cand+M_PI, qi_cand-M_PI};
                                    bool accepted = false;
                                    for (double qi_try : qi_tries) {
                                        for (double shift : kShifts) {
                                            double qi_test = qi_try + shift;
                                            if (qi_test >= lo_i - 1e-10 &&
                                                qi_test <= hi_i + 1e-10) {
                                                qi_test = std::max(lo_i, std::min(hi_i, qi_test));
                                                q_candidates.push_back({qi_test, qj_cand});
                                                accepted = true;
                                                break;
                                            }
                                        }
                                        if (accepted) break;
                                    }
                                }

                                if (!q_candidates.empty()) {
                                    const std::size_t raw_count = q_candidates.size();
                                    const bool do_dedup = dedup_rt.decide(raw_count);
                                    if (do_dedup) {
                                        std::sort(q_candidates.begin(), q_candidates.end(),
                                                  [](const std::pair<double, double>& a,
                                                     const std::pair<double, double>& b) {
                                                      if (std::abs(a.first - b.first) >= 1e-10)
                                                          return a.first < b.first;
                                                      return a.second < b.second;
                                                  });
                                        q_candidates.erase(
                                            std::unique(q_candidates.begin(), q_candidates.end(),
                                                        [](const std::pair<double, double>& a,
                                                           const std::pair<double, double>& b) {
                                                            return std::abs(a.first - b.first) < 1e-10 &&
                                                                   std::abs(a.second - b.second) < 1e-10;
                                                        }),
                                            q_candidates.end());
                                        dedup_rt.record_applied(raw_count, q_candidates.size());
                                    } else {
                                        dedup_rt.record_skipped(raw_count);
                                    }

                                    for (const auto& qc : q_candidates) {
                                        q[pi] = qc.first;
                                        q[pj] = qc.second;
                                        eval_and_update_from(robot, q, std::min(pi, pj),
                                                        n_sub, inv_n,
                                                        map, n_act, link_seg_ext, ws);
                                        ++fk_calls;
                                        ++found;
                                    }
                                }
                            }
                        }
                        continue;  // skip the QR path below
                    }

                    Eigen::Matrix<double, 9, 9> A_mat;
                    int row = 0;

                    double* lo_vals = (pi <= pj) ? qi_vals : qj_vals;
                    double* hi_vals = (pi <= pj) ? qj_vals : qi_vals;

                    for (int si = 0; si < 3; ++si) {
                        q[j_lo] = lo_vals[si];
                        ws.compute_joint(robot, q, j_lo);
                        for (int k = j_lo + 1; k < j_hi; ++k)
                            ws.compute_joint(robot, q, k);

                        for (int sj = 0; sj < 3; ++sj) {
                            q[j_hi] = hi_vals[sj];
                            ws.compute_from(robot, q, j_hi);
                            ++fk_calls;
                            if (V + 1 >= ws.np) { valid = false; break; }

                            pos_V_cache[row] = ws.pos(V);
                            pos_V1_cache[row] = ws.pos(V + 1);

                            int orig_si = (pi <= pj) ? si : sj;
                            int orig_sj = (pi <= pj) ? sj : si;
                            double ci_v = std::cos(qi_vals[orig_si]);
                            double si_v = std::sin(qi_vals[orig_si]);
                            double cj_v = std::cos(qj_vals[orig_sj]);
                            double sj_v = std::sin(qj_vals[orig_sj]);

                            A_mat(row, 0) = ci_v * cj_v;
                            A_mat(row, 1) = ci_v * sj_v;
                            A_mat(row, 2) = si_v * cj_v;
                            A_mat(row, 3) = si_v * sj_v;
                            A_mat(row, 4) = ci_v;
                            A_mat(row, 5) = si_v;
                            A_mat(row, 6) = cj_v;
                            A_mat(row, 7) = sj_v;
                            A_mat(row, 8) = 1.0;
                            ++row;
                        }
                        if (!valid) break;
                    }
                    if (!valid) continue;

                    auto qr = A_mat.colPivHouseholderQr();

                    for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                        int eval_link = (eval_idx == 0) ? V + 1 : V;
                        if (eval_link < 1 || eval_link >= n + 2) continue;
                        const Eigen::Vector3d* pos_cache =
                            (eval_idx == 0) ? pos_V1_cache : pos_V_cache;

                        Eigen::Matrix<double, 9, 3> B_mat;
                        for (int r = 0; r < 9; ++r)
                            for (int dd = 0; dd < 3; ++dd)
                                B_mat(r, dd) = pos_cache[r][dd];

                        Eigen::Matrix<double, 9, 3> coeff_3d = qr.solve(B_mat);

                        for (int d = 0; d < 3; ++d) {
                            if (aa_prefix && !aa_axis.axis_can_improve[d]) continue;
                            double a1 = coeff_3d(0,d), a2 = coeff_3d(1,d);
                            double a3 = coeff_3d(2,d), a4 = coeff_3d(3,d);
                            double a5 = coeff_3d(4,d), a6 = coeff_3d(5,d);
                            double a7 = coeff_3d(6,d), a8 = coeff_3d(7,d);

                            double poly_arr[9];
                            build_symbolic_poly8(a1,a2,a3,a4,a5,a6,a7,a8, poly_arr);

                            FixedRoots tj_roots;
                            solve_poly_in_interval(poly_arr, 8,
                                                   tj_lo_pre, tj_hi_pre, tj_roots);

                            std::vector<std::pair<double, double>> q_candidates;
                            q_candidates.reserve(16);

                            for (double tj_root : tj_roots) {
                                double qj_cand;
                                if (!half_angle_to_q(tj_root, lo_j, hi_j, qj_cand))
                                    continue;

                                double tj2 = tj_root * tj_root;
                                double denom_j = 1.0 + tj2;
                                double cj_v2 = (1.0 - tj2) / denom_j;
                                double sj_v2 = 2.0 * tj_root / denom_j;

                                double P = a3*cj_v2 + a4*sj_v2 + a6;
                                double Q = a1*cj_v2 + a2*sj_v2 + a5;
                                double qi_cand = std::atan2(P, Q);

                                double qi_tries[3] = {qi_cand, qi_cand+M_PI, qi_cand-M_PI};
                                bool accepted = false;
                                for (double qi_try : qi_tries) {
                                    for (double shift : kShifts) {
                                        double qi_test = qi_try + shift;
                                        if (qi_test >= lo_i - 1e-10 &&
                                            qi_test <= hi_i + 1e-10) {
                                            qi_test = std::max(lo_i, std::min(hi_i, qi_test));
                                            q_candidates.push_back({qi_test, qj_cand});
                                            accepted = true;
                                            break;
                                        }
                                    }
                                    if (accepted) break;
                                }
                            }

                            if (!q_candidates.empty()) {
                                const std::size_t raw_count = q_candidates.size();
                                const bool do_dedup = dedup_rt.decide(raw_count);
                                if (do_dedup) {
                                    std::sort(q_candidates.begin(), q_candidates.end(),
                                              [](const std::pair<double, double>& a,
                                                 const std::pair<double, double>& b) {
                                                  if (std::abs(a.first - b.first) >= 1e-10)
                                                      return a.first < b.first;
                                                  return a.second < b.second;
                                              });
                                    q_candidates.erase(
                                        std::unique(q_candidates.begin(), q_candidates.end(),
                                                    [](const std::pair<double, double>& a,
                                                       const std::pair<double, double>& b) {
                                                        return std::abs(a.first - b.first) < 1e-10 &&
                                                               std::abs(a.second - b.second) < 1e-10;
                                                    }),
                                        q_candidates.end());
                                    dedup_rt.record_applied(raw_count, q_candidates.size());
                                } else {
                                    dedup_rt.record_skipped(raw_count);
                                }

                                for (const auto& qc : q_candidates) {
                                    q[pi] = qc.first;
                                    q[pj] = qc.second;
                                    eval_and_update_from(robot, q, std::min(pi, pj),
                                                    n_sub, inv_n,
                                                    map, n_act, link_seg_ext, ws);
                                    ++fk_calls;
                                    ++found;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (n_found) *n_found = found;
    if (n_fk_calls) *n_fk_calls = fk_calls;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Phase 3: Coordinate-wise sweep (basic)
// ══════════════════════════════════════════════════════════════════════════�?

static void solve_interior(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    int max_free, int max_sweeps,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    const AnalyticalCriticalConfig& config,
    DedupMetrics* dedup_metrics,
    const char* skip_link,
    const AAMatrix4* aa_prefix,
    int ci_begin,
    int ci_end,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;
    DedupAdaptiveRuntime dedup_rt{&config, dedup_metrics};

    FKWorkspace ws;
    ws.resize(n + 2);

    for (int ci = ci_begin; ci < ci_end; ++ci) {
        if (skip_link && skip_link[ci]) continue;
        int V = map[ci];
        AAAxisImproveSummary aa_axis;
        if (aa_prefix) {
            aa_axis = aa_link_axis_improve_summary(
                aa_prefix, V, n_sub, inv_n, link_seg_ext[ci]);
            if (!aa_axis.can_improve) continue;
        }
        int nj = std::min(V + 1, n);
        if (nj < 3 || nj > max_free) continue;

        for (int face = 0; face < 6; ++face) {
            int dim = face / 2;
            bool is_min = (face % 2 == 0);
            if (aa_prefix && !aa_axis.axis_can_improve[dim]) continue;

            Eigen::VectorXd q = link_seg_ext[ci][0].configs[face];
            if (q.size() != n) continue;

            for (int sweep = 0; sweep < max_sweeps; ++sweep) {
                bool improved = false;

                for (int j = 0; j < nj; ++j) {
                    double lo_j = intervals[j].lo, hi_j = intervals[j].hi;
                    if (hi_j - lo_j < 1e-12) continue;

                    ws.set_identity();
                    for (int k = 0; k < j; ++k)
                        ws.compute_joint(robot, q, k);

                    // ── Phase F: direct coefficient extraction for P3 ──
                    if (config.enable_p3_direct_coeff) {
                        const Eigen::Matrix4d& prefix = ws.tf[j];
                        const auto& dh_j = robot.dh_params()[j];
                        bool has_prox = (V >= 1);

                        Eigen::Vector3d suf_dist = compute_suffix_pos(robot, q, j, V + 1);
                        Coeff1D c_dist = extract_1d_coefficients(prefix, suf_dist, dh_j);

                        Coeff1D c_prox{};
                        if (has_prox) {
                            Eigen::Vector3d suf_prox = compute_suffix_pos(robot, q, j, V);
                            c_prox = extract_1d_coefficients(prefix, suf_prox, dh_j);
                        }

                        for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                            int eval_link = (eval_idx == 0) ? V + 1 : V;
                            if (eval_link < 1) continue;
                            const Coeff1D& cc = (eval_idx == 0) ? c_dist : c_prox;

                            double alpha_c = cc.alpha[dim];
                            double beta_c  = cc.beta[dim];
                            if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                                continue;

                            std::vector<double> q_candidates;
                            q_candidates.reserve(6);

                            double q_star = std::atan2(beta_c, alpha_c);
                            if (is_min) q_star += M_PI;

                            for (double shift : kShifts) {
                                double qtest = q_star + shift;
                                if (qtest >= lo_j - 1e-10 && qtest <= hi_j + 1e-10) {
                                    qtest = std::max(lo_j, std::min(hi_j, qtest));
                                    q_candidates.push_back(qtest);
                                }
                            }
                            double q_star2 = q_star + M_PI;
                            for (double shift : kShifts) {
                                double qtest = q_star2 + shift;
                                if (qtest >= lo_j - 1e-10 && qtest <= hi_j + 1e-10) {
                                    qtest = std::max(lo_j, std::min(hi_j, qtest));
                                    q_candidates.push_back(qtest);
                                }
                            }

                            if (!q_candidates.empty()) {
                                const std::size_t raw_count = q_candidates.size();
                                const bool do_dedup = dedup_rt.decide(raw_count);
                                if (do_dedup) {
                                    std::sort(q_candidates.begin(), q_candidates.end());
                                    q_candidates.erase(
                                        std::unique(q_candidates.begin(), q_candidates.end(),
                                                    [](double a, double b) {
                                                        return std::abs(a - b) < 1e-10;
                                                    }),
                                        q_candidates.end());
                                    dedup_rt.record_applied(raw_count, q_candidates.size());
                                } else {
                                    dedup_rt.record_skipped(raw_count);
                                }

                                for (double qtest : q_candidates) {
                                    q[j] = qtest;
                                    eval_and_update_from(robot, q, j, n_sub, inv_n,
                                                    map, n_act, link_seg_ext, ws);
                                    ++fk_calls;
                                    ++found;
                                    improved = true;
                                }
                            }
                        }
                    } else {
                        // ── Original QR path ──
                        double mid_j = 0.5 * (lo_j + hi_j);
                        double qvals[3] = { lo_j, mid_j, hi_j };

                        Eigen::Matrix3d A_trig;
                        for (int si = 0; si < 3; ++si) {
                            A_trig(si, 0) = std::cos(qvals[si]);
                            A_trig(si, 1) = std::sin(qvals[si]);
                            A_trig(si, 2) = 1.0;
                        }
                        auto qr_A_int = A_trig.colPivHouseholderQr();

                        Eigen::Vector3d pts_dist[3], pts_prox[3];
                        bool has_prox = (V >= 1);
                        bool ok_all = true;
                        for (int si = 0; si < 3; ++si) {
                            double q_save = q[j];
                            q[j] = qvals[si];
                            ws.compute_from(robot, q, j);
                            ++fk_calls;
                            if (V + 1 >= ws.np) { q[j] = q_save; ok_all = false; break; }
                            pts_dist[si] = ws.pos(V + 1);
                            if (has_prox) pts_prox[si] = ws.pos(V);
                            q[j] = q_save;
                        }
                        if (!ok_all) continue;

                        for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                            int eval_link = (eval_idx == 0) ? V + 1 : V;
                            if (eval_link < 1) continue;
                            const Eigen::Vector3d* pts = (eval_idx == 0) ? pts_dist : pts_prox;

                            std::vector<double> q_candidates;
                            q_candidates.reserve(6);

                            {
                                Eigen::Vector3d b_vec;
                                b_vec << pts[0][dim], pts[1][dim], pts[2][dim];
                                Eigen::Vector3d coeff = qr_A_int.solve(b_vec);

                                double alpha_c = coeff[0], beta_c = coeff[1];
                                if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                                    continue;

                                double q_star = std::atan2(beta_c, alpha_c);
                                if (is_min) q_star += M_PI;

                                for (double shift : kShifts) {
                                    double qtest = q_star + shift;
                                    if (qtest >= lo_j - 1e-10 && qtest <= hi_j + 1e-10) {
                                        qtest = std::max(lo_j, std::min(hi_j, qtest));
                                        q_candidates.push_back(qtest);
                                    }
                                }
                                double q_star2 = q_star + M_PI;
                                for (double shift : kShifts) {
                                    double qtest = q_star2 + shift;
                                    if (qtest >= lo_j - 1e-10 && qtest <= hi_j + 1e-10) {
                                        qtest = std::max(lo_j, std::min(hi_j, qtest));
                                        q_candidates.push_back(qtest);
                                    }
                                }
                            }

                            if (!q_candidates.empty()) {
                                const std::size_t raw_count = q_candidates.size();
                                const bool do_dedup = dedup_rt.decide(raw_count);
                                if (do_dedup) {
                                    std::sort(q_candidates.begin(), q_candidates.end());
                                    q_candidates.erase(
                                        std::unique(q_candidates.begin(), q_candidates.end(),
                                                    [](double a, double b) {
                                                        return std::abs(a - b) < 1e-10;
                                                    }),
                                        q_candidates.end());
                                    dedup_rt.record_applied(raw_count, q_candidates.size());
                                } else {
                                    dedup_rt.record_skipped(raw_count);
                                }

                                for (double qtest : q_candidates) {
                                    q[j] = qtest;
                                    eval_and_update_from(robot, q, j, n_sub, inv_n,
                                                    map, n_act, link_seg_ext, ws);
                                    ++fk_calls;
                                    ++found;
                                    improved = true;
                                }
                            }
                        }
                    }
                }

                if (!improved) break;
            }
        }
    }

    if (n_found) *n_found = found;
    if (n_fk_calls) *n_fk_calls = fk_calls;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Phase 2.5a: Pair-Constrained 1D Solver
// ══════════════════════════════════════════════════════════════════════════�?

static void solve_pair_constrained_1d(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    const std::vector<std::pair<int,int>>& pairs,
    int max_bg,
    bool kpi2_backgrounds,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    const AnalyticalCriticalConfig& config,
    DedupMetrics* dedup_metrics,
    const char* skip_link,
    const AAMatrix4* aa_prefix,
    int ci_begin,
    int ci_end,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;
    DedupAdaptiveRuntime dedup_rt{&config, dedup_metrics};

    FKWorkspace ws;
    ws.resize(n + 2);

    std::vector<std::vector<double>> bg_vals(n);
    for (int j = 0; j < n; ++j)
        bg_vals[j] = build_bg_values(intervals[j].lo, intervals[j].hi, kpi2_backgrounds);

    for (int ci = ci_begin; ci < ci_end; ++ci) {
        if (skip_link && skip_link[ci]) continue;
        int V = map[ci];
        AAAxisImproveSummary aa_axis;
        if (aa_prefix) {
            aa_axis = aa_link_axis_improve_summary(
                aa_prefix, V, n_sub, inv_n, link_seg_ext[ci]);
            if (!aa_axis.can_improve) continue;
        }
        int nj = std::min(V + 1, n);

        for (auto& [pi, pj] : pairs) {
            if (pi >= nj || pj >= nj) continue;
            double lo_i = intervals[pi].lo, hi_i = intervals[pi].hi;
            double lo_j = intervals[pj].lo, hi_j = intervals[pj].hi;
            if (hi_i - lo_i < 1e-12 || hi_j - lo_j < 1e-12) continue;

            double sum_lo = lo_i + lo_j, sum_hi = hi_i + hi_j;
            int k_lo = static_cast<int>(std::ceil(sum_lo / HALF_PI - 1e-9));
            int k_hi = static_cast<int>(std::floor(sum_hi / HALF_PI + 1e-9));

            for (int kk = k_lo; kk <= k_hi; ++kk) {
                double C = kk * HALF_PI;
                double eff_lo = std::max(lo_i, C - hi_j);
                double eff_hi = std::min(hi_i, C - lo_j);
                if (eff_hi - eff_lo < 1e-12) continue;

                // Stack-allocated background arrays (max 7 joints → max 5 "other")
                constexpr int MAX_OTHER_1D = 8;
                constexpr int MAX_BG_PER_JOINT = 16;
                int other_idx_arr[MAX_OTHER_1D];
                double other_bg_arr[MAX_OTHER_1D][MAX_BG_PER_JOINT];
                int other_bg_sz[MAX_OTHER_1D];
                int n_other = 0;
                long long total_bg = 1;
                for (int jj = 0; jj < nj; ++jj) {
                    if (jj == pi || jj == pj) continue;
                    if (n_other >= MAX_OTHER_1D) break;
                    other_idx_arr[n_other] = jj;
                    const auto& bv = bg_vals[jj];
                    int sz = std::min(static_cast<int>(bv.size()), MAX_BG_PER_JOINT);
                    for (int bi = 0; bi < sz; ++bi) other_bg_arr[n_other][bi] = bv[bi];
                    other_bg_sz[n_other] = sz;
                    total_bg *= sz;
                    ++n_other;
                    if (total_bg > max_bg) break;
                }
                if (total_bg > max_bg) {
                    n_other = 0;
                    total_bg = 1;
                    for (int jj = 0; jj < nj; ++jj) {
                        if (jj == pi || jj == pj) continue;
                        if (n_other >= MAX_OTHER_1D) break;
                        other_idx_arr[n_other] = jj;
                        other_bg_arr[n_other][0] = intervals[jj].lo;
                        other_bg_arr[n_other][1] = intervals[jj].hi;
                        other_bg_sz[n_other] = 2;
                        total_bg *= 2;
                        ++n_other;
                        if (total_bg > max_bg) { total_bg = max_bg; break; }
                    }
                }

                // Precompute constraint trig values (used by both direct and QR paths)
                double cosC = std::cos(C), sinC = std::sin(C);

                double eff_mid = 0.5 * (eff_lo + eff_hi);
                double qvals[3] = { eff_lo, eff_mid, eff_hi };
                Eigen::Matrix3d A_basis;
                for (int si = 0; si < 3; ++si) {
                    A_basis(si, 0) = std::cos(qvals[si]);
                    A_basis(si, 1) = std::sin(qvals[si]);
                    A_basis(si, 2) = 1.0;
                }
                auto qr_A = A_basis.colPivHouseholderQr();

                int j_first = std::min(pi, pj);

                int bg_counter[MAX_OTHER_1D] = {};
                Eigen::VectorXd q(n);
                for (long long bgc = 0; bgc < total_bg; ++bgc) {
                    for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();
                    for (int oi = 0; oi < n_other; ++oi)
                        q[other_idx_arr[oi]] = other_bg_arr[oi][bg_counter[oi]];

                    ws.set_identity();
                    for (int k = 0; k < j_first; ++k)
                        ws.compute_joint(robot, q, k);

                    // ── Phase D+: P2.5a direct coefficient extraction ──
                    // Optimized: extract α,β directly from 2D coefficients + constraint,
                    // bypassing 3-point evaluation and QR solve entirely.
                    if (config.enable_p25_direct_coeff) {
                        int j_lo = std::min(pi, pj);
                        int j_hi = std::max(pi, pj);
                        const Eigen::Matrix4d& prefix_lo = ws.tf[j_lo];
                        const auto& dh_lo = robot.dh_params()[j_lo];
                        const auto& dh_hi = robot.dh_params()[j_hi];
                        Eigen::Matrix4d mid = compute_middle_matrix(robot, q, j_lo, j_hi);
                        const int max_eval = n + (robot.has_tool() ? 1 : 0);

                        for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                            int eval_link = (eval_idx == 0) ? V + 1 : V;
                            if (eval_link < 1 || eval_link > max_eval) continue;

                            Eigen::Vector3d suf = compute_suffix_pos(robot, q, j_hi, eval_link);
                            Coeff2D c2d = extract_2d_coefficients(prefix_lo, dh_lo, mid, dh_hi, suf);

                            for (int d = 0; d < 3; ++d) {
                                if (aa_prefix && !aa_axis.axis_can_improve[d]) continue;

                                // Direct α,β from 2D coefficients + constraint qi+qj=C.
                                // The constrained function f(qi) has cos(qi)/sin(qi) coefficients:
                                //   pi≤pj: q_lo=qi, q_hi=C−qi → α = a4 + a6·cosC + a7·sinC
                                //                                 β = a5 + a6·sinC − a7·cosC
                                //   pi>pj: q_lo=C−qi, q_hi=qi  → α = a4·cosC + a5·sinC + a6
                                //                                 β = a4·sinC − a5·cosC + a7
                                double alpha_c, beta_c;
                                if (pi <= pj) {
                                    alpha_c = c2d.a[4][d] + c2d.a[6][d]*cosC + c2d.a[7][d]*sinC;
                                    beta_c  = c2d.a[5][d] + c2d.a[6][d]*sinC - c2d.a[7][d]*cosC;
                                } else {
                                    alpha_c = c2d.a[4][d]*cosC + c2d.a[5][d]*sinC + c2d.a[6][d];
                                    beta_c  = c2d.a[4][d]*sinC - c2d.a[5][d]*cosC + c2d.a[7][d];
                                }
                                if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                                    continue;

                                std::vector<double> qi_candidates;
                                qi_candidates.reserve(6);
                                double q_star = std::atan2(beta_c, alpha_c);
                                double q_cands[2] = { q_star, q_star + M_PI };
                                if (q_cands[1] > M_PI + 0.1) q_cands[1] -= 2 * M_PI;

                                for (double qc : q_cands) {
                                    for (double shift : kShifts) {
                                        double qi_test = qc + shift;
                                        if (qi_test >= eff_lo - 1e-10 &&
                                            qi_test <= eff_hi + 1e-10) {
                                            qi_test = std::max(eff_lo, std::min(eff_hi, qi_test));
                                            qi_candidates.push_back(qi_test);
                                        }
                                    }
                                }

                                if (!qi_candidates.empty()) {
                                    const std::size_t raw_count = qi_candidates.size();
                                    const bool do_dedup = dedup_rt.decide(raw_count);
                                    if (do_dedup) {
                                        std::sort(qi_candidates.begin(), qi_candidates.end());
                                        qi_candidates.erase(
                                            std::unique(qi_candidates.begin(), qi_candidates.end(),
                                                        [](double a, double b) {
                                                            return std::abs(a - b) < 1e-10;
                                                        }),
                                            qi_candidates.end());
                                        dedup_rt.record_applied(raw_count, qi_candidates.size());
                                    } else {
                                        dedup_rt.record_skipped(raw_count);
                                    }

                                    for (double qi_test : qi_candidates) {
                                        double qj_test = C - qi_test;
                                        if (qj_test < lo_j - 1e-10 || qj_test > hi_j + 1e-10)
                                            continue;
                                        qj_test = std::max(lo_j, std::min(hi_j, qj_test));
                                        q[pi] = qi_test;
                                        q[pj] = qj_test;
                                        eval_and_update_from(robot, q, std::min(pi, pj),
                                                        n_sub, inv_n,
                                                        map, n_act, link_seg_ext, ws);
                                        ++fk_calls;
                                        ++found;
                                    }
                                }
                            }
                        }
                        goto next_bg_2_5a;
                    }

                    Eigen::Vector3d pts_dist[3], pts_prox[3];
                    bool has_prox = (V >= 1);
                    bool ok = true;
                    for (int si = 0; si < 3; ++si) {
                        q[pi] = qvals[si];
                        q[pj] = C - qvals[si];
                        ws.compute_from(robot, q, j_first);
                        ++fk_calls;
                        if (V + 1 >= ws.np) { ok = false; break; }
                        pts_dist[si] = ws.pos(V + 1);
                        if (has_prox) pts_prox[si] = ws.pos(V);
                    }
                    if (!ok) goto next_bg_2_5a;

                    for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                        int eval_link = (eval_idx == 0) ? V + 1 : V;
                        if (eval_link < 1) continue;
                        const Eigen::Vector3d* pts = (eval_idx == 0) ? pts_dist : pts_prox;

                        Eigen::Vector3d b_vec, coeff;
                        for (int d = 0; d < 3; ++d) {
                            if (aa_prefix && !aa_axis.axis_can_improve[d]) continue;
                            std::vector<double> qi_candidates;
                            qi_candidates.reserve(6);

                            b_vec << pts[0][d], pts[1][d], pts[2][d];
                            coeff = qr_A.solve(b_vec);
                            double alpha_c = coeff[0], beta_c = coeff[1];
                            if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                                continue;

                            double q_star = std::atan2(beta_c, alpha_c);
                            double candidates[2] = { q_star, q_star + M_PI };
                            if (candidates[1] > M_PI + 0.1) candidates[1] -= 2 * M_PI;

                            for (double qc : candidates) {
                                for (double shift : kShifts) {
                                    double qi_test = qc + shift;
                                    if (qi_test >= eff_lo - 1e-10 &&
                                        qi_test <= eff_hi + 1e-10) {
                                        qi_test = std::max(eff_lo, std::min(eff_hi, qi_test));
                                        qi_candidates.push_back(qi_test);
                                    }
                                }
                            }

                            if (!qi_candidates.empty()) {
                                const std::size_t raw_count = qi_candidates.size();
                                const bool do_dedup = dedup_rt.decide(raw_count);
                                if (do_dedup) {
                                    std::sort(qi_candidates.begin(), qi_candidates.end());
                                    qi_candidates.erase(
                                        std::unique(qi_candidates.begin(), qi_candidates.end(),
                                                    [](double a, double b) {
                                                        return std::abs(a - b) < 1e-10;
                                                    }),
                                        qi_candidates.end());
                                    dedup_rt.record_applied(raw_count, qi_candidates.size());
                                } else {
                                    dedup_rt.record_skipped(raw_count);
                                }

                                for (double qi_test : qi_candidates) {
                                    double qj_test = C - qi_test;
                                    if (qj_test < lo_j - 1e-10 || qj_test > hi_j + 1e-10)
                                        continue;
                                    qj_test = std::max(lo_j, std::min(hi_j, qj_test));

                                    q[pi] = qi_test;
                                    q[pj] = qj_test;
                                    eval_and_update_from(robot, q, std::min(pi, pj),
                                                    n_sub, inv_n,
                                                    map, n_act, link_seg_ext, ws);
                                    ++fk_calls;
                                    ++found;
                                }
                            }
                        }
                    }
                    next_bg_2_5a:;

                    for (int oi = n_other - 1; oi >= 0; --oi) {
                        if (++bg_counter[oi] < other_bg_sz[oi]) break;
                        bg_counter[oi] = 0;
                    }
                }
            }
        }
    }

    if (n_found) *n_found = found;
    if (n_fk_calls) *n_fk_calls = fk_calls;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Phase 2.5b: Pair-Constrained 2D Solver
// ══════════════════════════════════════════════════════════════════════════�?

static void solve_pair_constrained_2d(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    const std::vector<std::pair<int,int>>& pairs,
    int max_bg,
    bool kpi2_backgrounds,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    const AnalyticalCriticalConfig& config,
    DedupMetrics* dedup_metrics,
    const char* skip_link,
    const AAMatrix4* aa_prefix,
    int ci_begin,
    int ci_end,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;
    DedupAdaptiveRuntime dedup_rt{&config, dedup_metrics};

    FKWorkspace ws;
    ws.resize(n + 2);

    std::vector<std::vector<double>> bg_vals(n);
    for (int j = 0; j < n; ++j)
        bg_vals[j] = build_bg_values(intervals[j].lo, intervals[j].hi, kpi2_backgrounds);

    for (int ci = ci_begin; ci < ci_end; ++ci) {
        if (skip_link && skip_link[ci]) continue;
        int V = map[ci];
        AAAxisImproveSummary aa_axis;
        if (aa_prefix) {
            aa_axis = aa_link_axis_improve_summary(
                aa_prefix, V, n_sub, inv_n, link_seg_ext[ci]);
            if (!aa_axis.can_improve) continue;
        }
        int nj = std::min(V + 1, n);

        for (auto& [pi, pj] : pairs) {
            if (pi >= nj || pj >= nj) continue;
            double lo_i = intervals[pi].lo, hi_i = intervals[pi].hi;
            double lo_j = intervals[pj].lo, hi_j = intervals[pj].hi;
            if (hi_i - lo_i < 1e-12 || hi_j - lo_j < 1e-12) continue;

            double sum_lo = lo_i + lo_j, sum_hi = hi_i + hi_j;
            int k_lo = static_cast<int>(std::ceil(sum_lo / HALF_PI - 1e-9));
            int k_hi = static_cast<int>(std::floor(sum_hi / HALF_PI + 1e-9));

            for (int kk = k_lo; kk <= k_hi; ++kk) {
                double C = kk * HALF_PI;
                double eff_lo = std::max(lo_i, C - hi_j);
                double eff_hi = std::min(hi_i, C - lo_j);
                if (eff_hi - eff_lo < 1e-12) continue;

                for (int pm = 0; pm < nj; ++pm) {
                    if (pm == pi || pm == pj) continue;
                    double lo_m = intervals[pm].lo, hi_m = intervals[pm].hi;
                    if (hi_m - lo_m < 1e-12) continue;

                    double tm_lo_pre = std::tan(lo_m * 0.5);
                    double tm_hi_pre = std::tan(hi_m * 0.5);

                    // Stack-allocated background arrays (max 7 joints − 3 free = max 4 "other")
                    constexpr int MAX_OTHER_2D = 8;
                    constexpr int MAX_BG_PER_J2 = 16;
                    int bg_idx_arr[MAX_OTHER_2D];
                    double other_bg_2d[MAX_OTHER_2D][MAX_BG_PER_J2];
                    int other_bg_sz2[MAX_OTHER_2D];
                    int n_bg = 0;
                    long long total_bg = 1;
                    for (int jj = 0; jj < nj; ++jj) {
                        if (jj == pi || jj == pj || jj == pm) continue;
                        if (n_bg >= MAX_OTHER_2D) break;
                        bg_idx_arr[n_bg] = jj;
                        const auto& bv = bg_vals[jj];
                        int sz = std::min(static_cast<int>(bv.size()), MAX_BG_PER_J2);
                        for (int bi = 0; bi < sz; ++bi) other_bg_2d[n_bg][bi] = bv[bi];
                        other_bg_sz2[n_bg] = sz;
                        total_bg *= sz;
                        ++n_bg;
                        if (total_bg > max_bg) break;
                    }
                    if (total_bg > max_bg) {
                        n_bg = 0; total_bg = 1;
                        for (int jj = 0; jj < nj; ++jj) {
                            if (jj == pi || jj == pj || jj == pm) continue;
                            if (n_bg >= MAX_OTHER_2D) break;
                            bg_idx_arr[n_bg] = jj;
                            other_bg_2d[n_bg][0] = intervals[jj].lo;
                            other_bg_2d[n_bg][1] = intervals[jj].hi;
                            other_bg_sz2[n_bg] = 2;
                            total_bg *= 2;
                            ++n_bg;
                            if (total_bg > max_bg) { total_bg = max_bg; break; }
                        }
                    }

                    int bg_cnt[MAX_OTHER_2D] = {};

                    double qi_vals[3] = { eff_lo, 0.5*(eff_lo+eff_hi), eff_hi };
                    double qm_vals[3] = { lo_m, 0.5*(lo_m+hi_m), hi_m };

                    // H6: Kronecker decomposition — A = B_qi ⊗ B_qm
                    // A^{-1} = B_qi^{-1} ⊗ B_qm^{-1}; replaces 9×9 QR
                    // with two cheap 3×3 inverses
                    Eigen::Matrix3d B_qi_2d, B_qm_2d;
                    for (int k = 0; k < 3; ++k) {
                        B_qi_2d(k,0) = std::cos(qi_vals[k]);
                        B_qi_2d(k,1) = std::sin(qi_vals[k]);
                        B_qi_2d(k,2) = 1.0;
                        B_qm_2d(k,0) = std::cos(qm_vals[k]);
                        B_qm_2d(k,1) = std::sin(qm_vals[k]);
                        B_qm_2d(k,2) = 1.0;
                    }
                    Eigen::Matrix3d B_qi_inv_2d = B_qi_2d.inverse();
                    Eigen::Matrix3d B_qm_invT_2d = B_qm_2d.inverse().transpose();

                    int j_first_2d = std::min({pi, pj, pm});

                    // ── Hoist bg-independent computation outside bg loop ──
                    // Sort {pi, pj, pm} into chain order (constant per pm)
                    int sj_pre[3] = {pi, pj, pm};
                    std::sort(sj_pre, sj_pre + 3);
                    const int j1 = sj_pre[0], j2 = sj_pre[1], j3 = sj_pre[2];
                    const bool j1_is_pm = (j1 == pm);
                    const bool j2_is_pm = (j2 == pm);
                    const bool j3_is_pm = (j3 == pm);
                    const int max_eval = n + (robot.has_tool() ? 1 : 0);
                    const bool has_V  = (V >= 1 && V <= max_eval);
                    const bool has_V1 = (V + 1 >= 1 && V + 1 <= max_eval);
                    if (!has_V1) continue;  // skip this pm entirely

                    // Precompute DH transforms — independent of background
                    auto build_tf_pre = [&](int joint, double q_val) -> Eigen::Matrix4d {
                        const auto& dh = robot.dh_params()[joint];
                        double theta = (dh.joint_type == 0) ? q_val + dh.theta : dh.theta;
                        double d_val = (dh.joint_type == 0) ? dh.d : q_val + dh.d;
                        return dh_transform(dh.alpha, dh.a, d_val, theta);
                    };
                    Eigen::Matrix4d tf_j1_pre[3], tf_j2_pre[3], tf_j3_pre[3];
                    for (int k = 0; k < 3; ++k) {
                        double q1k = j1_is_pm ? qm_vals[k]
                                   : (j1 == pi ? qi_vals[k] : C - qi_vals[k]);
                        tf_j1_pre[k] = build_tf_pre(j1, q1k);

                        double q2k = j2_is_pm ? qm_vals[k]
                                   : (j2 == pi ? qi_vals[k] : C - qi_vals[k]);
                        tf_j2_pre[k] = build_tf_pre(j2, q2k);

                        double q3k = j3_is_pm ? qm_vals[k]
                                   : (j3 == pi ? qi_vals[k] : C - qi_vals[k]);
                        tf_j3_pre[k] = build_tf_pre(j3, q3k);
                    }

                    Eigen::VectorXd q(n);
                    for (long long bgc = 0; bgc < total_bg; ++bgc) {
                        for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();
                        for (int oi = 0; oi < n_bg; ++oi)
                            q[bg_idx_arr[oi]] = other_bg_2d[oi][bg_cnt[oi]];

                        ws.set_identity();
                        for (int k = 0; k < j_first_2d; ++k)
                            ws.compute_joint(robot, q, k);

                        // ── Phase D: P2.5b direct coefficient extraction ──
                        if (config.enable_p25_direct_coeff) {
                            Eigen::Matrix4d mid12 = compute_middle_matrix(robot, q, j1, j2);
                            Eigen::Matrix4d mid23 = compute_middle_matrix(robot, q, j2, j3);
                            const Eigen::Matrix4d& prefix_j1 = ws.tf[j1];

                            // Compute suffix_pos for V and V+1
                            Eigen::Vector3d suf_V, suf_V1;
                            if (has_V)  suf_V  = compute_suffix_pos(robot, q, j3, V);
                            suf_V1 = compute_suffix_pos(robot, q, j3, V + 1);

                            // Precompute left = prefix * T_j1[k] * mid12
                            Eigen::Matrix4d left_pre[3];
                            for (int k = 0; k < 3; ++k)
                                left_pre[k] = prefix_j1 * tf_j1_pre[k] * mid12;

                            // Precompute right = mid23 * T_j3[k] * [suf; 1]
                            Eigen::Vector4d sfx1_vec; sfx1_vec << suf_V1, 1.0;
                            Eigen::Vector4d right_V1_pre[3];
                            for (int k = 0; k < 3; ++k)
                                right_V1_pre[k] = mid23 * (tf_j3_pre[k] * sfx1_vec);

                            Eigen::Vector4d right_V_pre[3];
                            if (has_V) {
                                Eigen::Vector4d sfx0_vec; sfx0_vec << suf_V, 1.0;
                                for (int k = 0; k < 3; ++k)
                                    right_V_pre[k] = mid23 * (tf_j3_pre[k] * sfx0_vec);
                            }

                            // Evaluate 9 sample positions using precomputed matrices
                            // pos = left[k1] * T_j2[k2] * right[k3]
                            Eigen::Vector3d pos_V_cache_2d_d[9], pos_V1_cache_2d_d[9];
                            int row = 0;
                            for (int si = 0; si < 3; ++si) {
                                for (int sm = 0; sm < 3; ++sm) {
                                    const int k1 = j1_is_pm ? sm : si;
                                    const int k2 = j2_is_pm ? sm : si;
                                    const int k3 = j3_is_pm ? sm : si;

                                    Eigen::Vector4d v = tf_j2_pre[k2] * right_V1_pre[k3];
                                    pos_V1_cache_2d_d[row] = (left_pre[k1] * v).head<3>();

                                    if (has_V) {
                                        v = tf_j2_pre[k2] * right_V_pre[k3];
                                        pos_V_cache_2d_d[row] = (left_pre[k1] * v).head<3>();
                                    }
                                    ++row;
                                }
                            }

                            for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                                int eval_link = (eval_idx == 0) ? V + 1 : V;
                                if (eval_link < 1 || eval_link > max_eval) continue;
                                const Eigen::Vector3d* pos_cache =
                                    (eval_idx == 0) ? pos_V1_cache_2d_d : pos_V_cache_2d_d;

                                for (int d = 0; d < 3; ++d) {
                                    if (aa_prefix && !aa_axis.axis_can_improve[d]) continue;

                                    // H6: Kronecker coefficient extraction
                                    Eigen::Matrix3d Y_2d;
                                    for (int si = 0; si < 3; ++si)
                                        for (int sm = 0; sm < 3; ++sm)
                                            Y_2d(si, sm) = pos_cache[si*3+sm][d];
                                    Eigen::Matrix3d X_2d = B_qi_inv_2d * Y_2d * B_qm_invT_2d;

                                    double a1=X_2d(0,0), a2=X_2d(0,1);
                                    double a3=X_2d(1,0), a4=X_2d(1,1);
                                    double a5=X_2d(0,2), a6=X_2d(1,2);
                                    double a7=X_2d(2,0), a8=X_2d(2,1);

                                    // Early skip: if qm-dependent coefficients are tiny,
                                    // no meaningful critical point w.r.t. qm exists
                                    double qm_dep = std::abs(a1) + std::abs(a2)
                                                  + std::abs(a3) + std::abs(a4)
                                                  + std::abs(a7) + std::abs(a8);
                                    if (qm_dep < 1e-12) continue;

                                    double pa[9];
                                    build_symbolic_poly8(a1,a2,a3,a4,a5,a6,a7,a8, pa);

                                    FixedRoots tm_roots;
                                    solve_poly_in_interval(pa, 8, tm_lo_pre, tm_hi_pre, tm_roots, 16);

                                    std::vector<Eigen::Vector3d> triplet_candidates;
                                    triplet_candidates.reserve(
                                        static_cast<std::size_t>(tm_roots.size()) * 3);

                                    for (double tm_r : tm_roots) {
                                        double qm_cand;
                                        if (!half_angle_to_q(tm_r, lo_m, hi_m, qm_cand))
                                            continue;

                                        double tm2 = tm_r*tm_r;
                                        double den_m = 1.0 + tm2;
                                        double cm_v = (1.0 - tm2)/den_m;
                                        double sm_v = 2.0*tm_r/den_m;

                                        double P = a3*cm_v + a4*sm_v + a6;
                                        double Q = a1*cm_v + a2*sm_v + a5;
                                        double qi_cand = std::atan2(P, Q);

                                        double qi_tries[3] = {qi_cand, qi_cand+M_PI, qi_cand-M_PI};
                                        bool accepted = false;
                                        for (double qi_try : qi_tries) {
                                            for (double shift : kShifts) {
                                                double qi_test = qi_try + shift;
                                                if (qi_test >= eff_lo - 1e-10 &&
                                                    qi_test <= eff_hi + 1e-10) {
                                                    qi_test = std::max(eff_lo,
                                                              std::min(eff_hi, qi_test));
                                                    double qj_test = C - qi_test;
                                                    if (qj_test < lo_j - 1e-10 ||
                                                        qj_test > hi_j + 1e-10) continue;
                                                    qj_test = std::max(lo_j,
                                                              std::min(hi_j, qj_test));
                                                    triplet_candidates.emplace_back(
                                                        qi_test, qj_test, qm_cand);
                                                    accepted = true;
                                                    break;
                                                }
                                            }
                                            if (accepted) break;
                                        }
                                    }

                                    if (!triplet_candidates.empty()) {
                                        const std::size_t raw_cnt = triplet_candidates.size();
                                        const bool do_dedup_t = dedup_rt.decide(raw_cnt);
                                        if (do_dedup_t) {
                                            std::sort(triplet_candidates.begin(),
                                                      triplet_candidates.end(),
                                                      [](const Eigen::Vector3d& a,
                                                         const Eigen::Vector3d& b) {
                                                          if (std::abs(a[2]-b[2])>=1e-10) return a[2]<b[2];
                                                          if (std::abs(a[0]-b[0])>=1e-10) return a[0]<b[0];
                                                          return a[1] < b[1];
                                                      });
                                            triplet_candidates.erase(
                                                std::unique(triplet_candidates.begin(),
                                                            triplet_candidates.end(),
                                                            [](const Eigen::Vector3d& a,
                                                               const Eigen::Vector3d& b) {
                                                                return std::abs(a[0]-b[0])<1e-10 &&
                                                                       std::abs(a[1]-b[1])<1e-10 &&
                                                                       std::abs(a[2]-b[2])<1e-10;
                                                            }),
                                                triplet_candidates.end());
                                            dedup_rt.record_applied(raw_cnt,
                                                                    triplet_candidates.size());
                                        } else {
                                            dedup_rt.record_skipped(raw_cnt);
                                        }

                                        for (const auto& cand : triplet_candidates) {
                                            q[pi] = cand[0];
                                            q[pj] = cand[1];
                                            q[pm] = cand[2];
                                            eval_and_update_from(robot, q,
                                                            std::min({pi, pj, pm}),
                                                            n_sub, inv_n,
                                                            map, n_act,
                                                            link_seg_ext, ws);
                                            ++fk_calls;
                                            ++found;
                                        }
                                    }
                                }
                            }
                            goto next_bg_2_5b;
                        }

                        Eigen::Vector3d pos_V_cache_2d[9], pos_V1_cache_2d[9];
                        bool valid = true;
                        {
                            int row = 0;
                            for (int si = 0; si < 3 && valid; ++si) {
                                q[pi] = qi_vals[si];
                                q[pj] = C - qi_vals[si];
                                for (int sm = 0; sm < 3 && valid; ++sm) {
                                    q[pm] = qm_vals[sm];
                                    ws.compute_from(robot, q, j_first_2d);
                                    ++fk_calls;
                                    if (V + 1 >= ws.np) { valid = false; break; }
                                    pos_V_cache_2d[row] = ws.pos(V);
                                    pos_V1_cache_2d[row] = ws.pos(V + 1);
                                    ++row;
                                }
                            }
                        }
                        if (!valid) goto next_bg_2_5b;

                        for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                            int eval_link = (eval_idx == 0) ? V + 1 : V;
                            if (eval_link < 1 || eval_link >= n + 2) continue;
                            const Eigen::Vector3d* pos_cache =
                                (eval_idx == 0) ? pos_V1_cache_2d : pos_V_cache_2d;

                            for (int d = 0; d < 3; ++d) {
                                if (aa_prefix && !aa_axis.axis_can_improve[d]) continue;

                                // H6: Kronecker coefficient extraction
                                Eigen::Matrix3d Y_2d;
                                for (int si = 0; si < 3; ++si)
                                    for (int sm = 0; sm < 3; ++sm)
                                        Y_2d(si, sm) = pos_cache[si*3+sm][d];
                                Eigen::Matrix3d X_2d = B_qi_inv_2d * Y_2d * B_qm_invT_2d;

                                double a1=X_2d(0,0), a2=X_2d(0,1);
                                double a3=X_2d(1,0), a4=X_2d(1,1);
                                double a5=X_2d(0,2), a6=X_2d(1,2);
                                double a7=X_2d(2,0), a8=X_2d(2,1);

                                // Early skip: if qm-dependent coefficients are tiny,
                                // no meaningful critical point w.r.t. qm exists
                                double qm_dep = std::abs(a1) + std::abs(a2)
                                              + std::abs(a3) + std::abs(a4)
                                              + std::abs(a7) + std::abs(a8);
                                if (qm_dep < 1e-12) continue;

                                double pa[9];
                                build_symbolic_poly8(a1,a2,a3,a4,a5,a6,a7,a8, pa);

                                FixedRoots tm_roots;
                                solve_poly_in_interval(pa, 8,
                                                       tm_lo_pre, tm_hi_pre, tm_roots, 16);

                                std::vector<Eigen::Vector3d> triplet_candidates;
                                triplet_candidates.reserve(static_cast<std::size_t>(tm_roots.size()) * 3);
                                std::size_t raw_triplets = 0;

                                for (double tm_r : tm_roots) {
                                    double qm_cand;
                                    if (!half_angle_to_q(tm_r, lo_m, hi_m, qm_cand))
                                        continue;

                                    double tm2 = tm_r*tm_r;
                                    double den_m = 1.0 + tm2;
                                    double cm_v = (1.0 - tm2)/den_m;
                                    double sm_v = 2.0*tm_r/den_m;

                                    double P = a3*cm_v + a4*sm_v + a6;
                                    double Q = a1*cm_v + a2*sm_v + a5;
                                    double qi_cand = std::atan2(P, Q);

                                    bool accepted_this_root = false;

                                    double qi_tries[3] = {qi_cand, qi_cand+M_PI, qi_cand-M_PI};
                                    for (double qi_try : qi_tries) {
                                        for (double shift : kShifts) {
                                            double qi_test = qi_try + shift;
                                            if (qi_test >= eff_lo - 1e-10 &&
                                                qi_test <= eff_hi + 1e-10) {
                                                qi_test = std::max(eff_lo,
                                                          std::min(eff_hi, qi_test));
                                                double qj_test = C - qi_test;
                                                if (qj_test < lo_j - 1e-10 ||
                                                    qj_test > hi_j + 1e-10)
                                                    continue;
                                                qj_test = std::max(lo_j,
                                                          std::min(hi_j, qj_test));

                                                ++raw_triplets;

                                                triplet_candidates.emplace_back(
                                                    qi_test, qj_test, qm_cand);
                                                accepted_this_root = true;
                                                break;
                                            }
                                        }
                                        if (accepted_this_root) break;
                                    }
                                }

                                if (!triplet_candidates.empty()) {
                                    const bool do_dedup_triplets = dedup_rt.decide(raw_triplets);
                                    if (do_dedup_triplets) {
                                        std::sort(triplet_candidates.begin(), triplet_candidates.end(),
                                                  [](const Eigen::Vector3d& a,
                                                     const Eigen::Vector3d& b) {
                                                      if (std::abs(a[2] - b[2]) >= 1e-10)
                                                          return a[2] < b[2];
                                                      if (std::abs(a[0] - b[0]) >= 1e-10)
                                                          return a[0] < b[0];
                                                      return a[1] < b[1];
                                                  });
                                        triplet_candidates.erase(
                                            std::unique(triplet_candidates.begin(),
                                                        triplet_candidates.end(),
                                                        [](const Eigen::Vector3d& a,
                                                           const Eigen::Vector3d& b) {
                                                            return std::abs(a[0] - b[0]) < 1e-10 &&
                                                                   std::abs(a[1] - b[1]) < 1e-10 &&
                                                                   std::abs(a[2] - b[2]) < 1e-10;
                                                        }),
                                            triplet_candidates.end());
                                        dedup_rt.record_applied(raw_triplets,
                                                                triplet_candidates.size());
                                    } else {
                                        dedup_rt.record_skipped(raw_triplets);
                                    }

                                    for (const auto& cand : triplet_candidates) {
                                        q[pi] = cand[0];
                                        q[pj] = cand[1];
                                        q[pm] = cand[2];
                                        eval_and_update_from(robot, q,
                                                        std::min({pi, pj, pm}),
                                                        n_sub, inv_n,
                                                        map, n_act,
                                                        link_seg_ext, ws);
                                        ++fk_calls;
                                        ++found;
                                    }
                                }
                            }
                        }
                        next_bg_2_5b:;

                        for (int oi = n_bg - 1; oi >= 0; --oi) {
                            if (++bg_cnt[oi] < other_bg_sz2[oi]) break;
                            bg_cnt[oi] = 0;
                        }
                    }
                }
            }
        }
    }

    if (n_found) *n_found = found;
    if (n_fk_calls) *n_fk_calls = fk_calls;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Phase 3+: Improved interior solver (multi-start + pair coupling)
// ══════════════════════════════════════════════════════════════════════════�?

static void solve_interior_improved(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    int max_free, int max_sweeps,
    int n_restarts,
    bool pair_coupling,
    const std::vector<std::pair<int,int>>& priority_pairs,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    const AnalyticalCriticalConfig& config,
    DedupMetrics* dedup_metrics,
    const char* skip_link,
    const AAMatrix4* aa_prefix,
    int ci_begin,
    int ci_end,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;
    DedupAdaptiveRuntime dedup_rt{&config, dedup_metrics};
    FKWorkspace ws;
    ws.resize(n + 2);

    for (int ci = ci_begin; ci < ci_end; ++ci) {
        if (skip_link && skip_link[ci]) continue;
        int V = map[ci];
        AAAxisImproveSummary aa_axis;
        if (aa_prefix) {
            aa_axis = aa_link_axis_improve_summary(
                aa_prefix, V, n_sub, inv_n, link_seg_ext[ci]);
            if (!aa_axis.can_improve) continue;
        }
        int nj = std::min(V + 1, n);
        if (nj < 3 || nj > max_free) continue;

        std::vector<std::pair<int,int>> link_pairs;
        for (auto& [a, b] : priority_pairs) {
            if (a < nj && b < nj) link_pairs.push_back({a, b});
        }

        for (int face = 0; face < 6; ++face) {
            int dim = face / 2;
            bool is_min = (face % 2 == 0);
            if (aa_prefix && !aa_axis.axis_can_improve[dim]) continue;

            std::vector<Eigen::VectorXd> starts;

            {
                Eigen::VectorXd q0 = link_seg_ext[ci][0].configs[face];
                if (q0.size() == n) starts.push_back(q0);
            }

            for (int ri = 0; ri < n_restarts &&
                 ri < static_cast<int>(link_pairs.size()); ++ri) {
                auto [pa, pb] = link_pairs[ri % link_pairs.size()];
                Eigen::VectorXd q_seed = starts.empty()
                    ? Eigen::VectorXd::Zero(n) : starts[0];

                double sum_ab = q_seed[pa] + q_seed[pb];
                double nearest_k = std::round(sum_ab / HALF_PI) * HALF_PI;
                double delta = (nearest_k - sum_ab) * 0.5;
                double new_a = std::max(intervals[pa].lo,
                               std::min(intervals[pa].hi, q_seed[pa] + delta));
                double new_b = std::max(intervals[pb].lo,
                               std::min(intervals[pb].hi, q_seed[pb] + delta));
                q_seed[pa] = new_a;
                q_seed[pb] = new_b;
                starts.push_back(q_seed);
            }

            if (static_cast<int>(starts.size()) < n_restarts + 2) {
                Eigen::VectorXd q_kpi2(n);
                for (int j = 0; j < n; ++j) {
                    double mid = intervals[j].mid();
                    double best = mid;
                    double best_dist = 1e30;
                    for (int kk = -20; kk <= 20; ++kk) {
                        double a = kk * HALF_PI;
                        if (a >= intervals[j].lo && a <= intervals[j].hi) {
                            double dist = std::abs(a - mid);
                            if (dist < best_dist) { best_dist = dist; best = a; }
                        }
                    }
                    q_kpi2[j] = best;
                }
                starts.push_back(q_kpi2);
            }

            for (auto& q_start : starts) {
                Eigen::VectorXd q = q_start;

                for (int sweep = 0; sweep < max_sweeps; ++sweep) {
                    bool improved = false;

                    for (int j = 0; j < nj; ++j) {
                        double lo_j = intervals[j].lo, hi_j = intervals[j].hi;
                        if (hi_j - lo_j < 1e-12) continue;

                        ws.set_identity();
                        for (int k = 0; k < j; ++k)
                            ws.compute_joint(robot, q, k);
                        // Ensure full FK chain is valid: tf[j+1..n+1]
                        // are needed by pair-coupling when partner > j
                        // and no candidates were evaluated.
                        ws.compute_from(robot, q, j);

                        // ── Phase F: direct coefficient extraction for P3 ──
                        if (config.enable_p3_direct_coeff) {
                            const Eigen::Matrix4d& prefix = ws.tf[j];
                            const auto& dh_j = robot.dh_params()[j];
                            bool has_prox = (V >= 1);

                            Eigen::Vector3d suf_dist = compute_suffix_pos(robot, q, j, V + 1);
                            Coeff1D c_dist = extract_1d_coefficients(prefix, suf_dist, dh_j);

                            Coeff1D c_prox{};
                            if (has_prox) {
                                Eigen::Vector3d suf_prox = compute_suffix_pos(robot, q, j, V);
                                c_prox = extract_1d_coefficients(prefix, suf_prox, dh_j);
                            }

                            for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                                int eval_link = (eval_idx == 0) ? V + 1 : V;
                                if (eval_link < 1) continue;
                                const Coeff1D& cc = (eval_idx == 0) ? c_dist : c_prox;

                                double alpha_c = cc.alpha[dim];
                                double beta_c  = cc.beta[dim];
                                if (std::abs(alpha_c) < 1e-15 &&
                                    std::abs(beta_c) < 1e-15)
                                    continue;

                                std::vector<double> q_candidates;
                                q_candidates.reserve(6);

                                double q_star = std::atan2(beta_c, alpha_c);
                                if (is_min) q_star += M_PI;

                                for (double shift : kShifts) {
                                    double qtest = q_star + shift;
                                    if (qtest >= lo_j - 1e-10 &&
                                        qtest <= hi_j + 1e-10) {
                                        qtest = std::max(lo_j, std::min(hi_j, qtest));
                                        q_candidates.push_back(qtest);
                                    }
                                }
                                double q_star2 = q_star + M_PI;
                                for (double shift : kShifts) {
                                    double qtest = q_star2 + shift;
                                    if (qtest >= lo_j - 1e-10 &&
                                        qtest <= hi_j + 1e-10) {
                                        qtest = std::max(lo_j, std::min(hi_j, qtest));
                                        q_candidates.push_back(qtest);
                                    }
                                }

                                if (!q_candidates.empty()) {
                                    const std::size_t raw_count = q_candidates.size();
                                    const bool do_dedup = dedup_rt.decide(raw_count);
                                    if (do_dedup) {
                                        std::sort(q_candidates.begin(), q_candidates.end());
                                        q_candidates.erase(
                                            std::unique(q_candidates.begin(), q_candidates.end(),
                                                        [](double a, double b) {
                                                            return std::abs(a - b) < 1e-10;
                                                        }),
                                            q_candidates.end());
                                        dedup_rt.record_applied(raw_count, q_candidates.size());
                                    } else {
                                        dedup_rt.record_skipped(raw_count);
                                    }

                                    for (double qtest : q_candidates) {
                                        q[j] = qtest;
                                        eval_and_update_from(robot, q, j, n_sub, inv_n,
                                                        map, n_act, link_seg_ext, ws);
                                        ++fk_calls;
                                        ++found;
                                        improved = true;
                                    }
                                }
                            }
                        } else {
                            // ── Original QR path ──
                            double mid_j = 0.5 * (lo_j + hi_j);
                            double qvals[3] = { lo_j, mid_j, hi_j };

                            Eigen::Matrix3d A_trig;
                            for (int si = 0; si < 3; ++si) {
                                A_trig(si, 0) = std::cos(qvals[si]);
                                A_trig(si, 1) = std::sin(qvals[si]);
                                A_trig(si, 2) = 1.0;
                            }
                            auto qr_A_imp = A_trig.colPivHouseholderQr();

                            Eigen::Vector3d pts_dist[3], pts_prox[3];
                            bool has_prox = (V >= 1);
                            bool ok = true;
                            for (int si = 0; si < 3; ++si) {
                                double q_save = q[j];
                                q[j] = qvals[si];
                                ws.compute_from(robot, q, j);
                                ++fk_calls;
                                if (V + 1 >= ws.np) {
                                    q[j] = q_save; ok = false; break;
                                }
                                pts_dist[si] = ws.pos(V + 1);
                                if (has_prox) pts_prox[si] = ws.pos(V);
                                q[j] = q_save;
                            }
                            if (!ok) continue;

                            for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                                int eval_link = (eval_idx == 0) ? V + 1 : V;
                                if (eval_link < 1) continue;
                                const Eigen::Vector3d* pts =
                                    (eval_idx == 0) ? pts_dist : pts_prox;

                                std::vector<double> q_candidates;
                                q_candidates.reserve(6);

                                Eigen::Vector3d b_vec;
                                b_vec << pts[0][dim], pts[1][dim], pts[2][dim];
                                Eigen::Vector3d coeff = qr_A_imp.solve(b_vec);
                                double alpha_c = coeff[0], beta_c = coeff[1];
                                if (std::abs(alpha_c) < 1e-15 &&
                                    std::abs(beta_c) < 1e-15)
                                    continue;

                                double q_star = std::atan2(beta_c, alpha_c);
                                if (is_min) q_star += M_PI;

                                for (double shift : kShifts) {
                                    double qtest = q_star + shift;
                                    if (qtest >= lo_j - 1e-10 &&
                                        qtest <= hi_j + 1e-10) {
                                        qtest = std::max(lo_j, std::min(hi_j, qtest));
                                        q_candidates.push_back(qtest);
                                    }
                                }
                                double q_star2 = q_star + M_PI;
                                for (double shift : kShifts) {
                                    double qtest = q_star2 + shift;
                                    if (qtest >= lo_j - 1e-10 &&
                                        qtest <= hi_j + 1e-10) {
                                        qtest = std::max(lo_j, std::min(hi_j, qtest));
                                        q_candidates.push_back(qtest);
                                    }
                                }

                                if (!q_candidates.empty()) {
                                    const std::size_t raw_count = q_candidates.size();
                                    const bool do_dedup = dedup_rt.decide(raw_count);
                                    if (do_dedup) {
                                        std::sort(q_candidates.begin(), q_candidates.end());
                                        q_candidates.erase(
                                            std::unique(q_candidates.begin(), q_candidates.end(),
                                                        [](double a, double b) {
                                                            return std::abs(a - b) < 1e-10;
                                                        }),
                                            q_candidates.end());
                                        dedup_rt.record_applied(raw_count, q_candidates.size());
                                    } else {
                                        dedup_rt.record_skipped(raw_count);
                                    }

                                    for (double qtest : q_candidates) {
                                        q[j] = qtest;
                                        eval_and_update_from(robot, q, j, n_sub, inv_n,
                                                        map, n_act, link_seg_ext, ws);
                                        ++fk_calls;
                                        ++found;
                                        improved = true;
                                    }
                                }
                            }
                        }

                        // Pair-coupled update
                        if (pair_coupling) {
                            for (auto& [pa, pb] : link_pairs) {
                                int partner = -1;
                                if (pa == j) partner = pb;
                                else if (pb == j) partner = pa;
                                if (partner < 0 || partner >= nj) continue;

                                double sum_cur = q[j] + q[partner];
                                double nearest_c =
                                    std::round(sum_cur / HALF_PI) * HALF_PI;
                                if (std::abs(sum_cur - nearest_c) > 0.3) continue;

                                double new_partner = nearest_c - q[j];
                                if (new_partner >= intervals[partner].lo - 1e-10 &&
                                    new_partner <= intervals[partner].hi + 1e-10) {
                                    new_partner = std::max(intervals[partner].lo,
                                                  std::min(intervals[partner].hi,
                                                           new_partner));
                                    q[partner] = new_partner;
                                    eval_and_update_from(robot, q, partner,
                                                    n_sub, inv_n,
                                                    map, n_act, link_seg_ext, ws);
                                    ++fk_calls; ++found;
                                    improved = true;
                                }
                            }
                        }
                    }

                    if (!improved) break;
                }
            }
        }
    }

    if (n_found) *n_found = found;
    if (n_fk_calls) *n_fk_calls = fk_calls;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  derive_aabb_critical_analytical �?full analytical pipeline
// ══════════════════════════════════════════════════════════════════════════�?

void derive_aabb_critical_analytical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const AnalyticalCriticalConfig& config,
    float* out_aabb,
    AnalyticalCriticalStats* out_stats,
    float* out_endpoint_aabb)
{
    assert(n_sub >= 1);
    const int n       = robot.n_joints();
    const int n_act   = robot.n_active_links();
    const int* map    = robot.active_link_map();
    const double* rad = robot.active_link_radii();

    const float inv_n = 1.0f / static_cast<float>(n_sub);

    // Initialize endpoint interval AABBs
    if (out_endpoint_aabb) {
        for (int ci = 0; ci < n_act; ++ci) {
            for (int s = 0; s <= n_sub; ++s) {
                float* fa = out_endpoint_aabb + (ci * (n_sub + 1) + s) * 6;
                fa[0] = fa[1] = fa[2] =  1e30f;
                fa[3] = fa[4] = fa[5] = -1e30f;
            }
        }
    }

    // Per active-link, per sub-segment extremes tracker
    std::vector<std::vector<LinkExtremes>> link_seg_ext(n_act);
    for (int ci = 0; ci < n_act; ++ci) {
        link_seg_ext[ci].resize(n_sub);
        for (int s = 0; s < n_sub; ++s)
            link_seg_ext[ci][s].init(n);
    }

    AnalyticalCriticalStats stats{};
    DedupMetrics dedup_p1{}, dedup_p2{}, dedup_p25{}, dedup_p3{};

    // ──── Phase 0: kπ/2 baseline enumeration ─────────────────────────
    if (config.keep_kpi2_baseline) {
        std::vector<std::vector<double>> per_joint(n);
        for (int j = 0; j < n; ++j)
            per_joint[j] = crit_angles(intervals[j].lo, intervals[j].hi);

        int max_V = 0;
        for (int ci = 0; ci < n_act; ++ci)
            max_V = std::max(max_V, map[ci]);
        int n_joints_needed = std::min(max_V + 1, n);

        auto csets = build_csets(per_joint, intervals, n_joints_needed);

        Eigen::VectorXd q(n);
        for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

        FKWorkspace ws;
        ws.resize(n + 2);
        ws.set_identity();
        crit_enum_fk(robot, csets, q, 0, n_joints_needed, ws, [&]() {
            ++stats.n_phase0_vertices;
            for (int ci = 0; ci < n_act; ++ci) {
                int V = map[ci];
                if (V + 1 >= ws.np) continue;
                const Eigen::Vector3d p_prox = ws.pos(V);
                const Eigen::Vector3d p_dist = ws.pos(V + 1);
                for (int s = 0; s < n_sub; ++s) {
                    float t0 = s * inv_n;
                    float t1 = (s + 1) * inv_n;
                    Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                    Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);
                    link_seg_ext[ci][s].update(s0, q);
                    link_seg_ext[ci][s].update(s1, q);

                    if (out_endpoint_aabb) {
                        float* fa0 = out_endpoint_aabb + (ci * (n_sub + 1) + s) * 6;
                        float* fa1 = out_endpoint_aabb + (ci * (n_sub + 1) + s + 1) * 6;
                        for (int d = 0; d < 3; ++d) {
                            float v0 = static_cast<float>(s0[d]);
                            float v1 = static_cast<float>(s1[d]);
                            if (v0 < fa0[d])     fa0[d]     = v0;
                            if (v0 > fa0[d + 3]) fa0[d + 3] = v0;
                            if (v1 < fa1[d])     fa1[d]     = v1;
                            if (v1 > fa1[d + 3]) fa1[d + 3] = v1;
                        }
                    }
                }
            }
        });
    }

    // ──── AA Pruning: compute skip_link[] after Phase 0 ──────────────
    std::vector<char> skip_link_vec(n_act, false);
    char* skip_link = nullptr;
    AAFKWorkspace aa_ws;
    bool aa_ready = false;
    if (config.enable_aa_pruning) {
        aa_ws.compute(robot, intervals.data());
        aa_ready = true;
        int pruned_segments = 0;
        const int pruned_links = refresh_phase_skip_links_from_aa(
            aa_ws.prefix, map, n_act, n_sub, inv_n, link_seg_ext,
            skip_link_vec, &pruned_segments);
        stats.n_aa_pruned_links = pruned_links;
        stats.n_aa_pruned_segments += static_cast<int64_t>(pruned_segments);
        stats.n_aa_pruned_phase_checks += n_act;
        skip_link = skip_link_vec.data();
    }

    auto refresh_skip_for_phase = [&](const std::vector<std::vector<LinkExtremes>>& ext_ref) -> int {
        if (!aa_ready) return -1;
        int pruned_segments = 0;
        const int pruned_links = refresh_phase_skip_links_from_aa(
            aa_ws.prefix, map, n_act, n_sub, inv_n, ext_ref,
            skip_link_vec, &pruned_segments);
        stats.n_aa_pruned_segments += static_cast<int64_t>(pruned_segments);
        stats.n_aa_pruned_phase_checks += n_act;
        skip_link = skip_link_vec.data();
        return pruned_links;
    };

    // ──── Phase 1: 1D edge critical points ───────────────────────────
    if (config.enable_edge_solve) {
        const int pruned_links = refresh_skip_for_phase(link_seg_ext);
        if (pruned_links == n_act) {
            stats.n_phase1_edges = 0;
            stats.n_phase1_fk_calls = 0;
        } else {
        int edge_found = 0, edge_fk = 0;
        const int n_threads = resolve_parallel_threads(config, n_act);
        if (n_threads <= 1) {
            solve_edges(robot, intervals, n_sub, inv_n,
                        map, n_act, link_seg_ext,
                        config,
                        &dedup_p1,
                        skip_link,
                        aa_ready ? aa_ws.prefix : nullptr,
                        0, n_act,
                        &edge_found, &edge_fk);
        } else {
            std::vector<std::thread> workers;
            std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
            std::vector<DedupMetrics> part_dedup(n_threads);
            workers.reserve(n_threads);

            int ci0 = 0;
            const int base = n_act / n_threads;
            const int rem = n_act % n_threads;
            for (int t = 0; t < n_threads; ++t) {
                const int span = base + (t < rem ? 1 : 0);
                const int begin = ci0;
                const int end = begin + span;
                ci0 = end;
                workers.emplace_back([&, t, begin, end]() {
                    solve_edges(robot, intervals, n_sub, inv_n,
                                map, n_act, link_seg_ext,
                                config,
                                &part_dedup[t],
                                skip_link,
                                aa_ready ? aa_ws.prefix : nullptr,
                                begin, end,
                                &part_found[t], &part_fk[t]);
                });
            }
            for (auto& th : workers) th.join();
            for (int t = 0; t < n_threads; ++t) {
                edge_found += part_found[t];
                edge_fk += part_fk[t];
                merge_dedup_metrics(dedup_p1, part_dedup[t]);
            }
        }
        stats.n_phase1_edges = edge_found;
        stats.n_phase1_fk_calls = edge_fk;
        }
    }

    // ──── dual_phase3 checkpoint ─────────────────────────────────────
    std::vector<std::vector<LinkExtremes>> ext_checkpoint_01;
    if (config.dual_phase3 && config.enable_interior_solve) {
        ext_checkpoint_01 = link_seg_ext;
    }

    // ──── Phase 2: 2D face critical points ───────────────────────────
    if (config.enable_face_solve) {
        const int pruned_links = refresh_skip_for_phase(link_seg_ext);
        if (pruned_links == n_act) {
            stats.n_phase2_faces = 0;
            stats.n_phase2_fk_calls = 0;
        } else {
        std::vector<std::pair<int,int>> face_pairs;
        if (config.face_all_pairs) {
            for (int i = 0; i < n; ++i)
                for (int j = i + 1; j < n; ++j)
                    face_pairs.push_back({i, j});
        } else {
            face_pairs = robot.coupled_pairs();
            for (int i = 0; i < n - 1; ++i) {
                std::pair<int,int> p = {i, i+1};
                bool duplicate = false;
                for (auto& fp : face_pairs)
                    if (fp == p) { duplicate = true; break; }
                if (!duplicate) face_pairs.push_back(p);
            }
        }

        int face_found = 0, face_fk = 0;
        const int n_threads = resolve_parallel_threads(config, n_act);
        if (n_threads <= 1) {
            solve_faces(robot, intervals, n_sub, inv_n,
                        map, n_act, face_pairs, link_seg_ext,
                        config,
                        &dedup_p2,
                        skip_link,
                        aa_ready ? aa_ws.prefix : nullptr,
                        0, n_act,
                        &face_found, &face_fk);
        } else {
            std::vector<std::thread> workers;
            std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
            std::vector<DedupMetrics> part_dedup(n_threads);
            workers.reserve(n_threads);

            int ci0 = 0;
            const int base = n_act / n_threads;
            const int rem = n_act % n_threads;
            for (int t = 0; t < n_threads; ++t) {
                const int span = base + (t < rem ? 1 : 0);
                const int begin = ci0;
                const int end = begin + span;
                ci0 = end;
                workers.emplace_back([&, t, begin, end]() {
                    solve_faces(robot, intervals, n_sub, inv_n,
                               map, n_act, face_pairs, link_seg_ext,
                               config,
                               &part_dedup[t],
                               skip_link,
                               aa_ready ? aa_ws.prefix : nullptr,
                               begin, end,
                               &part_found[t], &part_fk[t]);
                });
            }
            for (auto& th : workers) th.join();
            for (int t = 0; t < n_threads; ++t) {
                face_found += part_found[t];
                face_fk += part_fk[t];
                merge_dedup_metrics(dedup_p2, part_dedup[t]);
            }
        }
        stats.n_phase2_faces = face_found;
        stats.n_phase2_fk_calls = face_fk;
        }
    }

    // ──── Phase 2.5: Pair-constrained solvers ────────────────────────
    {
        const int pruned_links = refresh_skip_for_phase(link_seg_ext);
        if (pruned_links == n_act) {
            stats.n_phase25a_pair1d = 0;
            stats.n_phase25b_pair2d = 0;
            stats.n_phase25_fk_calls = 0;
        } else {
        std::vector<std::pair<int,int>> pc_pairs;
        if (config.pair_all_pairs) {
            for (int i = 0; i < n; ++i)
                for (int j = i + 1; j < n; ++j)
                    pc_pairs.push_back({i, j});
        } else {
            pc_pairs = robot.coupled_pairs();
            for (int i = 0; i < n - 1; ++i) {
                std::pair<int,int> p = {i, i+1};
                bool dup = false;
                for (auto& fp : pc_pairs) if (fp == p) { dup = true; break; }
                if (!dup) pc_pairs.push_back(p);
            }
            for (int i = 0; i < n; ++i) {
                for (int j = i + 2; j < n; ++j) {
                    std::pair<int,int> p = {i, j};
                    bool dup = false;
                    for (auto& fp : pc_pairs) if (fp == p) { dup = true; break; }
                    if (!dup) pc_pairs.push_back(p);
                }
            }
        }

        int p25_fk = 0;
        const int n_threads = resolve_parallel_threads(config, n_act);

        if (config.enable_pair_1d) {
            int p1d = 0, p1d_fk = 0;
            if (n_threads <= 1) {
                solve_pair_constrained_1d(robot, intervals, n_sub, inv_n,
                                          map, n_act, pc_pairs,
                                          config.pair_max_bg,
                                          config.pair_kpi2_backgrounds,
                                          link_seg_ext,
                                          config,
                                          &dedup_p25,
                                          skip_link,
                                          aa_ready ? aa_ws.prefix : nullptr,
                                          0, n_act,
                                          &p1d, &p1d_fk);
            } else {
                std::vector<std::thread> workers;
                std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
                std::vector<DedupMetrics> part_dedup(n_threads);
                workers.reserve(n_threads);

                int ci0 = 0;
                const int base = n_act / n_threads;
                const int rem = n_act % n_threads;
                for (int t = 0; t < n_threads; ++t) {
                    const int span = base + (t < rem ? 1 : 0);
                    const int begin = ci0;
                    const int end = begin + span;
                    ci0 = end;
                    workers.emplace_back([&, t, begin, end]() {
                        solve_pair_constrained_1d(robot, intervals, n_sub, inv_n,
                                                  map, n_act, pc_pairs,
                                                  config.pair_max_bg,
                                                  config.pair_kpi2_backgrounds,
                                                  link_seg_ext,
                                                  config,
                                                  &part_dedup[t],
                                                  skip_link,
                                                  aa_ready ? aa_ws.prefix : nullptr,
                                                  begin, end,
                                                  &part_found[t], &part_fk[t]);
                    });
                }
                for (auto& th : workers) th.join();
                for (int t = 0; t < n_threads; ++t) {
                    p1d += part_found[t];
                    p1d_fk += part_fk[t];
                    merge_dedup_metrics(dedup_p25, part_dedup[t]);
                }
            }
            stats.n_phase25a_pair1d = p1d;
            p25_fk += p1d_fk;
        }

        if (config.enable_pair_2d) {
            int p2d = 0, p2d_fk = 0;
            if (n_threads <= 1) {
                solve_pair_constrained_2d(robot, intervals, n_sub, inv_n,
                                          map, n_act, pc_pairs,
                                          config.pair_max_bg,
                                          config.pair_kpi2_backgrounds,
                                          link_seg_ext,
                                          config,
                                          &dedup_p25,
                                          skip_link,
                                          aa_ready ? aa_ws.prefix : nullptr,
                                          0, n_act,
                                          &p2d, &p2d_fk);
            } else {
                std::vector<std::thread> workers;
                std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
                std::vector<DedupMetrics> part_dedup(n_threads);
                workers.reserve(n_threads);

                int ci0 = 0;
                const int base = n_act / n_threads;
                const int rem = n_act % n_threads;
                for (int t = 0; t < n_threads; ++t) {
                    const int span = base + (t < rem ? 1 : 0);
                    const int begin = ci0;
                    const int end = begin + span;
                    ci0 = end;
                    workers.emplace_back([&, t, begin, end]() {
                        solve_pair_constrained_2d(robot, intervals, n_sub, inv_n,
                                                  map, n_act, pc_pairs,
                                                  config.pair_max_bg,
                                                  config.pair_kpi2_backgrounds,
                                                  link_seg_ext,
                                                  config,
                                                  &part_dedup[t],
                                                  skip_link,
                                                  aa_ready ? aa_ws.prefix : nullptr,
                                                  begin, end,
                                                  &part_found[t], &part_fk[t]);
                    });
                }
                for (auto& th : workers) th.join();
                for (int t = 0; t < n_threads; ++t) {
                    p2d += part_found[t];
                    p2d_fk += part_fk[t];
                    merge_dedup_metrics(dedup_p25, part_dedup[t]);
                }
            }
            stats.n_phase25b_pair2d = p2d;
            p25_fk += p2d_fk;
        }

        stats.n_phase25_fk_calls = p25_fk;
        }
    }

    // ──── Phase 3+: Higher-dimensional interior ──────────────────────
    if (config.enable_interior_solve) {
        const int pruned_links = refresh_skip_for_phase(link_seg_ext);
        if (pruned_links == n_act) {
            stats.n_phase3_interior = 0;
            stats.n_phase3_fk_calls = 0;
        } else {
        std::vector<std::pair<int,int>> int_pairs;
        {
            auto cp = robot.coupled_pairs();
            int_pairs.insert(int_pairs.end(), cp.begin(), cp.end());
            for (int i = 0; i < n - 1; ++i) {
                std::pair<int,int> p = {i, i+1};
                bool dup = false;
                for (auto& fp : int_pairs) if (fp == p) { dup = true; break; }
                if (!dup) int_pairs.push_back(p);
            }
            for (int i = 0; i < n - 2; i += 2) {
                std::pair<int,int> p = {i, i+2};
                bool dup = false;
                for (auto& fp : int_pairs) if (fp == p) { dup = true; break; }
                if (!dup) int_pairs.push_back(p);
            }
        }

        int int_found = 0, int_fk = 0;
        const int n_threads = resolve_parallel_threads(config, n_act);
        if (n_threads <= 1) {
            if (config.improved_interior) {
                solve_interior_improved(robot, intervals, n_sub, inv_n,
                                        map, n_act,
                                        config.interior_max_free,
                                        config.interior_max_sweeps,
                                        config.interior_n_restarts,
                                        config.interior_pair_coupling,
                                        int_pairs,
                                        link_seg_ext,
                                        config,
                                        &dedup_p3,
                                        skip_link,
                                        aa_ready ? aa_ws.prefix : nullptr,
                                        0, n_act,
                                        &int_found, &int_fk);
            } else {
                solve_interior(robot, intervals, n_sub, inv_n,
                               map, n_act,
                               config.interior_max_free,
                               config.interior_max_sweeps,
                               link_seg_ext,
                               config,
                               &dedup_p3,
                               skip_link,
                               aa_ready ? aa_ws.prefix : nullptr,
                               0, n_act,
                               &int_found, &int_fk);
            }
        } else {
            std::vector<std::thread> workers;
            std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
            std::vector<DedupMetrics> part_dedup(n_threads);
            workers.reserve(n_threads);

            int ci0 = 0;
            const int base = n_act / n_threads;
            const int rem = n_act % n_threads;
            for (int t = 0; t < n_threads; ++t) {
                const int span = base + (t < rem ? 1 : 0);
                const int begin = ci0;
                const int end = begin + span;
                ci0 = end;
                workers.emplace_back([&, t, begin, end]() {
                    if (config.improved_interior) {
                        solve_interior_improved(robot, intervals, n_sub, inv_n,
                                                map, n_act,
                                                config.interior_max_free,
                                                config.interior_max_sweeps,
                                                config.interior_n_restarts,
                                                config.interior_pair_coupling,
                                                int_pairs,
                                                link_seg_ext,
                                                config,
                                                &part_dedup[t],
                                                skip_link,
                                                aa_ready ? aa_ws.prefix : nullptr,
                                                begin, end,
                                                &part_found[t], &part_fk[t]);
                    } else {
                        solve_interior(robot, intervals, n_sub, inv_n,
                                       map, n_act,
                                       config.interior_max_free,
                                       config.interior_max_sweeps,
                                       link_seg_ext,
                                       config,
                                       &part_dedup[t],
                                       skip_link,
                                       aa_ready ? aa_ws.prefix : nullptr,
                                       begin, end,
                                       &part_found[t], &part_fk[t]);
                    }
                });
            }
            for (auto& th : workers) th.join();
            for (int t = 0; t < n_threads; ++t) {
                int_found += part_found[t];
                int_fk += part_fk[t];
                merge_dedup_metrics(dedup_p3, part_dedup[t]);
            }
        }
        stats.n_phase3_interior = int_found;
        stats.n_phase3_fk_calls = int_fk;

        // dual_phase3
        if (config.dual_phase3 && !ext_checkpoint_01.empty()) {
            const int pruned_links2 = refresh_skip_for_phase(ext_checkpoint_01);
            if (pruned_links2 != n_act) {
            int int_found2 = 0, int_fk2 = 0;
            if (n_threads <= 1) {
                if (config.improved_interior) {
                    solve_interior_improved(robot, intervals, n_sub, inv_n,
                                            map, n_act,
                                            config.interior_max_free,
                                            config.interior_max_sweeps,
                                            config.interior_n_restarts,
                                            config.interior_pair_coupling,
                                            int_pairs,
                                            ext_checkpoint_01,
                                            config,
                                            &dedup_p3,
                                            skip_link,
                                            aa_ready ? aa_ws.prefix : nullptr,
                                            0, n_act,
                                            &int_found2, &int_fk2);
                } else {
                    solve_interior(robot, intervals, n_sub, inv_n,
                                   map, n_act,
                                   config.interior_max_free,
                                   config.interior_max_sweeps,
                                   ext_checkpoint_01,
                                   config,
                                   &dedup_p3,
                                   skip_link,
                                   aa_ready ? aa_ws.prefix : nullptr,
                                   0, n_act,
                                   &int_found2, &int_fk2);
                }
            } else {
                std::vector<std::thread> workers;
                std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
                std::vector<DedupMetrics> part_dedup(n_threads);
                workers.reserve(n_threads);

                int ci0 = 0;
                const int base = n_act / n_threads;
                const int rem = n_act % n_threads;
                for (int t = 0; t < n_threads; ++t) {
                    const int span = base + (t < rem ? 1 : 0);
                    const int begin = ci0;
                    const int end = begin + span;
                    ci0 = end;
                    workers.emplace_back([&, t, begin, end]() {
                        if (config.improved_interior) {
                            solve_interior_improved(robot, intervals, n_sub, inv_n,
                                                    map, n_act,
                                                    config.interior_max_free,
                                                    config.interior_max_sweeps,
                                                    config.interior_n_restarts,
                                                    config.interior_pair_coupling,
                                                    int_pairs,
                                                    ext_checkpoint_01,
                                                    config,
                                                    &part_dedup[t],
                                                    skip_link,
                                                    aa_ready ? aa_ws.prefix : nullptr,
                                                    begin, end,
                                                    &part_found[t], &part_fk[t]);
                        } else {
                            solve_interior(robot, intervals, n_sub, inv_n,
                                           map, n_act,
                                           config.interior_max_free,
                                           config.interior_max_sweeps,
                                           ext_checkpoint_01,
                                           config,
                                           &part_dedup[t],
                                           skip_link,
                                           aa_ready ? aa_ws.prefix : nullptr,
                                           begin, end,
                                           &part_found[t], &part_fk[t]);
                        }
                    });
                }
                for (auto& th : workers) th.join();
                for (int t = 0; t < n_threads; ++t) {
                    int_found2 += part_found[t];
                    int_fk2 += part_fk[t];
                    merge_dedup_metrics(dedup_p3, part_dedup[t]);
                }
            }
            for (int ci = 0; ci < n_act; ++ci) {
                for (int s = 0; s < n_sub; ++s) {
                    auto& dst = link_seg_ext[ci][s];
                    const auto& src = ext_checkpoint_01[ci][s];
                    for (int d = 0; d < 3; ++d) {
                        if (src.vals[d * 2] < dst.vals[d * 2]) {
                            dst.vals[d * 2] = src.vals[d * 2];
                            dst.configs[d * 2] = src.configs[d * 2];
                        }
                        if (src.vals[d * 2 + 1] > dst.vals[d * 2 + 1]) {
                            dst.vals[d * 2 + 1] = src.vals[d * 2 + 1];
                            dst.configs[d * 2 + 1] = src.configs[d * 2 + 1];
                        }
                    }
                }
            }
            stats.n_phase3_interior += int_found2;
            stats.n_phase3_fk_calls += int_fk2;
            }
        }
        }
    }

    stats.n_dedup_raw_candidates_p1 = dedup_p1.raw_candidates;
    stats.n_dedup_unique_candidates_p1 = dedup_p1.unique_candidates;
    stats.n_dedup_applied_sets_p1 = dedup_p1.applied_sets;
    stats.n_dedup_skipped_sets_p1 = dedup_p1.skipped_sets;

    stats.n_dedup_raw_candidates_p2 = dedup_p2.raw_candidates;
    stats.n_dedup_unique_candidates_p2 = dedup_p2.unique_candidates;
    stats.n_dedup_applied_sets_p2 = dedup_p2.applied_sets;
    stats.n_dedup_skipped_sets_p2 = dedup_p2.skipped_sets;

    stats.n_dedup_raw_candidates_p25 = dedup_p25.raw_candidates;
    stats.n_dedup_unique_candidates_p25 = dedup_p25.unique_candidates;
    stats.n_dedup_applied_sets_p25 = dedup_p25.applied_sets;
    stats.n_dedup_skipped_sets_p25 = dedup_p25.skipped_sets;

    stats.n_dedup_raw_candidates_p3 = dedup_p3.raw_candidates;
    stats.n_dedup_unique_candidates_p3 = dedup_p3.unique_candidates;
    stats.n_dedup_applied_sets_p3 = dedup_p3.applied_sets;
    stats.n_dedup_skipped_sets_p3 = dedup_p3.skipped_sets;

    // ──── Write output AABBs ─────────────────────────────────────────
    for (int ci = 0; ci < n_act; ++ci) {
        float r = (rad) ? static_cast<float>(rad[ci]) : 0.f;
        for (int s = 0; s < n_sub; ++s) {
            float* a = out_aabb + (ci * n_sub + s) * 6;
            const auto& ext = link_seg_ext[ci][s];
            a[0] = static_cast<float>(ext.vals[0]) - r;  // x_min
            a[1] = static_cast<float>(ext.vals[2]) - r;  // y_min
            a[2] = static_cast<float>(ext.vals[4]) - r;  // z_min
            a[3] = static_cast<float>(ext.vals[1]) + r;  // x_max
            a[4] = static_cast<float>(ext.vals[3]) + r;  // y_max
            a[5] = static_cast<float>(ext.vals[5]) + r;  // z_max
        }
    }

    // ──── Post-hoc: update endpoint_aabb from extreme configs ────────
    if (out_endpoint_aabb) {
        std::vector<Eigen::VectorXd> unique_cfgs;
        {
            auto disc = [](const Eigen::VectorXd& v) {
                std::vector<int> k(v.size());
                for (int i = 0; i < v.size(); ++i)
                    k[i] = static_cast<int>(std::round(v[i] * 1e6));
                return k;
            };
            std::set<std::vector<int>> seen;
            for (int ci = 0; ci < n_act; ++ci) {
                for (int s = 0; s < n_sub; ++s) {
                    for (int f = 0; f < 6; ++f) {
                        const auto& cfg = link_seg_ext[ci][s].configs[f];
                        if (seen.insert(disc(cfg)).second)
                            unique_cfgs.push_back(cfg);
                    }
                }
            }
        }

        FKWorkspace ws_post;
        Eigen::VectorXd q_post(n);
        for (auto& cfg : unique_cfgs) {
            q_post = cfg;
            ws_post.compute(robot, q_post);
            for (int ci = 0; ci < n_act; ++ci) {
                int V = map[ci];
                if (V + 1 >= ws_post.np) continue;
                const Eigen::Vector3d p_prox = ws_post.pos(V);
                const Eigen::Vector3d p_dist = ws_post.pos(V + 1);
                for (int s = 0; s <= n_sub; ++s) {
                    float t = s * inv_n;
                    Eigen::Vector3d pt = p_prox + double(t) * (p_dist - p_prox);
                    float* fa = out_endpoint_aabb + (ci * (n_sub + 1) + s) * 6;
                    for (int d = 0; d < 3; ++d) {
                        float v = static_cast<float>(pt[d]);
                        if (v < fa[d])     fa[d]     = v;
                        if (v > fa[d + 3]) fa[d + 3] = v;
                    }
                }
            }
        }
    }

    if (out_stats) *out_stats = stats;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  derive_aabb_critical_analytical_with_configs �?variant with config output
// ══════════════════════════════════════════════════════════════════════════�?

void derive_aabb_critical_analytical_with_configs(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const AnalyticalCriticalConfig& config,
    float* out_aabb,
    Eigen::VectorXd* out_configs,
    AnalyticalCriticalStats* out_stats)
{
    assert(n_sub >= 1);
    const int n       = robot.n_joints();
    const int n_act   = robot.n_active_links();
    const int* map    = robot.active_link_map();
    const double* rad = robot.active_link_radii();

    const float inv_n = 1.0f / static_cast<float>(n_sub);

    std::vector<std::vector<LinkExtremes>> link_seg_ext(n_act);
    for (int ci = 0; ci < n_act; ++ci) {
        link_seg_ext[ci].resize(n_sub);
        for (int s = 0; s < n_sub; ++s)
            link_seg_ext[ci][s].init(n);
    }

    AnalyticalCriticalStats stats{};
    DedupMetrics dedup_p1{}, dedup_p2{}, dedup_p25{}, dedup_p3{};

    // Phase 0
    if (config.keep_kpi2_baseline) {
        std::vector<std::vector<double>> per_joint(n);
        for (int j = 0; j < n; ++j)
            per_joint[j] = crit_angles(intervals[j].lo, intervals[j].hi);
        int max_V = 0;
        for (int ci = 0; ci < n_act; ++ci)
            max_V = std::max(max_V, map[ci]);
        int n_joints_needed = std::min(max_V + 1, n);
        auto csets = build_csets(per_joint, intervals, n_joints_needed);
        Eigen::VectorXd q(n);
        for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();
        FKWorkspace ws;
        ws.resize(n + 2);
        ws.set_identity();
        crit_enum_fk(robot, csets, q, 0, n_joints_needed, ws, [&]() {
            ++stats.n_phase0_vertices;
            for (int ci = 0; ci < n_act; ++ci) {
                int V = map[ci];
                if (V + 1 >= ws.np) continue;
                const Eigen::Vector3d p_prox = ws.pos(V);
                const Eigen::Vector3d p_dist = ws.pos(V + 1);
                for (int s = 0; s < n_sub; ++s) {
                    float t0 = s * inv_n;
                    float t1 = (s + 1) * inv_n;
                    Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                    Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);
                    link_seg_ext[ci][s].update(s0, q);
                    link_seg_ext[ci][s].update(s1, q);
                }
            }
        });
    }

    // AA Pruning
    std::vector<char> skip_link_vec(n_act, false);
    char* skip_link = nullptr;
    AAFKWorkspace aa_ws;
    bool aa_ready = false;
    if (config.enable_aa_pruning) {
        aa_ws.compute(robot, intervals.data());
        aa_ready = true;
        int pruned_segments = 0;
        const int pruned_links = refresh_phase_skip_links_from_aa(
            aa_ws.prefix, map, n_act, n_sub, inv_n, link_seg_ext,
            skip_link_vec, &pruned_segments);
        stats.n_aa_pruned_links = pruned_links;
        stats.n_aa_pruned_segments += static_cast<int64_t>(pruned_segments);
        stats.n_aa_pruned_phase_checks += n_act;
        skip_link = skip_link_vec.data();
    }

    auto refresh_skip_for_phase = [&](const std::vector<std::vector<LinkExtremes>>& ext_ref) -> int {
        if (!aa_ready) return -1;
        int pruned_segments = 0;
        const int pruned_links = refresh_phase_skip_links_from_aa(
            aa_ws.prefix, map, n_act, n_sub, inv_n, ext_ref,
            skip_link_vec, &pruned_segments);
        stats.n_aa_pruned_segments += static_cast<int64_t>(pruned_segments);
        stats.n_aa_pruned_phase_checks += n_act;
        skip_link = skip_link_vec.data();
        return pruned_links;
    };

    // Phase 1
    if (config.enable_edge_solve) {
        const int pruned_links = refresh_skip_for_phase(link_seg_ext);
        if (pruned_links == n_act) {
            stats.n_phase1_edges = 0;
            stats.n_phase1_fk_calls = 0;
        } else {
        int ef = 0, efk = 0;
        const int n_threads = resolve_parallel_threads(config, n_act);
        if (n_threads <= 1) {
            solve_edges(robot, intervals, n_sub, inv_n, map, n_act,
                        link_seg_ext,
                        config,
                        &dedup_p1,
                        skip_link,
                        aa_ready ? aa_ws.prefix : nullptr,
                        0, n_act,
                        &ef, &efk);
        } else {
            std::vector<std::thread> workers;
            std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
            std::vector<DedupMetrics> part_dedup(n_threads);
            workers.reserve(n_threads);

            int ci0 = 0;
            const int base = n_act / n_threads;
            const int rem = n_act % n_threads;
            for (int t = 0; t < n_threads; ++t) {
                const int span = base + (t < rem ? 1 : 0);
                const int begin = ci0;
                const int end = begin + span;
                ci0 = end;
                workers.emplace_back([&, t, begin, end]() {
                    solve_edges(robot, intervals, n_sub, inv_n, map, n_act,
                                link_seg_ext,
                                config,
                                &part_dedup[t],
                                skip_link,
                                aa_ready ? aa_ws.prefix : nullptr,
                                begin, end,
                                &part_found[t], &part_fk[t]);
                });
            }
            for (auto& th : workers) th.join();
            for (int t = 0; t < n_threads; ++t) {
                ef += part_found[t];
                efk += part_fk[t];
                merge_dedup_metrics(dedup_p1, part_dedup[t]);
            }
        }
        stats.n_phase1_edges = ef;
        stats.n_phase1_fk_calls = efk;
        }
    }

    // dual_phase3 checkpoint
    std::vector<std::vector<LinkExtremes>> ext_checkpoint_01;
    if (config.dual_phase3 && config.enable_interior_solve) {
        ext_checkpoint_01 = link_seg_ext;
    }

    // Phase 2
    if (config.enable_face_solve) {
        const int pruned_links = refresh_skip_for_phase(link_seg_ext);
        if (pruned_links == n_act) {
            stats.n_phase2_faces = 0;
            stats.n_phase2_fk_calls = 0;
        } else {
        std::vector<std::pair<int,int>> face_pairs;
        if (config.face_all_pairs) {
            for (int i = 0; i < n; ++i)
                for (int j = i + 1; j < n; ++j)
                    face_pairs.push_back({i, j});
        } else {
            face_pairs = robot.coupled_pairs();
            for (int i = 0; i < n - 1; ++i) {
                std::pair<int,int> p = {i, i+1};
                bool dup = false;
                for (auto& fp : face_pairs) if (fp == p) { dup = true; break; }
                if (!dup) face_pairs.push_back(p);
            }
        }
        int ff = 0, ffk = 0;
        const int n_threads = resolve_parallel_threads(config, n_act);
        if (n_threads <= 1) {
            solve_faces(robot, intervals, n_sub, inv_n, map, n_act, face_pairs,
                        link_seg_ext,
                        config,
                        &dedup_p2,
                        skip_link,
                        aa_ready ? aa_ws.prefix : nullptr,
                        0, n_act,
                        &ff, &ffk);
        } else {
            std::vector<std::thread> workers;
            std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
            std::vector<DedupMetrics> part_dedup(n_threads);
            workers.reserve(n_threads);

            int ci0 = 0;
            const int base = n_act / n_threads;
            const int rem = n_act % n_threads;
            for (int t = 0; t < n_threads; ++t) {
                const int span = base + (t < rem ? 1 : 0);
                const int begin = ci0;
                const int end = begin + span;
                ci0 = end;
                workers.emplace_back([&, t, begin, end]() {
                    solve_faces(robot, intervals, n_sub, inv_n, map, n_act, face_pairs,
                                link_seg_ext,
                                config,
                                &part_dedup[t],
                                skip_link,
                                aa_ready ? aa_ws.prefix : nullptr,
                                begin, end,
                                &part_found[t], &part_fk[t]);
                });
            }
            for (auto& th : workers) th.join();
            for (int t = 0; t < n_threads; ++t) {
                ff += part_found[t];
                ffk += part_fk[t];
                merge_dedup_metrics(dedup_p2, part_dedup[t]);
            }
        }
        stats.n_phase2_faces = ff;
        stats.n_phase2_fk_calls = ffk;
        }
    }

    // Phase 2.5
    {
        const int pruned_links = refresh_skip_for_phase(link_seg_ext);
        if (pruned_links == n_act) {
            stats.n_phase25a_pair1d = 0;
            stats.n_phase25b_pair2d = 0;
            stats.n_phase25_fk_calls = 0;
        } else {
        std::vector<std::pair<int,int>> pc_pairs;
        if (config.pair_all_pairs) {
            for (int i = 0; i < n; ++i)
                for (int j = i + 1; j < n; ++j)
                    pc_pairs.push_back({i, j});
        } else {
            pc_pairs = robot.coupled_pairs();
            for (int i = 0; i < n - 1; ++i) {
                std::pair<int,int> p = {i, i+1};
                bool dup = false;
                for (auto& fp : pc_pairs) if (fp == p) { dup = true; break; }
                if (!dup) pc_pairs.push_back(p);
            }
            for (int i = 0; i < n; ++i)
                for (int j = i + 2; j < n; ++j) {
                    std::pair<int,int> p = {i, j};
                    bool dup = false;
                    for (auto& fp : pc_pairs) if (fp == p) { dup = true; break; }
                    if (!dup) pc_pairs.push_back(p);
                }
        }
        int p25_fk = 0;
        const int n_threads = resolve_parallel_threads(config, n_act);
        if (config.enable_pair_1d) {
            int p1d = 0, p1d_fk = 0;
            if (n_threads <= 1) {
                solve_pair_constrained_1d(robot, intervals, n_sub, inv_n,
                                          map, n_act, pc_pairs,
                                          config.pair_max_bg,
                                          config.pair_kpi2_backgrounds,
                                          link_seg_ext,
                                          config,
                                          &dedup_p25,
                                          skip_link,
                                          aa_ready ? aa_ws.prefix : nullptr,
                                          0, n_act,
                                          &p1d, &p1d_fk);
            } else {
                std::vector<std::thread> workers;
                std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
                std::vector<DedupMetrics> part_dedup(n_threads);
                workers.reserve(n_threads);

                int ci0 = 0;
                const int base = n_act / n_threads;
                const int rem = n_act % n_threads;
                for (int t = 0; t < n_threads; ++t) {
                    const int span = base + (t < rem ? 1 : 0);
                    const int begin = ci0;
                    const int end = begin + span;
                    ci0 = end;
                    workers.emplace_back([&, t, begin, end]() {
                        solve_pair_constrained_1d(robot, intervals, n_sub, inv_n,
                                                  map, n_act, pc_pairs,
                                                  config.pair_max_bg,
                                                  config.pair_kpi2_backgrounds,
                                                  link_seg_ext,
                                                  config,
                                                  &part_dedup[t],
                                                  skip_link,
                                                  aa_ready ? aa_ws.prefix : nullptr,
                                                  begin, end,
                                                  &part_found[t], &part_fk[t]);
                    });
                }
                for (auto& th : workers) th.join();
                for (int t = 0; t < n_threads; ++t) {
                    p1d += part_found[t];
                    p1d_fk += part_fk[t];
                    merge_dedup_metrics(dedup_p25, part_dedup[t]);
                }
            }
            stats.n_phase25a_pair1d = p1d;
            p25_fk += p1d_fk;
        }
        if (config.enable_pair_2d) {
            int p2d = 0, p2d_fk = 0;
            if (n_threads <= 1) {
                solve_pair_constrained_2d(robot, intervals, n_sub, inv_n,
                                          map, n_act, pc_pairs,
                                          config.pair_max_bg,
                                          config.pair_kpi2_backgrounds,
                                          link_seg_ext,
                                          config,
                                          &dedup_p25,
                                          skip_link,
                                          aa_ready ? aa_ws.prefix : nullptr,
                                          0, n_act,
                                          &p2d, &p2d_fk);
            } else {
                std::vector<std::thread> workers;
                std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
                std::vector<DedupMetrics> part_dedup(n_threads);
                workers.reserve(n_threads);

                int ci0 = 0;
                const int base = n_act / n_threads;
                const int rem = n_act % n_threads;
                for (int t = 0; t < n_threads; ++t) {
                    const int span = base + (t < rem ? 1 : 0);
                    const int begin = ci0;
                    const int end = begin + span;
                    ci0 = end;
                    workers.emplace_back([&, t, begin, end]() {
                        solve_pair_constrained_2d(robot, intervals, n_sub, inv_n,
                                                  map, n_act, pc_pairs,
                                                  config.pair_max_bg,
                                                  config.pair_kpi2_backgrounds,
                                                  link_seg_ext,
                                                  config,
                                                  &part_dedup[t],
                                                  skip_link,
                                                  aa_ready ? aa_ws.prefix : nullptr,
                                                  begin, end,
                                                  &part_found[t], &part_fk[t]);
                    });
                }
                for (auto& th : workers) th.join();
                for (int t = 0; t < n_threads; ++t) {
                    p2d += part_found[t];
                    p2d_fk += part_fk[t];
                    merge_dedup_metrics(dedup_p25, part_dedup[t]);
                }
            }
            stats.n_phase25b_pair2d = p2d;
            p25_fk += p2d_fk;
        }
        stats.n_phase25_fk_calls = p25_fk;
        }
    }

    // Phase 3+
    if (config.enable_interior_solve) {
        const int pruned_links = refresh_skip_for_phase(link_seg_ext);
        if (pruned_links == n_act) {
            stats.n_phase3_interior = 0;
            stats.n_phase3_fk_calls = 0;
        } else {
        std::vector<std::pair<int,int>> int_pairs;
        {
            auto cp = robot.coupled_pairs();
            int_pairs.insert(int_pairs.end(), cp.begin(), cp.end());
            for (int i = 0; i < n - 1; ++i) {
                std::pair<int,int> p = {i, i+1};
                bool dup = false;
                for (auto& fp : int_pairs) if (fp == p) { dup = true; break; }
                if (!dup) int_pairs.push_back(p);
            }
            for (int i = 0; i < n - 2; i += 2) {
                std::pair<int,int> p = {i, i+2};
                bool dup = false;
                for (auto& fp : int_pairs) if (fp == p) { dup = true; break; }
                if (!dup) int_pairs.push_back(p);
            }
        }
        int ii = 0, ifk = 0;
        const int n_threads = resolve_parallel_threads(config, n_act);
        if (n_threads <= 1) {
            if (config.improved_interior) {
                solve_interior_improved(robot, intervals, n_sub, inv_n,
                                        map, n_act,
                                        config.interior_max_free,
                                        config.interior_max_sweeps,
                                        config.interior_n_restarts,
                                        config.interior_pair_coupling,
                                        int_pairs,
                                        link_seg_ext,
                                        config,
                                        &dedup_p3,
                                        skip_link,
                                        aa_ready ? aa_ws.prefix : nullptr,
                                        0, n_act,
                                        &ii, &ifk);
            } else {
                solve_interior(robot, intervals, n_sub, inv_n, map, n_act,
                               config.interior_max_free, config.interior_max_sweeps,
                               link_seg_ext,
                               config,
                               &dedup_p3,
                               skip_link,
                               aa_ready ? aa_ws.prefix : nullptr,
                               0, n_act,
                               &ii, &ifk);
            }
        } else {
            std::vector<std::thread> workers;
            std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
            std::vector<DedupMetrics> part_dedup(n_threads);
            workers.reserve(n_threads);

            int ci0 = 0;
            const int base = n_act / n_threads;
            const int rem = n_act % n_threads;
            for (int t = 0; t < n_threads; ++t) {
                const int span = base + (t < rem ? 1 : 0);
                const int begin = ci0;
                const int end = begin + span;
                ci0 = end;
                workers.emplace_back([&, t, begin, end]() {
                    if (config.improved_interior) {
                        solve_interior_improved(robot, intervals, n_sub, inv_n,
                                                map, n_act,
                                                config.interior_max_free,
                                                config.interior_max_sweeps,
                                                config.interior_n_restarts,
                                                config.interior_pair_coupling,
                                                int_pairs,
                                                link_seg_ext,
                                                config,
                                                &part_dedup[t],
                                                skip_link,
                                                aa_ready ? aa_ws.prefix : nullptr,
                                                begin, end,
                                                &part_found[t], &part_fk[t]);
                    } else {
                        solve_interior(robot, intervals, n_sub, inv_n, map, n_act,
                                       config.interior_max_free, config.interior_max_sweeps,
                                       link_seg_ext,
                                       config,
                                       &part_dedup[t],
                                       skip_link,
                                       aa_ready ? aa_ws.prefix : nullptr,
                                       begin, end,
                                       &part_found[t], &part_fk[t]);
                    }
                });
            }
            for (auto& th : workers) th.join();
            for (int t = 0; t < n_threads; ++t) {
                ii += part_found[t];
                ifk += part_fk[t];
                merge_dedup_metrics(dedup_p3, part_dedup[t]);
            }
        }
        stats.n_phase3_interior = ii;
        stats.n_phase3_fk_calls = ifk;

        if (config.dual_phase3 && !ext_checkpoint_01.empty()) {
            const int pruned_links2 = refresh_skip_for_phase(ext_checkpoint_01);
            if (pruned_links2 != n_act) {
            int ii2 = 0, ifk2 = 0;
            if (n_threads <= 1) {
                if (config.improved_interior) {
                    solve_interior_improved(robot, intervals, n_sub, inv_n,
                                            map, n_act,
                                            config.interior_max_free,
                                            config.interior_max_sweeps,
                                            config.interior_n_restarts,
                                            config.interior_pair_coupling,
                                            int_pairs,
                                            ext_checkpoint_01,
                                            config,
                                            &dedup_p3,
                                            skip_link,
                                            aa_ready ? aa_ws.prefix : nullptr,
                                            0, n_act,
                                            &ii2, &ifk2);
                } else {
                    solve_interior(robot, intervals, n_sub, inv_n, map, n_act,
                                   config.interior_max_free, config.interior_max_sweeps,
                                   ext_checkpoint_01,
                                   config,
                                   &dedup_p3,
                                   skip_link,
                                   aa_ready ? aa_ws.prefix : nullptr,
                                   0, n_act,
                                   &ii2, &ifk2);
                }
            } else {
                std::vector<std::thread> workers;
                std::vector<int> part_found(n_threads, 0), part_fk(n_threads, 0);
                std::vector<DedupMetrics> part_dedup(n_threads);
                workers.reserve(n_threads);

                int ci0 = 0;
                const int base = n_act / n_threads;
                const int rem = n_act % n_threads;
                for (int t = 0; t < n_threads; ++t) {
                    const int span = base + (t < rem ? 1 : 0);
                    const int begin = ci0;
                    const int end = begin + span;
                    ci0 = end;
                    workers.emplace_back([&, t, begin, end]() {
                        if (config.improved_interior) {
                            solve_interior_improved(robot, intervals, n_sub, inv_n,
                                                    map, n_act,
                                                    config.interior_max_free,
                                                    config.interior_max_sweeps,
                                                    config.interior_n_restarts,
                                                    config.interior_pair_coupling,
                                                    int_pairs,
                                                    ext_checkpoint_01,
                                                    config,
                                                    &part_dedup[t],
                                                    skip_link,
                                                    aa_ready ? aa_ws.prefix : nullptr,
                                                    begin, end,
                                                    &part_found[t], &part_fk[t]);
                        } else {
                            solve_interior(robot, intervals, n_sub, inv_n, map, n_act,
                                           config.interior_max_free, config.interior_max_sweeps,
                                           ext_checkpoint_01,
                                           config,
                                           &part_dedup[t],
                                           skip_link,
                                           aa_ready ? aa_ws.prefix : nullptr,
                                           begin, end,
                                           &part_found[t], &part_fk[t]);
                        }
                    });
                }
                for (auto& th : workers) th.join();
                for (int t = 0; t < n_threads; ++t) {
                    ii2 += part_found[t];
                    ifk2 += part_fk[t];
                    merge_dedup_metrics(dedup_p3, part_dedup[t]);
                }
            }
            for (int ci = 0; ci < n_act; ++ci) {
                for (int s = 0; s < n_sub; ++s) {
                    auto& dst = link_seg_ext[ci][s];
                    const auto& src = ext_checkpoint_01[ci][s];
                    for (int d = 0; d < 3; ++d) {
                        if (src.vals[d * 2] < dst.vals[d * 2]) {
                            dst.vals[d * 2] = src.vals[d * 2];
                            dst.configs[d * 2] = src.configs[d * 2];
                        }
                        if (src.vals[d * 2 + 1] > dst.vals[d * 2 + 1]) {
                            dst.vals[d * 2 + 1] = src.vals[d * 2 + 1];
                            dst.configs[d * 2 + 1] = src.configs[d * 2 + 1];
                        }
                    }
                }
            }
            stats.n_phase3_interior += ii2;
            stats.n_phase3_fk_calls += ifk2;
            }
        }
        }
    }

    stats.n_dedup_raw_candidates_p1 = dedup_p1.raw_candidates;
    stats.n_dedup_unique_candidates_p1 = dedup_p1.unique_candidates;
    stats.n_dedup_applied_sets_p1 = dedup_p1.applied_sets;
    stats.n_dedup_skipped_sets_p1 = dedup_p1.skipped_sets;

    stats.n_dedup_raw_candidates_p2 = dedup_p2.raw_candidates;
    stats.n_dedup_unique_candidates_p2 = dedup_p2.unique_candidates;
    stats.n_dedup_applied_sets_p2 = dedup_p2.applied_sets;
    stats.n_dedup_skipped_sets_p2 = dedup_p2.skipped_sets;

    stats.n_dedup_raw_candidates_p25 = dedup_p25.raw_candidates;
    stats.n_dedup_unique_candidates_p25 = dedup_p25.unique_candidates;
    stats.n_dedup_applied_sets_p25 = dedup_p25.applied_sets;
    stats.n_dedup_skipped_sets_p25 = dedup_p25.skipped_sets;

    stats.n_dedup_raw_candidates_p3 = dedup_p3.raw_candidates;
    stats.n_dedup_unique_candidates_p3 = dedup_p3.unique_candidates;
    stats.n_dedup_applied_sets_p3 = dedup_p3.applied_sets;
    stats.n_dedup_skipped_sets_p3 = dedup_p3.skipped_sets;

    // Write output AABBs + configs
    for (int ci = 0; ci < n_act; ++ci) {
        float r = (rad) ? static_cast<float>(rad[ci]) : 0.f;
        for (int s = 0; s < n_sub; ++s) {
            int slot = ci * n_sub + s;
            float* a = out_aabb + slot * 6;
            const auto& ext = link_seg_ext[ci][s];
            a[0] = static_cast<float>(ext.vals[0]) - r;
            a[1] = static_cast<float>(ext.vals[2]) - r;
            a[2] = static_cast<float>(ext.vals[4]) - r;
            a[3] = static_cast<float>(ext.vals[1]) + r;
            a[4] = static_cast<float>(ext.vals[3]) + r;
            a[5] = static_cast<float>(ext.vals[5]) + r;

            if (out_configs) {
                for (int face = 0; face < 6; ++face)
                    out_configs[slot * 6 + face] = ext.configs[face];
            }
        }
    }

    if (out_stats) *out_stats = stats;
}

} // namespace envelope
} // namespace sbf
