// SafeBoxForest v2 — Critical-Point Envelope Derive implementation
#define _USE_MATH_DEFINES
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/robot/fk.h"           // fk_link_positions

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <functional>
#include <numeric>
#include <random>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double kShifts[3] = {0.0, 2*M_PI, -2*M_PI};

namespace sbf {
namespace envelope {

// ─── Critical-angle enumeration helpers ─────────────────────────────────────

static constexpr long long CRIT_MAX_COMBOS = 60'000;

// Build candidate angles for one joint interval
static std::vector<double> crit_angles(double lo, double hi) {
    std::vector<double> v = { lo, hi };
    for (int k = -20; k <= 20; ++k) {
        double a = k * HALF_PI;
        if (a > lo && a < hi)
            v.push_back(a);
    }
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

// Build Cartesian-product sets for joints [0..n_joints-1].
// If product exceeds CRIT_MAX_COMBOS, fall back to {lo, mid, hi}.
static std::vector<std::vector<double>> build_csets(
    const std::vector<std::vector<double>>& per_joint,
    const std::vector<Interval>& intervals,
    int n_joints)
{
    std::vector<std::vector<double>> csets(n_joints);
    long long total = 1;
    for (int j = 0; j < n_joints; ++j) {
        csets[j] = per_joint[j];
        total *= static_cast<long long>(csets[j].size());
        if (total > CRIT_MAX_COMBOS) {
            for (int j2 = 0; j2 < n_joints; ++j2) {
                double lo = intervals[j2].lo, hi = intervals[j2].hi;
                csets[j2] = { lo, 0.5 * (lo + hi), hi };
            }
            break;
        }
    }
    return csets;
}

// Recursive Cartesian-product enumeration
static void crit_enum(
    const std::vector<std::vector<double>>& csets,
    Eigen::VectorXd& q, int depth,
    const std::function<void()>& cb)
{
    if (depth == static_cast<int>(csets.size())) { cb(); return; }
    for (double v : csets[depth]) {
        q[depth] = v;
        crit_enum(csets, q, depth + 1, cb);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  derive_crit_frames — critical-point enumeration → FrameStore format
// ═════════════════════════════════════════════════════════════════════════════

void derive_crit_frames(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    float* out_frames,
    int* out_n_combos)
{
    const int n = robot.n_joints();
    const bool has_tool = robot.has_tool();
    const int n_frames = n + (has_tool ? 1 : 0);

    // Initialise frames to empty intervals [+inf, -inf]
    for (int k = 0; k < n_frames; ++k) {
        out_frames[k * 6 + 0] = out_frames[k * 6 + 1] = out_frames[k * 6 + 2] =  1e30f;
        out_frames[k * 6 + 3] = out_frames[k * 6 + 4] = out_frames[k * 6 + 5] = -1e30f;
    }

    // Build per-joint critical angles
    std::vector<std::vector<double>> per_joint(n);
    for (int j = 0; j < n; ++j)
        per_joint[j] = crit_angles(intervals[j].lo, intervals[j].hi);

    auto csets = build_csets(per_joint, intervals, n);

    Eigen::VectorXd q(n);
    for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

    int n_combos = 0;

    crit_enum(csets, q, 0, [&]() {
        ++n_combos;
        auto pos = fk_link_positions(robot, q);
        // pos has n+1+(has_tool?1:0) entries
        // pos[0] = base (constant), pos[k+1] = frame k
        const int np = static_cast<int>(pos.size());

        for (int k = 0; k < n_frames; ++k) {
            int pi = k + 1;  // pos index for frame k
            if (pi >= np) continue;
            float* f = out_frames + k * 6;
            for (int d = 0; d < 3; ++d) {
                float v = static_cast<float>(pos[pi][d]);
                f[d]     = std::min(f[d],     v);   // lo
                f[d + 3] = std::max(f[d + 3], v);   // hi
            }
        }
    });

    if (out_n_combos) *out_n_combos = n_combos;
}

// ═════════════════════════════════════════════════════════════════════════════
//  derive_aabb_critical
// ═════════════════════════════════════════════════════════════════════════════

void derive_aabb_critical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    float* out_aabb)
{
    assert(n_sub >= 1);
    const int n       = robot.n_joints();
    const int n_act   = robot.n_active_links();
    const int* map    = robot.active_link_map();
    const double* rad = robot.active_link_radii();

    const int total_slots = n_act * n_sub;

    // Initialise all AABBs to empty
    for (int k = 0; k < total_slots; ++k) {
        out_aabb[k*6+0] = out_aabb[k*6+1] = out_aabb[k*6+2] =  1e30f;
        out_aabb[k*6+3] = out_aabb[k*6+4] = out_aabb[k*6+5] = -1e30f;
    }

    // Build per-joint critical angles
    std::vector<std::vector<double>> per_joint(n);
    for (int j = 0; j < n; ++j)
        per_joint[j] = crit_angles(intervals[j].lo, intervals[j].hi);

    // Deepest active link determines how many joints we need
    int max_V = 0;
    for (int ci = 0; ci < n_act; ++ci)
        max_V = std::max(max_V, map[ci]);
    // positions[V+1] depends on joints [0..V], so need V+1 joints
    int n_joints = std::min(max_V + 1, n);

    auto csets = build_csets(per_joint, intervals, n_joints);

    Eigen::VectorXd q(n);
    for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

    const float inv_n = 1.0f / static_cast<float>(n_sub);

    crit_enum(csets, q, 0, [&]() {
        auto pos = fk_link_positions(robot, q);
        const int np = static_cast<int>(pos.size());

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            if (V + 1 >= np) continue;

            const Eigen::Vector3d& p_prox = pos[V];
            const Eigen::Vector3d& p_dist = pos[V + 1];

            for (int s = 0; s < n_sub; ++s) {
                float t0 = s * inv_n;
                float t1 = (s + 1) * inv_n;

                // Sub-segment endpoints via linear interpolation
                Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);

                float* a = out_aabb + (ci * n_sub + s) * 6;
                for (int k = 0; k < 3; ++k) {
                    float vs = static_cast<float>(s0[k]);
                    float ve = static_cast<float>(s1[k]);
                    a[k]     = std::min(a[k],     std::min(vs, ve));
                    a[k + 3] = std::max(a[k + 3], std::max(vs, ve));
                }
            }
        }
    });

    // Inflate by link radius
    for (int ci = 0; ci < n_act; ++ci) {
        float r = (rad) ? static_cast<float>(rad[ci]) : 0.f;
        for (int s = 0; s < n_sub; ++s) {
            float* a = out_aabb + (ci * n_sub + s) * 6;
            a[0] -= r; a[1] -= r; a[2] -= r;
            a[3] += r; a[4] += r; a[5] += r;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  derive_obb_critical
// ═════════════════════════════════════════════════════════════════════════════

// Helper: PCA OBB from point cloud with world-frame centre
static void pca_obb_world(
    const std::vector<Eigen::Vector3d>& pts,
    float* out,           // CRIT_OBB_FLOATS
    double radius = 0.0)
{
    if (pts.size() < 2) {
        std::fill(out, out + CRIT_OBB_FLOATS, 0.0f);
        out[6] = 1.0f; out[10] = 1.0f; out[14] = 1.0f;
        return;
    }

    // ── Covariance ───────────────────────────────────────────────────
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (auto& p : pts) mean += p;
    mean /= static_cast<double>(pts.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (auto& p : pts) {
        Eigen::Vector3d d = p - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(pts.size());
    cov.diagonal().array() += 1e-12;

    // ── PCA eigenvectors ─────────────────────────────────────────────
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    const Eigen::Vector3d ax = solver.eigenvectors().col(2).normalized();
    const Eigen::Vector3d ay = solver.eigenvectors().col(1).normalized();
    const Eigen::Vector3d az = (ax.cross(ay)).normalized();

    // ── Project → half-extents ───────────────────────────────────────
    double lo[3] = {  1e30,  1e30,  1e30 };
    double hi[3] = { -1e30, -1e30, -1e30 };
    const Eigen::Vector3d* axes[3] = { &ax, &ay, &az };

    for (auto& p : pts) {
        for (int k = 0; k < 3; ++k) {
            double proj = axes[k]->dot(p);
            if (proj < lo[k]) lo[k] = proj;
            if (proj > hi[k]) hi[k] = proj;
        }
    }
    for (int k = 0; k < 3; ++k) { lo[k] -= radius; hi[k] += radius; }

    // ── Centre in local frame ────────────────────────────────────────
    double c_local[3];
    for (int k = 0; k < 3; ++k)
        c_local[k] = 0.5 * (lo[k] + hi[k]);

    // ── Centre in WORLD frame  (c_local[0]*ax + c_local[1]*ay + c_local[2]*az)
    Eigen::Vector3d c_world = c_local[0] * ax + c_local[1] * ay + c_local[2] * az;

    // ── Write slot ───────────────────────────────────────────────────
    out[0] = static_cast<float>(c_world[0]);
    out[1] = static_cast<float>(c_world[1]);
    out[2] = static_cast<float>(c_world[2]);
    out[3] = static_cast<float>(0.5 * (hi[0] - lo[0]));
    out[4] = static_cast<float>(0.5 * (hi[1] - lo[1]));
    out[5] = static_cast<float>(0.5 * (hi[2] - lo[2]));
    out[6]  = static_cast<float>(ax[0]); out[7]  = static_cast<float>(ax[1]); out[8]  = static_cast<float>(ax[2]);
    out[9]  = static_cast<float>(ay[0]); out[10] = static_cast<float>(ay[1]); out[11] = static_cast<float>(ay[2]);
    out[12] = static_cast<float>(az[0]); out[13] = static_cast<float>(az[1]); out[14] = static_cast<float>(az[2]);
}

void derive_obb_critical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    float* out_obbs)
{
    assert(n_sub >= 1);
    const int n       = robot.n_joints();
    const int n_act   = robot.n_active_links();
    const int* map    = robot.active_link_map();
    const double* rad = robot.active_link_radii();

    const int total_slots = n_act * n_sub;

    // Build per-joint critical angles
    std::vector<std::vector<double>> per_joint(n);
    for (int j = 0; j < n; ++j)
        per_joint[j] = crit_angles(intervals[j].lo, intervals[j].hi);

    int max_V = 0;
    for (int ci = 0; ci < n_act; ++ci)
        max_V = std::max(max_V, map[ci]);
    int n_joints = std::min(max_V + 1, n);

    auto csets = build_csets(per_joint, intervals, n_joints);

    Eigen::VectorXd q(n);
    for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

    const float inv_n = 1.0f / static_cast<float>(n_sub);

    // Collect point clouds per (link, sub-segment)
    std::vector<std::vector<Eigen::Vector3d>> clouds(total_slots);

    crit_enum(csets, q, 0, [&]() {
        auto pos = fk_link_positions(robot, q);
        const int np = static_cast<int>(pos.size());

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            if (V + 1 >= np) continue;

            const Eigen::Vector3d& p_prox = pos[V];
            const Eigen::Vector3d& p_dist = pos[V + 1];

            for (int s = 0; s < n_sub; ++s) {
                float t0 = s * inv_n;
                float t1 = (s + 1) * inv_n;

                Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);

                auto& cloud = clouds[ci * n_sub + s];
                cloud.push_back(s0);
                cloud.push_back(s1);
            }
        }
    });

    // PCA fit each sub-segment
    for (int ci = 0; ci < n_act; ++ci) {
        double r = (rad) ? rad[ci] : 0.0;
        for (int s = 0; s < n_sub; ++s) {
            int idx = ci * n_sub + s;
            pca_obb_world(clouds[idx], out_obbs + idx * CRIT_OBB_FLOATS, r);
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  collide_obb_obs  —  15-axis SAT with interleaved obstacle format
// ═════════════════════════════════════════════════════════════════════════════
//
// OBB slot: [cx cy cz hx hy hz Ax(3) Ay(3) Az(3)]   (world-frame centre)
// Obstacle: [lo_x hi_x lo_y hi_y lo_z hi_z]          (interleaved)

static bool obb_aabb_sat(const float* obb, const float* obs_inter)
{
    // OBB
    const double oc[3] = { obb[0], obb[1], obb[2] };
    const double oh[3] = { obb[3], obb[4], obb[5] };
    const double Ax[3] = { obb[6],  obb[7],  obb[8]  };
    const double Ay[3] = { obb[9],  obb[10], obb[11] };
    const double Az[3] = { obb[12], obb[13], obb[14] };

    // AABB (interleaved → centre / half-extent)
    const double ac[3] = { 0.5*(obs_inter[0]+obs_inter[1]),
                            0.5*(obs_inter[2]+obs_inter[3]),
                            0.5*(obs_inter[4]+obs_inter[5]) };
    const double ah[3] = { 0.5*(obs_inter[1]-obs_inter[0]),
                            0.5*(obs_inter[3]-obs_inter[2]),
                            0.5*(obs_inter[5]-obs_inter[4]) };

    const double t[3] = { oc[0]-ac[0], oc[1]-ac[1], oc[2]-ac[2] };
    const double* R[3] = { Ax, Ay, Az };

    double AR[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            AR[i][j] = std::fabs(R[i][j]) + 1e-6;

    // Case 1: L = world axis e_j
    for (int j = 0; j < 3; ++j) {
        double dist = std::fabs(t[j]);
        double po   = oh[0]*AR[0][j] + oh[1]*AR[1][j] + oh[2]*AR[2][j];
        if (dist > po + ah[j]) return false;
    }
    // Case 2: L = OBB axis i
    for (int i = 0; i < 3; ++i) {
        double dist = std::fabs(R[i][0]*t[0] + R[i][1]*t[1] + R[i][2]*t[2]);
        double pa   = ah[0]*AR[i][0] + ah[1]*AR[i][1] + ah[2]*AR[i][2];
        if (dist > oh[i] + pa) return false;
    }
    // Case 3: L = e_j × OBB_axis_i  (9 cross-product axes)
    for (int i = 0; i < 3; ++i) {
        // j=0: L = (0, -R[i][2], R[i][1])
        {
            double dist = std::fabs(-R[i][2]*t[1] + R[i][1]*t[2]);
            double po = oh[0]*(AR[i][2]*AR[0][1]+AR[i][1]*AR[0][2])
                      + oh[1]*(AR[i][2]*AR[1][1]+AR[i][1]*AR[1][2])
                      + oh[2]*(AR[i][2]*AR[2][1]+AR[i][1]*AR[2][2]);
            double pa = ah[1]*AR[i][2] + ah[2]*AR[i][1];
            if (dist > po + pa) return false;
        }
        // j=1: L = (R[i][2], 0, -R[i][0])
        {
            double dist = std::fabs(R[i][2]*t[0] - R[i][0]*t[2]);
            double po = oh[0]*(AR[i][2]*AR[0][0]+AR[i][0]*AR[0][2])
                      + oh[1]*(AR[i][2]*AR[1][0]+AR[i][0]*AR[1][2])
                      + oh[2]*(AR[i][2]*AR[2][0]+AR[i][0]*AR[2][2]);
            double pa = ah[0]*AR[i][2] + ah[2]*AR[i][0];
            if (dist > po + pa) return false;
        }
        // j=2: L = (-R[i][1], R[i][0], 0)
        {
            double dist = std::fabs(-R[i][1]*t[0] + R[i][0]*t[1]);
            double po = oh[0]*(AR[i][1]*AR[0][0]+AR[i][0]*AR[0][1])
                      + oh[1]*(AR[i][1]*AR[1][0]+AR[i][0]*AR[1][1])
                      + oh[2]*(AR[i][1]*AR[2][0]+AR[i][0]*AR[2][1]);
            double pa = ah[0]*AR[i][1] + ah[1]*AR[i][0];
            if (dist > po + pa) return false;
        }
    }
    return true;  // not separated → collision
}

bool collide_obb_obs(
    const float* obb_slots, int n_slots,
    const float* obs_compact, int n_obs)
{
    for (int i = 0; i < n_slots; ++i) {
        const float* obb = obb_slots + i * CRIT_OBB_FLOATS;
        for (int j = 0; j < n_obs; ++j) {
            if (obb_aabb_sat(obb, obs_compact + j * 6))
                return true;
        }
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════════
//  volume_obb_slots
// ═════════════════════════════════════════════════════════════════════════════

double volume_obb_slots(const float* obb_slots, int n_slots)
{
    double total = 0.0;
    for (int i = 0; i < n_slots; ++i) {
        const float* p = obb_slots + i * CRIT_OBB_FLOATS;
        double hx = p[3], hy = p[4], hz = p[5];
        if (hx > 0 && hy > 0 && hz > 0)
            total += 8.0 * double(hx) * double(hy) * double(hz);
    }
    return total;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Enhanced critical sampling — 3-stage pipeline (ported from Python v4)
// ═════════════════════════════════════════════════════════════════════════════

// Helper: per-link AABB extremes tracker (6 directions × value + config)
struct LinkExtremes {
    // [x_min, x_max, y_min, y_max, z_min, z_max]
    double vals[6];
    Eigen::VectorXd configs[6];

    void init(int n_joints) {
        for (int i = 0; i < 6; ++i) {
            vals[i] = (i % 2 == 0) ? 1e30 : -1e30;  // min=+inf, max=-inf
            configs[i] = Eigen::VectorXd::Zero(n_joints);
        }
    }

    void update(const Eigen::Vector3d& pos, const Eigen::VectorXd& q) {
        for (int d = 0; d < 3; ++d) {
            double v = pos[d];
            if (v < vals[d * 2]) {       // d_min
                vals[d * 2] = v;
                configs[d * 2] = q;
            }
            if (v > vals[d * 2 + 1]) {   // d_max
                vals[d * 2 + 1] = v;
                configs[d * 2 + 1] = q;
            }
        }
    }
};

// ──── Stage 1+: Coupled constraint point generation ──────────────────────
// Generates additional critical configs where qi + qj = k*pi/2 (pairs)
// and qa + qb + qc = k*pi/2 (triples)

static std::vector<Eigen::VectorXd> generate_coupled_constraint_points(
    int n_joints,
    const std::vector<Interval>& intervals,
    const std::vector<std::vector<double>>& key_values,
    const std::vector<std::pair<int,int>>& coupled_pairs,
    const std::vector<std::tuple<int,int,int>>& coupled_triples)
{
    using PT = std::tuple<std::vector<double>>;
    std::set<std::vector<int>> seen;  // discretize to avoid dups
    std::vector<Eigen::VectorXd> points;

    auto discretize = [](double v) -> int { return (int)std::round(v * 1e6); };

    auto add_point = [&](const Eigen::VectorXd& pt) {
        std::vector<int> key(pt.size());
        for (int i = 0; i < pt.size(); ++i) key[i] = discretize(pt[i]);
        if (seen.insert(key).second)
            points.push_back(pt);
    };

    auto make_defaults = [&]() -> Eigen::VectorXd {
        Eigen::VectorXd d(n_joints);
        for (int i = 0; i < n_joints; ++i) d[i] = intervals[i].mid();
        return d;
    };

    // Strategy 3: Any two-joint sum constraint qi + qj = k*π/2
    for (int i = 0; i < n_joints; ++i) {
        for (double vi : key_values[i]) {
            for (int j = i + 1; j < n_joints; ++j) {
                double lo_j = intervals[j].lo, hi_j = intervals[j].hi;
                for (int k = -4; k <= 4; ++k) {
                    double qj = k * HALF_PI - vi;
                    if (qj >= lo_j && qj <= hi_j) {
                        // With all-lo background
                        Eigen::VectorXd pt(n_joints);
                        for (int x = 0; x < n_joints; ++x) pt[x] = intervals[x].lo;
                        pt[i] = vi; pt[j] = qj;
                        add_point(pt);
                        // With all-hi background
                        for (int x = 0; x < n_joints; ++x) pt[x] = intervals[x].hi;
                        pt[i] = vi; pt[j] = qj;
                        add_point(pt);
                    }
                }
            }
        }
    }

    // Strategy 4: Specific coupled pairs with lo/hi/mid backgrounds
    for (auto& [pi_idx, pj_idx] : coupled_pairs) {
        if (pi_idx >= n_joints || pj_idx >= n_joints) continue;
        double lo_j = intervals[pj_idx].lo, hi_j = intervals[pj_idx].hi;
        for (double vi : key_values[pi_idx]) {
            for (int k = -4; k <= 4; ++k) {
                double qj = k * HALF_PI - vi;
                if (qj < lo_j || qj > hi_j) continue;
                // Three backgrounds: lo, hi, mid
                for (int bg = 0; bg < 3; ++bg) {
                    Eigen::VectorXd pt(n_joints);
                    for (int x = 0; x < n_joints; ++x) {
                        if (bg == 0) pt[x] = intervals[x].lo;
                        else if (bg == 1) pt[x] = intervals[x].hi;
                        else pt[x] = intervals[x].mid();
                    }
                    pt[pi_idx] = vi;
                    pt[pj_idx] = qj;
                    add_point(pt);
                }
            }
        }
    }

    // Strategy 5-6: Coupled triples qa + qb + qc = k*π/2
    for (auto& [a, b, c] : coupled_triples) {
        if (a >= n_joints || b >= n_joints || c >= n_joints) continue;

        // For each permutation of (a,b,c) as (i1, i2, i_solve)
        int perms[][3] = {{a,b,c}, {a,c,b}, {b,c,a}};
        for (auto& perm : perms) {
            int i1 = perm[0], i2 = perm[1], i_s = perm[2];
            double lo_s = intervals[i_s].lo, hi_s = intervals[i_s].hi;
            for (double v1 : key_values[i1]) {
                for (double v2 : key_values[i2]) {
                    for (int k = -5; k <= 5; ++k) {
                        double vs = k * HALF_PI - v1 - v2;
                        if (vs < lo_s || vs > hi_s) continue;

                        // Background: all other joints at lo / hi
                        std::vector<int> bg_joints;
                        for (int x = 0; x < n_joints; ++x)
                            if (x != a && x != b && x != c)
                                bg_joints.push_back(x);

                        // Enumerate 2^(bg_joints.size()) backgrounds (capped)
                        int n_bg = static_cast<int>(bg_joints.size());
                        int n_combos = std::min(1 << n_bg, 16); // cap at 16
                        for (int mask = 0; mask < n_combos; ++mask) {
                            Eigen::VectorXd pt = make_defaults();
                            for (int bi = 0; bi < n_bg; ++bi) {
                                int jj = bg_joints[bi];
                                pt[jj] = (mask & (1 << bi))
                                    ? intervals[jj].hi : intervals[jj].lo;
                            }
                            pt[i1] = v1; pt[i2] = v2; pt[i_s] = vs;
                            add_point(pt);
                        }
                    }
                }
            }
        }
    }

    return points;
}

// ──── Stage 2: Manifold random sampling ──────────────────────────────────

static std::vector<Eigen::VectorXd> generate_manifold_samples(
    int n_joints,
    const std::vector<Interval>& intervals,
    const std::vector<std::tuple<int,int,int>>& coupled_triples,
    int n_per,
    std::mt19937& rng)
{
    std::vector<Eigen::VectorXd> samples;
    if (n_joints < 2) return samples;

    auto rand_in = [&](double lo, double hi) -> double {
        return std::uniform_real_distribution<double>(lo, hi)(rng);
    };

    auto make_random = [&]() -> Eigen::VectorXd {
        Eigen::VectorXd pt(n_joints);
        for (int i = 0; i < n_joints; ++i)
            pt[i] = rand_in(intervals[i].lo, intervals[i].hi);
        return pt;
    };

    // Two-joint constraint manifold: qi + qj = k*π/2
    for (int i = 0; i < n_joints; ++i) {
        double lo_i = intervals[i].lo, hi_i = intervals[i].hi;
        for (int j = i + 1; j < n_joints; ++j) {
            double lo_j = intervals[j].lo, hi_j = intervals[j].hi;
            for (int k = -4; k <= 4; ++k) {
                double tgt = k * HALF_PI;
                double qi_lo = std::max(lo_i, tgt - hi_j);
                double qi_hi = std::min(hi_i, tgt - lo_j);
                if (qi_lo > qi_hi) continue;
                for (int s = 0; s < n_per; ++s) {
                    Eigen::VectorXd pt = make_random();
                    double qi = rand_in(qi_lo, qi_hi);
                    pt[i] = qi;
                    pt[j] = tgt - qi;
                    samples.push_back(pt);
                }
            }
        }
    }

    // Three-joint constraint manifold: qa + qb + qc = k*π/2
    for (auto& [a, b, c] : coupled_triples) {
        if (a >= n_joints || b >= n_joints || c >= n_joints) continue;
        double lo_a = intervals[a].lo, hi_a = intervals[a].hi;
        double lo_b = intervals[b].lo, hi_b = intervals[b].hi;
        double lo_c = intervals[c].lo, hi_c = intervals[c].hi;
        for (int k = -5; k <= 5; ++k) {
            double tgt = k * HALF_PI;
            for (int s = 0; s < n_per; ++s) {
                double qa = rand_in(lo_a, hi_a);
                double qb = rand_in(lo_b, hi_b);
                double qc = tgt - qa - qb;
                if (qc >= lo_c && qc <= hi_c) {
                    Eigen::VectorXd pt = make_random();
                    pt[a] = qa; pt[b] = qb; pt[c] = qc;
                    samples.push_back(pt);
                }
            }
        }
    }

    return samples;
}

// ──── Stage 3: Projected L-BFGS-B local optimization ─────────────────────

// Minimal projected L-BFGS-B for box-constrained optimization (SLOW: finite-difference)
// Optimise f(x) subject to lb <= x <= ub
static void lbfgs_b_minimize(
    const std::function<double(const Eigen::VectorXd&)>& func,
    Eigen::VectorXd& x,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    int max_iter,
    double ftol)
{
    const int n = static_cast<int>(x.size());
    if (n == 0) return;
    const int m = 5;  // L-BFGS memory
    const double eps_g = 1e-6;
    const double h_fd = 1e-7;  // finite difference step

    // Project x into bounds
    auto project = [&](Eigen::VectorXd& v) {
        for (int i = 0; i < n; ++i)
            v[i] = std::max(lb[i], std::min(ub[i], v[i]));
    };

    // Gradient via central finite differences
    auto gradient = [&](const Eigen::VectorXd& xc) -> Eigen::VectorXd {
        Eigen::VectorXd g(n);
        Eigen::VectorXd xp = xc;
        for (int i = 0; i < n; ++i) {
            double xi_save = xp[i];
            double h = std::max(h_fd, std::abs(xi_save) * 1e-8);
            xp[i] = std::min(ub[i], xi_save + h);
            double fp = func(xp);
            xp[i] = std::max(lb[i], xi_save - h);
            double fm = func(xp);
            g[i] = (fp - fm) / (xp[i] - (xi_save - h) + (xi_save + h) - xp[i]);
            // Correction for asymmetric steps at bounds
            double actual_h = (std::min(ub[i], xi_save + h) - std::max(lb[i], xi_save - h));
            if (actual_h > 1e-15)
                g[i] = (fp - fm) / actual_h;
            xp[i] = xi_save;
        }
        return g;
    };

    project(x);
    double f_prev = func(x);
    Eigen::VectorXd g = gradient(x);

    // L-BFGS storage
    std::vector<Eigen::VectorXd> S(m), Y(m);
    std::vector<double> rho(m, 0.0);
    int mem_count = 0;

    for (int iter = 0; iter < max_iter; ++iter) {
        // Check projected gradient convergence
        double pg_norm = 0.0;
        for (int i = 0; i < n; ++i) {
            double gi = g[i];
            if (x[i] <= lb[i] + 1e-12 && gi > 0) gi = 0;
            if (x[i] >= ub[i] - 1e-12 && gi < 0) gi = 0;
            pg_norm += gi * gi;
        }
        if (std::sqrt(pg_norm) < eps_g) break;

        // L-BFGS two-loop recursion to get direction
        Eigen::VectorXd q = g;
        int bound = std::min(mem_count, m);
        std::vector<double> alpha_hist(bound);
        for (int i = bound - 1; i >= 0; --i) {
            int idx = (mem_count - 1 - (bound - 1 - i)) % m;
            alpha_hist[i] = rho[idx] * S[idx].dot(q);
            q -= alpha_hist[i] * Y[idx];
        }

        // Scale by gamma
        double gamma = 1.0;
        if (mem_count > 0) {
            int last = (mem_count - 1) % m;
            double yy = Y[last].dot(Y[last]);
            if (yy > 1e-15)
                gamma = S[last].dot(Y[last]) / yy;
        }
        Eigen::VectorXd r = gamma * q;

        for (int i = 0; i < bound; ++i) {
            int idx = (mem_count - bound + i) % m;
            double beta = rho[idx] * Y[idx].dot(r);
            r += (alpha_hist[i] - beta) * S[idx];
        }

        Eigen::VectorXd d = -r;

        // Projected backtracking line search
        double step = 1.0;
        const double c1 = 1e-4;
        for (int ls = 0; ls < 20; ++ls) {
            Eigen::VectorXd x_new = x + step * d;
            project(x_new);
            double f_new = func(x_new);
            double descent = g.dot(x_new - x);
            if (f_new <= f_prev + c1 * descent || step < 1e-12) {
                Eigen::VectorXd s = x_new - x;
                Eigen::VectorXd g_new = gradient(x_new);
                Eigen::VectorXd y = g_new - g;
                double sy = s.dot(y);
                if (sy > 1e-15) {
                    int idx = mem_count % m;
                    S[idx] = s;
                    Y[idx] = y;
                    rho[idx] = 1.0 / sy;
                    ++mem_count;
                }

                double f_change = std::abs(f_new - f_prev);
                x = x_new;
                g = g_new;
                f_prev = f_new;

                if (f_change < ftol) return;  // converged
                break;
            }
            step *= 0.5;
        }
    }
}

// ──── FAST L-BFGS-B with analytical Jacobian ─────────────────────────────
// Uses analytical FK Jacobian instead of finite-difference gradients.
// Only optimises joints [0..n_opt-1] that actually affect the target link.
// Cost per iteration: 1 FK + n_opt cross-products (vs 2*n+1 FK calls for FD).

static void lbfgs_b_minimize_fast(
    const Robot& robot,
    int eval_link,           // transform index (1-based frame)
    int dim,                 // 0=x, 1=y, 2=z
    double sign,             // +1 for minimize, -1 for maximize
    Eigen::VectorXd& q,     // full joint vector (modified in-place)
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    int n_opt,               // number of joints to optimise [0..n_opt-1]
    int max_iter,
    double ftol,
    Eigen::Matrix4d* tf_buf) // pre-allocated buffer of size >= eval_link+1
{
    if (n_opt <= 0 || eval_link < 1) return;
    const int m = 5;   // L-BFGS memory
    const double eps_g = 1e-6;

    // Project joints [0..n_opt-1] into bounds
    auto project = [&]() {
        for (int i = 0; i < n_opt; ++i)
            q[i] = std::max(lb[i], std::min(ub[i], q[i]));
    };

    // Evaluate function value + analytical gradient simultaneously
    // grad_out must have size >= n_opt
    auto eval_fg = [&](double* grad_out) -> double {
        fk_transforms_inplace(robot, q, tf_buf);
        double fval = fk_position_gradient(
            robot, tf_buf, eval_link, dim, n_opt, grad_out);
        // Apply sign: we always minimise sign*fval
        fval *= sign;
        for (int i = 0; i < n_opt; ++i) grad_out[i] *= sign;
        return fval;
    };

    project();

    // Use stack-allocated arrays for small n_opt (typically <= 7)
    std::vector<double> g_buf(n_opt);
    double f_prev = eval_fg(g_buf.data());

    // Map gradient to Eigen for L-BFGS algebra
    Eigen::Map<Eigen::VectorXd> g(g_buf.data(), n_opt);

    // L-BFGS storage (small vectors of size n_opt)
    std::vector<Eigen::VectorXd> S(m), Y(m);
    std::vector<double> rho(m, 0.0);
    int mem_count = 0;

    std::vector<double> g_new_buf(n_opt);

    for (int iter = 0; iter < max_iter; ++iter) {
        // Check projected gradient convergence
        double pg_norm = 0.0;
        for (int i = 0; i < n_opt; ++i) {
            double gi = g[i];
            if (q[i] <= lb[i] + 1e-12 && gi > 0) gi = 0;
            if (q[i] >= ub[i] - 1e-12 && gi < 0) gi = 0;
            pg_norm += gi * gi;
        }
        if (std::sqrt(pg_norm) < eps_g) break;

        // L-BFGS two-loop recursion
        Eigen::VectorXd r_vec = g;  // use as q_vec in two-loop
        int bound = std::min(mem_count, m);
        std::vector<double> alpha_hist(bound);
        for (int i = bound - 1; i >= 0; --i) {
            int idx = (mem_count - 1 - (bound - 1 - i)) % m;
            alpha_hist[i] = rho[idx] * S[idx].dot(r_vec);
            r_vec -= alpha_hist[i] * Y[idx];
        }

        double gamma = 1.0;
        if (mem_count > 0) {
            int last = (mem_count - 1) % m;
            double yy = Y[last].dot(Y[last]);
            if (yy > 1e-15)
                gamma = S[last].dot(Y[last]) / yy;
        }
        r_vec *= gamma;

        for (int i = 0; i < bound; ++i) {
            int idx = (mem_count - bound + i) % m;
            double beta = rho[idx] * Y[idx].dot(r_vec);
            r_vec += (alpha_hist[i] - beta) * S[idx];
        }

        // Search direction d = -H*g
        Eigen::VectorXd d = -r_vec;

        // Save current q values for rollback
        Eigen::VectorXd q_save(n_opt);
        for (int i = 0; i < n_opt; ++i) q_save[i] = q[i];

        // Projected backtracking line search
        double step = 1.0;
        const double c1 = 1e-4;
        bool accepted = false;
        for (int ls = 0; ls < 20; ++ls) {
            // Trial point
            for (int i = 0; i < n_opt; ++i)
                q[i] = q_save[i] + step * d[i];
            project();

            double f_new = eval_fg(g_new_buf.data());

            // Compute actual displacement
            Eigen::VectorXd s_vec(n_opt);
            for (int i = 0; i < n_opt; ++i)
                s_vec[i] = q[i] - q_save[i];
            double descent = g.dot(s_vec);

            if (f_new <= f_prev + c1 * descent || step < 1e-12) {
                Eigen::Map<Eigen::VectorXd> g_new(g_new_buf.data(), n_opt);
                Eigen::VectorXd y_vec = g_new - g;
                double sy = s_vec.dot(y_vec);
                if (sy > 1e-15) {
                    int idx = mem_count % m;
                    S[idx] = s_vec;
                    Y[idx] = y_vec;
                    rho[idx] = 1.0 / sy;
                    ++mem_count;
                }

                double f_change = std::abs(f_new - f_prev);
                // Update gradient
                for (int i = 0; i < n_opt; ++i) g_buf[i] = g_new_buf[i];
                f_prev = f_new;
                accepted = true;

                if (f_change < ftol) return;  // converged
                break;
            }
            step *= 0.5;
        }

        if (!accepted) {
            // Restore q
            for (int i = 0; i < n_opt; ++i) q[i] = q_save[i];
            break;
        }
    }
}

// ──── Stage 3 driver: optimize AABB extremes per active link ─────────────
// SLOW version: uses finite-difference L-BFGS-B, pre-screens all seeds

static void optimize_aabb_extremes_slow(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const CriticalSamplingConfig& config,
    // Per active-link sub-segment extremes: [n_act][n_sub]
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    const std::vector<Eigen::VectorXd>& constraint_seeds,
    int* out_n_calls)
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();
    int total_calls = 0;

    Eigen::VectorXd lb(n), ub(n);
    for (int j = 0; j < n; ++j) {
        lb[j] = intervals[j].lo;
        ub[j] = intervals[j].hi;
    }

    const float inv_n = 1.0f / static_cast<float>(n_sub);

    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];

        // For each of the 6 AABB face directions
        for (int face = 0; face < 6; ++face) {
            int dim = face / 2;        // 0=x, 1=y, 2=z
            bool is_min = (face % 2 == 0);

            for (int eval_link : {V + 1, V}) {
                if (eval_link < 1) continue;

                auto objective = [&](const Eigen::VectorXd& q) -> double {
                    auto pos = fk_link_positions(robot, q);
                    if (eval_link >= static_cast<int>(pos.size())) return 0.0;
                    double v = pos[eval_link][dim];
                    return is_min ? v : -v;
                };

                // Collect seeds: current best + constraint seeds
                std::vector<Eigen::VectorXd> seeds;

                for (int s = 0; s < n_sub; ++s) {
                    const auto& ext = link_seg_ext[ci][s];
                    const auto& cfg = ext.configs[face];
                    if (cfg.size() == n) {
                        bool dup = false;
                        for (auto& seed : seeds) {
                            if ((seed - cfg).norm() < 1e-6) { dup = true; break; }
                        }
                        if (!dup) seeds.push_back(cfg);
                    }
                }

                if (!constraint_seeds.empty()) {
                    int best_idx = -1;
                    double best_val = 1e30;
                    for (int si = 0; si < static_cast<int>(constraint_seeds.size()); ++si) {
                        double v = objective(constraint_seeds[si]);
                        if (v < best_val) {
                            best_val = v;
                            best_idx = si;
                        }
                    }
                    if (best_idx >= 0) {
                        seeds.push_back(constraint_seeds[best_idx]);
                        if (config.lbfgs_n_seeds >= 2 &&
                            constraint_seeds.size() > 1) {
                            double max_dist = -1;
                            int far_idx = -1;
                            for (int si = 0; si < static_cast<int>(constraint_seeds.size()); ++si) {
                                if (si == best_idx) continue;
                                double d = (constraint_seeds[si] - constraint_seeds[best_idx]).norm();
                                if (d > max_dist) {
                                    max_dist = d;
                                    far_idx = si;
                                }
                            }
                            if (far_idx >= 0)
                                seeds.push_back(constraint_seeds[far_idx]);
                        }
                    }
                }

                if (seeds.empty()) continue;

                for (auto& seed : seeds) {
                    Eigen::VectorXd x = seed;
                    lbfgs_b_minimize(objective, x, lb, ub,
                                     config.lbfgs_max_iter, config.lbfgs_ftol);
                    ++total_calls;

                    auto pos = fk_link_positions(robot, x);
                    int np = static_cast<int>(pos.size());
                    if (V + 1 >= np) continue;

                    const Eigen::Vector3d& p_prox = pos[V];
                    const Eigen::Vector3d& p_dist = pos[V + 1];

                    for (int s = 0; s < n_sub; ++s) {
                        float t0 = s * inv_n;
                        float t1 = (s + 1) * inv_n;
                        Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                        Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);
                        link_seg_ext[ci][s].update(s0, x);
                        link_seg_ext[ci][s].update(s1, x);
                    }
                }
            }
        }
    }

    if (out_n_calls) *out_n_calls = total_calls;
}

// ──── FAST Stage 3 driver: analytical Jacobian + smart seeds ─────────────
// Key accelerations:
//   1. Analytical FK Jacobian → ~14× cheaper gradient (n cross-products vs 2n FK)
//   2. Restrict optimisation to joints [0..eval_link-1] only
//   3. Use tracked best configs as seeds directly (no O(N_seeds) pre-screening)
//   4. Pre-allocated transform buffer (no heap allocation per FK call)

static void optimize_aabb_extremes_fast(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const CriticalSamplingConfig& config,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    int* out_n_calls)
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();
    int total_calls = 0;

    Eigen::VectorXd lb(n), ub(n);
    for (int j = 0; j < n; ++j) {
        lb[j] = intervals[j].lo;
        ub[j] = intervals[j].hi;
    }

    const float inv_n = 1.0f / static_cast<float>(n_sub);

    // Pre-allocate transform buffer (max size: n+2 for tool)
    const int tf_buf_size = n + 2;
    std::vector<Eigen::Matrix4d> tf_buf(tf_buf_size);

    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];

        for (int face = 0; face < 6; ++face) {
            int dim = face / 2;        // 0=x, 1=y, 2=z
            bool is_min = (face % 2 == 0);
            double sign = is_min ? 1.0 : -1.0;

            // Evaluate both eval_links (V and V+1)
            for (int eval_link : {V + 1, V}) {
                if (eval_link < 1) continue;

                // Number of joints that affect this link
                int n_opt = std::min(eval_link, n);
                if (n_opt == 0) continue;

                // Smart seed selection: use already-tracked best configs
                // from all sub-segments, no expensive pre-screening
                std::vector<Eigen::VectorXd> seeds;

                for (int s = 0; s < n_sub; ++s) {
                    const auto& ext = link_seg_ext[ci][s];
                    const auto& cfg = ext.configs[face];
                    if (cfg.size() == n) {
                        bool dup = false;
                        for (auto& seed : seeds) {
                            if ((seed - cfg).squaredNorm() < 1e-12) {
                                dup = true;
                                break;
                            }
                        }
                        if (!dup) seeds.push_back(cfg);
                    }
                }

                // Also add the best config from opposite face as exploration seed
                if (config.lbfgs_n_seeds >= 2) {
                    int opp_face = (face % 2 == 0) ? face + 1 : face - 1;
                    for (int s = 0; s < n_sub && (int)seeds.size() < config.lbfgs_n_seeds + 1; ++s) {
                        const auto& cfg = link_seg_ext[ci][s].configs[opp_face];
                        if (cfg.size() == n) {
                            bool dup = false;
                            for (auto& seed : seeds) {
                                if ((seed - cfg).squaredNorm() < 1e-12) {
                                    dup = true;
                                    break;
                                }
                            }
                            if (!dup) seeds.push_back(cfg);
                        }
                    }
                }

                // Limit number of seeds
                if ((int)seeds.size() > config.lbfgs_n_seeds + 1)
                    seeds.resize(config.lbfgs_n_seeds + 1);

                for (auto& seed : seeds) {
                    Eigen::VectorXd q = seed;
                    lbfgs_b_minimize_fast(
                        robot, eval_link, dim, sign,
                        q, lb, ub, n_opt,
                        config.lbfgs_max_iter, config.lbfgs_ftol,
                        tf_buf.data());
                    ++total_calls;

                    // Evaluate final position and update extremes
                    fk_transforms_inplace(robot, q, tf_buf.data());
                    Eigen::Vector3d p_prox = tf_buf[V].block<3,1>(0,3);
                    Eigen::Vector3d p_dist = tf_buf[V+1].block<3,1>(0,3);

                    for (int s = 0; s < n_sub; ++s) {
                        float t0 = s * inv_n;
                        float t1 = (s + 1) * inv_n;
                        Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                        Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);
                        link_seg_ext[ci][s].update(s0, q);
                        link_seg_ext[ci][s].update(s1, q);
                    }
                }
            }
        }
    }

    if (out_n_calls) *out_n_calls = total_calls;
}

// ═════════════════════════════════════════════════════════════════════════════
//  derive_aabb_critical_enhanced  — 3-stage pipeline
// ═════════════════════════════════════════════════════════════════════════════

void derive_aabb_critical_enhanced(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const CriticalSamplingConfig& config,
    float* out_aabb,
    CriticalStats* out_stats)
{
    assert(n_sub >= 1);
    const int n       = robot.n_joints();
    const int n_act   = robot.n_active_links();
    const int* map    = robot.active_link_map();
    const double* rad = robot.active_link_radii();

    const int total_slots = n_act * n_sub;
    const float inv_n = 1.0f / static_cast<float>(n_sub);

    // Per active-link, per sub-segment extremes tracker
    std::vector<std::vector<LinkExtremes>> link_seg_ext(n_act);
    for (int ci = 0; ci < n_act; ++ci) {
        link_seg_ext[ci].resize(n_sub);
        for (int s = 0; s < n_sub; ++s)
            link_seg_ext[ci][s].init(n);
    }

    CriticalStats stats{};

    // ──── Build per-joint critical angles ────────────────────────────
    std::vector<std::vector<double>> per_joint(n);
    for (int j = 0; j < n; ++j)
        per_joint[j] = crit_angles(intervals[j].lo, intervals[j].hi);

    int max_V = 0;
    for (int ci = 0; ci < n_act; ++ci)
        max_V = std::max(max_V, map[ci]);
    int n_joints_needed = std::min(max_V + 1, n);

    // ──── Stage 1: Base critical enumeration ─────────────────────────
    auto csets = build_csets(per_joint, intervals, n_joints_needed);

    Eigen::VectorXd q(n);
    for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

    crit_enum(csets, q, 0, [&]() {
        ++stats.n_stage1_base;
        auto pos = fk_link_positions(robot, q);
        const int np = static_cast<int>(pos.size());

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            if (V + 1 >= np) continue;
            const Eigen::Vector3d& p_prox = pos[V];
            const Eigen::Vector3d& p_dist = pos[V + 1];

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

    // ──── Stage 1+: Coupled constraint points ────────────────────────
    std::vector<Eigen::VectorXd> constraint_points;
    if (config.enable_coupled_constraints) {
        constraint_points = generate_coupled_constraint_points(
            n_joints_needed,
            intervals,
            per_joint,
            robot.coupled_pairs(),
            robot.coupled_triples());

        stats.n_stage1_coupled = static_cast<int>(constraint_points.size());

        // Evaluate all constraint points
        for (const auto& cp : constraint_points) {
            // Build full q with mid values for remaining joints
            Eigen::VectorXd q_full(n);
            for (int j = 0; j < n; ++j) q_full[j] = intervals[j].mid();
            for (int j = 0; j < std::min(n_joints_needed, (int)cp.size()); ++j)
                q_full[j] = cp[j];

            auto pos = fk_link_positions(robot, q_full);
            const int np = static_cast<int>(pos.size());

            for (int ci = 0; ci < n_act; ++ci) {
                int V = map[ci];
                if (V + 1 >= np) continue;
                const Eigen::Vector3d& p_prox = pos[V];
                const Eigen::Vector3d& p_dist = pos[V + 1];

                for (int s = 0; s < n_sub; ++s) {
                    float t0 = s * inv_n;
                    float t1 = (s + 1) * inv_n;
                    Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                    Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);
                    link_seg_ext[ci][s].update(s0, q_full);
                    link_seg_ext[ci][s].update(s1, q_full);
                }
            }
        }
    }

    // ──── Stage 2: Manifold random sampling ──────────────────────────
    std::vector<Eigen::VectorXd> manifold_samples;
    if (config.enable_manifold_sampling) {
        std::mt19937 rng(42);  // deterministic seed for reproducibility
        manifold_samples = generate_manifold_samples(
            n_joints_needed,
            intervals,
            robot.coupled_triples(),
            config.manifold_n_per,
            rng);

        stats.n_stage2_manifold = static_cast<int>(manifold_samples.size());

        for (const auto& ms : manifold_samples) {
            Eigen::VectorXd q_full(n);
            for (int j = 0; j < n; ++j) q_full[j] = intervals[j].mid();
            for (int j = 0; j < std::min(n_joints_needed, (int)ms.size()); ++j)
                q_full[j] = ms[j];

            auto pos = fk_link_positions(robot, q_full);
            const int np = static_cast<int>(pos.size());

            for (int ci = 0; ci < n_act; ++ci) {
                int V = map[ci];
                if (V + 1 >= np) continue;
                const Eigen::Vector3d& p_prox = pos[V];
                const Eigen::Vector3d& p_dist = pos[V + 1];

                for (int s = 0; s < n_sub; ++s) {
                    float t0 = s * inv_n;
                    float t1 = (s + 1) * inv_n;
                    Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                    Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);
                    link_seg_ext[ci][s].update(s0, q_full);
                    link_seg_ext[ci][s].update(s1, q_full);
                }
            }
        }
    }

    // ──── Stage 3: L-BFGS-B optimization ─────────────────────────────
    if (config.enable_lbfgs_optimization) {
        int n_calls = 0;

        if (config.use_analytical_jacobian && config.smart_seed_selection) {
            // FAST path: analytical Jacobian + smart seed selection
            optimize_aabb_extremes_fast(
                robot, intervals, n_sub, config,
                link_seg_ext, &n_calls);
        } else {
            // SLOW path: finite-difference gradient + pre-screening all seeds
            std::vector<Eigen::VectorXd> opt_seeds;
            opt_seeds.reserve(constraint_points.size() + manifold_samples.size());
            for (auto& cp : constraint_points) {
                Eigen::VectorXd q_full(n);
                for (int j = 0; j < n; ++j) q_full[j] = intervals[j].mid();
                for (int j = 0; j < std::min(n_joints_needed, (int)cp.size()); ++j)
                    q_full[j] = cp[j];
                opt_seeds.push_back(q_full);
            }
            for (auto& ms : manifold_samples) {
                Eigen::VectorXd q_full(n);
                for (int j = 0; j < n; ++j) q_full[j] = intervals[j].mid();
                for (int j = 0; j < std::min(n_joints_needed, (int)ms.size()); ++j)
                    q_full[j] = ms[j];
                opt_seeds.push_back(q_full);
            }

            optimize_aabb_extremes_slow(
                robot, intervals, n_sub, config,
                link_seg_ext, opt_seeds, &n_calls);
        }

        stats.n_stage3_lbfgs = n_calls;
    }

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

    if (out_stats) *out_stats = stats;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Analytical critical-point enumeration  (zero-optimisation approach)
// ═════════════════════════════════════════════════════════════════════════════
//
// Mathematical foundation for revolute-only DH chains:
//
// For a single free joint q_j with all others fixed, the FK position
// component p_d (d = x,y,z) of link V has the form:
//
//   p_d(q_j) = α·cos(q_j) + β·sin(q_j) + γ
//
// where α, β, γ depend on all other joint values and DH parameters.
// Setting dp_d/dq_j = 0 gives  −α·sin(q_j) + β·cos(q_j) = 0,
// hence  q_j* = atan2(β, α)  and  q_j* + π  (two candidates).
//
// For two free joints (q_i, q_j), p_d is a bilinear trig function:
//   p_d = a1·ci·cj + a2·ci·sj + a3·si·cj + a4·si·sj
//       + a5·ci + a6·si + a7·cj + a8·sj + a9
//
// The gradient-zero system ∂p_d/∂q_i = 0, ∂p_d/∂q_j = 0 can be
// reduced to a degree-8 polynomial in t_j = tan(q_j/2) via
// Weierstrass substitution and univariate elimination.
// Solved exactly via companion-matrix eigenvalues.
//
// For k ≥ 3 free joints, we use coordinate-wise sweep: iteratively
// solve 1D sub-problems for each joint while fixing others, repeating
// until convergence. This is deterministic but not provably complete.
// ═════════════════════════════════════════════════════════════════════════════

// Helper: evaluate FK and update link extremes for a single config
static void eval_and_update(
    const Robot& robot,
    const Eigen::VectorXd& q,
    int n_sub, float inv_n,
    const int* map, int n_act,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext)
{
    auto pos = fk_link_positions(robot, q);
    const int np = static_cast<int>(pos.size());
    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        if (V + 1 >= np) continue;
        const Eigen::Vector3d& p_prox = pos[V];
        const Eigen::Vector3d& p_dist = pos[V + 1];
        for (int s = 0; s < n_sub; ++s) {
            float t0 = s * inv_n;
            float t1 = (s + 1) * inv_n;
            Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
            Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);
            link_seg_ext[ci][s].update(s0, q);
            link_seg_ext[ci][s].update(s1, q);
        }
    }
}

// ──── Phase 1: 1D Edge Solver (atan2) ────────────────────────────────────
//
// For each active link V, for each coordinate dimension d (x,y,z),
// for each free joint j ∈ [0..V], with all other joints on boundary:
//   Fit p_d(q_j) = α·cos(q_j) + β·sin(q_j) + γ  using 3-point sampling
//   Solve  q_j* = atan2(β, α)  → 2 candidates
//
// Background combinations: other joints take {lo} or {hi} → 2^(V-1) combos
// For manageable cost, if V>5 we sample a limited set of backgrounds.

static void solve_edges(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;

    // For each active link
    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        int nj = std::min(V + 1, n);  // joints affecting this link

        // Number of background combos: 2^(nj-1), cap at 128
        int n_bg = nj - 1;
        int max_bg = std::min(1 << n_bg, 128);

        // For each free joint j
        for (int j = 0; j < nj; ++j) {
            double lo_j = intervals[j].lo, hi_j = intervals[j].hi;
            if (hi_j - lo_j < 1e-12) continue;  // degenerate interval

            // 3 sample points: lo, mid, hi
            double mid_j = 0.5 * (lo_j + hi_j);
            double qvals[3] = { lo_j, mid_j, hi_j };

            // Enumerate backgrounds
            for (int bg = 0; bg < max_bg; ++bg) {
                Eigen::VectorXd q(n);
                for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();

                // Set background joints to lo/hi based on bg bitmask
                int bit = 0;
                for (int jj = 0; jj < nj; ++jj) {
                    if (jj == j) continue;
                    q[jj] = (bg & (1 << bit)) ? intervals[jj].hi : intervals[jj].lo;
                    ++bit;
                }

                // Evaluate FK at 3 sample points along q_j
                Eigen::Vector3d pts[3];
                // We need positions for link V and V+1
                for (int si = 0; si < 3; ++si) {
                    q[j] = qvals[si];
                    auto pos = fk_link_positions(robot, q);
                    ++fk_calls;
                    // For each link-eval (proximal V and distal V+1), take both
                    // But simpler: store the two positions and process below
                    // Store midpoint of sub-segment for now — for n_sub=1 both
                    // endpoints matter equally.
                    // Actually, store both endpoints for max coverage.
                    // We'll fit α,β,γ for each eval_link and each dim.
                    int np = static_cast<int>(pos.size());
                    if (V + 1 >= np) goto next_bg;
                    // Use distal position (V+1) as primary; proximal V will also be checked
                    pts[si] = pos[V + 1];
                }

                {
                    // Fit p_d(q_j) = α·cos(q_j) + β·sin(q_j) + γ
                    // Using 3 samples: [cos(q0) sin(q0) 1] [α β γ]ᵀ = p_d(q0)
                    Eigen::Matrix3d A;
                    for (int si = 0; si < 3; ++si) {
                        A(si, 0) = std::cos(qvals[si]);
                        A(si, 1) = std::sin(qvals[si]);
                        A(si, 2) = 1.0;
                    }
                    // Solve for each dimension d
                    Eigen::Vector3d b_vec;
                    Eigen::Vector3d coeff;

                    for (int d = 0; d < 3; ++d) {
                        b_vec << pts[0][d], pts[1][d], pts[2][d];
                        coeff = A.colPivHouseholderQr().solve(b_vec);
                        double alpha_c = coeff[0];  // coefficient of cos
                        double beta_c  = coeff[1];  // coefficient of sin

                        if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                            continue;  // constant in this dim

                        // Critical point: q* = atan2(β, α)
                        double q_star = std::atan2(beta_c, alpha_c);

                        // Two candidates: q_star and q_star ± π
                        double candidates[2] = { q_star, q_star + M_PI };
                        if (candidates[1] > M_PI + 0.1) candidates[1] -= 2 * M_PI;

                        for (double qc : candidates) {
                            // Also try shifted by 2π for boundary cases
                            for (double shift : kShifts) {
                                double qtest = qc + shift;
                                if (qtest >= lo_j - 1e-12 && qtest <= hi_j + 1e-12) {
                                    qtest = std::max(lo_j, std::min(hi_j, qtest));
                                    q[j] = qtest;
                                    eval_and_update(robot, q, n_sub, inv_n,
                                                    map, n_act, link_seg_ext);
                                    ++fk_calls;
                                    ++found;
                                    break;
                                }
                            }
                        }
                    }

                    // Also do the same for proximal link V (if V >= 1)
                    if (V >= 1) {
                        Eigen::Vector3d pts_prox[3];
                        for (int si = 0; si < 3; ++si) {
                            q[j] = qvals[si];
                            auto pos = fk_link_positions(robot, q);
                            ++fk_calls;
                            pts_prox[si] = pos[V];
                        }
                        for (int d = 0; d < 3; ++d) {
                            b_vec << pts_prox[0][d], pts_prox[1][d], pts_prox[2][d];
                            coeff = A.colPivHouseholderQr().solve(b_vec);
                            double alpha_c = coeff[0];
                            double beta_c  = coeff[1];
                            if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                                continue;
                            double q_star = std::atan2(beta_c, alpha_c);
                            double candidates[2] = { q_star, q_star + M_PI };
                            if (candidates[1] > M_PI + 0.1) candidates[1] -= 2 * M_PI;
                            for (double qcc : candidates) {
                                for (double shift : kShifts) {
                                    double qtest = qcc + shift;
                                    if (qtest >= lo_j - 1e-12 && qtest <= hi_j + 1e-12) {
                                        qtest = std::max(lo_j, std::min(hi_j, qtest));
                                        q[j] = qtest;
                                        eval_and_update(robot, q, n_sub, inv_n,
                                                        map, n_act, link_seg_ext);
                                        ++fk_calls;
                                        ++found;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                next_bg:;
            }
        }
    }

    if (n_found) *n_found = found;
    if (n_fk_calls) *n_fk_calls = fk_calls;
}

// ──── Phase 2: 2D Face Solver (companion-matrix polynomial roots) ────────
//
// For two free joints (i,j) and all others fixed on boundaries:
//   p_d(qi, qj) = a1·ci·cj + a2·ci·sj + a3·si·cj + a4·si·sj
//               + a5·ci + a6·si + a7·cj + a8·sj + a9
//
// Gradient-zero conditions:
//   ∂p_d/∂qi = -a1·si·cj - a2·si·sj + a3·ci·cj + a4·ci·sj - a5·si + a6·ci = 0
//   ∂p_d/∂qj = -a1·ci·sj + a2·ci·cj - a3·si·sj + a4·si·cj - a7·sj + a8·cj = 0
//
// Weierstrass substitution t = tan(q/2):  cos = (1-t²)/(1+t²), sin = 2t/(1+t²)
//
// After elimination, this yields a polynomial of degree ≤ 8 in t_j.
//
// We solve using companion-matrix eigenvalues (Eigen's EigenSolver).

// Helper: solve real roots of polynomial c[0] + c[1]*t + ... + c[deg]*t^deg
static std::vector<double> solve_polynomial_real(const double* c, int deg) {
    std::vector<double> roots;
    // Remove leading zeros
    while (deg > 0 && std::abs(c[deg]) < 1e-15) --deg;
    if (deg <= 0) return roots;

    if (deg == 1) {
        roots.push_back(-c[0] / c[1]);
        return roots;
    }
    if (deg == 2) {
        double disc = c[1]*c[1] - 4*c[2]*c[0];
        if (disc >= 0) {
            double sq = std::sqrt(disc);
            roots.push_back((-c[1] + sq) / (2*c[2]));
            roots.push_back((-c[1] - sq) / (2*c[2]));
        }
        return roots;
    }

    // General: companion matrix
    Eigen::MatrixXd comp = Eigen::MatrixXd::Zero(deg, deg);
    for (int i = 1; i < deg; ++i)
        comp(i, i - 1) = 1.0;
    double lead_inv = 1.0 / c[deg];
    for (int i = 0; i < deg; ++i)
        comp(i, deg - 1) = -c[i] * lead_inv;

    Eigen::EigenSolver<Eigen::MatrixXd> solver(comp, false);
    const auto& vals = solver.eigenvalues();
    for (int i = 0; i < deg; ++i) {
        if (std::abs(vals[i].imag()) < 1e-8)
            roots.push_back(vals[i].real());
    }
    return roots;
}

// Helper: from half-angle t = tan(q/2), recover q in [lo, hi]
static bool half_angle_to_q(double t, double lo, double hi, double& q_out) {
    double q = 2.0 * std::atan(t);
    for (double shift : kShifts) {
        double qtest = q + shift;
        if (qtest >= lo - 1e-10 && qtest <= hi + 1e-10) {
            q_out = std::max(lo, std::min(hi, qtest));
            return true;
        }
    }
    return false;
}

static void solve_faces(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    const std::vector<std::pair<int,int>>& face_pairs,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;

    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        int nj = std::min(V + 1, n);

        for (auto& [pi, pj] : face_pairs) {
            if (pi >= nj || pj >= nj) continue;
            double lo_i = intervals[pi].lo, hi_i = intervals[pi].hi;
            double lo_j = intervals[pj].lo, hi_j = intervals[pj].hi;
            if (hi_i - lo_i < 1e-12 || hi_j - lo_j < 1e-12) continue;

            // Background joints: all except pi, pj → {lo, hi}
            int n_bg_joints = nj - 2;
            int max_bg = std::min(1 << std::max(n_bg_joints, 0), 64);

            for (int bg = 0; bg < max_bg; ++bg) {
                Eigen::VectorXd q(n);
                for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();

                // Set background
                int bit = 0;
                for (int jj = 0; jj < nj; ++jj) {
                    if (jj == pi || jj == pj) continue;
                    q[jj] = (bg & (1 << bit)) ? intervals[jj].hi : intervals[jj].lo;
                    ++bit;
                }

                // Sample 3×3 grid to fit 9-coefficient bilinear trig model
                double qi_vals[3] = { lo_i, 0.5*(lo_i+hi_i), hi_i };
                double qj_vals[3] = { lo_j, 0.5*(lo_j+hi_j), hi_j };

                // For each eval_link (V and V+1)
                for (int eval_link : {V + 1, V}) {
                    if (eval_link < 1 || eval_link >= n + 2) continue;

                    // [3*3 = 9 samples]
                    Eigen::Matrix<double, 9, 9> A_mat;
                    Eigen::Matrix<double, 9, 3> B_mat;  // 3 dims

                    int row = 0;
                    for (int si = 0; si < 3; ++si) {
                        double ci_v = std::cos(qi_vals[si]);
                        double si_v = std::sin(qi_vals[si]);
                        for (int sj = 0; sj < 3; ++sj) {
                            double cj_v = std::cos(qj_vals[sj]);
                            double sj_v = std::sin(qj_vals[sj]);

                            q[pi] = qi_vals[si];
                            q[pj] = qj_vals[sj];
                            auto pos = fk_link_positions(robot, q);
                            ++fk_calls;
                            int np = (int)pos.size();
                            if (eval_link >= np) goto next_face_bg;

                            // Basis: ci*cj, ci*sj, si*cj, si*sj, ci, si, cj, sj, 1
                            A_mat(row, 0) = ci_v * cj_v;
                            A_mat(row, 1) = ci_v * sj_v;
                            A_mat(row, 2) = si_v * cj_v;
                            A_mat(row, 3) = si_v * sj_v;
                            A_mat(row, 4) = ci_v;
                            A_mat(row, 5) = si_v;
                            A_mat(row, 6) = cj_v;
                            A_mat(row, 7) = sj_v;
                            A_mat(row, 8) = 1.0;

                            for (int d = 0; d < 3; ++d)
                                B_mat(row, d) = pos[eval_link][d];
                            ++row;
                        }
                    }

                    {
                        // Solve for coefficients: A_mat * coeffs = B_mat
                        auto qr = A_mat.colPivHouseholderQr();

                        for (int d = 0; d < 3; ++d) {
                            Eigen::VectorXd rhs = B_mat.col(d);
                            Eigen::VectorXd coeff = qr.solve(rhs);
                            double a1 = coeff[0], a2 = coeff[1];
                            double a3 = coeff[2], a4 = coeff[3];
                            double a5 = coeff[4], a6 = coeff[5];
                            double a7 = coeff[6], a8 = coeff[7];
                            // a9 = coeff[8] (constant, not needed for gradient)

                            // ∂p/∂qi = 0:
                            //   -a1·si·cj - a2·si·sj + a3·ci·cj + a4·ci·sj - a5·si + a6·ci = 0
                            //   → tan(qi) = (a3·cj + a4·sj + a6) / (a1·cj + a2·sj + a5)
                            //
                            // ∂p/∂qj = 0:
                            //   -a1·ci·sj + a2·ci·cj - a3·si·sj + a4·si·cj - a7·sj + a8·cj = 0
                            //
                            // Substituting tan(qi) into the second equation via
                            // Weierstrass: let ti = tan(qi/2), tj = tan(qj/2)
                            //   ci = (1-ti²)/(1+ti²), si = 2ti/(1+ti²)
                            //   cj = (1-tj²)/(1+tj²), sj = 2tj/(1+tj²)
                            //
                            // From ∂p/∂qi = 0:
                            //   (a3·cj + a4·sj + a6)·ci = (a1·cj + a2·sj + a5)·si
                            //   Let P(tj) = a3·cj + a4·sj + a6 = a3(1-tj²)/(1+tj²) + a4·2tj/(1+tj²) + a6
                            //             = [a3(1-tj²) + 2a4·tj + a6(1+tj²)] / (1+tj²)
                            //             = [(a6-a3)tj² + 2a4·tj + (a3+a6)] / (1+tj²)
                            //   Let Q(tj) = a1·cj + a2·sj + a5 = [(a5-a1)tj² + 2a2·tj + (a1+a5)] / (1+tj²)
                            //
                            // Then si·P(tj) = ci·Q(tj) → tan(qi) = P(tj)/Q(tj)
                            //   ci = Q / sqrt(P²+Q²),  si = P / sqrt(P²+Q²)
                            //
                            // Substituting into ∂p/∂qj = 0:
                            //   (-a1·sj + a2·cj)·ci + (-a3·sj + a4·cj)·si - a7·sj + a8·cj = 0
                            //   ci·(-a1·sj + a2·cj) + si·(-a3·sj + a4·cj) + (-a7·sj + a8·cj) = 0
                            //
                            // Substituting ci = Q/R, si = P/R where R = sqrt(P²+Q²):
                            //   Q·(-a1·sj + a2·cj) + P·(-a3·sj + a4·cj) + R·(-a7·sj + a8·cj) = 0
                            //
                            // The R term is problematic (square root). Instead, rearrange:
                            //   [Q·(-a1·sj + a2·cj) + P·(-a3·sj + a4·cj)]² = (-a7·sj + a8·cj)²·(P²+Q²)
                            //
                            // This becomes a polynomial in tj via Weierstrass.
                            // Each of Q, P is degree 2 in tj (over (1+tj²)),
                            // sj, cj are degree 1,2 in tj (over (1+tj²)).
                            // After clearing denominators (1+tj²)^k, we get degree ≤ 8.

                            // Numerically: evaluate the resultant polynomial coefficients
                            // by sampling at 9 points in tj, then fitting.
                            // The polynomial is degree ≤ 8 so 9 samples suffice.

                            // Strategy: define F(tj) = LHS² - RHS² = 0 in terms of tj
                            // Sample F at 9 points, fit degree-8 polynomial, solve.

                            double tj_samples[9];
                            double F_values[9];
                            for (int k = 0; k < 9; ++k)
                                tj_samples[k] = -2.0 + k * 0.5;  // -2, -1.5, ..., 2

                            for (int k = 0; k < 9; ++k) {
                                double tj = tj_samples[k];
                                double tj2 = tj * tj;
                                double denom_j = 1.0 + tj2;
                                double cj_v = (1.0 - tj2) / denom_j;
                                double sj_v = 2.0 * tj / denom_j;

                                // P = a3·cj + a4·sj + a6
                                double P = a3*cj_v + a4*sj_v + a6;
                                // Q = a1·cj + a2·sj + a5
                                double Q = a1*cj_v + a2*sj_v + a5;

                                // A_term = Q·(-a1·sj + a2·cj) + P·(-a3·sj + a4·cj)
                                double A_term = Q*(-a1*sj_v + a2*cj_v) + P*(-a3*sj_v + a4*cj_v);
                                // B_term = -a7·sj + a8·cj
                                double B_term = -a7*sj_v + a8*cj_v;

                                // F = A² - B²·(P²+Q²)
                                F_values[k] = A_term * A_term - B_term * B_term * (P*P + Q*Q);
                            }

                            // Fit degree-8 polynomial to F_values at tj_samples
                            // Using Vandermonde matrix
                            Eigen::Matrix<double, 9, 9> V_mat;
                            for (int k = 0; k < 9; ++k) {
                                double pw = 1.0;
                                for (int p = 0; p < 9; ++p) {
                                    V_mat(k, p) = pw;
                                    pw *= tj_samples[k];
                                }
                            }
                            Eigen::Map<Eigen::Matrix<double, 9, 1>> F_vec(F_values);
                            Eigen::VectorXd poly_c = V_mat.colPivHouseholderQr().solve(F_vec);

                            // Solve polynomial
                            double poly_arr[9];
                            for (int p = 0; p < 9; ++p) poly_arr[p] = poly_c[p];

                            auto tj_roots = solve_polynomial_real(poly_arr, 8);

                            for (double tj_root : tj_roots) {
                                double qj_cand;
                                if (!half_angle_to_q(tj_root, lo_j, hi_j, qj_cand))
                                    continue;

                                // Recover qi from tan(qi) = P(tj)/Q(tj)
                                double tj2 = tj_root * tj_root;
                                double denom_j = 1.0 + tj2;
                                double cj_v = (1.0 - tj2) / denom_j;
                                double sj_v = 2.0 * tj_root / denom_j;

                                double P = a3*cj_v + a4*sj_v + a6;
                                double Q = a1*cj_v + a2*sj_v + a5;
                                double qi_cand = std::atan2(P, Q);

                                // Try qi_cand and qi_cand ± π
                                double qi_tries[3] = {qi_cand, qi_cand + M_PI, qi_cand - M_PI};
                                for (double qi_try : qi_tries) {
                                    for (double shift : kShifts) {
                                        double qi_test = qi_try + shift;
                                        if (qi_test >= lo_i - 1e-10 && qi_test <= hi_i + 1e-10) {
                                            qi_test = std::max(lo_i, std::min(hi_i, qi_test));
                                            q[pi] = qi_test;
                                            q[pj] = qj_cand;

                                            // Verify gradient is approximately zero
                                            // (spurious roots from squaring)
                                            eval_and_update(robot, q, n_sub, inv_n,
                                                            map, n_act, link_seg_ext);
                                            ++fk_calls;
                                            ++found;
                                            goto next_root;
                                        }
                                    }
                                }
                                next_root:;
                            }
                        }
                    }
                    next_face_bg:;
                }
            }
        }
    }

    if (n_found) *n_found = found;
    if (n_fk_calls) *n_fk_calls = fk_calls;
}

// ──── Phase 3+: Higher-dimensional coordinate-wise sweep ─────────────────
//
// For k ≥ 3 free joints, iterative approach:
//   1. Start from current best configs in LinkExtremes
//   2. For each free joint j in turn, solve the 1D sub-problem
//      (atan2 as in Phase 1) while fixing all other free joints
//   3. Repeat for `max_sweeps` rounds
//
// This is analogous to block-coordinate descent but uses exact 1D solves.
// Not provably complete for k ≥ 3, but deterministic and cheap.

static void solve_interior(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    int max_free, int max_sweeps,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;

    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        int nj = std::min(V + 1, n);
        if (nj < 3 || nj > max_free) continue;

        // For each AABB face direction (6 directions)
        for (int face = 0; face < 6; ++face) {
            int dim = face / 2;
            bool is_min = (face % 2 == 0);

            // Start from best config for this face
            // Use sub-segment 0 (n_sub=1 typically)
            Eigen::VectorXd q = link_seg_ext[ci][0].configs[face];
            if (q.size() != n) continue;

            // Iterative coordinate-wise sweep
            for (int sweep = 0; sweep < max_sweeps; ++sweep) {
                bool improved = false;

                for (int j = 0; j < nj; ++j) {
                    double lo_j = intervals[j].lo, hi_j = intervals[j].hi;
                    if (hi_j - lo_j < 1e-12) continue;

                    // Sample 3 points along q_j (others fixed)
                    double mid_j = 0.5 * (lo_j + hi_j);
                    double qvals[3] = { lo_j, mid_j, hi_j };

                    // For both eval_links: V and V+1
                    for (int eval_link : {V + 1, V}) {
                        if (eval_link < 1) continue;

                        Eigen::Vector3d pts[3];
                        for (int si = 0; si < 3; ++si) {
                            double q_save = q[j];
                            q[j] = qvals[si];
                            auto pos = fk_link_positions(robot, q);
                            ++fk_calls;
                            int np = (int)pos.size();
                            if (eval_link >= np) { q[j] = q_save; goto next_eval; }
                            pts[si] = pos[eval_link];
                            q[j] = q_save;
                        }

                        {
                            Eigen::Matrix3d A;
                            for (int si = 0; si < 3; ++si) {
                                A(si, 0) = std::cos(qvals[si]);
                                A(si, 1) = std::sin(qvals[si]);
                                A(si, 2) = 1.0;
                            }
                            Eigen::Vector3d b_vec;
                            b_vec << pts[0][dim], pts[1][dim], pts[2][dim];
                            Eigen::Vector3d coeff = A.colPivHouseholderQr().solve(b_vec);

                            double alpha_c = coeff[0], beta_c = coeff[1];
                            if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                                continue;

                            double q_star = std::atan2(beta_c, alpha_c);
                            // For min: we want the minimum → largest negative contribution
                            // For max: we want the maximum → check both candidates
                            if (is_min) q_star += M_PI;  // atan2(β,α)+π gives the minimum

                            // Normalize to [lo, hi]
                            for (double shift : kShifts) {
                                double qtest = q_star + shift;
                                if (qtest >= lo_j - 1e-10 && qtest <= hi_j + 1e-10) {
                                    qtest = std::max(lo_j, std::min(hi_j, qtest));
                                    double old_j = q[j];
                                    q[j] = qtest;

                                    // Evaluate and update
                                    eval_and_update(robot, q, n_sub, inv_n,
                                                    map, n_act, link_seg_ext);
                                    ++fk_calls;
                                    ++found;
                                    improved = true;
                                    break;
                                }
                            }
                            // Also try the other candidate
                            double q_star2 = q_star + M_PI;
                            for (double shift : kShifts) {
                                double qtest = q_star2 + shift;
                                if (qtest >= lo_j - 1e-10 && qtest <= hi_j + 1e-10) {
                                    qtest = std::max(lo_j, std::min(hi_j, qtest));
                                    q[j] = qtest;
                                    eval_and_update(robot, q, n_sub, inv_n,
                                                    map, n_act, link_seg_ext);
                                    ++fk_calls;
                                    ++found;
                                    break;
                                }
                            }
                        }
                        next_eval:;
                    }
                }

                if (!improved) break;
            }
        }
    }

    if (n_found) *n_found = found;
    if (n_fk_calls) *n_fk_calls = fk_calls;
}

// ──── Helper: build background set for one joint ─────────────────────────
// Returns {lo, hi} and optionally kπ/2 values within (lo, hi).
static std::vector<double> build_bg_values(double lo, double hi, bool add_kpi2) {
    std::vector<double> bg = { lo, hi };
    if (add_kpi2) {
        for (int k = -20; k <= 20; ++k) {
            double a = k * HALF_PI;
            if (a > lo + 1e-12 && a < hi - 1e-12)
                bg.push_back(a);
        }
    }
    return bg;
}

// ──── Phase 2.5a: Pair-Constrained 1D Solver ─────────────────────────────
//
// For each pair (i,j) and each k s.t. qi+qj = kπ/2 lies in the feasible
// range, substitute qj = C − qi and solve the resulting 1-DOF problem:
//   p_d(qi) = α·cos(qi) + β·sin(qi) + γ
// using atan2, same as Phase 1 edges.
//
// Background: other joints on {lo, hi} (optionally + kπ/2 values).
// This specifically targets the ~46% of Phase 3 extrema that satisfy
// qi+qj = kπ/2 for some pair — dominant pairs being (0,2), (2,4), (2,5).

static void solve_pair_constrained_1d(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    const std::vector<std::pair<int,int>>& pairs,
    int max_bg,
    bool kpi2_backgrounds,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;

    // Pre-compute per-joint background value sets
    std::vector<std::vector<double>> bg_vals(n);
    for (int j = 0; j < n; ++j)
        bg_vals[j] = build_bg_values(intervals[j].lo, intervals[j].hi, kpi2_backgrounds);

    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        int nj = std::min(V + 1, n);

        for (auto& [pi, pj] : pairs) {
            if (pi >= nj || pj >= nj) continue;
            double lo_i = intervals[pi].lo, hi_i = intervals[pi].hi;
            double lo_j = intervals[pj].lo, hi_j = intervals[pj].hi;
            if (hi_i - lo_i < 1e-12 || hi_j - lo_j < 1e-12) continue;

            // Enumerate constraint values C = kπ/2 where qi+qj = C is feasible
            double sum_lo = lo_i + lo_j, sum_hi = hi_i + hi_j;
            int k_lo = (int)std::ceil(sum_lo / HALF_PI - 1e-9);
            int k_hi = (int)std::floor(sum_hi / HALF_PI + 1e-9);

            for (int kk = k_lo; kk <= k_hi; ++kk) {
                double C = kk * HALF_PI;

                // Effective qi range: [max(lo_i, C-hi_j), min(hi_i, C-lo_j)]
                double eff_lo = std::max(lo_i, C - hi_j);
                double eff_hi = std::min(hi_i, C - lo_j);
                if (eff_hi - eff_lo < 1e-12) continue;

                // Build backgrounds for other joints
                // Count total combos; skip if too many
                std::vector<std::vector<double>> other_bg;
                std::vector<int> other_idx;
                long long total_bg = 1;
                for (int jj = 0; jj < nj; ++jj) {
                    if (jj == pi || jj == pj) continue;
                    other_idx.push_back(jj);
                    other_bg.push_back(bg_vals[jj]);
                    total_bg *= (long long)bg_vals[jj].size();
                    if (total_bg > max_bg) break;
                }
                if (total_bg > max_bg) {
                    // Fall back to {lo, hi} only
                    other_bg.clear();
                    other_idx.clear();
                    total_bg = 1;
                    for (int jj = 0; jj < nj; ++jj) {
                        if (jj == pi || jj == pj) continue;
                        other_idx.push_back(jj);
                        other_bg.push_back({intervals[jj].lo, intervals[jj].hi});
                        total_bg *= 2;
                        if (total_bg > max_bg) {
                            total_bg = max_bg;
                            break;
                        }
                    }
                }

                int n_other = (int)other_idx.size();

                // Enumerate backgrounds via mixed-radix counter
                std::vector<int> bg_counter(n_other, 0);
                for (long long bg = 0; bg < total_bg; ++bg) {
                    Eigen::VectorXd q(n);
                    for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();

                    // Set background joints
                    for (int oi = 0; oi < n_other; ++oi)
                        q[other_idx[oi]] = other_bg[oi][bg_counter[oi]];

                    // Sample 3 qi values in effective range
                    double eff_mid = 0.5 * (eff_lo + eff_hi);
                    double qvals[3] = { eff_lo, eff_mid, eff_hi };

                    // For each eval_link (distal V+1, proximal V)
                    for (int eval_link : {V + 1, V}) {
                        if (eval_link < 1) continue;

                        Eigen::Vector3d pts[3];
                        bool ok = true;
                        for (int si = 0; si < 3; ++si) {
                            q[pi] = qvals[si];
                            q[pj] = C - qvals[si];
                            auto pos = fk_link_positions(robot, q);
                            ++fk_calls;
                            if (eval_link >= (int)pos.size()) { ok = false; break; }
                            pts[si] = pos[eval_link];
                        }
                        if (!ok) continue;

                        // Fit sinusoidal model and solve via atan2
                        Eigen::Matrix3d A;
                        for (int si = 0; si < 3; ++si) {
                            A(si, 0) = std::cos(qvals[si]);
                            A(si, 1) = std::sin(qvals[si]);
                            A(si, 2) = 1.0;
                        }
                        Eigen::Vector3d b_vec, coeff;
                        for (int d = 0; d < 3; ++d) {
                            b_vec << pts[0][d], pts[1][d], pts[2][d];
                            coeff = A.colPivHouseholderQr().solve(b_vec);
                            double alpha_c = coeff[0], beta_c = coeff[1];
                            if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                                continue;

                            double q_star = std::atan2(beta_c, alpha_c);
                            double candidates[2] = { q_star, q_star + M_PI };
                            if (candidates[1] > M_PI + 0.1) candidates[1] -= 2 * M_PI;

                            for (double qc : candidates) {
                                for (double shift : kShifts) {
                                    double qi_test = qc + shift;
                                    if (qi_test >= eff_lo - 1e-10 && qi_test <= eff_hi + 1e-10) {
                                        qi_test = std::max(eff_lo, std::min(eff_hi, qi_test));
                                        double qj_test = C - qi_test;
                                        if (qj_test < lo_j - 1e-10 || qj_test > hi_j + 1e-10)
                                            continue;
                                        qj_test = std::max(lo_j, std::min(hi_j, qj_test));

                                        q[pi] = qi_test;
                                        q[pj] = qj_test;
                                        eval_and_update(robot, q, n_sub, inv_n,
                                                        map, n_act, link_seg_ext);
                                        ++fk_calls;
                                        ++found;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    // Increment mixed-radix counter
                    for (int oi = n_other - 1; oi >= 0; --oi) {
                        if (++bg_counter[oi] < (int)other_bg[oi].size()) break;
                        bg_counter[oi] = 0;
                    }
                }
            }
        }
    }

    if (n_found) *n_found = found;
    if (n_fk_calls) *n_fk_calls = fk_calls;
}

// ──── Phase 2.5b: Pair-Constrained 2D Solver ─────────────────────────────
//
// For pair (i,j) with qi+qj = C = kπ/2, plus one additional free joint m:
//   Substitute qj = C − qi, so the 2 free variables are (qi, qm).
//   The FK function p_d(qi, qm) is bilinear-trigonometric (9-coeff model):
//     a1·Ci·Cm + a2·Ci·Sm + a3·Si·Cm + a4·Si·Sm + a5·Ci + a6·Si + a7·Cm + a8·Sm + a9
//   where Ci=cos(qi), Si=sin(qi), Cm=cos(qm), Sm=sin(qm).
//   (The coupling cos(C−qi) expands to cos(C)·cos(qi)+sin(C)·sin(qi),
//    which stays linear in Ci,Si — so the 9-coeff model is exact.)
//
//   We solve using the companion-matrix method from Phase 2 (degree-8 poly).

static void solve_pair_constrained_2d(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, float inv_n,
    const int* map, int n_act,
    const std::vector<std::pair<int,int>>& pairs,
    int max_bg,
    bool kpi2_backgrounds,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;

    std::vector<std::vector<double>> bg_vals(n);
    for (int j = 0; j < n; ++j)
        bg_vals[j] = build_bg_values(intervals[j].lo, intervals[j].hi, kpi2_backgrounds);

    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        int nj = std::min(V + 1, n);

        for (auto& [pi, pj] : pairs) {
            if (pi >= nj || pj >= nj) continue;
            double lo_i = intervals[pi].lo, hi_i = intervals[pi].hi;
            double lo_j = intervals[pj].lo, hi_j = intervals[pj].hi;
            if (hi_i - lo_i < 1e-12 || hi_j - lo_j < 1e-12) continue;

            double sum_lo = lo_i + lo_j, sum_hi = hi_i + hi_j;
            int k_lo = (int)std::ceil(sum_lo / HALF_PI - 1e-9);
            int k_hi = (int)std::floor(sum_hi / HALF_PI + 1e-9);

            for (int kk = k_lo; kk <= k_hi; ++kk) {
                double C = kk * HALF_PI;
                double eff_lo = std::max(lo_i, C - hi_j);
                double eff_hi = std::min(hi_i, C - lo_j);
                if (eff_hi - eff_lo < 1e-12) continue;

                // For each additional free joint m (not pi, pj)
                for (int pm = 0; pm < nj; ++pm) {
                    if (pm == pi || pm == pj) continue;
                    double lo_m = intervals[pm].lo, hi_m = intervals[pm].hi;
                    if (hi_m - lo_m < 1e-12) continue;

                    // Build backgrounds for remaining joints
                    std::vector<int> bg_idx;
                    std::vector<std::vector<double>> other_bg;
                    long long total_bg = 1;
                    for (int jj = 0; jj < nj; ++jj) {
                        if (jj == pi || jj == pj || jj == pm) continue;
                        bg_idx.push_back(jj);
                        other_bg.push_back(bg_vals[jj]);
                        total_bg *= (long long)bg_vals[jj].size();
                        if (total_bg > max_bg) break;
                    }
                    if (total_bg > max_bg) {
                        other_bg.clear(); bg_idx.clear(); total_bg = 1;
                        for (int jj = 0; jj < nj; ++jj) {
                            if (jj == pi || jj == pj || jj == pm) continue;
                            bg_idx.push_back(jj);
                            other_bg.push_back({intervals[jj].lo, intervals[jj].hi});
                            total_bg *= 2;
                            if (total_bg > max_bg) { total_bg = max_bg; break; }
                        }
                    }

                    int n_bg = (int)bg_idx.size();
                    std::vector<int> bg_cnt(n_bg, 0);

                    for (long long bgc = 0; bgc < total_bg; ++bgc) {
                        Eigen::VectorXd q(n);
                        for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();
                        for (int oi = 0; oi < n_bg; ++oi)
                            q[bg_idx[oi]] = other_bg[oi][bg_cnt[oi]];

                        // 3×3 sample grid: (qi, qm)
                        double qi_vals[3] = { eff_lo, 0.5*(eff_lo+eff_hi), eff_hi };
                        double qm_vals[3] = { lo_m, 0.5*(lo_m+hi_m), hi_m };

                        for (int eval_link : {V + 1, V}) {
                            if (eval_link < 1) continue;

                            Eigen::Matrix<double, 9, 9> A_mat;
                            Eigen::Matrix<double, 9, 3> B_mat;
                            int row = 0;
                            bool ok = true;
                            for (int si = 0; si < 3 && ok; ++si) {
                                double ci_v = std::cos(qi_vals[si]);
                                double si_v = std::sin(qi_vals[si]);
                                for (int sm = 0; sm < 3 && ok; ++sm) {
                                    double cm_v = std::cos(qm_vals[sm]);
                                    double sm_v = std::sin(qm_vals[sm]);
                                    q[pi] = qi_vals[si];
                                    q[pj] = C - qi_vals[si];  // pair constraint
                                    q[pm] = qm_vals[sm];
                                    auto pos = fk_link_positions(robot, q);
                                    ++fk_calls;
                                    if (eval_link >= (int)pos.size()) { ok = false; break; }

                                    A_mat(row, 0) = ci_v * cm_v;
                                    A_mat(row, 1) = ci_v * sm_v;
                                    A_mat(row, 2) = si_v * cm_v;
                                    A_mat(row, 3) = si_v * sm_v;
                                    A_mat(row, 4) = ci_v;
                                    A_mat(row, 5) = si_v;
                                    A_mat(row, 6) = cm_v;
                                    A_mat(row, 7) = sm_v;
                                    A_mat(row, 8) = 1.0;
                                    for (int d = 0; d < 3; ++d)
                                        B_mat(row, d) = pos[eval_link][d];
                                    ++row;
                                }
                            }
                            if (!ok || row < 9) continue;

                            auto qr = A_mat.colPivHouseholderQr();
                            for (int d = 0; d < 3; ++d) {
                                Eigen::VectorXd coeff = qr.solve(B_mat.col(d));
                                double a1=coeff[0], a2=coeff[1], a3=coeff[2], a4=coeff[3];
                                double a5=coeff[4], a6=coeff[5], a7=coeff[6], a8=coeff[7];

                                // Same companion-matrix approach as solve_faces:
                                // build F(tm) = 0 polynomial via sampling
                                double tm_samp[9], F_val[9];
                                for (int k = 0; k < 9; ++k)
                                    tm_samp[k] = -2.0 + k * 0.5;

                                for (int k = 0; k < 9; ++k) {
                                    double tm = tm_samp[k], tm2 = tm*tm;
                                    double den_m = 1.0 + tm2;
                                    double cm_v = (1.0 - tm2) / den_m;
                                    double sm_v = 2.0 * tm / den_m;

                                    double P = a3*cm_v + a4*sm_v + a6;
                                    double Q = a1*cm_v + a2*sm_v + a5;
                                    double A_t = Q*(-a1*sm_v + a2*cm_v)
                                               + P*(-a3*sm_v + a4*cm_v);
                                    double B_t = -a7*sm_v + a8*cm_v;
                                    F_val[k] = A_t*A_t - B_t*B_t*(P*P + Q*Q);
                                }

                                Eigen::Matrix<double,9,9> V_mat;
                                for (int k = 0; k < 9; ++k) {
                                    double pw = 1.0;
                                    for (int p = 0; p < 9; ++p) {
                                        V_mat(k, p) = pw;
                                        pw *= tm_samp[k];
                                    }
                                }
                                Eigen::Map<Eigen::Matrix<double,9,1>> F_vec(F_val);
                                Eigen::VectorXd pc = V_mat.colPivHouseholderQr().solve(F_vec);

                                double pa[9];
                                for (int p = 0; p < 9; ++p) pa[p] = pc[p];
                                auto tm_roots = solve_polynomial_real(pa, 8);

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
                                    for (double qi_try : qi_tries) {
                                        for (double shift : kShifts) {
                                            double qi_test = qi_try + shift;
                                            if (qi_test >= eff_lo - 1e-10 &&
                                                qi_test <= eff_hi + 1e-10) {
                                                qi_test = std::max(eff_lo, std::min(eff_hi, qi_test));
                                                double qj_test = C - qi_test;
                                                if (qj_test < lo_j - 1e-10 || qj_test > hi_j + 1e-10)
                                                    continue;
                                                qj_test = std::max(lo_j, std::min(hi_j, qj_test));

                                                q[pi] = qi_test;
                                                q[pj] = qj_test;
                                                q[pm] = qm_cand;
                                                eval_and_update(robot, q, n_sub, inv_n,
                                                                map, n_act, link_seg_ext);
                                                ++fk_calls;
                                                ++found;
                                                goto next_tm_root;
                                            }
                                        }
                                    }
                                    next_tm_root:;
                                }
                            }
                        }

                        // Increment mixed-radix counter
                        for (int oi = n_bg - 1; oi >= 0; --oi) {
                            if (++bg_cnt[oi] < (int)other_bg[oi].size()) break;
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

// ──── Improved Phase 3: Multi-start + Pair-Aware Coordinate Sweep ────────
//
// Enhancements over original solve_interior:
//   1. Multi-start: begin from current best AND from kπ/2-grid seeds
//   2. Pair-coupled sweep: when optimizing qi, also update qj s.t.
//      qi+qj stays on the nearest kπ/2 value (if it improves)
//   3. More aggressive sweeps with restart on plateau

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
    int* n_found, int* n_fk_calls)
{
    const int n = robot.n_joints();
    int found = 0, fk_calls = 0;

    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        int nj = std::min(V + 1, n);
        if (nj < 3 || nj > max_free) continue;

        // Build priority pairs limited to joints affecting this link
        std::vector<std::pair<int,int>> link_pairs;
        for (auto& [a, b] : priority_pairs) {
            if (a < nj && b < nj) link_pairs.push_back({a, b});
        }

        for (int face = 0; face < 6; ++face) {
            int dim = face / 2;
            bool is_min = (face % 2 == 0);

            // Collect starting configs
            std::vector<Eigen::VectorXd> starts;

            // Start 0: current best from Phase 0-2.5
            {
                Eigen::VectorXd q0 = link_seg_ext[ci][0].configs[face];
                if (q0.size() == n) starts.push_back(q0);
            }

            // Start 1..n_restarts: perturbed seeds at kπ/2 manifolds
            // For each priority pair (a,b), try setting qa+qb to nearest kπ/2
            for (int ri = 0; ri < n_restarts && ri < (int)link_pairs.size(); ++ri) {
                auto [pa, pb] = link_pairs[ri % link_pairs.size()];
                Eigen::VectorXd q_seed = starts.empty()
                    ? Eigen::VectorXd::Zero(n)
                    : starts[0];

                // Set qa+qb to nearest kπ/2, splitting evenly
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

            // Add kπ/2-midpoint seed
            if ((int)starts.size() < n_restarts + 2) {
                Eigen::VectorXd q_kpi2(n);
                for (int j = 0; j < n; ++j) {
                    // Pick nearest kπ/2 within [lo, hi], or midpoint
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

            // Run coordinate-wise sweep from each start
            for (auto& q_start : starts) {
                Eigen::VectorXd q = q_start;

                for (int sweep = 0; sweep < max_sweeps; ++sweep) {
                    bool improved = false;

                    for (int j = 0; j < nj; ++j) {
                        double lo_j = intervals[j].lo, hi_j = intervals[j].hi;
                        if (hi_j - lo_j < 1e-12) continue;

                        double mid_j = 0.5 * (lo_j + hi_j);
                        double qvals[3] = { lo_j, mid_j, hi_j };

                        for (int eval_link : {V + 1, V}) {
                            if (eval_link < 1) continue;

                            Eigen::Vector3d pts[3];
                            bool ok = true;
                            for (int si = 0; si < 3; ++si) {
                                double q_save = q[j];
                                q[j] = qvals[si];
                                auto pos = fk_link_positions(robot, q);
                                ++fk_calls;
                                if (eval_link >= (int)pos.size()) {
                                    q[j] = q_save; ok = false; break;
                                }
                                pts[si] = pos[eval_link];
                                q[j] = q_save;
                            }
                            if (!ok) continue;

                            Eigen::Matrix3d A;
                            for (int si = 0; si < 3; ++si) {
                                A(si, 0) = std::cos(qvals[si]);
                                A(si, 1) = std::sin(qvals[si]);
                                A(si, 2) = 1.0;
                            }
                            Eigen::Vector3d b_vec;
                            b_vec << pts[0][dim], pts[1][dim], pts[2][dim];
                            Eigen::Vector3d coeff = A.colPivHouseholderQr().solve(b_vec);
                            double alpha_c = coeff[0], beta_c = coeff[1];
                            if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15)
                                continue;

                            double q_star = std::atan2(beta_c, alpha_c);
                            if (is_min) q_star += M_PI;

                            for (double shift : kShifts) {
                                double qtest = q_star + shift;
                                if (qtest >= lo_j - 1e-10 && qtest <= hi_j + 1e-10) {
                                    qtest = std::max(lo_j, std::min(hi_j, qtest));
                                    q[j] = qtest;
                                    eval_and_update(robot, q, n_sub, inv_n,
                                                    map, n_act, link_seg_ext);
                                    ++fk_calls; ++found;
                                    improved = true;
                                    break;
                                }
                            }
                            // Also try the other candidate
                            double q_star2 = q_star + M_PI;
                            for (double shift : kShifts) {
                                double qtest = q_star2 + shift;
                                if (qtest >= lo_j - 1e-10 && qtest <= hi_j + 1e-10) {
                                    qtest = std::max(lo_j, std::min(hi_j, qtest));
                                    q[j] = qtest;
                                    eval_and_update(robot, q, n_sub, inv_n,
                                                    map, n_act, link_seg_ext);
                                    ++fk_calls; ++found;
                                    break;
                                }
                            }
                        }

                        // Pair-coupled update: if j is part of a priority pair,
                        // also adjust the paired joint to maintain qi+qj ≈ nearest kπ/2
                        if (pair_coupling) {
                            for (auto& [pa, pb] : link_pairs) {
                                int partner = -1;
                                if (pa == j) partner = pb;
                                else if (pb == j) partner = pa;
                                if (partner < 0 || partner >= nj) continue;

                                double sum_cur = q[j] + q[partner];
                                double nearest_c = std::round(sum_cur / HALF_PI) * HALF_PI;
                                if (std::abs(sum_cur - nearest_c) > 0.3) continue;

                                double new_partner = nearest_c - q[j];
                                if (new_partner >= intervals[partner].lo - 1e-10 &&
                                    new_partner <= intervals[partner].hi + 1e-10) {
                                    new_partner = std::max(intervals[partner].lo,
                                                  std::min(intervals[partner].hi, new_partner));
                                    q[partner] = new_partner;
                                    eval_and_update(robot, q, n_sub, inv_n,
                                                    map, n_act, link_seg_ext);
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

// ═════════════════════════════════════════════════════════════════════════════
//  derive_aabb_critical_analytical — full analytical pipeline
// ═════════════════════════════════════════════════════════════════════════════

void derive_aabb_critical_analytical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const AnalyticalCriticalConfig& config,
    float* out_aabb,
    AnalyticalCriticalStats* out_stats)
{
    assert(n_sub >= 1);
    const int n       = robot.n_joints();
    const int n_act   = robot.n_active_links();
    const int* map    = robot.active_link_map();
    const double* rad = robot.active_link_radii();

    const int total_slots = n_act * n_sub;
    const float inv_n = 1.0f / static_cast<float>(n_sub);

    // Per active-link, per sub-segment extremes tracker
    std::vector<std::vector<LinkExtremes>> link_seg_ext(n_act);
    for (int ci = 0; ci < n_act; ++ci) {
        link_seg_ext[ci].resize(n_sub);
        for (int s = 0; s < n_sub; ++s)
            link_seg_ext[ci][s].init(n);
    }

    AnalyticalCriticalStats stats{};

    // ──── Phase 0: kπ/2 baseline enumeration (vertices + standard crit) ──
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

        crit_enum(csets, q, 0, [&]() {
            ++stats.n_phase0_vertices;
            eval_and_update(robot, q, n_sub, inv_n, map, n_act, link_seg_ext);
        });
    }

    // ──── Phase 1: 1D edge critical points ───────────────────────────
    if (config.enable_edge_solve) {
        int edge_found = 0, edge_fk = 0;
        solve_edges(robot, intervals, n_sub, inv_n,
                    map, n_act, link_seg_ext,
                    &edge_found, &edge_fk);
        stats.n_phase1_edges = edge_found;
        stats.n_phase1_fk_calls = edge_fk;
    }

    // ──── dual_phase3 checkpoint: save Phase 0+1 state ───────────────
    // When dual_phase3 is enabled, we save the state after Phase 0+1
    // so we can later run Phase 3 from this checkpoint (no Phase 2 basin
    // shift), while also running Phase 3 from the full Phase 0+1+2+2.5 state.
    std::vector<std::vector<LinkExtremes>> ext_checkpoint_01;
    if (config.dual_phase3 && config.enable_interior_solve) {
        ext_checkpoint_01 = link_seg_ext;  // deep copy
    }

    // ──── Phase 2: 2D face critical points ───────────────────────────
    if (config.enable_face_solve) {
        // Build face pairs list
        std::vector<std::pair<int,int>> face_pairs;
        if (config.face_all_pairs) {
            // All C(n, 2) pairs
            for (int i = 0; i < n; ++i)
                for (int j = i + 1; j < n; ++j)
                    face_pairs.push_back({i, j});
        } else {
            // Only coupled pairs from robot config
            face_pairs = robot.coupled_pairs();
            // Also add adjacent pairs if not already present
            for (int i = 0; i < n - 1; ++i) {
                std::pair<int,int> p = {i, i+1};
                bool duplicate = false;
                for (auto& fp : face_pairs)
                    if (fp == p) { duplicate = true; break; }
                if (!duplicate) face_pairs.push_back(p);
            }
        }

        int face_found = 0, face_fk = 0;
        solve_faces(robot, intervals, n_sub, inv_n,
                    map, n_act, face_pairs, link_seg_ext,
                    &face_found, &face_fk);
        stats.n_phase2_faces = face_found;
        stats.n_phase2_fk_calls = face_fk;
    }

    // ──── Phase 2.5: Pair-constrained solvers ────────────────────────
    {
        // Build pair list for Phase 2.5
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
            // Add skip-1 pairs: (0,2), (2,4), (0,4), (2,5), (0,5)
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

        if (config.enable_pair_1d) {
            int p1d = 0, p1d_fk = 0;
            solve_pair_constrained_1d(robot, intervals, n_sub, inv_n,
                                      map, n_act, pc_pairs,
                                      config.pair_max_bg,
                                      config.pair_kpi2_backgrounds,
                                      link_seg_ext,
                                      &p1d, &p1d_fk);
            stats.n_phase25a_pair1d = p1d;
            p25_fk += p1d_fk;
        }

        if (config.enable_pair_2d) {
            int p2d = 0, p2d_fk = 0;
            solve_pair_constrained_2d(robot, intervals, n_sub, inv_n,
                                      map, n_act, pc_pairs,
                                      config.pair_max_bg,
                                      config.pair_kpi2_backgrounds,
                                      link_seg_ext,
                                      &p2d, &p2d_fk);
            stats.n_phase25b_pair2d = p2d;
            p25_fk += p2d_fk;
        }

        stats.n_phase25_fk_calls = p25_fk;
    }

    // ──── Phase 3+: Higher-dimensional interior ──────────────────────
    if (config.enable_interior_solve) {
        // Build pair list for improved interior
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
            // Add skip-1 pairs (0,2), (2,4), etc. — dominant from Exp-24
            for (int i = 0; i < n - 2; i += 2) {
                std::pair<int,int> p = {i, i+2};
                bool dup = false;
                for (auto& fp : int_pairs) if (fp == p) { dup = true; break; }
                if (!dup) int_pairs.push_back(p);
            }
        }

        int int_found = 0, int_fk = 0;
        if (config.improved_interior) {
            solve_interior_improved(robot, intervals, n_sub, inv_n,
                                    map, n_act,
                                    config.interior_max_free,
                                    config.interior_max_sweeps,
                                    config.interior_n_restarts,
                                    config.interior_pair_coupling,
                                    int_pairs,
                                    link_seg_ext,
                                    &int_found, &int_fk);
        } else {
            solve_interior(robot, intervals, n_sub, inv_n,
                           map, n_act,
                           config.interior_max_free,
                           config.interior_max_sweeps,
                           link_seg_ext,
                           &int_found, &int_fk);
        }
        stats.n_phase3_interior = int_found;
        stats.n_phase3_fk_calls = int_fk;

        // ── dual_phase3: run Phase 3 again from Phase 0+1 checkpoint ──
        // This avoids the "basin shift" where Phase 2's companion-matrix
        // results change Phase 3's starting configuration, sometimes
        // converging to a worse local optimum.
        if (config.dual_phase3 && !ext_checkpoint_01.empty()) {
            int int_found2 = 0, int_fk2 = 0;
            if (config.improved_interior) {
                solve_interior_improved(robot, intervals, n_sub, inv_n,
                                        map, n_act,
                                        config.interior_max_free,
                                        config.interior_max_sweeps,
                                        config.interior_n_restarts,
                                        config.interior_pair_coupling,
                                        int_pairs,
                                        ext_checkpoint_01,
                                        &int_found2, &int_fk2);
            } else {
                solve_interior(robot, intervals, n_sub, inv_n,
                               map, n_act,
                               config.interior_max_free,
                               config.interior_max_sweeps,
                               ext_checkpoint_01,
                               &int_found2, &int_fk2);
            }
            // Merge checkpoint results into main state: min(lo), max(hi)
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

    if (out_stats) *out_stats = stats;
}

// ═════════════════════════════════════════════════════════════════════════════
//  derive_aabb_critical_analytical_with_configs — same pipeline + config output
// ═════════════════════════════════════════════════════════════════════════════

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

    const int total_slots = n_act * n_sub;
    const float inv_n = 1.0f / static_cast<float>(n_sub);

    std::vector<std::vector<LinkExtremes>> link_seg_ext(n_act);
    for (int ci = 0; ci < n_act; ++ci) {
        link_seg_ext[ci].resize(n_sub);
        for (int s = 0; s < n_sub; ++s)
            link_seg_ext[ci][s].init(n);
    }

    AnalyticalCriticalStats stats{};

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
        crit_enum(csets, q, 0, [&]() {
            ++stats.n_phase0_vertices;
            eval_and_update(robot, q, n_sub, inv_n, map, n_act, link_seg_ext);
        });
    }

    // Phase 1
    if (config.enable_edge_solve) {
        int ef = 0, efk = 0;
        solve_edges(robot, intervals, n_sub, inv_n, map, n_act, link_seg_ext, &ef, &efk);
        stats.n_phase1_edges = ef;
        stats.n_phase1_fk_calls = efk;
    }

    // Phase 2
    if (config.enable_face_solve) {
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
        solve_faces(robot, intervals, n_sub, inv_n, map, n_act, face_pairs,
                    link_seg_ext, &ff, &ffk);
        stats.n_phase2_faces = ff;
        stats.n_phase2_fk_calls = ffk;
    }

    // Phase 2.5: pair-constrained
    {
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
        if (config.enable_pair_1d) {
            int p1d = 0, p1d_fk = 0;
            solve_pair_constrained_1d(robot, intervals, n_sub, inv_n,
                                      map, n_act, pc_pairs,
                                      config.pair_max_bg,
                                      config.pair_kpi2_backgrounds,
                                      link_seg_ext, &p1d, &p1d_fk);
            stats.n_phase25a_pair1d = p1d;
            p25_fk += p1d_fk;
        }
        if (config.enable_pair_2d) {
            int p2d = 0, p2d_fk = 0;
            solve_pair_constrained_2d(robot, intervals, n_sub, inv_n,
                                      map, n_act, pc_pairs,
                                      config.pair_max_bg,
                                      config.pair_kpi2_backgrounds,
                                      link_seg_ext, &p2d, &p2d_fk);
            stats.n_phase25b_pair2d = p2d;
            p25_fk += p2d_fk;
        }
        stats.n_phase25_fk_calls = p25_fk;
    }

    // Phase 3+
    if (config.enable_interior_solve) {
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
        if (config.improved_interior) {
            solve_interior_improved(robot, intervals, n_sub, inv_n,
                                    map, n_act,
                                    config.interior_max_free,
                                    config.interior_max_sweeps,
                                    config.interior_n_restarts,
                                    config.interior_pair_coupling,
                                    int_pairs,
                                    link_seg_ext, &ii, &ifk);
        } else {
            solve_interior(robot, intervals, n_sub, inv_n, map, n_act,
                           config.interior_max_free, config.interior_max_sweeps,
                           link_seg_ext, &ii, &ifk);
        }
        stats.n_phase3_interior = ii;
        stats.n_phase3_fk_calls = ifk;
    }

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
                // face mapping: ext.configs[0]=x_min, [1]=x_max, [2]=y_min,
                //   [3]=y_max, [4]=z_min, [5]=z_max
                for (int face = 0; face < 6; ++face)
                    out_configs[slot * 6 + face] = ext.configs[face];
            }
        }
    }

    if (out_stats) *out_stats = stats;
}

} // namespace envelope
} // namespace sbf
