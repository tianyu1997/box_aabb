// SafeBoxForest v6 — GCPC Single-Tier Cache (Phase B4)
//
// Analytical boundary (Phase 0-2) + cached interior critical points.
// Single-tier flat storage (no v4 4-tier hierarchy).
#include <sbf/envelope/gcpc_source.h>
#include <sbf/envelope/analytical_source.h>

#include <cassert>
#include <cstring>
#include <fstream>

namespace sbf {

// ─── GcpcCache implementation ───────────────────────────────────────────────

void GcpcCache::add_point(const Eigen::VectorXd& q) {
    if (n_dims_ == 0) {
        n_dims_ = static_cast<int>(q.size());
    }
    assert(static_cast<int>(q.size()) == n_dims_);
    points_.push_back(q);
}

std::vector<Eigen::VectorXd> GcpcCache::lookup(
    const std::vector<Interval>& intervals) const
{
    std::vector<Eigen::VectorXd> result;
    int n = static_cast<int>(intervals.size());
    for (const auto& pt : points_) {
        if (static_cast<int>(pt.size()) != n) continue;
        bool inside = true;
        for (int j = 0; j < n; ++j) {
            if (pt[j] < intervals[j].lo - 1e-12 ||
                pt[j] > intervals[j].hi + 1e-12) {
                inside = false;
                break;
            }
        }
        if (inside) {
            result.push_back(pt);
        }
    }
    return result;
}

// Binary format: [magic(4B)] [n_dims(4B)] [n_points(4B)] [flat doubles]
static constexpr uint32_t GCPC_MAGIC = 0x47435043;  // "GCPC"

void GcpcCache::save(const std::string& path) const {
    std::ofstream ofs(path, std::ios::binary);
    assert(ofs.is_open());

    uint32_t magic = GCPC_MAGIC;
    uint32_t nd = static_cast<uint32_t>(n_dims_);
    uint32_t np = static_cast<uint32_t>(points_.size());

    ofs.write(reinterpret_cast<const char*>(&magic), 4);
    ofs.write(reinterpret_cast<const char*>(&nd), 4);
    ofs.write(reinterpret_cast<const char*>(&np), 4);

    for (const auto& pt : points_) {
        ofs.write(reinterpret_cast<const char*>(pt.data()),
                  n_dims_ * static_cast<int>(sizeof(double)));
    }
}

GcpcCache GcpcCache::load(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    assert(ifs.is_open());

    GcpcCache cache;
    uint32_t magic = 0, nd = 0, np = 0;
    ifs.read(reinterpret_cast<char*>(&magic), 4);
    assert(magic == GCPC_MAGIC);
    ifs.read(reinterpret_cast<char*>(&nd), 4);
    ifs.read(reinterpret_cast<char*>(&np), 4);

    cache.n_dims_ = static_cast<int>(nd);
    cache.points_.resize(np);
    for (uint32_t i = 0; i < np; ++i) {
        cache.points_[i].resize(nd);
        ifs.read(reinterpret_cast<char*>(cache.points_[i].data()),
                 nd * sizeof(double));
    }
    return cache;
}

// ─── Scalar FK for expanding GCPC cache points ─────────────────────────────
namespace {

void scalar_fk_pos(const Robot& robot, const double* q,
                   double positions[][3])
{
    const auto& dh = robot.dh_params();
    int n = robot.n_joints();
    double T[16];
    for (int i = 0; i < 16; ++i) T[i] = 0.0;
    T[0] = T[5] = T[10] = T[15] = 1.0;
    positions[0][0] = 0.0; positions[0][1] = 0.0; positions[0][2] = 0.0;

    for (int j = 0; j < n; ++j) {
        double d_val = (dh[j].joint_type == 1) ? (q[j] + dh[j].d) : dh[j].d;
        double angle = (dh[j].joint_type == 0) ? (q[j] + dh[j].theta) : dh[j].theta;
        double ct = std::cos(angle), st = std::sin(angle);
        double ca = std::cos(dh[j].alpha), sa = std::sin(dh[j].alpha);
        double A[16] = {
            ct,      -st,      0.0,  dh[j].a,
            st*ca,    ct*ca,  -sa,  -d_val*sa,
            st*sa,    ct*sa,   ca,   d_val*ca,
            0.0,      0.0,    0.0,   1.0
        };
        double R[16];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                R[r*4+c] = T[r*4]*A[c] + T[r*4+1]*A[4+c]
                          + T[r*4+2]*A[8+c] + T[r*4+3]*A[12+c];
        R[12] = 0.0; R[13] = 0.0; R[14] = 0.0; R[15] = 1.0;
        for (int i = 0; i < 16; ++i) T[i] = R[i];
        positions[j+1][0] = T[3];
        positions[j+1][1] = T[7];
        positions[j+1][2] = T[11];
    }

    // Tool frame
    if (robot.has_tool()) {
        const auto& tool_f = *robot.tool_frame();
        double ct = std::cos(tool_f.theta), st = std::sin(tool_f.theta);
        double ca = std::cos(tool_f.alpha), sa = std::sin(tool_f.alpha);
        double A[16] = {
            ct,      -st,      0.0,  tool_f.a,
            st*ca,    ct*ca,  -sa,  -tool_f.d*sa,
            st*sa,    ct*sa,   ca,   tool_f.d*ca,
            0.0,      0.0,    0.0,   1.0
        };
        double R[16];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                R[r*4+c] = T[r*4]*A[c] + T[r*4+1]*A[4+c]
                          + T[r*4+2]*A[8+c] + T[r*4+3]*A[12+c];
        R[12] = 0.0; R[13] = 0.0; R[14] = 0.0; R[15] = 1.0;
        for (int i = 0; i < 16; ++i) T[i] = R[i];
        positions[n+1][0] = T[3];
        positions[n+1][1] = T[7];
        positions[n+1][2] = T[11];
    }
}

void expand_endpoints(const double positions[][3],
                      const int* alm, int n_act, float* out)
{
    for (int ci = 0; ci < n_act; ++ci) {
        int V = alm[ci];
        float* p = out + (ci * 2) * 6;
        for (int d = 0; d < 3; ++d) {
            float v = static_cast<float>(positions[V][d]);
            if (v < p[d])     p[d]     = v;
            if (v > p[d + 3]) p[d + 3] = v;
        }
        float* dd = out + (ci * 2 + 1) * 6;
        for (int d = 0; d < 3; ++d) {
            float v = static_cast<float>(positions[V + 1][d]);
            if (v < dd[d])     dd[d]     = v;
            if (v > dd[d + 3]) dd[d + 3] = v;
        }
    }
}

}  // anonymous namespace

EndpointIAABBResult compute_endpoint_iaabb_gcpc(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const GcpcCache& cache)
{
    // Step 1: Analytical boundary (Phase 0-2)
    EndpointIAABBResult result = compute_endpoint_iaabb_analytical(
        robot, intervals, /*max_phase=*/2);
    result.source = EndpointSource::GCPC;

    // Step 2: Expand with cached interior critical points
    if (!cache.empty()) {
        auto hits = cache.lookup(intervals);
        double positions[MAX_TF][3];
        const int n_act = result.n_active_links;
        const int* alm = robot.active_link_map();

        for (const auto& pt : hits) {
            scalar_fk_pos(robot, pt.data(), positions);
            expand_endpoints(positions, alm, n_act,
                             result.endpoint_iaabbs.data());
        }
    }

    return result;
}

}  // namespace sbf
