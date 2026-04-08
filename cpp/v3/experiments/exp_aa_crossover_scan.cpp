// ═══════════════════════════════════════════════════════════════════════════
//  Quick scan: AA crossover threshold vs volume / timing
//
//  Uses the SAME trial generation as exp_pipeline_benchmark (seed=42,
//  40% narrow / 40% medium / 20% wide).
//
//  For each crossover value, computes IFK+SubAABB(n_sub=1) volume using
//  the hybrid IA/AA path.  Reports:
//    - # trials where AA was triggered (max_w ≤ crossover)
//    - avg volume (SubAABB, n_sub=1)
//    - avg frame compute time
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/robot/affine_fk.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/common/types.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

struct Timer {
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point t0;
    void start() { t0 = Clock::now(); }
    double us() const {
        return std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
    }
};

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <robot.json> [n_trials=50]\n", argv[0]);
        return 1;
    }

    Robot robot = Robot::from_json(argv[1]);
    int N = (argc >= 3) ? std::atoi(argv[2]) : 50;
    int n_joints = robot.n_joints();
    int n_act = robot.n_active_links();
    int n_frames = n_joints + (robot.has_tool() ? 1 : 0);
    const int* alm = robot.active_link_map();
    const double* lr = robot.active_link_radii();

    // ── Same trial generation as exp_pipeline_benchmark ──
    std::mt19937 rng(42);
    const auto& lim = robot.joint_limits().limits;

    std::discrete_distribution<int> width_dist({40, 40, 20});
    const double fracs[] = {0.05, 0.20, 0.60};

    struct Trial {
        std::vector<Interval> ivs;
        double max_w;
        int frac_idx;
    };
    std::vector<Trial> trials(N);

    for (int i = 0; i < N; ++i) {
        int fi = width_dist(rng);
        double frac = fracs[fi];
        trials[i].ivs.resize(n_joints);
        trials[i].max_w = 0;
        trials[i].frac_idx = fi;
        for (int j = 0; j < n_joints; ++j) {
            double lo = lim[j].lo, hi = lim[j].hi, range = hi - lo;
            std::uniform_real_distribution<double> d(lo, hi);
            double c = d(rng);
            double hw = range * frac * 0.5;
            trials[i].ivs[j].lo = std::max(lo, c - hw);
            trials[i].ivs[j].hi = std::min(hi, c + hw);
            double w = trials[i].ivs[j].hi - trials[i].ivs[j].lo;
            trials[i].max_w = std::max(trials[i].max_w, w);
        }
    }

    // ── Print per-trial max width distribution ──
    std::printf("=== Per-trial max_interval_width distribution ===\n");
    std::printf("  Narrow (frac=0.05): ");
    int cnt[3] = {};
    double min_w[3] = {1e30, 1e30, 1e30};
    double max_w[3] = {0, 0, 0};
    for (int i = 0; i < N; ++i) {
        int fi = trials[i].frac_idx;
        cnt[fi]++;
        min_w[fi] = std::min(min_w[fi], trials[i].max_w);
        max_w[fi] = std::max(max_w[fi], trials[i].max_w);
    }
    const char* names[] = {"Narrow(0.05)", "Medium(0.20)", "Wide(0.60)"};
    for (int fi = 0; fi < 3; ++fi)
        std::printf("  %s: n=%d, max_w in [%.4f, %.4f]\n",
                    names[fi], cnt[fi], min_w[fi], max_w[fi]);

    // sorted max_w histogram (full percentiles)
    std::vector<double> all_w(N);
    for (int i = 0; i < N; ++i) all_w[i] = trials[i].max_w;
    std::sort(all_w.begin(), all_w.end());
    std::printf("\n  Sorted max_w (all %d values):\n    ", N);
    for (int i = 0; i < N; ++i) {
        std::printf("%.4f ", all_w[i]);
        if ((i + 1) % 10 == 0) std::printf("\n    ");
    }
    std::printf("\n\n  Percentiles: p10=%.4f  p25=%.4f  p50=%.4f  p75=%.4f  p90=%.4f  max=%.4f\n\n",
        all_w[N/10], all_w[N/4], all_w[N/2], all_w[3*N/4], all_w[9*N/10], all_w.back());

    // ── Sweep crossover values (fine grid around sweet spot) ──
    std::vector<double> crossovers;
    // coarse: 0
    crossovers.push_back(0.0);
    // fine grid 0.05 .. 0.50 step 0.01
    for (double v = 0.05; v <= 0.501; v += 0.01)
        crossovers.push_back(v);
    // coarse tail
    for (double v : {0.60, 0.75, 1.00, 1.50, 2.00, 3.00})
        crossovers.push_back(v);
    int n_co = static_cast<int>(crossovers.size());

    float base_pos[3] = {0.f, 0.f, 0.f};
    std::vector<float> lrf(n_act);
    for (int i = 0; i < n_act; ++i) lrf[i] = lr ? static_cast<float>(lr[i]) : 0.f;

    std::printf("%-10s %6s %10s %10s %10s %10s %10s %10s\n",
                "crossover", "n_AA", "avg_vol", "avg_us", "aa_us", "ia_us", "vol_ratio", "spd_ratio");
    std::printf("---------- ------ ---------- ---------- ---------- ---------- ---------- ----------\n");

    // Baseline: pure IA (crossover=0)
    double baseline_vol = 0;
    double baseline_us = 0;

    for (int ci = 0; ci < n_co; ++ci) {
        double co = crossovers[ci];
        int n_aa = 0;
        double sum_vol = 0;
        double sum_us = 0;
        double sum_aa_us = 0;
        double sum_ia_us = 0;
        int cnt_aa = 0, cnt_ia = 0;

        for (int t = 0; t < N; ++t) {
            Timer tm; tm.start();

            std::vector<float> frames(n_frames * 6);
            bool used_aa = false;

            if (co > 0 && trials[t].max_w <= co) {
                // AA path
                auto aa_fk = aa_compute_fk(robot, trials[t].ivs);
                for (int k = 0; k < n_frames; ++k) {
                    int fi = k + 1;
                    double lo3[3], hi3[3];
                    aa_position_bounds(aa_fk.prefix[fi], lo3, hi3);
                    frames[k*6+0] = static_cast<float>(lo3[0]);
                    frames[k*6+1] = static_cast<float>(lo3[1]);
                    frames[k*6+2] = static_cast<float>(lo3[2]);
                    frames[k*6+3] = static_cast<float>(hi3[0]);
                    frames[k*6+4] = static_cast<float>(hi3[1]);
                    frames[k*6+5] = static_cast<float>(hi3[2]);
                }
                used_aa = true;
            } else {
                // IA path
                auto fk = compute_fk_full(robot, trials[t].ivs);
                for (int k = 0; k < n_frames; ++k) {
                    int fi = k + 1;
                    frames[k*6+0] = static_cast<float>(fk.prefix_lo[fi][3]);
                    frames[k*6+1] = static_cast<float>(fk.prefix_lo[fi][7]);
                    frames[k*6+2] = static_cast<float>(fk.prefix_lo[fi][11]);
                    frames[k*6+3] = static_cast<float>(fk.prefix_hi[fi][3]);
                    frames[k*6+4] = static_cast<float>(fk.prefix_hi[fi][7]);
                    frames[k*6+5] = static_cast<float>(fk.prefix_hi[fi][11]);
                }
            }

            // Compute SubAABB volume (n_sub=1)
            double vol = 0;
            for (int ci2 = 0; ci2 < n_act; ++ci2) {
                int parent_fi = (alm[ci2] == 0) ? -1 : alm[ci2] - 1;
                int link_fi = alm[ci2];
                float r = lrf[ci2];
                float aabb[6];
                derive_aabb_subdivided(frames.data(), n_frames,
                                       parent_fi, link_fi,
                                       1, r, base_pos, aabb);
                double dx = std::max(0.0, double(aabb[3] - aabb[0]));
                double dy = std::max(0.0, double(aabb[4] - aabb[1]));
                double dz = std::max(0.0, double(aabb[5] - aabb[2]));
                vol += dx * dy * dz;
            }

            double elapsed = tm.us();
            sum_vol += vol;
            sum_us += elapsed;
            if (used_aa) {
                ++n_aa;
                sum_aa_us += elapsed;
                cnt_aa++;
            } else {
                sum_ia_us += elapsed;
                cnt_ia++;
            }
        }

        double avg_vol = sum_vol / N;
        double avg_us = sum_us / N;
        double avg_aa_us = cnt_aa > 0 ? sum_aa_us / cnt_aa : 0;
        double avg_ia_us = cnt_ia > 0 ? sum_ia_us / cnt_ia : 0;
        if (ci == 0) { baseline_vol = avg_vol; baseline_us = avg_us; }
        double vol_ratio = (baseline_vol > 0) ? avg_vol / baseline_vol : 1.0;
        double spd_ratio = (baseline_us > 0) ? avg_us / baseline_us : 1.0;

        std::printf("%-10.2f %6d %10.6f %10.1f %10.1f %10.1f %10.4f %10.4f\n",
                    co, n_aa, avg_vol, avg_us, avg_aa_us, avg_ia_us, vol_ratio, spd_ratio);
    }

    return 0;
}
