// ═══════════════════════════════════════════════════════════════════════════
//  Quick benchmark: compare all 3×3 pipeline combinations (time + volume)
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/envelope/frame_source.h"
#include "sbf/envelope/envelope_type.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

static const char* ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

struct Timer {
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point t0;
    void start() { t0 = Clock::now(); }
    double us() const {
        return std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
    }
};

int main() {
    Robot robot = Robot::from_json(ROBOT_PATH);
    const int N = 50;  // trials
    std::mt19937 rng(42);

    // Generate random C-space boxes: 40% narrow, 40% medium, 20% wide
    auto lim = robot.joint_limits().limits;
    int nj = robot.n_joints();
    std::vector<std::vector<Interval>> boxes(N);
    for (int t = 0; t < N; ++t) {
        double frac;
        if (t < N * 4 / 10)      frac = 0.05;
        else if (t < N * 8 / 10) frac = 0.15;
        else                      frac = 0.40;
        boxes[t].resize(nj);
        for (int j = 0; j < nj; ++j) {
            std::uniform_real_distribution<double> d(lim[j].lo, lim[j].hi);
            double c = d(rng);
            double hw = (lim[j].hi - lim[j].lo) * frac * 0.5;
            boxes[t][j] = {std::max(lim[j].lo, c - hw),
                           std::min(lim[j].hi, c + hw)};
        }
    }

    const char* src_names[] = {"IFK", "CritSample", "AnalyticalCrit", "GCPC"};
    const char* env_names[] = {"SubAABB", "SubAABB_Grid", "Hull16_Grid"};

    std::cout << "\n";
    std::cout << "+--------------------------------+---------------+---------------+--------------------+\n";
    std::cout << "|  Pipeline Benchmark: " << N << " trials (AnalyticalCrit: 5), robot=" << nj << "DOF, "
              << robot.n_active_links() << " links  |\n";
    std::cout << "+--------------------------------+---------------+---------------+--------------------+\n";
    std::cout << "|  Pipeline                      |  Volume (m^3) |  Source (us)  |  Envelope (us)     |\n";
    std::cout << "+--------------------------------+---------------+---------------+--------------------+\n";

    for (int s = 0; s < 3; ++s) {
        auto src_method = static_cast<FrameSourceMethod>(s);
        FrameSourceConfig src_cfg;
        if (s == 0) src_cfg = FrameSourceConfig::ifk();
        if (s == 1) src_cfg = FrameSourceConfig::crit_sampling();
        if (s == 2) src_cfg = FrameSourceConfig::analytical_critical();

        // AnalyticalCrit is ~1000x slower, use fewer trials
        int n_trials = (s == 2) ? 5 : N;

        for (int e = 0; e < 3; ++e) {
            auto env_type = static_cast<EnvelopeType>(e);
            auto env_cfg = default_envelope_config(src_method, env_type);

            char label[40];
            snprintf(label, sizeof(label), "%s+%s", src_names[s], env_names[e]);
            std::cerr << "[DBG] Starting " << label << " ..." << std::flush;

            double total_src_us = 0, total_env_us = 0, total_vol = 0;
            Timer t;

            try {
                for (int trial = 0; trial < n_trials; ++trial) {
                    // Source
                    t.start();
                    auto src_res = compute_frame_source(
                        src_cfg, robot, boxes[trial]);
                    total_src_us += t.us();

                    // Envelope
                    t.start();
                    auto env_res = compute_envelope_repr(env_cfg, robot, src_res);
                    total_env_us += t.us();

                    total_vol += env_res.volume;
                }
            } catch (const std::exception& ex) {
                std::cerr << " EXCEPTION: " << ex.what() << std::endl;
                std::cout << "|  " << std::left << std::setw(29) << label
                          << "|  EXCEPTION   |               |                    |\n";
                std::cout << std::flush;
                continue;
            }

            double avg_src_us = total_src_us / n_trials;
            double avg_env_us = total_env_us / n_trials;
            double avg_vol    = total_vol / n_trials;

            std::cerr << " done." << std::endl;

            std::cout << "|  " << std::left << std::setw(29) << label
                      << "|  " << std::right << std::setw(11) << std::fixed
                      << std::setprecision(4) << avg_vol
                      << "  |  " << std::setw(11) << std::setprecision(1)
                      << avg_src_us
                      << "  |  " << std::setw(16) << std::setprecision(1)
                      << avg_env_us << "  |\n";
            std::cout << std::flush;
        }

        if (s < 2) {
            std::cout << "+--------------------------------+---------------+---------------+--------------------+\n";
            std::cout << std::flush;
        }
    }

    std::cout << "+--------------------------------+---------------+---------------+--------------------+\n";
    std::cout << "\n";
    std::cout << std::flush;

    return 0;
}
