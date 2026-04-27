#pragma once
#include <chrono>

namespace sbf::util {

// Monotonic wall-clock timer.
class Timer {
public:
    void start();
    double elapsed_ms() const;
    double elapsed_s() const;

private:
    std::chrono::steady_clock::time_point t0_{};
};

}  // namespace sbf::util
