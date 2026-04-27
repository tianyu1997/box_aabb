#include "sbf/util/timer.h"

namespace sbf::util {

void Timer::start() {
    t0_ = std::chrono::steady_clock::now();
}

double Timer::elapsed_ms() const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(now - t0_).count();
}

double Timer::elapsed_s() const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - t0_).count();
}

}  // namespace sbf::util
