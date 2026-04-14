#pragma once
/// @file log.h
/// @brief Lightweight logging macros for SafeBoxForest.
///
/// Usage:
///   SBF_INFO("[PLN] lect=%.0fms (nodes=%d)", time_ms, n_nodes);
///   SBF_DEBUG("[GRW] miss at (%.3f, %.3f)", q[0], q[1]);
///
/// Control:
///   sbf::set_log_level(sbf::LogLevel::SILENT);   // compile-time-style
///   SBF_LOG_LEVEL=0 ./my_binary                   // env override
///
/// The macros expand to a single branch + fprintf; when the level is below
/// the threshold, the format arguments are not evaluated.

#include <cstdio>
#include <cstdlib>

namespace sbf {

enum class LogLevel : int {
    SILENT = 0,
    ERROR  = 1,
    WARN   = 2,
    INFO   = 3,
    DEBUG  = 4,
    TRACE  = 5
};

inline LogLevel& global_log_level() {
    static LogLevel lvl = LogLevel::INFO;
    return lvl;
}

inline void set_log_level(LogLevel lvl) { global_log_level() = lvl; }

inline void init_log_level_from_env() {
    if (const char* e = std::getenv("SBF_LOG_LEVEL")) {
        int v = std::atoi(e);
        if (v >= 0 && v <= 5) global_log_level() = static_cast<LogLevel>(v);
    }
}

}  // namespace sbf

// Core macro — checks level, formats to stderr, appends newline.
#define SBF_LOG(level, fmt, ...)                                               \
    do {                                                                       \
        if (static_cast<int>(::sbf::global_log_level()) >=                     \
            static_cast<int>(level))                                           \
            std::fprintf(stderr, fmt "\n", ##__VA_ARGS__);                     \
    } while (0)

#define SBF_ERROR(fmt, ...) SBF_LOG(::sbf::LogLevel::ERROR, fmt, ##__VA_ARGS__)
#define SBF_WARN(fmt, ...)  SBF_LOG(::sbf::LogLevel::WARN,  fmt, ##__VA_ARGS__)
#define SBF_INFO(fmt, ...)  SBF_LOG(::sbf::LogLevel::INFO,  fmt, ##__VA_ARGS__)
#define SBF_DEBUG(fmt, ...) SBF_LOG(::sbf::LogLevel::DEBUG, fmt, ##__VA_ARGS__)
#define SBF_TRACE(fmt, ...) SBF_LOG(::sbf::LogLevel::TRACE, fmt, ##__VA_ARGS__)
