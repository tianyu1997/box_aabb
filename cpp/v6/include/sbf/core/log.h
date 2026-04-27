#pragma once
/// @file log.h
/// @brief Lightweight logging macros for SafeBoxForest.
///
/// Usage:
///   SBF_INFO("[PLN] lect=%.0fms (nodes=%d)", time_ms, n_nodes);
///   SBF_DEBUG("[GRW] miss at (%.3f, %.3f)", q[0], q[1]);
///   SBF_TRACE("[FFB] step=%d node=%d depth=%d", k, idx, d);  // very verbose
///
/// Control:
///   sbf::set_log_level(sbf::LogLevel::SILENT);   // compile-time-style
///   SBF_LOG_LEVEL=0 ./my_binary                   // env override
///   SBF_LOG_FILE=/tmp/build.log ./my_binary       // redirect log to file
///
/// At program startup call `sbf::init_log_from_env()` to honour both env
/// variables.  The macros expand to a single branch + fprintf; when the
/// level is below the threshold, the format arguments are not evaluated.

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef SBF_LOG_DIR
#define SBF_LOG_DIR "log"
#endif

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

/// Mutable log sink.  Defaults to stderr; can be redirected to a file
/// (usually via `set_log_file`).  When set to a file, the file handle is
/// owned by `set_log_file` (closed on next call).
inline std::FILE*& global_log_sink() {
    static std::FILE* sink = stderr;
    return sink;
}

inline std::FILE*& global_owned_log_file() {
    static std::FILE* owned = nullptr;
    return owned;
}

/// Redirect log output to a file (append mode).  Pass nullptr / empty to
/// revert to stderr.  The previous owned file (if any) is closed.
inline bool set_log_file(const char* path) {
    if (auto& owned = global_owned_log_file(); owned) {
        std::fclose(owned); owned = nullptr;
    }
    if (!path || !*path) {
        global_log_sink() = stderr;
        return true;
    }
    std::FILE* f = std::fopen(path, "w");
    if (!f) {
        global_log_sink() = stderr;
        std::fprintf(stderr, "[LOG] failed to open %s; staying on stderr\n",
                     path);
        return false;
    }
    std::setvbuf(f, nullptr, _IOLBF, 0);  // line-buffered for crash safety
    global_owned_log_file() = f;
    global_log_sink() = f;
    return true;
}

inline void init_log_level_from_env() {
    if (const char* e = std::getenv("SBF_LOG_LEVEL")) {
        int v = std::atoi(e);
        if (v >= 0 && v <= 5) global_log_level() = static_cast<LogLevel>(v);
    }
}

inline void init_log_file_from_env() {
    if (const char* e = std::getenv("SBF_LOG_FILE")) {
        if (!*e) return;
        // Escape hatches: keep stderr.
        if (std::string(e) == "-" || std::string(e) == "stderr") {
            global_log_sink() = stderr;
            return;
        }
        set_log_file(e);
        return;
    }
    // Default: write to <SBF_LOG_DIR>/sbf_<YYYYmmdd_HHMMSS>_<pid>.log
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
    // Best-effort mkdir (ignore errors; set_log_file will fall back to stderr).
    ::mkdir(SBF_LOG_DIR, 0755);
    char path[1024];
    std::snprintf(path, sizeof(path), "%s/sbf_%s_pid%d.log",
                  SBF_LOG_DIR, ts, (int)::getpid());
    set_log_file(path);
    std::fprintf(stderr, "[LOG] writing to %s\n", path);
}

/// Convenience: honour both SBF_LOG_LEVEL and SBF_LOG_FILE in one call.
inline void init_log_from_env() {
    init_log_level_from_env();
    init_log_file_from_env();
}

}  // namespace sbf

// Core macro — checks level, formats to the configured sink, appends newline.
#define SBF_LOG(level, fmt, ...)                                               \
    do {                                                                       \
        if (static_cast<int>(::sbf::global_log_level()) >=                     \
            static_cast<int>(level))                                           \
            std::fprintf(::sbf::global_log_sink(),                             \
                         fmt "\n", ##__VA_ARGS__);                             \
    } while (0)

#define SBF_ERROR(fmt, ...) SBF_LOG(::sbf::LogLevel::ERROR, fmt, ##__VA_ARGS__)
#define SBF_WARN(fmt, ...)  SBF_LOG(::sbf::LogLevel::WARN,  fmt, ##__VA_ARGS__)
#define SBF_INFO(fmt, ...)  SBF_LOG(::sbf::LogLevel::INFO,  fmt, ##__VA_ARGS__)
#define SBF_DEBUG(fmt, ...) SBF_LOG(::sbf::LogLevel::DEBUG, fmt, ##__VA_ARGS__)
#define SBF_TRACE(fmt, ...) SBF_LOG(::sbf::LogLevel::TRACE, fmt, ##__VA_ARGS__)
