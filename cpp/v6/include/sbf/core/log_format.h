#pragma once
/// @file log_format.h
/// @brief Helpers for formatting commonly-logged objects (Eigen vectors,
///        intervals, FFB results) into compact one-line strings suitable
///        for SBF_TRACE / SBF_DEBUG output.

#include <Eigen/Core>
#include <cstdio>
#include <string>
#include <vector>

#include <sbf/core/types.h>

namespace sbf {

/// Format an Eigen vector as `[a,b,c,...]` with %.3f precision.  Truncates
/// to `max_dims` entries when the vector is wider (appends "...").  Caller
/// owns the returned std::string.
inline std::string fmt_vec(const Eigen::Ref<const Eigen::VectorXd>& v,
                           int max_dims = 8, int precision = 3) {
    char buf[32];
    std::string out;
    out.reserve(v.size() * 10 + 4);
    out.push_back('[');
    int n = static_cast<int>(v.size());
    int shown = std::min(n, max_dims);
    for (int i = 0; i < shown; ++i) {
        std::snprintf(buf, sizeof(buf), "%.*f", precision, v[i]);
        out += buf;
        if (i + 1 < shown) out.push_back(',');
    }
    if (n > shown) out += ",...";
    out.push_back(']');
    return out;
}

/// Format interval vector as "[(-3.14,3.14),(-3.14,1.57),...]"
/// with {lo,hi} per interval, precision %.3f by default.
template<typename Container>
inline std::string fmt_intervals(const Container& ivs, int precision = 3) {
    char buf[64];
    std::string out;
    out.reserve(ivs.size() * 20 + 4);
    out.push_back('[');
    for (size_t i = 0; i < ivs.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "(%.*f,%.*f)",
                      precision, ivs[i].lo, precision, ivs[i].hi);
        out += buf;
        if (i + 1 < ivs.size()) out.push_back(',');
    }
    out.push_back(']');
    return out;
}

}  // namespace sbf
