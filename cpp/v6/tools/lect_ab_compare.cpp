#include <sbf/lect/lect_io.h>
#include <sbf/core/robot.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>

namespace fs = std::filesystem;
using namespace sbf;

struct Agg {
    std::uint64_t a_bytes = 0;
    std::uint64_t b_bytes = 0;
    int count = 0;
};

struct Record {
    std::string path;
    std::string robot;
    std::string difficulty;
    std::string scene;
    std::string variant;
    std::uint64_t a_bytes = 0;
    std::uint64_t b_bytes = 0;
};

static std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        if (c == '\\' || c == '"') {
            out.push_back('\\');
            out.push_back(c);
        } else if (c == '\n') {
            out += "\\n";
        } else {
            out.push_back(c);
        }
    }
    return out;
}

static bool parse_scene_info(const fs::path& p,
                             std::string& robot,
                             std::string& difficulty,
                             std::string& scene_key) {
    static const std::regex kSceneRe("(ur5|panda)_(easy|medium|hard)_[0-9]{2}");
    std::string s = p.generic_string();
    std::smatch m;
    if (!std::regex_search(s, m, kSceneRe)) return false;
    scene_key = m.str(0);
    robot = m.str(1);
    difficulty = m.str(2);
    return true;
}

static bool load_robot_for_group(const std::string& robot_name, Robot& robot) {
    if (robot_name == "ur5") {
        robot = Robot::from_json("data/ur5.json");
        return true;
    }
    if (robot_name == "panda") {
        robot = Robot::from_json("data/panda.json");
        return true;
    }
    return false;
}

int main(int argc, char** argv) {
    fs::path input_root = "experiments/results_paper/exp5_random_scenes/lect_cache";
    fs::path output_json = "experiments/results_paper/lect_ab_real_exp5.json";
    fs::path temp_dir = "experiments/results_paper/_lect_ab_tmp";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--input-root" && i + 1 < argc) {
            input_root = argv[++i];
        } else if (a == "--output-json" && i + 1 < argc) {
            output_json = argv[++i];
        } else if (a == "--temp-dir" && i + 1 < argc) {
            temp_dir = argv[++i];
        } else if (a == "--help") {
            std::cout << "Usage: lect_ab_compare [--input-root DIR] [--output-json FILE] [--temp-dir DIR]\\n";
            return 0;
        }
    }

    if (!fs::exists(input_root)) {
        std::cerr << "Input root not found: " << input_root << "\\n";
        return 1;
    }

    fs::create_directories(temp_dir);

    std::map<std::string, Robot> robots;
    std::vector<Record> records;
    std::map<std::string, Agg> by_robot;
    std::map<std::string, Agg> by_difficulty;
    std::map<std::string, Agg> by_scene;
    std::map<std::string, Agg> by_variant;

    int idx = 0;
    int skipped = 0;
    for (auto it = fs::recursive_directory_iterator(input_root);
         it != fs::recursive_directory_iterator(); ++it) {
        if (!it->is_regular_file()) continue;
        const fs::path in_path = it->path();
        if (in_path.extension() != ".lect") continue;

        std::string robot_name, difficulty, scene;
        if (!parse_scene_info(in_path, robot_name, difficulty, scene)) {
            ++skipped;
            continue;
        }

        if (!robots.count(robot_name)) {
            Robot robot;
            if (!load_robot_for_group(robot_name, robot)) {
                ++skipped;
                continue;
            }
            robots.emplace(robot_name, std::move(robot));
        }

        LECT lect;
        if (!lect_load_binary(lect, robots.at(robot_name), in_path.string())) {
            ++skipped;
            continue;
        }

        fs::path a_path = temp_dir / ("ab_" + std::to_string(idx) + "_A.lect");
        fs::path b_path = temp_dir / ("ab_" + std::to_string(idx) + "_B.lect");
        ++idx;

        setenv("SBF_LECT_FORCE_2CH", "1", 1);
        if (!lect_save_binary(lect, a_path.string())) {
            ++skipped;
            continue;
        }
        unsetenv("SBF_LECT_FORCE_2CH");
        if (!lect_save_binary(lect, b_path.string())) {
            ++skipped;
            continue;
        }

        std::uint64_t a_bytes = static_cast<std::uint64_t>(fs::file_size(a_path));
        std::uint64_t b_bytes = static_cast<std::uint64_t>(fs::file_size(b_path));

        Record rec;
        rec.path = in_path.generic_string();
        rec.robot = robot_name;
        rec.difficulty = difficulty;
        rec.scene = scene;
        rec.variant = (in_path.generic_string().find("/ifk/") != std::string::npos)
            ? "ifk"
            : "non_ifk";
        rec.a_bytes = a_bytes;
        rec.b_bytes = b_bytes;
        records.push_back(rec);

        by_robot[robot_name].a_bytes += a_bytes;
        by_robot[robot_name].b_bytes += b_bytes;
        by_robot[robot_name].count += 1;

        by_difficulty[difficulty].a_bytes += a_bytes;
        by_difficulty[difficulty].b_bytes += b_bytes;
        by_difficulty[difficulty].count += 1;

        by_scene[scene].a_bytes += a_bytes;
        by_scene[scene].b_bytes += b_bytes;
        by_scene[scene].count += 1;

        by_variant[rec.variant].a_bytes += a_bytes;
        by_variant[rec.variant].b_bytes += b_bytes;
        by_variant[rec.variant].count += 1;
    }

    unsetenv("SBF_LECT_FORCE_2CH");

    auto pct_reduce = [](std::uint64_t a, std::uint64_t b) {
        if (a == 0) return 0.0;
        return 100.0 * static_cast<double>(a - b) / static_cast<double>(a);
    };

    std::ofstream out(output_json);
    if (!out.is_open()) {
        std::cerr << "Failed to write json: " << output_json << "\\n";
        return 2;
    }

    out << "{\n";
    out << "  \"input_root\": \"" << json_escape(input_root.generic_string()) << "\",\n";
    out << "  \"records_count\": " << records.size() << ",\n";
    out << "  \"skipped\": " << skipped << ",\n";

    out << "  \"by_robot\": {\n";
    {
        bool first = true;
        for (const auto& kv : by_robot) {
            if (!first) out << ",\n";
            first = false;
            out << "    \"" << json_escape(kv.first) << "\": {\"count\": " << kv.second.count
                << ", \"a_bytes\": " << kv.second.a_bytes
                << ", \"b_bytes\": " << kv.second.b_bytes
                << ", \"reduction_pct\": " << std::fixed << std::setprecision(4)
                << pct_reduce(kv.second.a_bytes, kv.second.b_bytes) << "}";
        }
        out << "\n";
    }
    out << "  },\n";

    out << "  \"by_variant\": {\n";
    {
        bool first = true;
        for (const auto& kv : by_variant) {
            if (!first) out << ",\n";
            first = false;
            out << "    \"" << json_escape(kv.first) << "\": {\"count\": " << kv.second.count
                << ", \"a_bytes\": " << kv.second.a_bytes
                << ", \"b_bytes\": " << kv.second.b_bytes
                << ", \"reduction_pct\": " << std::fixed << std::setprecision(4)
                << pct_reduce(kv.second.a_bytes, kv.second.b_bytes) << "}";
        }
        out << "\n";
    }
    out << "  },\n";

    out << "  \"by_difficulty\": {\n";
    {
        bool first = true;
        for (const auto& kv : by_difficulty) {
            if (!first) out << ",\n";
            first = false;
            out << "    \"" << json_escape(kv.first) << "\": {\"count\": " << kv.second.count
                << ", \"a_bytes\": " << kv.second.a_bytes
                << ", \"b_bytes\": " << kv.second.b_bytes
                << ", \"reduction_pct\": " << std::fixed << std::setprecision(4)
                << pct_reduce(kv.second.a_bytes, kv.second.b_bytes) << "}";
        }
        out << "\n";
    }
    out << "  },\n";

    out << "  \"by_scene\": {\n";
    {
        bool first = true;
        for (const auto& kv : by_scene) {
            if (!first) out << ",\n";
            first = false;
            out << "    \"" << json_escape(kv.first) << "\": {\"count\": " << kv.second.count
                << ", \"a_bytes\": " << kv.second.a_bytes
                << ", \"b_bytes\": " << kv.second.b_bytes
                << ", \"reduction_pct\": " << std::fixed << std::setprecision(4)
                << pct_reduce(kv.second.a_bytes, kv.second.b_bytes) << "}";
        }
        out << "\n";
    }
    out << "  },\n";

    out << "  \"records\": [\n";
    for (size_t i = 0; i < records.size(); ++i) {
        const auto& r = records[i];
        out << "    {\"path\": \"" << json_escape(r.path)
            << "\", \"robot\": \"" << json_escape(r.robot)
            << "\", \"difficulty\": \"" << json_escape(r.difficulty)
            << "\", \"scene\": \"" << json_escape(r.scene)
            << "\", \"variant\": \"" << json_escape(r.variant)
            << "\", \"a_bytes\": " << r.a_bytes
            << ", \"b_bytes\": " << r.b_bytes
            << ", \"reduction_pct\": " << std::fixed << std::setprecision(4)
            << pct_reduce(r.a_bytes, r.b_bytes) << "}";
        if (i + 1 < records.size()) out << ",";
        out << "\n";
    }
    out << "  ]\n";
    out << "}\n";

    auto print_agg = [&](const std::string& title,
                         const std::map<std::string, Agg>& agg) {
        std::cout << "\\n[" << title << "]\\n";
        for (const auto& kv : agg) {
            std::cout << "  " << kv.first
                      << " count=" << kv.second.count
                      << " A=" << kv.second.a_bytes
                      << " B=" << kv.second.b_bytes
                      << " reduce=" << std::fixed << std::setprecision(2)
                      << pct_reduce(kv.second.a_bytes, kv.second.b_bytes)
                      << "%\\n";
        }
    };

    std::uint64_t total_a = 0, total_b = 0;
    for (const auto& r : records) {
        total_a += r.a_bytes;
        total_b += r.b_bytes;
    }

    std::cout << "Processed=" << records.size() << " skipped=" << skipped << "\\n";
    std::cout << "TOTAL A=" << total_a << " B=" << total_b
              << " reduce=" << std::fixed << std::setprecision(2)
              << pct_reduce(total_a, total_b) << "%\\n";
    print_agg("by_robot", by_robot);
    print_agg("by_difficulty", by_difficulty);
    print_agg("by_variant", by_variant);
    std::cout << "\\nJSON written: " << output_json << "\\n";
    return 0;
}
