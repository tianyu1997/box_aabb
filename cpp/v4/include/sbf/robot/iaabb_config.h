// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — IAABBConfig: DH → iAABB 预处理配置
//  Module: sbf::robot
//
//  通用 DH 参数预处理器，将任意 N-DOF 串联 DH 链转换为专用于
//  interval AABB 生成的紧凑配置。预处理包括：
//
//    1. 活跃链接识别
//       - Link 0（基座坐标系）始终跳过
//       - 链接长度 = sqrt(a² + d²) < threshold 的链接视为零长度，跳过
//       - 默认 threshold = 0.1（长度小于 10 cm 的链接视为零长度）
//
//    2. 配对端点布局 (paired endpoint layout)
//       - n_active_endpoints = n_active × 2（每个活跃链接的近端 + 远端）
//       - 消除非活跃端点的存储浪费
//
//    3. 逐链接关节裁剪元数据
//       - n_affecting_joints[ci]：影响活跃链接 ci 位置的关节数量
//       - 用于 iFK 每链接提前终止
//
//    4. iFK 截止帧
//       - last_active_frame：任何活跃链接引用的最高帧索引
//       - iFK 无需计算超出此帧的前缀变换
//
//  使用方式:
//    Robot robot = Robot::from_json("panda.json");
//    IAABBConfig cfg = IAABBConfig::from_robot(robot);
//    cfg.save_json("panda_iaabb.json");   // 持久化
//    // ... 后续 iAABB 代码只使用 cfg ...
//    auto cfg2 = IAABBConfig::load_json("panda_iaabb.json");
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <string>
#include <vector>

namespace sbf {

class Robot;  // forward declaration

struct IAABBConfig {

    // ── 构造参数 ─────────────────────────────────────────────────────
    /// 零长度阈值：sqrt(a² + d²) < threshold 的链接视为零长度并跳过。
    double zero_length_threshold = 0.1;

    // ── 机器人维度（拷贝自 Robot，使配置自包含）────────────────────────
    int  n_joints     = 0;
    bool has_tool     = false;
    int  n_total_links = 0;     // n_joints + has_tool

    // ── 活跃链接元数据 ──────────────────────────────────────────────
    int n_active           = 0;   // 非跳过链接数量
    int n_active_endpoints = 0;   // = n_active × 2 (配对布局)
    int last_active_frame  = 0;   // 活跃链接引用的最大帧索引 + 1

    /// active_link_map[ci] = 活跃链接 ci 的原始 DH 帧索引。
    /// 例如 Panda (threshold=0.1): [2, 4], IIWA14: [2, 4, 7]。
    std::vector<int> active_link_map;

    /// 紧凑顺序的胶囊半径（按 ci 索引）。
    std::vector<double> active_link_radii;

    /// 每个活跃链接的 DH 长度 = sqrt(a² + d²)。
    std::vector<double> active_link_lengths;

    /// n_affecting_joints[ci] = 从关节 0 开始影响活跃链接 ci 位置的关节数。
    /// 对于串联 DH 链：= min(active_link_map[ci] + 1, n_joints)。
    /// iFK 可据此跳过对更远关节的重算。
    std::vector<int> n_affecting_joints;

    // ── 全链状态（用于调试和检查）──────────────────────────────────
    /// 所有链接的 DH 长度（索引 0..n_total_links-1）。
    std::vector<double> all_link_lengths;

    /// 每个链接是否活跃（true）或被跳过（false）。
    std::vector<bool> link_is_active;

    // ── 工厂方法 ─────────────────────────────────────────────────────
    /// 从 Robot 的 DH 参数构建 iAABB 配置。
    /// 通用方法，适用于任意 N-DOF 串联 DH 链。
    static IAABBConfig from_robot(const Robot& robot,
                                  double threshold = 0.1);

    // ── JSON 序列化 / 反序列化 ──────────────────────────────────────
    void save_json(const std::string& path) const;
    static IAABBConfig load_json(const std::string& path);

    // ── 摘要字符串（用于日志/调试）──────────────────────────────────
    std::string summary() const;
};

} // namespace sbf
