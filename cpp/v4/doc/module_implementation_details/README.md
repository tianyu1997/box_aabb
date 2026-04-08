# v4 模块实现细节文档

本目录包含你指定的 5 个核心模块的详细实现说明：

1. endpoint_iaabb_details.md
2. link_iaabb_details.md
3. lect_details.md
4. ffb_details.md
5. sbf_grower_details.md

文档重点覆盖：
- 核心数据结构与内存布局
- 主要执行流程
- 关键优化点
- 缓存与增量更新机制
- 碰撞与安全语义
- 已知边界与 TODO 状态
- 建议扩展与测试方向

参考代码目录：
- include/sbf/**
- src/envelope/**
- src/forest/**
- src/core/**
