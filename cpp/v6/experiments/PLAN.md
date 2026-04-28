# v6 实验工作约定

这份文件不再维护逐条实验待办，而只保留 `cpp/v6/experiments` 的使用约定，避免继续把旧版本运行计划、旧环境命令和当前目录结构混在一起。

## 当前路由

- 论文实验入口在 `paper/`，默认从 `paper/run_all.py` 开始。
- C++ 实验源码入口是本目录下的 `exp*.cpp`。
- 批处理型历史 runner 在 `scripts/`；它们适合做阶段性诊断，不是当前 paper 的首选入口。
- 历史计划和分项说明在 `doc/`；仅作背景材料，不再视为当前执行手册。

## 结果口径

- `results_paper/` 是当前论文结果目录。
- 其他 `results_*` 目录默认按 rerun、quickcheck、临时验证、历史保留理解；除非脚本显式指定，不自动视为论文权威结果。
- 若一个实验同时存在 `paper/` 包装层与根目录 `../scripts/` 实现，则以前者负责论文输出口径，后者负责权威实现或底层复现逻辑。

## 当前约束

- 默认只在 `cpp/v6` 内 rerun 论文实验；`cpp/v7` 仅用于显式要求的诊断。
- Exp.3 的权威 SBF build/query 在 `../scripts/run_online_query_comparison.py`，paper-facing 入口在 `paper/04_e2e_baselines_combined.py`。
- 当前论文主线已移除 LinkIAABB-Grid；活跃比较只保留 `LinkIAABB` 与 `Hull16_Grid`。

## 维护原则

- 优先补 README 和入口说明，不轻易重命名已被结果目录或脚本引用的实验文件。
- 若目录里保留了重复编号、冻结脚本或历史快照，优先在 README 中标出“当前入口/历史保留”，而不是直接删除。
