# Experiments

SafeBoxForest v5 论文实验集合。

## 目录结构

```
experiments/
├── scripts/          # 实验运行脚本
│   ├── run_all.py           # 全部实验一键运行 (S1→S2→S5→S3→S4→gen)
│   ├── run_s1.py            # 仅运行 S1 (Envelope Tightness)
│   ├── run_s3s4_fast.py     # S3+S4 快速子集 (仅 IFK+CritSample)
│   ├── run_s3s4_isolated.py # S3+S4 子进程隔离模式 (防内存累积)
│   ├── run_all_wrapper.py   # PowerShell Start-Process 适配器
│   └── gen_paper.py         # 生成表格+图形+编译 PDF
├── results/          # 实验结果
│   ├── s1_envelope_tightness/
│   ├── s2_envelope_timing/
│   ├── s3_e2e/
│   ├── s4_baselines/
│   └── s5_scalability/
├── tools/            # 调试辅助工具
│   ├── check_cp.py          # 检查 S3 checkpoint
│   ├── monitor.py           # 轮询实验进度
│   └── test_crash.py        # 崩溃复现测试
└── doc/              # 实验文档 (EXP1–EXP7)
    ├── README.md
    └── EXP{1-7}_*.md
```

## 实验列表

| # | 实验 | 脚本阶段 | 论文表格/图 | 状态 |
|---|------|----------|-------------|------|
| 1 | Envelope Volume Tightness | S1 | Table 1 | ✅ Full (500 boxes) |
| 2 | Envelope Timing | S2 | Table 2 | ✅ Full (1000×50) |
| 3 | E2E Planning | S3 | Table 3, Fig 2 | 🔄 部分 (630/1080 IFK+CritSample) |
| 4 | Baselines | S4 | Table 4, Fig 3 | ❌ 待运行 |
| 5 | Scalability | S5 | Table 7, Fig 4 | ✅ Full |
| 6 | LECT Cache | 独立 bench | Table 6 | ✅ 完整 |

## 运行方式

所有脚本从 **v5/ 根目录** 运行:

```powershell
cd safeboxforest/v5
$env:PYTHONPATH = "build_x64/Release;python"

# 全部运行
python experiments/scripts/run_all.py

# 仅 S3+S4 (IFK+CritSample)
python experiments/scripts/run_s3s4_fast.py

# 仅 S1
python experiments/scripts/run_s1.py
```

## 加速方案

所有实验脚本自动使用 C++ 加速后端:
- **IFK**: ~5 μs (DH-chain decomposition)
- **CritSample**: ~22 μs (DH-based analytical + early exit)
- **Analytical**: ~27 μs @ width≤0.15 rad (early exit), ~15,432 μs (full solver)
- **GCPC**: ~27.6 μs @ width≤0.10 rad (early exit), ~13,013 μs (full lookup)
- **LECT Cache**: 始终启用, 2–9× 加速 (IFK/CritSample)

无 Python fallback — 所有 endpoint source 和 envelope 计算均通过 `_sbf5_cpp.pyd` C++ 绑定执行。
