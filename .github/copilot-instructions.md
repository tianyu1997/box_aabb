# Repository Instructions

- Preserve the original v6 paper entry files at `cpp/v6/doc/box_aabb_v6_paper_en.tex` and `cpp/v6/doc/box_aabb_v6_paper_zh.tex` unless the task explicitly asks to edit them.
- Keep active paper edits under `cpp/v6/doc/paper/`; edit `cpp/v6/doc/paper/sbf_paper_en.tex` and `cpp/v6/doc/paper/sbf_paper_zh.tex` instead of recreating `cpp/v6/doc/v7_based_draft/`.
- Keep active experiment reruns in `cpp/v6`; do not route paper experiments or baseline reproduction through `cpp/v7` unless the task explicitly asks for a v7 diagnostic.
- For Exp.3 SBF reproduction, use `cpp/v6/experiments/paper/04_e2e_baselines_combined.py` as the paper-facing entry point and `cpp/v6/scripts/run_online_query_comparison.py` as the authoritative SBF build/query implementation.
- The authoritative paper SBF cached-query artifact is `cpp/v6/experiments/results_paper/marcucci_combined.json`.
- LinkIAABB-Grid is retired from the active v6 paper/experiment pipeline; prefer `LinkIAABB` and `Hull16_Grid` in current experiments unless the task explicitly asks for legacy archaeology.
