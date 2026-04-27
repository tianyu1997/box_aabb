# Repository Instructions

- Preserve the original v6 paper entry files at `cpp/v6/doc/box_aabb_v6_paper_en.tex` and `cpp/v6/doc/box_aabb_v6_paper_zh.tex` unless the task explicitly asks to edit them.
- For the current paper-writing pivot, keep the working v7-based draft under `cpp/v6/doc/v7_based_draft/`; edit `cpp/v6/doc/v7_based_draft/sbf_paper_en.tex` and `cpp/v6/doc/v7_based_draft/sbf_paper_zh.tex` instead of overwriting the root v6 paper files.
- For Exp.3 SBF reproduction, use `cpp/v7/experiments/scripts/run_exp3_reproducible.py` as the single entry point.
- The authoritative paper SBF cached-query artifact is `cpp/v7/experiments/results_nightly/full/marcucci.json`.
- Treat live `exp_marcucci_cached` reruns as diagnostic unless the task explicitly asks to debug or change the live cached runner.
