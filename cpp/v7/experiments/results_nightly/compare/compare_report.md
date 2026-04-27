# v7 P7 — planner comparison

### 2dof_box

| planner | SR | avg_t_ms | avg_path_len | n_succ |
| --- | --- | --- | --- | --- |
| v7 SBF (full opt) | 0.667 | 16.4 | 2.336 | 2 |
| v7 OMPL RRT-Connect | 1.000 | 1.2 | 1.414 | 3 |
| Drake GCS (1-region) | 1.000 | 143.8 | 1.414 | 3 |

*v7 SBF time-per-box on `2dof_box`: **0.1445 ms/box***

### iiwa14_far

| planner | SR | avg_t_ms | avg_path_len | n_succ |
| --- | --- | --- | --- | --- |
| v7 SBF (full opt) | 1.000 | 16.2 | 1.055 | 3 |
| v7 OMPL RRT-Connect | 1.000 | 1.2 | 0.583 | 3 |
| Drake GCS (1-region) | 1.000 | 449.9 | 0.583 | 3 |

*v7 SBF time-per-box on `iiwa14_far`: **0.0815 ms/box***


## v6 anchor (Marcucci IIWA14, 16 obstacles)

| metric | v6 SBF |
| --- | --- |
| build_ms (median) | 444.0 |
| per_box_ms        | 0.3200 |


## v7-vs-v6 per-box throughput

| scene | v7 SBF ms/box | v6 SBF ms/box | ratio (v6 / v7) |
| --- | --- | --- | --- |
| 2dof_box | 0.1445 | 0.3200 | 2.21× |
| iiwa14_far | 0.0815 | 0.3200 | 3.93× |
