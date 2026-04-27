# v7 P7 — planner comparison

### 2dof_box

| planner | SR | avg_t_ms | avg_path_len | n_succ |
| --- | --- | --- | --- | --- |
| v7 SBF (full opt) | 1.000 | 33.4 | 1.728 | 20 |
| v7 OMPL RRT-Connect | 1.000 | 1.2 | 1.414 | 20 |
| Drake GCS (1-region) | 1.000 | 147.5 | 1.414 | 20 |

*v7 SBF time-per-box on `2dof_box`: **0.1910 ms/box***

### iiwa14_far

| planner | SR | avg_t_ms | avg_path_len | n_succ |
| --- | --- | --- | --- | --- |
| v7 SBF (full opt) | 1.000 | 90.3 | 0.978 | 20 |
| v7 OMPL RRT-Connect | 1.000 | 1.3 | 0.583 | 20 |
| Drake GCS (1-region) | 1.000 | 453.3 | 0.583 | 20 |

*v7 SBF time-per-box on `iiwa14_far`: **0.1119 ms/box***


## v6 anchor (Marcucci IIWA14, 16 obstacles)

| metric | v6 SBF |
| --- | --- |
| build_ms (median) | 444.0 |
| per_box_ms        | 0.3200 |


## v7-vs-v6 per-box throughput

| scene | v7 SBF ms/box | v6 SBF ms/box | ratio (v6 / v7) |
| --- | --- | --- | --- |
| 2dof_box | 0.1910 | 0.3200 | 1.68× |
| iiwa14_far | 0.1119 | 0.3200 | 2.86× |
