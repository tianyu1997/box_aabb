# v7 P6 experiment results


### main — 2dof_box

| scene | robot | trials | SR | n_succ | avg_t_ms | avg_opt_len | avg_raw_len |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 2dof_box | 2dof_planar | 20 | 1.000 | 20 | 33.4 | 1.728 | 2.244 |



### main — iiwa14_far

| scene | robot | trials | SR | n_succ | avg_t_ms | avg_opt_len | avg_raw_len |
| --- | --- | --- | --- | --- | --- | --- | --- |
| iiwa14_far | iiwa14 | 20 | 1.000 | 20 | 90.3 | 0.978 | 1.470 |



### threads — iiwa14_far

| n_threads | n_succ | avg_t_ms | speedup_vs_1t |
| --- | --- | --- | --- |
| 1 | 20 | 656.5 | 1.00 |
| 2 | 20 | 221.9 | 2.96 |
| 4 | 20 | 144.6 | 4.54 |
| 8 | 20 | 106.0 | 6.19 |



### pathopt_steps — iiwa14_far

| combo | n_succ | avg_opt_len | avg_raw_len | len_ratio | avg_opt_t_ms |
| --- | --- | --- | --- | --- | --- |
| raw | 20 | 1.470 | 1.470 | 1.000 | 0.0 |
| greedy | 20 | 1.124 | 1.470 | 0.764 | 0.1 |
| greedy+final | 20 | 1.124 | 1.470 | 0.764 | 0.2 |
| full | 20 | 0.978 | 1.470 | 0.665 | 3.9 |

