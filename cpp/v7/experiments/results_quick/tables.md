# v7 P6 experiment results


### main — 2dof_box

| scene | robot | trials | SR | n_succ | avg_t_ms | avg_opt_len | avg_raw_len |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 2dof_box | 2dof_planar | 3 | 1.000 | 3 | 15.6 | 2.122 | 2.442 |



### main — iiwa14_far

| scene | robot | trials | SR | n_succ | avg_t_ms | avg_opt_len | avg_raw_len |
| --- | --- | --- | --- | --- | --- | --- | --- |
| iiwa14_far | iiwa14 | 3 | 1.000 | 3 | 17.1 | 1.055 | 1.447 |



### pathopt_steps — iiwa14_far

| combo | n_succ | avg_opt_len | avg_raw_len | len_ratio | avg_opt_t_ms |
| --- | --- | --- | --- | --- | --- |
| raw | 3 | 1.447 | 1.447 | 1.000 | 0.0 |
| greedy | 3 | 1.055 | 1.447 | 0.729 | 0.0 |
| greedy+final | 3 | 1.055 | 1.447 | 0.729 | 0.1 |
| full | 3 | 0.918 | 1.447 | 0.634 | 1.8 |



### threads — iiwa14_far

| n_threads | n_succ | avg_t_ms | speedup_vs_1t |
| --- | --- | --- | --- |
| 1 | 3 | 84.2 | 1.00 |
| 2 | 3 | 50.3 | 1.67 |
| 4 | 3 | 26.2 | 3.21 |

