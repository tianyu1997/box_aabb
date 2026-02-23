"""Remove all goal_bias references from pipeline.py"""

filepath = r'c:\Users\TIAN\Documents\box_aabb\v3\src\planner\pipeline.py'
with open(filepath, 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Remove goal_bias from PandaGCSConfig
content = content.replace(
    '    max_boxes: int = 500\n    goal_bias: float = 0.10\n    guided_sample_ratio: float = 0.6',
    '    max_boxes: int = 500\n    guided_sample_ratio: float = 0.6'
)

# 2. Remove goal_bias from make_planner_config
content = content.replace(
    '        min_box_size=cfg.min_box_size,\n        goal_bias=cfg.goal_bias,\n        guided_sample_ratio=cfg.guided_sample_ratio,',
    '        min_box_size=cfg.min_box_size,\n        guided_sample_ratio=cfg.guided_sample_ratio,'
)

# 3. Remove goal_bias from _grow_partition_worker
content = content.replace(
    '    min_box_size = float(payload["min_box_size"])\n    goal_bias = float(payload.get("goal_bias", 0.1))\n    seed_val = payload.get("seed")',
    '    min_box_size = float(payload["min_box_size"])\n    seed_val = payload.get("seed")'
)

# 4. Rewrite _sample_batch_local to remove goal_bias
old_batch_local = '    def _sample_batch_local():\n        rolls = rng.uniform(size=batch_size)\n        configs = np.empty((batch_size, ndim), dtype=np.float64)\n        goal_mask = rolls < goal_bias\n        n_goal = int(goal_mask.sum())\n        if n_goal > 0:\n            noise = rng.normal(0.0, 0.3, size=(n_goal, ndim))\n            configs[goal_mask] = np.clip(q_goal + noise, lows, highs)\n        uni_mask = ~goal_mask\n        n_uni = int(uni_mask.sum())\n        if n_uni > 0:\n            configs[uni_mask] = rng.uniform(lows, highs, size=(n_uni, ndim))\n        collisions = local_checker.check_config_collision_batch(configs)\n        return [configs[i] for i in range(batch_size) if not collisions[i]]'
new_batch_local = '    def _sample_batch_local():\n        configs = rng.uniform(lows, highs, size=(batch_size, ndim))\n        collisions = local_checker.check_config_collision_batch(configs)\n        return [configs[i] for i in range(batch_size) if not collisions[i]]'
content = content.replace(old_batch_local, new_batch_local)

# 5. Remove goal_bias from parallel grow payload
content = content.replace(
    '                "min_box_size": planner.config.min_box_size,\n                "goal_bias": planner.config.goal_bias,\n                "seed": int(rng.integers(0, 2**31 - 1)),',
    '                "min_box_size": planner.config.min_box_size,\n                "seed": int(rng.integers(0, 2**31 - 1)),'
)

with open(filepath, 'w', encoding='utf-8') as f:
    f.write(content)

# Verify
count = 0
for i, line in enumerate(content.split('\n'), 1):
    if 'goal_bias' in line or 'goal_mask' in line:
        count += 1
        print(f'{i}: {line.rstrip()}')
print(f'\nRemaining goal_bias/goal_mask references: {count}')
