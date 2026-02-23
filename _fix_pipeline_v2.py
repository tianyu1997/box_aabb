"""Comprehensive removal of all goal_bias from pipeline.py"""

filepath = r'c:\Users\TIAN\Documents\box_aabb\v3\src\planner\pipeline.py'
with open(filepath, 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Find and fix each problematic line
new_lines = []
skip_until = -1
i = 0
while i < len(lines):
    line = lines[i]
    
    # Skip lines we've already handled
    if i < skip_until:
        i += 1
        continue
        
    stripped = line.strip()
    
    # 1. Remove "goal_bias: float = 0.10" from PandaGCSConfig
    if stripped == 'goal_bias: float = 0.10':
        # Skip this line entirely
        i += 1
        continue
    
    # 2. Remove "goal_bias=cfg.goal_bias," from make_planner_config
    if stripped == 'goal_bias=cfg.goal_bias,':
        i += 1
        continue
    
    # 3. Remove "goal_bias = float(payload.get(..." from _grow_partition_worker
    if stripped.startswith('goal_bias = float(payload.get('):
        i += 1
        continue
    
    # 4. Remove '"goal_bias": planner.config.goal_bias,' from parallel payload
    if stripped == '"goal_bias": planner.config.goal_bias,':
        i += 1
        continue
    
    # 5. Remove 'goal_bias = planner.config.goal_bias' or getattr variant
    if stripped.startswith('goal_bias = ') and ('planner.config' in stripped or 'getattr' in stripped):
        i += 1
        continue
    
    # 6. Replace _sample_batch_local (parallel worker) - detect start of old function
    if stripped == 'def _sample_batch_local():' and i + 1 < len(lines):
        # Check if next lines contain the old goal_mask pattern
        next_lines_text = ''.join(lines[i:i+15])
        if 'goal_mask' in next_lines_text or 'goal_bias' in next_lines_text:
            indent = line[:len(line) - len(line.lstrip())]
            inner = indent + '    '
            new_lines.append(f'{indent}def _sample_batch_local():\n')
            new_lines.append(f'{inner}configs = rng.uniform(lows, highs, size=(batch_size, ndim))\n')
            new_lines.append(f'{inner}collisions = local_checker.check_config_collision_batch(configs)\n')
            new_lines.append(f'{inner}return [configs[i] for i in range(batch_size) if not collisions[i]]\n')
            # Skip old function body
            j = i + 1
            while j < len(lines):
                sj = lines[j].strip()
                if sj == '' or (not lines[j][0].isspace() and sj):
                    break
                # Check if we've reached a line at the same indent level or less
                if lines[j].strip() and len(lines[j]) - len(lines[j].lstrip()) <= len(indent) and j > i + 1:
                    break
                # Check for return statement as end
                if 'return [configs[i]' in lines[j]:
                    j += 1
                    break
                j += 1
            i = j
            continue
        else:
            new_lines.append(line)
            i += 1
            continue
    
    # 7. Replace _sample_batch (serial grow) - detect start
    if stripped == 'def _sample_batch():' and i + 1 < len(lines):
        next_lines_text = ''.join(lines[i:i+30])
        if 'goal_mask' in next_lines_text or 'goal_bias' in next_lines_text:
            indent = line[:len(line) - len(line.lstrip())]
            inner = indent + '    '
            new_lines.append(f'{indent}def _sample_batch():\n')
            new_lines.append(f'{inner}rolls = rng.uniform(size=batch_size)\n')
            new_lines.append(f'{inner}configs = np.empty((batch_size, ndim), dtype=np.float64)\n')
            new_lines.append(f'{inner}if has_hier_tree:\n')
            new_lines.append(f'{inner}    guided_mask = rolls < guided_ratio\n')
            new_lines.append(f'{inner}else:\n')
            new_lines.append(f'{inner}    guided_mask = np.zeros(batch_size, dtype=bool)\n')
            new_lines.append(f'{inner}uniform_mask = ~guided_mask\n')
            new_lines.append(f'\n')
            new_lines.append(f'{inner}n_uni = int(uniform_mask.sum())\n')
            new_lines.append(f'{inner}if n_uni > 0:\n')
            new_lines.append(f'{inner}    configs[uniform_mask] = rng.uniform(lows, highs, size=(n_uni, ndim))\n')
            new_lines.append(f'\n')
            new_lines.append(f'{inner}guided_idxs = np.flatnonzero(guided_mask)\n')
            new_lines.append(f'{inner}for i in guided_idxs:\n')
            new_lines.append(f'{inner}    try:\n')
            new_lines.append(f'{inner}        q = planner.hier_tree.sample_unoccupied_seed(rng)\n')
            new_lines.append(f'{inner}    except ValueError:\n')
            new_lines.append(f'{inner}        q = None\n')
            new_lines.append(f'{inner}    if q is None:\n')
            new_lines.append(f'{inner}        q = rng.uniform(lows, highs)\n')
            new_lines.append(f'{inner}    configs[i] = q\n')
            new_lines.append(f'\n')
            new_lines.append(f'{inner}collisions = planner.collision_checker.check_config_collision_batch(\n')
            new_lines.append(f'{inner}    configs)\n')
            new_lines.append(f'{inner}return [configs[i] for i in range(batch_size) if not collisions[i]]\n')
            # Skip old function body
            j = i + 1
            while j < len(lines):
                if 'return [configs[i]' in lines[j]:
                    j += 1
                    break
                j += 1
            i = j
            continue
        else:
            new_lines.append(line)
            i += 1
            continue
    
    new_lines.append(line)
    i += 1

with open(filepath, 'w', encoding='utf-8') as f:
    f.writelines(new_lines)

# Verify
with open(filepath, 'r', encoding='utf-8') as f:
    content = f.read()

count = 0
for i, line in enumerate(content.split('\n'), 1):
    if 'goal_bias' in line or 'goal_mask' in line:
        count += 1
        print(f'{i}: {line.rstrip()}')

if count == 0:
    print('SUCCESS: All goal_bias/goal_mask references removed from pipeline.py')
else:
    print(f'WARNING: {count} references remaining')
