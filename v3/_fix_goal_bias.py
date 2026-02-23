f = 'src/planner/pipeline.py'
t = open(f, encoding='utf-8').read()
old = '    goal_bias = planner.config.goal_bias\n'
new = '    goal_bias = getattr(planner.config, "goal_bias", 0.1)\n'
assert old in t, 'not found'
t = t.replace(old, new, 1)
open(f, 'w', encoding='utf-8').write(t)
print('OK')
