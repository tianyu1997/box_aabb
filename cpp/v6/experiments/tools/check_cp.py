import json, os
os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
d = json.load(open('experiments/results/s3_e2e/_checkpoint.json'))
for x in d['trials'][-5:]:
    print(f"{x['scene']} planner={x.get('planner','?')} seed={x['seed']}")
print(f"Total: {len(d['trials'])}")
print("Keys:", list(d['trials'][0].keys()))
# Show planner field of last entry
print("Last planner repr:", repr(d['trials'][-1]['planner'])[:200])
