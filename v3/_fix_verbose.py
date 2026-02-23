"""One-shot script to add _VERBOSE/_log to pipeline.py and replace print→_log."""
import re

path = r"src\planner\pipeline.py"
with open(path, "r", encoding="utf-8") as f:
    content = f.read()

# 1. Check if already patched
if "_VERBOSE" in content:
    print("Already patched, skipping.")
    exit(0)

# 2. Add _VERBOSE flag and _log function after imports
marker = "cp = None  # GCS solver \u4e0d\u53ef\u7528"
idx = content.index(marker)
nl = content.index("\n", idx)

insert_code = """


# ═══════════════════════════════════════════════════════════════════════════
# Module-level verbose control
# ═══════════════════════════════════════════════════════════════════════════

_VERBOSE = True


def set_verbose(v: bool):
    \"\"\"开启/关闭 pipeline 模块的控制台输出.\"\"\"
    global _VERBOSE
    _VERBOSE = v


def _log(*args, **kwargs):
    if _VERBOSE:
        print(*args, **kwargs)"""

content = content[:nl] + insert_code + content[nl:]

# 3. Replace all pipeline print( calls with _log(
#    Match lines where stripped starts with print(f"    [ or print(f"\n or
#    print(f"      (the detail block)
lines = content.split("\n")
new_lines = []
replaced = 0
for line in lines:
    stripped = line.lstrip()
    if stripped.startswith('print(f"    [') or stripped.startswith('print(f"\\n'):
        indent = len(line) - len(stripped)
        new_lines.append(" " * indent + "_log(" + stripped[6:])
        replaced += 1
    elif stripped.startswith('print(f"      '):
        indent = len(line) - len(stripped)
        new_lines.append(" " * indent + "_log(" + stripped[6:])
        replaced += 1
    else:
        new_lines.append(line)

content = "\n".join(new_lines)
print(f"Replaced {replaced} print statements with _log")

# 4. Verify remaining
remaining = []
for i, line in enumerate(content.split("\n"), 1):
    s = line.lstrip()
    if s.startswith("print(") and "verbose" not in s and "..." not in s:
        remaining.append(f"  Line {i}: {line.rstrip()}")

print(f"Remaining print() calls: {len(remaining)}")
for r in remaining:
    print(r)

with open(path, "w", encoding="utf-8") as f:
    f.write(content)
print("File saved successfully.")
