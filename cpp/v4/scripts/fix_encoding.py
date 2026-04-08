"""Fix GBK encoding corruption in gcpc.cpp.

PowerShell's Get-Content read the UTF-8 file using system GBK encoding,
then Set-Content -Encoding UTF8 wrote the garbled text back as UTF-8.
This reverses the corruption.
"""
import re

filepath = 'src/envelope/gcpc.cpp'

with open(filepath, 'rb') as f:
    raw = f.read()

# Remove BOM if present
if raw[:3] == b'\xef\xbb\xbf':
    raw = raw[3:]

# Strategy: try to reverse the double-encoding.
# Original UTF-8 bytes were read as GBK → Unicode strings → written as UTF-8.
# To reverse: read current UTF-8 → Unicode, encode as GBK → original UTF-8 bytes,
# decode as UTF-8 → correct Unicode.
#
# Problem: this fails at boundaries (odd-byte sequences).
# Solution: process line by line, and for decorative lines, just regenerate them.

text = raw.decode('utf-8')
lines = text.split('\r\n') if '\r\n' in text else text.split('\n')

# Known garbled → correct character mappings
# (built by analyzing the GBK interpretation of UTF-8 bytes)
REPLACEMENTS = {
    '鈺愨晲': '═',   # U+2550 (E2 95 90)
    '鈹€': '─',      # U+2500 (E2 94 80)
    '鈹?': '─',      # U+2500 variant
    '鈥?': '—',      # U+2014 (E2 80 94) em-dash  
    '鈥?': '–',      # U+2013 (E2 80 93) en-dash
    '鈧€': '₀',      # U+2080 (E2 82 80) subscript 0
    '鈧?': '₁',      # U+2081 (E2 82 81) subscript 1
    '蟺': 'π',       # U+03C0 (CF 80) pi
    '脳': '×',       # U+00D7 (C3 97) multiplication
}

def fix_line(line):
    """Fix garbled characters in a single line."""
    # Check if this is a pure decorative ═══ line
    stripped = line.strip()
    if stripped.startswith('//') and all(c in '/ 鈺愨晲\t?' for c in stripped):
        # Pure decorative line — regenerate
        return '// ' + '═' * 75
    
    # For other lines, do targeted replacements
    result = line
    for garbled, correct in REPLACEMENTS.items():
        result = result.replace(garbled, correct)
    return result

fixed_lines = [fix_line(line) for line in lines]

# Now fix the duplicate Accessors section.
# Find and remove the first "Accessors" header + empty n_total_points stub.
# Pattern: empty line, decorative ═══, "//  Accessors", decorative ═══, empty, 
#          "int GcpcCache::n_total_points() const {", empty, empty,
#          decorative ═══, "//  Accessors", decorative ═══
# Replace with just the second one.

new_lines = []
i = 0
skip_until_real_accessors = False
while i < len(fixed_lines):
    line = fixed_lines[i]
    
    # Detect the duplicate pattern: first Accessors followed by empty n_total_points
    if (line.strip() == '//  Accessors' and 
        i + 4 < len(fixed_lines) and
        fixed_lines[i+2].strip() == 'int GcpcCache::n_total_points() const {' and
        fixed_lines[i+3].strip() == '' and 
        fixed_lines[i+4].strip() == ''):
        # Check if there's a second Accessors section after
        for j in range(i+4, min(i+10, len(fixed_lines))):
            if fixed_lines[j].strip() == '//  Accessors':
                # Found duplicate — skip the first one (from decoration before Accessors
                # to the empty n_total_points stub)
                # Go back to find the decoration line before this Accessors
                start_skip = i
                while start_skip > 0 and '═' in fixed_lines[start_skip-1]:
                    start_skip -= 1
                # Skip to just before the second Accessors decoration
                end_skip = j
                while end_skip > 0 and '═' in fixed_lines[end_skip-1]:
                    end_skip -= 1
                # Remove lines from start_skip to end_skip-1
                new_lines = new_lines[:-(i - start_skip)] if i > start_skip else new_lines
                i = end_skip
                break
        else:
            new_lines.append(line)
            i += 1
    else:
        new_lines.append(line)
        i += 1

# Write back as UTF-8 without BOM
with open(filepath, 'w', encoding='utf-8', newline='\r\n') as f:
    f.write('\r\n'.join(new_lines))

print(f'Fixed {len(lines)} lines → {len(new_lines)} lines')
print(f'Wrote to {filepath}')

# Verify: count remaining non-ASCII
remaining = sum(1 for line in new_lines if any(ord(c) > 127 for c in line))
print(f'Lines with non-ASCII remaining: {remaining}')
