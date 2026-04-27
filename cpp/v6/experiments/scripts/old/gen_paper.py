#!/usr/bin/env python3
"""Generate all tables + figures from experiment results, then build paper PDF.

Run from v5/:
    python _gen_paper.py
"""
import os, sys, subprocess

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, "python")
os.environ["PYTHONPATH"] = "build_x64/Release;python"

# 1) Tables (gen_paper_tables.py)
print("=" * 60)
print("STEP 1: Generate LaTeX tables")
print("=" * 60)
from scripts.gen_paper_tables import main as gen_tables
gen_tables()

# 2) Figures (gen_figures.py)
print("=" * 60)
print("STEP 2: Generate figures (PDF + PNG + HTML)")
print("=" * 60)
from scripts.gen_figures import main as gen_figures
gen_figures()

# 3) Build PDF
print("=" * 60)
print("STEP 3: Build paper PDF")
print("=" * 60)
os.chdir("paper")
for i in range(3):
    r = subprocess.run(["pdflatex", "-interaction=nonstopmode", "root.tex"],
                       capture_output=True, text=True)
    if i == 0:
        subprocess.run(["bibtex", "root"], capture_output=True, text=True)

if os.path.exists("root.pdf"):
    sz = os.path.getsize("root.pdf")
    print(f"\nSUCCESS: paper/root.pdf ({sz:,} bytes)")
else:
    print("\nFAILED: see paper/root.log")
