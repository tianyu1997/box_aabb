"""Common plotting utilities for SBF v6 paper figures.

IEEE T-RO style: 8pt fonts, single-column ~3.5in, double-column ~7in.
Grayscale-friendly color palette with distinct patterns.
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# ── IEEE T-RO figure sizes (inches) ──
SINGLE_COL = 3.5
DOUBLE_COL = 7.16
FIG_DPI = 300

# ── Color palette: distinguishable in color AND grayscale ──
COLORS = {
    'blue':    '#1f77b4',
    'orange':  '#ff7f0e',
    'green':   '#2ca02c',
    'red':     '#d62728',
    'purple':  '#9467bd',
    'brown':   '#8c564b',
    'pink':    '#e377c2',
    'gray':    '#7f7f7f',
    'olive':   '#bcbd22',
    'cyan':    '#17becf',
}

# Ordered palette for bar/line charts
PAL = list(COLORS.values())

# Hatching patterns for grayscale distinction
HATCHES = ['', '//', '\\\\', 'xx', '..', 'oo', '++']

# ── Global RC params (IEEE style) ──
def setup_ieee_style():
    plt.rcParams.update({
        'font.size': 8,
        'font.family': 'serif',
        'font.serif': ['Times New Roman', 'DejaVu Serif'],
        'axes.labelsize': 8,
        'axes.titlesize': 9,
        'legend.fontsize': 7,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'lines.linewidth': 1.0,
        'lines.markersize': 4,
        'figure.dpi': FIG_DPI,
        'savefig.dpi': FIG_DPI,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.02,
        'axes.grid': True,
        'grid.alpha': 0.3,
        'grid.linewidth': 0.5,
    })

# ── Output directory ──
FIG_DIR = Path(__file__).resolve().parent.parent / 'doc' / 'figures'

def savefig(fig, name, tight=True):
    """Save figure as PDF (for LaTeX) and PNG (for preview)."""
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    pdf_path = FIG_DIR / f'{name}.pdf'
    png_path = FIG_DIR / f'{name}.png'
    if tight:
        fig.savefig(pdf_path, bbox_inches='tight', pad_inches=0.02)
        fig.savefig(png_path, bbox_inches='tight', pad_inches=0.02)
    else:
        fig.savefig(pdf_path)
        fig.savefig(png_path)
    plt.close(fig)
    print(f'  → {pdf_path}')
    print(f'  → {png_path}')
    return pdf_path

# ── Data directory ──
DATA_DIR = Path(__file__).resolve().parent.parent / 'experiments' / 'results_backup_pre_accel'

def load_json(relpath):
    """Load a JSON file relative to DATA_DIR."""
    import json
    p = DATA_DIR / relpath
    with open(p) as f:
        return json.load(f)
