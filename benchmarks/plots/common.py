# workaround to select Agg as backend consistenly
import matplotlib as mpl  # type: ignore
import matplotlib.pyplot as plt  # type: ignore
import matplotlib.ticker as ticker
from matplotlib.colors import rgb_to_hsv, hsv_to_rgb, to_rgb
import seaborn as sns  # type: ignore
from typing import Any, Dict, List, Union
import pandas as pd
import os
import numpy as np

dir_path = os.path.dirname(os.path.realpath(__file__))
result_dir = os.path.join(dir_path, "../results")
mpl.use("Agg")
mpl.rcParams["text.latex.preamble"] = r"\usepackage{amsmath}"
mpl.rcParams["pdf.fonttype"] = 42
mpl.rcParams["ps.fonttype"] = 42
mpl.rcParams["font.family"] = "libertine"

# 3.3 inch for single column, 7 inch for double column
figwidth_third = 2
figwidth_half = 3.3
figwidth_full = 7
fig_height = 1.6
FONTSIZE=7

palette = sns.color_palette("pastel")
#sns.set(rc={"figure.figsize": (5, 5)})
sns.set_style("whitegrid")
sns.set_style("ticks", {"xtick.major.size": FONTSIZE, "ytick.major.size": FONTSIZE})
sns.set_context("paper", rc={"font.size": FONTSIZE, "axes.titlesize": FONTSIZE, "axes.labelsize": FONTSIZE})

def get_order(expe):
    match expe:
        case "micro":
            return ["mmap", "uCache"]#, 'TriCache']
        case "big_pages":
            return ["uCache"]
        case "io":
            return ['libaio', 'uStore', 'SPDK']
        case "kvs":
            return ['mmap', 'uCache']#, 'block']
        case "vmcache":
            return ['POSIX', 'uCache', 'exmap']
        case "duckdb":
            return ['DuckDB', 'uCache']#['DuckDB cold', 'DuckDB warm', 'uCache cold', 'uCache warm']

def darken(color):
    hue, saturation, value = rgb_to_hsv(to_rgb(color))
    return hsv_to_rgb((hue, saturation, value * 0.9))

hatch_def = [
    "//",
    '',
    'xx',
    '*',
    "--",
    "++",
    "||",
    "..",
    "oo",
    "\\\\",
]

def get_hatch_map(expe):
    match expe:
        case "micro":
            return [hatch_def[0], hatch_def[1]]
        case "big_pages":
            return {"uCache": hatch_def[1]}
        case "kvs":
            return {"mmap": hatch_def[0], 'uCache': hatch_def[1]}
        case "duckdb":
            return {'uCache': hatch_def[1], "DuckDB": hatch_def[2]}
        case "io":
            return {"libaio": hatch_def[0], 'uStore': hatch_def[1], "SPDK": hatch_def[2]}

def get_palette(expe):
    match expe:
        case "micro":
            return [palette[0], palette[1]]
        case "big_pages":
            return [palette[1]]
        case "vmcache":
            return [palette[0], palette[1], palette[2]]
        case "io":
            return [palette[0], palette[1], palette[2]]
        case "kvs":
            return [palette[0], palette[1], palette[2]]
        case "duckdb":
            return [palette[0], palette[1]]#{darken(palette[0]), palette[1], darken(palette[1])]

marker_def = [
    "o",
    "x",
    "D",
    "*",
    "+",
]

lower_better_str = "Lower is better ↓"
higher_better_str = "Higher is better ↑"
left_better_str = "Lower is better ←"
right_better_str = "Higher is better →"

marker_def = [
    "o",
    "x",
    "D",
    "*",
    "+",
]

lower_better_str = "Lower is better ↓"
higher_better_str = "Higher is better ↑"
left_better_str = "Lower is better ←"
right_better_str = "Higher is better →"
