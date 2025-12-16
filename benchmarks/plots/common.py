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
fig_height = 1.8
FONTSIZE=7

palette = sns.color_palette("pastel")
#sns.set(rc={"figure.figsize": (5, 5)})
sns.set_style("whitegrid")
sns.set_style("ticks", {"xtick.major.size": FONTSIZE, "ytick.major.size": FONTSIZE})
sns.set_context("paper", rc={"font.size": FONTSIZE, "axes.titlesize": FONTSIZE, "axes.labelsize": FONTSIZE})

def get_order(expe):
    match expe:
        case "micro":
            return ['uCache', 'mmap', 'TriCache']
        case "io":
            return ['uStore', 'libaio', 'SPDK']
        case "kvs":
            return ['ucache', 'mmap', 'blockcache']
        case "vmcache":
            return ['ucache', 'POSIX', 'exmap']
        case "duckdb":
            return ['DuckDB', "DuckDB\nw/ prefetch", "uCache", "uCache\nw/ prefetch"]

def darken(color):
    hue, saturation, value = rgb_to_hsv(to_rgb(color))
    return hsv_to_rgb((hue, saturation, value * 0.6))

hatch_def = [
    "",
    "//",
    "xx",
    "++",
    "--",
    "||",
    "..",
    "oo",
    "\\\\",
]

def get_hatches(data, column) -> []:
    return hatch_def[0:max(1, data[column].unique().size)]

def get_palette(data, column) -> []:
    return palette[0:max(1,data[column].unique().size)]

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
