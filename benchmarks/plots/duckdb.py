#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from common import *

def load_data() -> pd.DataFrame:
    df_tmp = pd.read_csv(result_dir+'/duckdb.csv')
    df = df_tmp.groupby(['system', 'prefetching'])['time'].mean().reset_index()
    df = df.assign(Prefetch=df['prefetching'].map({1:"\nw/ prefetch", 0:""}))
    df = df.assign(System=df['system'].map({'ucache': 'uCache', "duckdb": "DuckDB"})+df['Prefetch'])
    return df


def main():
    data = load_data()
    #print(data)

    fig, ax = plt.subplots(figsize=(figwidth_half, fig_height))
    plot = sns.barplot(data=data, x = "System", y = "time",legend=False, palette = [palette[2], darken(palette[2]), palette[0], darken(palette[0])],
                       order = get_order('duckdb'), hue="System",edgecolor="black"
        )
    for container in ax.containers:
        ax.bar_label(container, fontsize=FONTSIZE)
    # set hatch
    bars = ax.patches
    hatches = [hatch_def[2], hatch_def[3], hatch_def[0], hatch_def[4]]
    hs = []
    for bar, hatch in zip(bars, hatches):
        bar.set_hatch(hatch)
    plot.set_xticklabels(["DuckDB", "DuckDB\nw/ prefetch", "uCache", "uCache\nw/ prefetch"], fontsize=FONTSIZE)
    plot.set_yticklabels([0, 50, 100], fontsize=FONTSIZE)
    ax.set_ylabel("Time (s)")
    ax.set_xlabel("")
    ax.set_title(lower_better_str, fontsize=FONTSIZE, color="navy")
    plt.tight_layout()
    plt.savefig(os.path.join(result_dir, "duckdb.pdf"), format="pdf", pad_inches=0, bbox_inches="tight")


if __name__ == "__main__":
    main()
