#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from common import *

def load_data() -> pd.DataFrame:
    df = pd.read_csv(result_dir+'/microbench.csv')
    df = df[df.pageSize == 4096]
    df = df.assign(system=df['system'].map({'ucache': 'uCache', 'tricache': 'TriCache'}))
    return df


def main():
    data = load_data()
    #print(data)

    fig, ax = plt.subplots(figsize=(figwidth_half, fig_height))
    plot = sns.barplot(data=data, x = "threads", y = "ops", hue = "system", palette=get_palette(data, "system"), hue_order=get_order('micro'), edgecolor="black")
    #plot.set_xticks(sorted(data['threads'].unique()))
    plot.set_xticklabels([1, 16, 32, 64], size=FONTSIZE)
    plot.set_yticks([0, 200_000, 400_000, 600_000, 800_000, 1_000_000])
    plot.set_yticklabels([0, 200, 400, 600, 800, 1000], size=FONTSIZE)
    for container in ax.containers:
        ax.bar_label(container, fmt = (lambda x: '{:g}'.format(x/1_000)), fontsize=FONTSIZE)
    # set hatch
    bars = ax.patches
    hatches = get_hatches(data, "system")
    hs = []
    for h in hatches:
        for i in range(int(len(bars) / len(hatches))):
            hs.append(h)
    for bar, hatch in zip(bars, hs):
        bar.set_hatch(hatch)
    ax.set_ylabel("Throughput (K op/s)")
    ax.set_xlabel("# of threads")
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels, loc="upper left", title=None, fontsize=FONTSIZE
    #    bbox_to_anchor=(0.5, -0.15),
    #    ncol=2,
    )
    ax.set_title(higher_better_str, fontsize=FONTSIZE, color="navy")
    plt.tight_layout()
    plt.ylim((0, 1_100_000))
    plt.savefig(os.path.join(result_dir, "microbench.pdf"), format="pdf", pad_inches=0, bbox_inches="tight")


if __name__ == "__main__":
    main()
