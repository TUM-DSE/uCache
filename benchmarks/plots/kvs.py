#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from common import *

def load_data() -> pd.DataFrame:
    df = pd.read_csv(result_dir+'/rocksdb.csv')
    return df


def main():
    data = load_data()
    #print(data)

    fig, ax = plt.subplots(figsize=(figwidth_half, fig_height))
    plot = sns.barplot(data=data, x = "memsize", y = "throughput", 
                        hue = "system", order=sorted(data['memsize'].unique(), reverse=True),
                        hue_order=get_order('kvs'), palette = get_palette(data, 'system'),
                       edgecolor="black"
        )
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

    plot.set_xticklabels(ax.get_xticks(), fontsize=FONTSIZE)
    plot.set_yticks([0, 50_000, 100_000])
    plot.set_yticklabels([0, 50, 100], fontsize=FONTSIZE)
    ax.set_ylabel("Throughput (K op/s)")
    ax.set_xlabel("Memory Quota")
    ax.legend(loc="upper left", title=None,fontsize=FONTSIZE
    #    bbox_to_anchor=(0.5, -0.15),
    #    ncol=2,
    )
    ax.set_title(higher_better_str, fontsize=FONTSIZE, color="navy")
    plt.tight_layout()
    plt.savefig(os.path.join(result_dir, "kvs.pdf"), format="pdf", pad_inches=0, bbox_inches="tight")


if __name__ == "__main__":
    main()
