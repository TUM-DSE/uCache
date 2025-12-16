#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from common import *

def load_data() -> pd.DataFrame:
    df = pd.read_csv(result_dir+'/microbench.csv')
    df = df[(df.threads == 64) & (df.system=="ucache")]
    return df


def main():
    data = load_data()
    #print(data)

    fig, ax = plt.subplots(figsize=(figwidth_half, fig_height))
    plot = sns.barplot(data=data, x = "pageSize", y = "ops",legend=False, color=palette[0]
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
    #plot.set_xticks()
    plot.set_xticklabels([f'{x/1024:.0f}' for x in sorted(data['pageSize'].unique())], size=FONTSIZE)
    plot.set_yticks([0, 500_000, 1_000_000])
    plot.set_yticklabels([0, 500, 1000], size=FONTSIZE)
    ax.set_ylabel("Throughput (K op/s)")
    ax.set_xlabel("Buffer size (KiB)")
    ax.set_title(higher_better_str, fontsize=FONTSIZE, color="navy")
    plt.tight_layout()
    plt.ylim((0,1_100_000))
    plt.savefig(os.path.join(result_dir, "big_pages.pdf"), format="pdf", pad_inches=0, bbox_inches="tight")


if __name__ == "__main__":
    main()
