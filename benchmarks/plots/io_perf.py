#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from common import *

def load_data() -> pd.DataFrame:
    df = pd.read_csv(result_dir+'/io.csv')
    df = df[(df.nthreads == 64) & (df.system!="pread")]
    df = df.assign(system=df['system'].map({"ustore": "uStore", "spdk": "SPDK", "libaio": "libaio"}))
    return df


def main():
    data = load_data()
    #print(data)

    fig, ax = plt.subplots(figsize=(figwidth_half, fig_height))
    plot = sns.barplot(data=data, x = "queue_depth", y = "bandwidth", hue_order=get_order('io'), 
                        hue = "system", palette = get_palette(data, 'system'), edgecolor="black")
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
    #plot.set_xticks(sorted(data['queue_depth'].unique()))
    #plot.set_yticks([0, 200_000, 400_000, 600_000, 800_000, 1_000_000])
    #plot.set_yticklabels([0, 200, 400, 600, 800, 1_000])
    #plot.set_xticklabels(range(len(data)))
    plot.set_xticklabels(ax.get_xticks(), size=FONTSIZE)
    plot.set_yticklabels(ax.get_yticks(), size=FONTSIZE)
    ax.set_ylabel("Bandwdith (MiB/s)")
    ax.set_xlabel("Queue depth")
    ax.legend(loc="upper left", title=None, fontsize=FONTSIZE
    #    bbox_to_anchor=(0.5, -0.15),
    #    ncol=2,
    )
    ax.set_title(higher_better_str, fontsize=FONTSIZE, color="navy")
    plt.tight_layout()
    plt.savefig(os.path.join(result_dir, "io_perf.pdf"), format="pdf", pad_inches=0, bbox_inches="tight")


if __name__ == "__main__":
    main()
