#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from common import *

def load_data() -> pd.DataFrame:
    df_tmp = pd.read_csv(result_dir+'/vmcache.csv')
    df = df_tmp.groupby(['ts', 'system', 'threads'])['tx'].mean().reset_index()
    df = df.assign(system=df['system'].map({'ucache': 'uCache', 'vmcache': 'POSIX', 'exmap': 'exmap'}))
    return df


def main():
    data = load_data()
    #print(data)

    fig, ax = plt.subplots(figsize=(figwidth_half, fig_height))
    plot = sns.lineplot(ax=ax, data=data, x = "ts", y = "tx", 
                        hue = "system", style = "system", palette = get_palette("vmcache"),
                        hue_order = get_order('vmcache')
                        #marker="H"
        )
    plot.set_xticks([0, 100, 200, 300])
    plot.set_xticklabels([0, 100, 200, 300], fontsize=FONTSIZE)
    plot.set_yticks([0, 50_000, 100_000])
    plot.set_yticklabels([0, 50, 100], fontsize=FONTSIZE)
    ax.set_ylabel("Throughput (K tx/s)")
    ax.set_xlabel("Time (s)")
    ax.legend(loc="lower right", title=None,fontsize=FONTSIZE
    #    bbox_to_anchor=(0.5, -0.15),
    #    ncol=2,
    )
    ax.set_title(higher_better_str, fontsize=FONTSIZE, color="navy")
    plt.tight_layout()
    plt.grid()
    plt.savefig(os.path.join(result_dir, "vmcache.pdf"), format="pdf", pad_inches=0, bbox_inches="tight")


if __name__ == "__main__":
    main()
