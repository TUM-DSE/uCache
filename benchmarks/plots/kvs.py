#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from common import *

def load_data() -> pd.DataFrame:
    df = pd.read_csv(result_dir+'/microbench.csv')
    df = df[(df.pageSize == 4096) & (df.time >= 60) & (df.vm_size != 1000)]
    df = df.assign(system=df['system'].map({'ucache': 'uCache', 'mmap': 'mmap'}))
    return df


def main():
    data = load_data()
    #print(data)
    
    df = data.groupby(['system', 'pm_size'])['throughput'].mean().reset_index()
    overhead = df.pivot_table(index='pm_size', columns='system', values='throughput')
    overhead = overhead.assign(ov=overhead['uCache']/overhead['mmap'])
    ucache = overhead['uCache']
    mmap = overhead['mmap']
    print("{:2f}, {:2f}, {:2f}".format(ucache[64]/ucache[128], ucache[32]/ucache[64], ucache[16]/ucache[32]))
    print("{:2f}, {:2f}, {:2f}".format(mmap[64]/mmap[128], mmap[32]/mmap[64], mmap[16]/mmap[32]))
    print(ucache[16]/ucache[128])
    print(mmap[16]/mmap[128])

    fig, ax = plt.subplots(figsize=(figwidth_half, fig_height))
    plot = sns.barplot(ax=ax, data=data, x = "pm_size", y = "throughput", 
                        hue = "system", order=sorted(data['pm_size'].unique(), reverse=True),
                        hue_order=get_order('kvs'), palette = get_palette('kvs'),
                       edgecolor="black"
        )
    #for container in ax.containers:
    #    ax.bar_label(container, fmt = (lambda x: '{:g}'.format(x/1_000)), fontsize=FONTSIZE)
    # set hatch
    size = 4
    hatches = [hatch_def[0]] * size + [hatch_def[1]] * size
    for bar, hat in zip(ax.patches, hatches):
        bar.set_hatch(hat)
        if hat == hatch_def[1]:
            ax.text(bar.xy[0]+bar.get_width()/4, bar.get_height()*1.15, 'x'+str(round(overhead[overhead['uCache'] == bar.get_height()].ov.values[0])), color = darken(palette[3]), fontsize=FONTSIZE-1)


    ax.set_yscale("log")
    ax.set_xticklabels(sorted(data['pm_size'].unique(), reverse=True), fontsize=FONTSIZE)
    ax.set_yticks([10_000, 100_000, 1_000_000, 10_000_000])
    ax.set_yticklabels(ax.get_yticklabels(), fontsize=FONTSIZE)
    ax.get_yaxis().set_major_formatter(mpl.ticker.LogFormatterMathtext())
    ax.tick_params(axis='y', which='both', labelsize=FONTSIZE)
    ax.set_ylim(10_000, 10_000_000)
    ax.set_ylabel("Lookups (op/s)")
    ax.set_xlabel("Memory Quota (GiB)")
    handles, labels = ax.get_legend_handles_labels()
    leg = [
        mpl.patches.Patch(facecolor=get_palette("kvs")[0], hatch=hatch_def[0], edgecolor="black"),
        mpl.patches.Patch(facecolor=get_palette("kvs")[1], hatch=hatch_def[1], edgecolor="black"),
    ]
    ax.legend(leg, labels, loc="upper right", title=None,fontsize=FONTSIZE-1,
        #bbox_to_anchor=(0.55, 0.07),
        ncol=2,
    )
    ax.set_title(higher_better_str, fontsize=FONTSIZE, color="navy")
    plt.tight_layout()
    plt.savefig(os.path.join(result_dir, "kvs.pdf"), format="pdf", pad_inches=0, bbox_inches="tight")


if __name__ == "__main__":
    main()
