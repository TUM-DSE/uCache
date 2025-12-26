#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from common import *

def load_data() -> (pd.DataFrame, pd.DataFrame):
    df_tmp = pd.read_csv(result_dir+'/microbench.csv')
    df_tmp_th = df_tmp[(df_tmp.pageSize == 4096) & (df_tmp.time >= 60) & (df_tmp.vm_size == 1000)]
    df_tmp_th = df_tmp_th.assign(system=df_tmp_th['system'].map({'ucache': 'uCache', 'mmap': 'mmap'}))
    #df = df_tmp.groupby(['system', 'thread'])['throughput'].mean().reset_index()
    #df.throughput = df.throughput.round()
    df_tmp_bp = df_tmp[(df_tmp.thread == 64) & (df_tmp.system == "ucache") & (df_tmp.time >= 60) & (df_tmp.vm_size == 1000)]
    #df2 = df2_tmp.groupby(['system', 'pageSize','thread'])['throughput'].mean().reset_index()
    #df2.throughput = df2.throughput.round()
    return df_tmp_th, df_tmp_bp


def main():
    data_micro, data_bp = load_data()
    print(data_micro)
    mmap_64 = data_micro[(data_micro.system == "mmap") & (data_micro.thread == 64)].throughput.values[0]
    ucache_64 = data_micro[(data_micro.system == "uCache") & (data_micro.thread == 64)].throughput.values[0]
    ucache_32 = data_micro[(data_micro.system == "uCache") & (data_micro.thread == 32)].throughput.values[0]
    ucache_16 = data_micro[(data_micro.system == "uCache") & (data_micro.thread == 16)].throughput.values[0]
    ucache_1 = data_micro[(data_micro.system == "uCache") & (data_micro.thread == 1)].throughput.values[0]
    print("improvement between mmap and uCache at 64 threads: "+ str(ucache_64/mmap_64))
    print("Throughput per thread: " + str(ucache_1)+ " (1), " + str(ucache_16/16) + " (16), "+ str(ucache_32/32) + ' (32), ' + str(ucache_64/64) + ' (64)')
    
    data_bp['pageThroughput'] = (data_bp.throughput * data_bp.pageSize)/(10**9)
    #print(data_bp)

    fig, ax = plt.subplots(1, 2, figsize=(figwidth_half, fig_height), sharey=True)
    sns.barplot(ax=ax[0], data=data_micro, x = "thread", y = "throughput", hue = "system", palette=get_palette("micro"), hue_order=get_order('micro'), edgecolor="black")
    sns.barplot(ax=ax[1], data=data_bp, x = "pageSize", y = "throughput", color = palette[1], edgecolor="black", width=0.5)
    ax2 = ax[1].twinx()

    sns.pointplot(ax=ax2, data=data_bp, x='pageSize', y='pageThroughput', color = "grey", scale=0.4)

    size = int(len(ax[0].patches)/2)
    hatches = [hatch_def[0]] * size + [hatch_def[1]] * size
    for bar,hat in zip(ax[0].patches, hatches):
        bar.set_hatch(hat)

    ax[0].set_yscale("log")
    ax[0].set_yticks([100_000, 1_000_000])
    ax[0].get_yaxis().set_major_formatter(mpl.ticker.LogFormatterMathtext())
    ax[0].tick_params(axis='y', which='both', labelsize=FONTSIZE)
    ax[0].set_ylabel("Insertions/s", fontsize=FONTSIZE)
    ax[0].set_xlabel("# of threads", fontsize=FONTSIZE, labelpad=2)
    box = ax[0].get_position()
    ax[0].set_xticklabels([1, 16, 32, 64], size=FONTSIZE)
    ax[0].set_position([box.x0, box.y0+box.height*0.1, box.width, box.height*0.9])
    ax[0].set_title("4KiB pages", fontsize=FONTSIZE, color="black")
    
    ax[1].set_yscale("log")
    ax[1].set_yticks([100_000, 1_000_000])
    ax[1].get_yaxis().set_major_formatter(mpl.ticker.LogFormatterMathtext())
    ax[1].tick_params(axis='y', which='both', labelsize=FONTSIZE)
    ax[1].set_xticklabels([f'{x/1024:.0f}' for x in sorted(data_bp['pageSize'].unique())], size=FONTSIZE)
    #ax[1].set_yticks([0, 200_000, 400_000, 600_000, 800_000, 1_000_000])
    #ax[1].set_yticklabels([0, 0.2, 0.4, 0.6, 0.8, 1], size=FONTSIZE)
    #ax[1].set_ylabel("test", fontsize=FONTSIZE)
    ax[1].set_xlabel("Buffer size (KiB)", fontsize=FONTSIZE, labelpad=2)
    ax[1].set_title("64 threads", fontsize=FONTSIZE, color="black")
    
    ax2.set_ylabel("Throughput (GiB/s)", fontsize=FONTSIZE)
    #ax2.set_yscale("log")
    #ax2.get_yaxis().set_major_formatter(mpl.ticker.LogFormatterMathtext())
    ax2.tick_params(axis='y', which='both', labelsize=FONTSIZE)
    ax2.set_ylim(0, 13)

    handles, labels = ax[0].get_legend_handles_labels()
    leg = [
        mpl.patches.Patch(facecolor=get_palette("micro")[0], hatch=hatch_def[0], edgecolor="black"),
        mpl.patches.Patch(facecolor=get_palette("micro")[1], hatch=hatch_def[1], edgecolor="black")
    ]
    fig.legend(leg, labels, loc="upper center", title=None, fontsize=FONTSIZE-1,
        bbox_to_anchor=(0.5, 0.07),
        ncol=4,
    )
    ax[0].get_legend().remove()
    plt.suptitle(higher_better_str, fontsize=FONTSIZE, color="navy", y=0.91, x=0.55)
    plt.tight_layout()
    #plt.ylim((0, 1_300_000))
    plt.subplots_adjust(wspace=0.4)
    plt.savefig(os.path.join(result_dir, "microbench.pdf"), format="pdf", pad_inches=0, bbox_inches="tight")


if __name__ == "__main__":
    main()
