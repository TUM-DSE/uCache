#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from common import *

def load_data() -> pd.DataFrame:
    df_tmp = pd.read_csv(result_dir+'/io.csv')
    df_tmp['bw'] = [float(s.replace('bw=', '').replace('MiB/s', '').replace('GiB/s', '')) * (1024 if 'GiB/s' in s else 1) for s in df_tmp['bandwidth']]
    df_tmp = df_tmp[round(df_tmp.time) != 10]
    df = df_tmp.groupby(['system', 'batchSize', 'threads'])['bw'].mean().reset_index()
    df = df.assign(system=df['system'].map({"ufs": "uStore", "spdk": "SPDK", "libaio": "libaio"}))
    return df


def main():
    data = load_data()
    #print(data)
    
    fig, ax  = plt.subplots(1, 3, figsize=(figwidth_half, fig_height), sharey=True)
    data1 = data[data.threads == 4]
    data2 = data[data.threads == 32]
    data3 = data[data.threads == 64]
    sns.barplot(ax=ax[0], data=data1, x = "batchSize", y = "bw", hue_order=get_order('io'), 
                        hue = "system", palette = get_palette("io"), edgecolor="black", width=0.8)
    sns.barplot(ax=ax[1], data=data2, x = "batchSize", y = "bw", hue_order=get_order('io'), 
                        hue = "system", palette = get_palette("io"), edgecolor="black", width=0.8)
    sns.barplot(ax=ax[2], data=data3, x = "batchSize", y = "bw", hue_order=get_order('io'), 
                        hue = "system", palette = get_palette("io"), edgecolor="black", width=0.8)
   
    for i in range(3):
        bars = ax[i].patches
        for bar in bars:
            sys = data[data.bw == bar.get_height()].system
            if not sys.empty:
                bar.set_hatch(get_hatch_map("io")[sys.iloc[0]])
    
    ax[0].set_xticklabels(ax[0].get_xticklabels(), size=FONTSIZE)
    ax[0].set_yticks([0, 2500, 5000, 7500, 10000, 12500])
    ax[0].set_yticklabels([0, 2.5, 5, 7.5, 10, 12.5], size=FONTSIZE)
    ax[1].set_xticklabels(ax[1].get_xticklabels(), size=FONTSIZE)
    ax[2].set_xticklabels(ax[2].get_xticklabels(), size=FONTSIZE)
    
    ax[0].set_title("4 threads", fontsize=FONTSIZE, color="black")
    ax[1].set_title("32 threads", fontsize=FONTSIZE, color="black")
    ax[2].set_title("64 threads", fontsize=FONTSIZE, color="black")
    
    ax[0].set_ylabel("Bandwdith (GiB/s)")
    ax[0].set_xlabel("")
    ax[1].set_xlabel("Batch size")
    ax[1].set_ylabel("")
    ax[2].set_xlabel("")
    ax[2].set_ylabel("")
    
    ax[2].get_legend().remove()
    ax[1].get_legend().remove()
    ax[0].get_legend().remove()

    handles, labels = ax[0].get_legend_handles_labels()
    leg = [
        mpl.patches.Patch(facecolor=get_palette("io")[0], hatch=hatch_def[0], edgecolor="black"),
        mpl.patches.Patch(facecolor=get_palette("io")[1], hatch=hatch_def[1], edgecolor="black"),
        mpl.patches.Patch(facecolor=get_palette("io")[2], hatch=hatch_def[2], edgecolor="black")
    ]
    fig.legend(leg, labels, loc="upper center", title=None, fontsize=FONTSIZE-1,
        bbox_to_anchor=(0.55, 0.07),
        ncol=4,
    )
    plt.suptitle(higher_better_str, fontsize=FONTSIZE, color="navy", y=0.91, x=0.53)
    plt.tight_layout()
    plt.subplots_adjust(wspace=0.3)
    plt.savefig(os.path.join(result_dir, "io_perf.pdf"), format="pdf", pad_inches=0, bbox_inches="tight")

    df = data
    df['batchSize'] = [str(s) for s in df['batchSize']]
    df['threads'] = [str(s) for s in df['threads']]
    df = df.assign(ind=data['batchSize'] +'-'+ data['threads'])
    overhead = df.pivot_table(index='ind', columns='system', values='bw')
    overhead = overhead.assign(ov=overhead['SPDK']/overhead['uStore'])
    overhead = overhead.assign(impr=overhead['uStore']/overhead['libaio'])
    print(overhead)
    mean_overhead = overhead['ov'].mean()
    mean_impr = overhead['impr'].mean()
    print(mean_overhead)
    print(mean_impr)

if __name__ == "__main__":
    main()
