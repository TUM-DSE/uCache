#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from common import *

def load_data() -> pd.DataFrame:
    #query,repetition,time,system
    df_tmp = pd.read_csv(result_dir+'/duckdb.csv')
    df_tmp = df_tmp[df_tmp.repetition != 1]
    #df_tmp = df_tmp.assign(situation=np.where(df_tmp['repetition'] == 1, "cold", "warm"))
    #df = df_tmp.groupby(['system', 'query', 'situation'])['time'].mean().reset_index()
    df = df_tmp
    df = df.assign(system=df['system'].map({'ucache': 'uCache', 'duckdb': 'DuckDB'}))
    #df = df.assign(System=np.where(df['system'] =="DuckDB", "DuckDB", df['system'] + " " + df['situation']))
    #df = df.assign(System=df['system'] + " " + df['situation']) 
    return df


def main():
    data = load_data()
    #print(data)

    fig, ax = plt.subplots(figsize=(figwidth_full, fig_height))
    sns.barplot(ax=ax, data=data, x = "query", y = "time", palette = get_palette('duckdb'), order = sorted(data['query'].unique()),
                       hue_order = get_order('duckdb'), hue="system" , edgecolor="black"
        )
    
    df_4 = data[data['query'] == 4].groupby(['system', 'query'])['time'].mean().reset_index()
    print("Q4 improvement: {:.2f}".format(df_4[df_4['system']=="DuckDB"].time.values[0] / df_4[df_4['system']=="uCache"].time.values[0]))
    df_6 = data[data['query'] == 6].groupby(['system', 'query'])['time'].mean().reset_index()
    print("Q6 improvement: {:.2f}".format(df_6[df_6['system']=="DuckDB"].time.values[0] / df_6[df_6['system']=="uCache"].time.values[0]))
    df_17 = data[data['query'] == 17].groupby(['system', 'query'])['time'].mean().reset_index()
    print("Q17 improvement: {:.2f}".format(df_17[df_17['system']=="DuckDB"].time.values[0] / df_17[df_17['system']=="uCache"].time.values[0]))
    
    df = data.groupby(['system', 'query'])['time'].mean().reset_index()
    overhead = df.pivot_table(index='query', columns='system', values='time')
    overhead = overhead.assign(ov=overhead['DuckDB']/overhead['uCache'])
    mean_overhead = overhead['ov'].mean()
    print(mean_overhead)

    size = int(len(ax.patches)/2)
    for bar in ax.patches:
        if bar.xy[0]%1 > 0.5 and bar.xy[0]%1 < 0.7:
            bar.set_hatch(hatch_def[0])
        else:
            bar.set_hatch(hatch_def[1])

    ax.annotate("OOM", xy=(5.75, 1), size=FONTSIZE-2, color='red', rotation=90, weight='bold')
    ax.annotate("OOM", xy=(7.75, 1), size=FONTSIZE-2, color='red', rotation=90, weight='bold')
    ax.annotate("OOM", xy=(15.75, 1), size=FONTSIZE-2, color='red', rotation=90, weight='bold')
    ax.annotate("OOM", xy=(18.75, 1), size=FONTSIZE-2, color='red', rotation=90, weight='bold')
    #for container in ax.containers:
    #    ax.bar_label(container, fmt = (lambda x: '{:.1f}'.format(x)), fontsize=FONTSIZE)
    ax.set_xticklabels([x+1 if x < 14 else x+2 for x in ax.get_xticks()], fontsize=FONTSIZE)
    ax.set_yticks([0, 10, 20, 30])
    ax.set_yticklabels(ax.get_yticks(), fontsize=FONTSIZE)
    ax.set_ylim(0, 35)
    handles, labels = ax.get_legend_handles_labels()
    leg = [
        mpl.patches.Patch(facecolor=get_palette("duckdb")[0], hatch=hatch_def[0], edgecolor="black"),
        #mpl.patches.Patch(facecolor=get_palette("duckdb")[0], hatch=hatch_def[3], edgecolor="black"),
        mpl.patches.Patch(facecolor=get_palette("duckdb")[1], hatch=hatch_def[1], edgecolor="black"),
        #mpl.patches.Patch(facecolor=get_palette("duckdb")[3], hatch=hatch_def[4], edgecolor="black")
    ]
    ax.set_xlabel("Queries")
    ax.set_ylabel("Time (s)")
    ax.legend(leg, labels, loc="upper left", title=None, fontsize=FONTSIZE-1,
        #bbox_to_anchor=(0.5, 0.01),
        ncol=4,
    )
    ax.set_title(lower_better_str, fontsize=FONTSIZE, color="navy")
    plt.tight_layout()
    plt.savefig(os.path.join(result_dir, "duckdb.pdf"), format="pdf", pad_inches=0, bbox_inches="tight")


if __name__ == "__main__":
    main()
