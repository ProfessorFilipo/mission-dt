import json, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({"font.size":8,"font.family":"serif","figure.dpi":300,
                     "axes.grid":True,"grid.alpha":0.3})
import pathlib; R=str(pathlib.Path(__file__).resolve().parent.parent/"results")
e1=json.load(open(f"{R}/e1_scalability.json"))
N=[r["n_agents"] for r in e1]

# Fig A: frame compute time vs fleet size
fig,ax=plt.subplots(figsize=(3.45,2.1))
ax.plot(N,[r["frame_ms"]["mean"] for r in e1],"o-",ms=3.5,label="mean",color="#16243F")
ax.plot(N,[r["frame_ms"]["p99"] for r in e1],"s--",ms=3.5,label="p99",color="#C9A24A")
ax.plot(N,[r["frame_ms"]["max"] for r in e1],"^:",ms=3.5,label="max",color="#8a1f1f")
ax.axhline(125,color="k",lw=0.8,ls="-.")
ax.text(2,95,"frame deadline $T_f$ = 125 ms",fontsize=7)
ax.set_yscale("log");ax.set_xlabel("Fleet size (number of hybrid agents)")
ax.set_ylabel("$\\Delta^e$ frame compute time (ms)")
ax.legend(frameon=False,fontsize=7,loc="center right")
fig.tight_layout();fig.savefig(f"{R}/fig_scalability.pdf");fig.savefig(f"{R}/fig_scalability.png")

# Fig B: telemetry latency and stale updates vs fleet size
fig,ax=plt.subplots(figsize=(3.45,2.1))
ax.plot(N,[r["telemetry_lat_ms"]["p50"] for r in e1],"o-",ms=3.5,label="telemetry p50",color="#16243F")
ax.plot(N,[r["telemetry_lat_ms"]["p99"] for r in e1],"s--",ms=3.5,label="telemetry p99",color="#C9A24A")
ax.set_xlabel("Fleet size (number of hybrid agents)")
ax.set_ylabel("MQTT latency (ms)")
ax2=ax.twinx();ax2.grid(False)
stale=[100.0*r["stale_updates"]/(r["frames"]*r["n_agents"]) for r in e1]
ax2.plot(N,stale,"^:",ms=3.5,color="#8a1f1f",label="stale updates")
ax2.set_ylabel("Stale state updates (\\%)",color="#8a1f1f")
ax2.tick_params(axis="y",labelcolor="#8a1f1f")
h1,l1=ax.get_legend_handles_labels();h2,l2=ax2.get_legend_handles_labels()
ax.legend(h1+h2,l1+l2,frameon=False,fontsize=7,loc="upper left")
fig.tight_layout();fig.savefig(f"{R}/fig_latency.pdf");fig.savefig(f"{R}/fig_latency.png")

# Fig C: regulator effect
e2=json.load(open(f"{R}/e2_regulator.json"))
on,off=e2[0],e2[1]
fig,axs=plt.subplots(1,2,figsize=(3.45,1.9))
axs[0].bar(["ON\n(8 Hz)","OFF\n(50 Hz)"],[on["uplink_Bps"]/1024,off["uplink_Bps"]/1024],
           color=["#16243F","#C9A24A"],width=0.55)
axs[0].set_ylabel("Uplink usage (KiB/s)");axs[0].set_title("Bandwidth",fontsize=8)
axs[1].bar(["ON\n(8 Hz)","OFF\n(50 Hz)"],[on["dup_updates"],off["dup_updates"]],
           color=["#16243F","#C9A24A"],width=0.55)
axs[1].set_ylabel("Redundant updates");axs[1].set_title("Wasted samples",fontsize=8)
for a in axs:
    for c in a.containers: a.bar_label(c,fontsize=7,fmt="%.0f")
    a.margins(y=0.18)
fig.tight_layout();fig.savefig(f"{R}/fig_regulator.pdf");fig.savefig(f"{R}/fig_regulator.png")
print("figures ok")

# Fig D: E3 swarm propagation latency CDF
import os, pathlib
_R = str(pathlib.Path(__file__).resolve().parent.parent/"results")
if os.path.exists(f"{_R}/e3_swarm.json"):
    e3=json.load(open(f"{_R}/e3_swarm.json"))
    fig,ax=plt.subplots(figsize=(3.45,2.1))
    styles=[("-","#16243F"),("--","#C9A24A"),(":","#8a1f1f")]
    import numpy as np
    for r,(ls,c) in zip(e3,styles):
        v=np.sort(np.array(r["raw_swarm_lat_ms"]))
        y=np.arange(1,len(v)+1)/len(v)
        ax.plot(v,y,ls,color=c,lw=1.3,label=f"N={r['n_agents']} ({r['swarm_lat_ms']['n']} events)")
    ax.axvline(250,color="k",lw=0.8,ls="-.")
    ax.text(252,0.08,"2-frame bound\n(250 ms)",fontsize=6.5)
    ax.set_xlabel("Swarm-reaction propagation latency (ms)")
    ax.set_ylabel("CDF"); ax.set_xlim(0,290); ax.set_ylim(0,1.02)
    ax.legend(frameon=False,fontsize=7,loc="center right")
    fig.tight_layout()
    fig.savefig(f"{_R}/fig_swarm.pdf"); fig.savefig(f"{_R}/fig_swarm.png")
    print("fig_swarm ok")
