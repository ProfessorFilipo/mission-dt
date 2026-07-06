"""
Experiment battery for the Mission-DT paper. All numbers reported in the
paper come from these runs. Raw data is stored as JSON in results/.

E1  Scalability: N in {1,2,5,10,25,50,75,100} virtual agents, 30 s each.
    Metrics: frame compute time, frame overruns, telemetry latency,
    actuation latency, stale/duplicate updates.
E2  Bandwidth regulator: 10 agents, regulator ON (8 Hz) vs OFF (50 Hz).
    Metrics: bytes/s on the wire, latency, duplicate updates per frame.
"""
import gc
import json
import statistics as st
import sys
import time

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent))
from mission_dt.core import MissionDT
from mission_dt.agents import VirtualAgent, BASE_LAT, BASE_LON

RES = str(__import__("pathlib").Path(__file__).resolve().parent.parent / "results"); __import__("os").makedirs(RES, exist_ok=True)


def pctl(v, p):
    if not v:
        return None
    s = sorted(v)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))]


def run_trial(n_agents, duration=30.0, regulator=True, mix_aerial=True):
    gc.collect()
    dt = MissionDT()
    agents, goals = [], {}
    for i in range(n_agents):
        dom = "aerial" if (mix_aerial and i % 2) else "surface"
        a = VirtualAgent(f"ag{i:03d}", domain=dom,
                         regulator=regulator, duration_s=duration + 2)
        agents.append(a)
        goals[a.aid] = (BASE_LAT + 0.002 * (i % 7 - 3),
                        BASE_LON + 0.002 * (i // 7 - 3),
                        15.0 if dom == "aerial" else 0.0)
    time.sleep(1.0)          # registration settle
    for a in agents:
        a.start()
    dt.run(duration, goals=goals)
    for a in agents:
        a.join(timeout=5)
    lat_t = dt.msg_latencies
    lat_a = [x for a in agents for x in a.act_latencies]
    total_bytes = sum(a.bytes_out for a in agents)
    total_msgs = sum(a.msgs_out for a in agents)
    return {
        "n_agents": n_agents, "duration_s": duration, "regulator": regulator,
        "frames": dt.frames, "overruns": dt.frame_overruns,
        "stale_updates": dt.stale_updates, "dup_updates": dt.dup_updates,
        "frame_ms": {"mean": st.mean(dt.frame_compute) * 1e3,
                     "p99": pctl(dt.frame_compute, 99) * 1e3,
                     "max": max(dt.frame_compute) * 1e3},
        "telemetry_lat_ms": {"mean": st.mean(lat_t) * 1e3,
                             "p50": pctl(lat_t, 50) * 1e3,
                             "p99": pctl(lat_t, 99) * 1e3,
                             "max": max(lat_t) * 1e3, "n": len(lat_t)},
        "actuation_lat_ms": {"mean": st.mean(lat_a) * 1e3,
                             "p99": pctl(lat_a, 99) * 1e3, "n": len(lat_a)},
        "uplink_Bps": total_bytes / duration,
        "uplink_msgs_s": total_msgs / duration,
        "raw_frame_compute_ms": [x * 1e3 for x in dt.frame_compute],
        "raw_telemetry_lat_ms": [x * 1e3 for x in lat_t[:20000]],
    }


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "all"

    if which in ("e1", "all"):
        out = []
        for n in [1, 2, 5, 10, 25, 50, 75, 100]:
            print(f"[E1] N={n} ...", flush=True)
            r = run_trial(n)
            out.append(r); json.dump(out, open(f"{RES}/_partial.json","w"))
            print(f"     frames={r['frames']} overruns={r['overruns']} "
                  f"frame_p99={r['frame_ms']['p99']:.2f}ms "
                  f"tele_p99={r['telemetry_lat_ms']['p99']:.2f}ms "
                  f"stale={r['stale_updates']} dup={r['dup_updates']}", flush=True)
            time.sleep(2)
        json.dump(out, open(f"{RES}/e1_scalability.json", "w"))

    if which in ("e2", "all"):
        out = []
        for reg in (True, False):
            print(f"[E2] regulator={'ON' if reg else 'OFF'} ...", flush=True)
            r = run_trial(10, regulator=reg)
            out.append(r); json.dump(out, open(f"{RES}/_partial.json","w"))
            print(f"     uplink={r['uplink_Bps']/1024:.1f} KiB/s "
                  f"({r['uplink_msgs_s']:.0f} msg/s) "
                  f"tele_p99={r['telemetry_lat_ms']['p99']:.2f}ms "
                  f"dup={r['dup_updates']}", flush=True)
            time.sleep(2)
        json.dump(out, open(f"{RES}/e2_regulator.json", "w"))
    print("done")
