"""
E3 -- Swarm-reaction propagation latency.

Scenario: N agents start on a circle (radius R_M) and receive antipodal
goals, forcing everyone through the center and creating repeated
separation conflicts. The mission core runs with swarm=True: the phi
context (inter-agent distances) triggers a separation rule that
overrides lambda for conflicting neighbors.

Measured, per corrective actuation delivered to a neighbor:
    propagation latency = t_apply(neighbor) - t_pub(triggering telemetry)
i.e., the full path: maneuvering agent publishes telemetry -> mission
core frame (delta + phi + separation) -> corrective actuation received
by the neighbor. Upper bound by design: ~2 frames (250 ms) + delivery.

Also reports Delta^e frame cost with the O(N^2) phi computation on, to
show the twin still meets the 125 ms deadline with coordination active.
"""
import json
import math
import statistics as st
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mission_dt.core import MissionDT
from mission_dt.agents import VirtualAgent, BASE_LAT, BASE_LON

RES = str(Path(__file__).resolve().parent.parent / "results")


def pctl(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))] if s else None


def run_e3(n_agents, duration=40.0, radius_m=40.0, sep_m=12.0):
    dt = MissionDT(swarm=True, sep_m=sep_m)
    agents, goals = [], {}
    for i in range(n_agents):
        th = 2 * math.pi * i / n_agents
        dlat = radius_m * math.cos(th) / 111_320.0
        dlon = radius_m * math.sin(th) / (111_320.0 * math.cos(math.radians(BASE_LAT)))
        dom = "aerial" if i % 2 else "surface"
        a = VirtualAgent(f"sw{i:03d}", domain=dom, duration_s=duration + 2)
        a.lat, a.lon = BASE_LAT + dlat, BASE_LON + dlon
        a.yaw = th + math.pi          # facing the center
        agents.append(a)
        goals[a.aid] = (BASE_LAT - dlat, BASE_LON - dlon,   # antipode
                        15.0 if dom == "aerial" else 0.0)
    time.sleep(1.0)
    for a in agents:
        a.start()
    dt.run(duration, goals=goals)
    for a in agents:
        a.join(timeout=5)
    lat_sw = [x for a in agents for x in a.swarm_latencies]
    return {
        "n_agents": n_agents, "duration_s": duration, "sep_m": sep_m,
        "frames": dt.frames, "overruns": dt.frame_overruns,
        "avoid_events": dt.avoid_events,
        "frame_ms": {"mean": st.mean(dt.frame_compute) * 1e3,
                     "p99": pctl(dt.frame_compute, 99) * 1e3,
                     "max": max(dt.frame_compute) * 1e3},
        "swarm_lat_ms": {"mean": st.mean(lat_sw) * 1e3,
                         "p50": pctl(lat_sw, 50) * 1e3,
                         "p95": pctl(lat_sw, 95) * 1e3,
                         "p99": pctl(lat_sw, 99) * 1e3,
                         "max": max(lat_sw) * 1e3, "n": len(lat_sw)},
        "raw_swarm_lat_ms": [x * 1e3 for x in lat_sw[:50000]],
    }


if __name__ == "__main__":
    import os
    sizes = [int(x) for x in sys.argv[1:]] or [10, 25, 50]
    fn = f"{RES}/e3_swarm.json"
    out = json.load(open(fn)) if os.path.exists(fn) else []
    out = [r for r in out if r["n_agents"] not in sizes]
    for n in sizes:
        print(f"[E3] N={n} ...", flush=True)
        r = run_e3(n)
        out.append(r)
        json.dump(out, open(f"{RES}/e3_swarm.json", "w"))
        s = r["swarm_lat_ms"]
        print(f"     events={s['n']} lat p50={s['p50']:.0f}ms p95={s['p95']:.0f}ms "
              f"p99={s['p99']:.0f}ms max={s['max']:.0f}ms | "
              f"frame_p99={r['frame_ms']['p99']:.2f}ms overruns={r['overruns']}",
              flush=True)
        time.sleep(2)
    print("done")
