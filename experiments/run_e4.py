"""
E4 -- Twin fidelity under packet loss.

Question: how faithful is the twin's state estimate, and how does it
degrade when the network drops telemetry?

Method: virtual agents log their ground-truth pose at 50 Hz; a logging
subclass of the mission core records the twin's estimate (and the
stale flag) at every frame. Loss is injected at the regulator output
with i.i.d. probability p in {0, 5%, 10%} -- representative of harsh
maritime Wi-Fi. For every core sample we interpolate the ground truth
at the same instant and compute position error (metres) and heading
error (degrees, wrapped). Reported: RMSE, p99, worst error observed
during stale (dead-reckoned) frames.

    python experiments/run_e4.py [loss ...]      # default: 0 0.05 0.10
"""
import bisect
import json
import math
import statistics as st
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from mission_dt.core import MissionDT
from mission_dt.agents import VirtualAgent, BASE_LAT, BASE_LON

RES = str(ROOT / "results")


class LoggingDT(MissionDT):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.state_log = []   # (t, aid, lat, lon, yaw, stale)

    def _delta(self, rec, telems):
        super()._delta(rec, telems)
        if rec.last_seq >= 0:      # only after first real telemetry
            self.state_log.append((time.time(), rec.agent_id, rec.state.lat,
                                   rec.state.lon, rec.state.yaw, rec.stale))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def pctl(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))] if s else None


def run_e4(loss, n_agents=10, duration=30.0):
    dt = LoggingDT()
    agents, goals = [], {}
    for i in range(n_agents):
        dom = "aerial" if i % 2 else "surface"
        a = VirtualAgent(f"fd{i:02d}", domain=dom,
                         duration_s=duration + 2, loss=loss)
        agents.append(a)
        goals[a.aid] = (BASE_LAT + 0.002 * (i % 7 - 3),
                        BASE_LON + 0.002 * (i // 7 - 3),
                        15.0 if dom == "aerial" else 0.0)
    time.sleep(1.0)
    for a in agents:
        a.start()
    dt.run(duration, goals=goals)
    for a in agents:
        a.join(timeout=5)

    truth = {a.aid: a.truth_log for a in agents}
    times = {aid: [r[0] for r in log] for aid, log in truth.items()}
    pos_err, hdg_err, stale_err = [], [], []
    for (t, aid, lat, lon, yaw, stale) in dt.state_log:
        log, ts = truth[aid], times[aid]
        i = bisect.bisect_left(ts, t)
        if i == 0 or i >= len(ts):
            continue
        (t0, la0, lo0, y0), (t1, la1, lo1, y1) = log[i - 1], log[i]
        f = (t - t0) / max(1e-9, t1 - t0)
        tlat, tlon = la0 + f * (la1 - la0), lo0 + f * (lo1 - lo0)
        tyaw = y0 + f * (wrap_deg(math.degrees(y1 - y0)) * math.pi / 180.0)
        dy = (lat - tlat) * 111_320.0
        dx = (lon - tlon) * 111_320.0 * math.cos(math.radians(tlat))
        pe = math.hypot(dx, dy)
        he = abs(wrap_deg(math.degrees(yaw - tyaw)))
        pos_err.append(pe)
        hdg_err.append(he)
        if stale:
            stale_err.append(pe)
    frames_agents = len(dt.state_log)
    lost = sum(a.lost_msgs for a in agents)
    sent = sum(a.msgs_out for a in agents)
    return {
        "loss": loss, "n_agents": n_agents, "duration_s": duration,
        "samples": len(pos_err),
        "lost_msgs": lost, "sent_msgs": sent,
        "stale_pct": 100.0 * dt.stale_updates / max(1, frames_agents),
        "pos_rmse_m": math.sqrt(st.mean(e * e for e in pos_err)),
        "pos_p99_m": pctl(pos_err, 99),
        "pos_max_stale_m": max(stale_err) if stale_err else 0.0,
        "hdg_rmse_deg": math.sqrt(st.mean(e * e for e in hdg_err)),
        "hdg_p99_deg": pctl(hdg_err, 99),
    }


if __name__ == "__main__":
    import os
    losses = [float(x) for x in sys.argv[1:]] or [0.0, 0.05, 0.10]
    fn = f"{RES}/e4_fidelity.json"
    out = json.load(open(fn)) if os.path.exists(fn) else []
    out = [r for r in out if r["loss"] not in losses]
    for L in losses:
        print(f"[E4] loss={L:.0%} ...", flush=True)
        r = run_e4(L)
        out.append(r)
        json.dump(sorted(out, key=lambda x: x["loss"]), open(fn, "w"))
        print(f"     pos RMSE={r['pos_rmse_m']:.2f} m  p99={r['pos_p99_m']:.2f} m"
              f"  max(stale)={r['pos_max_stale_m']:.2f} m | "
              f"hdg RMSE={r['hdg_rmse_deg']:.1f} deg | stale={r['stale_pct']:.1f}%",
              flush=True)
        time.sleep(2)
    print("done")
