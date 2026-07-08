"""
Demo mission v3 -- configurable fleets and routes.

    python experiments/demo_mission.py [--config configs/mission_default.json]

Config (JSON):
  drones        {"aerial": N, "surface": M}   fleet composition
  profile       "orbitas" | "paralelo" | "explorador" | "aleatorio"
                | "checkpoints" (explicit routes)
  checkpoints   {"P01": [lat, lon, alt_m], ...}  alt>0 air corridor,
                alt=0 surface, alt<0 submerged
  assignments   {"sur00": ["P01","P02"], ...}  checkpoints may be shared
  radius_m, arrive_m, aerial_alt_m

Profiles (used when no explicit assignments):
  orbitas     antipodal patrol across a circle (endless crossings)
  paralelo    parallel lanes, back-and-forth transit
  explorador  surface vessels advance in line; aerial drones scout a
              moving point ahead of their paired vessel (dynamic goal)
  aleatorio   random waypoints per agent converging on a common
              destination, regenerated every lap

Checkpoints are published retained on missiondt/mission/checkpoints so
the 3D visualizer labels them (P01, P02, ...) with no config file.
"""
import argparse
import json
import math
import random
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from mission_dt.core import MissionDT
from mission_dt.agents import VirtualAgent, BASE_LAT, BASE_LON

DURATION_S = 3600.0
LOOKAHEAD_M = 25.0        # explorador: how far ahead aerial scouts fly


def offset(radius_m, theta, base=(BASE_LAT, BASE_LON)):
    return (base[0] + radius_m * math.cos(theta) / 111_320.0,
            base[1] + radius_m * math.sin(theta) /
            (111_320.0 * math.cos(math.radians(base[0]))))


def dist_m(lat1, lon1, lat2, lon2):
    dy = (lat2 - lat1) * 111_320.0
    dx = (lon2 - lon1) * 111_320.0 * math.cos(math.radians(lat1))
    return math.hypot(dx, dy)


# ---------------------------------------------------------------- profiles
def gen_orbitas(ids, cfg):
    routes, n = {}, len(ids)
    for i, aid in enumerate(ids):
        th = 2 * math.pi * i / n
        alt = cfg["aerial_alt_m"] if aid.startswith("aer") else 0.0
        p0, p1 = offset(cfg["radius_m"], th), offset(cfg["radius_m"], th + math.pi)
        routes[aid] = [(p1[0], p1[1], alt), (p0[0], p0[1], alt)]
    return routes


def gen_paralelo(ids, cfg):
    routes, n = {}, len(ids)
    lane_gap, leg = 12.0, cfg["radius_m"] * 2
    for i, aid in enumerate(ids):
        alt = cfg["aerial_alt_m"] if aid.startswith("aer") else 0.0
        off_lat = (i - (n - 1) / 2) * lane_gap / 111_320.0
        lon_m = 111_320.0 * math.cos(math.radians(BASE_LAT))
        a = (BASE_LAT + off_lat, BASE_LON - leg / 2 / lon_m)
        b = (BASE_LAT + off_lat, BASE_LON + leg / 2 / lon_m)
        routes[aid] = [(b[0], b[1], alt), (a[0], a[1], alt)]
    return routes


def gen_aleatorio(ids, cfg):
    routes = {}
    common = offset(cfg["radius_m"] * 0.8, random.uniform(0, 2 * math.pi))
    for aid in ids:
        alt = cfg["aerial_alt_m"] if aid.startswith("aer") else 0.0
        wps = [(*offset(random.uniform(10, cfg["radius_m"]),
                        random.uniform(0, 2 * math.pi)), alt)
               for _ in range(3)]
        wps.append((common[0], common[1], alt))
        routes[aid] = wps
    return routes


# ---------------------------------------------------------------- setup
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "configs" /
                                            "mission_default.json"))
    args = ap.parse_args()
    cfg = json.load(open(args.config))
    profile = cfg.get("profile", "orbitas")

    dt = MissionDT(swarm=True)              # core first
    # publish checkpoints (retained) for the visualizer labels
    dt.cli.publish("missiondt/mission/checkpoints",
                   json.dumps(cfg.get("checkpoints", {})), qos=1, retain=True)

    ids = [f"aer{i:02d}" for i in range(cfg["drones"].get("aerial", 0))] + \
          [f"sur{i:02d}" for i in range(cfg["drones"].get("surface", 0))]

    # routes: explicit assignments win; otherwise profile generator
    cps = cfg.get("checkpoints", {})
    if cfg.get("assignments"):
        routes = {aid: [tuple(cps[p]) for p in plist]
                  for aid, plist in cfg["assignments"].items() if aid in ids}
        for aid in ids:                       # unassigned agents hold position
            routes.setdefault(aid, None)
    elif profile == "paralelo":
        routes = gen_paralelo(ids, cfg)
    elif profile == "aleatorio":
        routes = gen_aleatorio(ids, cfg)
    elif profile == "explorador":
        routes = gen_paralelo(ids, cfg)       # vessels get lanes; aerial dynamic
    else:
        routes = gen_orbitas(ids, cfg)

    agents, goals, idx = [], {}, {}
    for i, aid in enumerate(ids):
        dom = "aerial" if aid.startswith("aer") else "surface"
        a = VirtualAgent(aid, domain=dom, duration_s=DURATION_S + 2)
        th = 2 * math.pi * i / max(1, len(ids))
        a.lat, a.lon = offset(cfg.get("radius_m", 45.0), th)
        agents.append(a)
        idx[aid] = 0
        r = routes.get(aid)
        goals[aid] = r[0] if r else (a.lat, a.lon,
                                     cfg["aerial_alt_m"] if dom == "aerial" else 0.0)

    surface_ids = [i2 for i2 in ids if i2.startswith("sur")]

    def pub_routes():
        payload = {aid: r for aid, r in routes.items()
                   if r and not (profile == "explorador"
                                 and aid.startswith("aer"))}
        dt.cli.publish("missiondt/mission/routes", json.dumps(payload),
                       qos=1, retain=True)

    pub_routes()

    def monitor():
        while True:
            time.sleep(0.25)
            for aid in ids:
                rec = dt.agents.get(aid)
                if rec is None:
                    continue
                # explorador: aerial goal = moving point ahead of paired vessel
                if profile == "explorador" and aid.startswith("aer") \
                        and surface_ids and not cfg.get("assignments"):
                    mate = dt.agents.get(
                        surface_ids[ids.index(aid) % len(surface_ids)])
                    if mate:
                        s = mate.state
                        goals[aid] = (
                            s.lat + LOOKAHEAD_M * math.cos(s.yaw) / 111_320.0,
                            s.lon + LOOKAHEAD_M * math.sin(s.yaw) /
                            (111_320.0 * math.cos(math.radians(s.lat))),
                            cfg["aerial_alt_m"])
                    continue
                r = routes.get(aid)
                if not r:
                    continue
                g = r[idx[aid] % len(r)]
                if dist_m(rec.state.lat, rec.state.lon, g[0], g[1]) \
                        < cfg.get("arrive_m", 3.0):
                    idx[aid] += 1
                    if profile == "aleatorio" and idx[aid] % len(r) == 0:
                        routes.update(gen_aleatorio([aid], cfg))  # new lap
                        pub_routes()
                    goals[aid] = routes[aid][idx[aid] % len(routes[aid])]

    time.sleep(1.0)
    for a in agents:
        a.start()
    threading.Thread(target=monitor, daemon=True).start()
    print(f"Mission running: {len(ids)} agents "
          f"({cfg['drones']}), profile='{profile}', swarm ON. Ctrl+C stops.")
    dt.run(DURATION_S, goals=goals)


if __name__ == "__main__":
    main()
