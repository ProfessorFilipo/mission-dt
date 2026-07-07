"""
Demo mission for the 3D visualizer -- run alongside viz/mission_viz.py.

10 hybrid agents (5 aerial, 5 surface) patrol WAYPOINT lists with swarm
coordination ON: each agent flies to its next waypoint (advancing when
within ARRIVE_M meters, looping forever), and the mission core's
separation rule makes neighbors veer apart when they conflict.

EDIT YOUR ROUTES HERE: change WAYPOINTS below. Each agent id maps to a
list of (lat, lon, alt_m) tuples. Default: antipodal patrol across the
circle center -- endless crossings, ideal for video recording.

Usage (three terminals):
    1) mosquitto        2) python experiments/demo_mission.py
    3) python viz/mission_viz.py
"""
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mission_dt.core import MissionDT
from mission_dt.agents import VirtualAgent, BASE_LAT, BASE_LON

N, RADIUS_M, DURATION_S, ARRIVE_M = 10, 45.0, 3600.0, 3.0


def offset(radius_m, theta):
    return (BASE_LAT + radius_m * math.cos(theta) / 111_320.0,
            BASE_LON + radius_m * math.sin(theta) /
            (111_320.0 * math.cos(math.radians(BASE_LAT))))

# ---------------------------------------------------------------- routes
WAYPOINTS = {}
for i in range(N):
    th = 2 * math.pi * i / N
    alt = 15.0 if i % 2 else 0.0
    p0, p1 = offset(RADIUS_M, th), offset(RADIUS_M, th + math.pi)
    WAYPOINTS[f"demo{i:02d}"] = [(p1[0], p1[1], alt), (p0[0], p0[1], alt)]
# Example of a custom square route for one agent (uncomment to try):
# WAYPOINTS["demo00"] = [(*offset(60, a), 0.0)
#                        for a in (0, math.pi/2, math.pi, 3*math.pi/2)]

# ---------------------------------------------------------------- setup
dt = MissionDT(swarm=True)          # core first (retained registers also ok)
agents, goals, idx = [], {}, {}
for i, (aid, wps) in enumerate(WAYPOINTS.items()):
    th = 2 * math.pi * i / N
    dom = "aerial" if wps[0][2] > 1.0 else "surface"
    a = VirtualAgent(aid, domain=dom, duration_s=DURATION_S + 2)
    a.lat, a.lon = offset(RADIUS_M, th)
    a.yaw = th + math.pi
    agents.append(a)
    idx[aid] = 0
    goals[aid] = wps[0]


def waypoint_monitor():
    """Advance each agent to its next waypoint on arrival (loops)."""
    while True:
        time.sleep(0.25)
        for aid, wps in WAYPOINTS.items():
            rec = dt.agents.get(aid)
            if rec is None:
                continue
            g = wps[idx[aid] % len(wps)]
            dy = (g[0] - rec.state.lat) * 111_320.0
            dx = (g[1] - rec.state.lon) * 111_320.0 * \
                math.cos(math.radians(rec.state.lat))
            if math.hypot(dx, dy) < ARRIVE_M:
                idx[aid] += 1
                goals[aid] = wps[idx[aid] % len(wps)]


time.sleep(1.0)
for a in agents:
    a.start()
threading.Thread(target=waypoint_monitor, daemon=True).start()
print(f"Mission running: {N} agents, swarm ON, waypoint patrol. "
      "Ctrl+C to stop.")
dt.run(DURATION_S, goals=goals)
