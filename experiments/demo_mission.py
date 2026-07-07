"""
Demo mission for the 3D visualizer -- run this alongside viz/mission_viz.py.

Scenario: 10 hybrid agents (5 aerial cones, 5 surface boxes) start on a
circle and receive antipodal goals, so everyone crosses the center.
The Mission-DT runs with swarm=True: watch neighbors veer away from
each other as the separation rule (phi context) issues corrective
actuation within <= 2 DT frames.

Usage (three terminals):
    1) mosquitto (broker running as service or in a console)
    2) python experiments/demo_mission.py
    3) python viz/mission_viz.py
Press F11/PrintScreen in the viz window for the paper screenshot; the
HUD shows agent count and rendering FPS.
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mission_dt.core import MissionDT
from mission_dt.agents import VirtualAgent, BASE_LAT, BASE_LON

N, RADIUS_M, DURATION_S = 10, 45.0, 600.0

agents, goals = [], {}
for i in range(N):
    th = 2 * math.pi * i / N
    dlat = RADIUS_M * math.cos(th) / 111_320.0
    dlon = RADIUS_M * math.sin(th) / (111_320.0 * math.cos(math.radians(BASE_LAT)))
    dom = "aerial" if i % 2 else "surface"
    a = VirtualAgent(f"demo{i:02d}", domain=dom, duration_s=DURATION_S + 2)
    a.lat, a.lon = BASE_LAT + dlat, BASE_LON + dlon
    a.yaw = th + math.pi
    agents.append(a)
    goals[a.aid] = (BASE_LAT - dlat, BASE_LON - dlon,
                    15.0 if dom == "aerial" else 0.0)

time.sleep(1.0)
for a in agents:
    a.start()
print(f"Mission running: {N} agents, swarm coordination ON. Ctrl+C to stop.")
MissionDT(swarm=True).run(DURATION_S, goals=goals)
