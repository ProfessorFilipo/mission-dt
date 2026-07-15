"""
Staged scene for Figure 7 -- a deterministic, motionless composition of
the 3D mission view, designed for screenshotting without any timing
pressure.

Unlike demo_mission.py (agents travel between checkpoints, so the
"good moment" -- vehicles in conflict -- is fleeting and unpredictable),
eight of the nine agents here are placed DIRECTLY at their final
position, with each agent's own goal equal to that same position. The
core's lambda controller then holds throttle at zero (distance-to-goal
< 1.5 m), so those eight agents simply stand still. The swarm
separation rule keeps running for real on this frozen configuration,
so the safety spheres you see are still a genuine, live computation --
not faked -- they just no longer change from second to second, because
nothing is moving. The ninth agent (sur05) is a real, moving patrol
boat that loops around the whole composition, to demonstrate the ROTA
route-line feature against a backdrop that is otherwise frozen.

Composition -- a compact diagonal chain showing the full spectrum of
separation states in one frame, with no large empty gaps:
  - sur00, sur01  : head-on pair, offset laterally and angled toward
                    each other (not perfectly co-linear) at ~11 m real
                    separation -> ORANGE conflict spheres, without the
                    two ~12 m-long hull models visually fusing
  - aer00, aer01  : aerial pair ~18 m apart, hovering above the same
                    cluster -> WHITE approach spheres
  - sur03, sur04  : near-collision pair ~2 m apart, headings
                    PERPENDICULAR to each other (one east, one north)
                    so the crossing reads as two distinct hulls rather
                    than a single fused blob -> RED spheres
  - sur02, aer02  : parked beyond the red pair, calm/normal operation,
                    no spheres
  - sur05         : a real, moving surface agent patrolling a loop
                    around the outside of all three clusters, staying
                    clear of each one -- enable ROTA to see its route
  - P01 (surface), P04 (air corridor, 20 m), P09 (submerged, -2 m):
    labelled checkpoints near the calm agents, for visual variety

All same-domain cross-cluster distances (among the frozen agents) are
kept above 24 m so the three clusters read as visually and
functionally distinct.

Note: this script runs the core with swarm=False. The safety spheres
you see in the 3D view are computed purely from same-domain distance
on the client side (viz/mission_viz.py), independent of whether the
core's separation override is active -- so the spheres are exactly as
correct either way. Turning the override off here avoids a fight
between it (which continuously steers agents away from their nearest
same-domain neighbour) and the frozen heading this script sets up.

Usage (three terminals):
    1) mosquitto (broker running)
    2) python experiments/staged_photo.py
    3) python viz/mission_viz.py
The eight frozen agents are stable from the first frame onward --
screenshot whenever you like. sur05 keeps moving in the background;
wait for it to be somewhere unobtrusive if you want it out of the shot,
or let it add a touch of life to the picture.
"""
import json
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mission_dt.core import MissionDT
from mission_dt.agents import VirtualAgent, BASE_LAT, BASE_LON

M_PER_DEG_LAT = 111_320.0
M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(BASE_LAT))
ARRIVE_M = 3.0


def offset(east_m, north_m, base=(BASE_LAT, BASE_LON)):
    """Return (lat, lon) of a point east_m/north_m metres from base."""
    return (base[0] + north_m / M_PER_DEG_LAT,
            base[1] + east_m / M_PER_DEG_LON)


def bearing_to(from_ll, to_ll):
    """Yaw (radians) pointing from from_ll to to_ll, matching the
    code's convention: yaw=0 -> north, yaw=pi/2 -> east."""
    dy = (to_ll[0] - from_ll[0]) * M_PER_DEG_LAT
    dx = (to_ll[1] - from_ll[1]) * M_PER_DEG_LON
    return math.atan2(dx, dy)


def freeze(agent, lat, lon, alt, yaw):
    """Place an agent at an exact pose and make it hold both position
    and heading there. The goal is set 1 m ahead along the intended
    yaw (not exactly at the agent's own position): a goal identical to
    the current position makes the bearing-to-goal calculation
    degenerate (near 0/0), letting the steering controller slowly
    rotate the vessel toward a meaningless heading over time even
    though it never translates. Offsetting the goal by 1 m keeps
    throttle at zero (still under the 1.5 m dead zone) while giving
    the controller a well-defined, unmoving bearing to settle onto and
    hold indefinitely.
    """
    agent.lat, agent.lon, agent.alt, agent.yaw = lat, lon, alt, yaw
    ghost_lat = lat + (1.0 * math.cos(yaw)) / M_PER_DEG_LAT
    ghost_lon = lon + (1.0 * math.sin(yaw)) / M_PER_DEG_LON
    return (ghost_lat, ghost_lon, alt)


# compact diagonal layout: main cluster -> near-collision bridge -> calm
MAIN_E, MAIN_N = 0.0, 0.0
BRIDGE_E, BRIDGE_N = 35.0, -18.0
CALM_E, CALM_N = 62.0, -34.0

checkpoints = {
    "P01": (*offset(CALM_E + 12, CALM_N - 6), 0.0),
    "P04": (*offset(CALM_E - 8, CALM_N + 12), 20.0),
    "P09": (*offset(CALM_E + 4, CALM_N + 4), -2.0),
}

agents, goals = [], {}

# --- main cluster: surface conflict pair (orange) ------------------------
# ~11 m apart (safely under the 12 m orange threshold) AND offset
# laterally + angled toward each other, instead of perfectly co-linear,
# so the two ~12 m-long hull models don't fully overlap.
p_sur00 = offset(MAIN_E - 5.4, MAIN_N + 1.2)
p_sur01 = offset(MAIN_E + 5.4, MAIN_N - 1.2)
a = VirtualAgent("sur00", domain="surface", duration_s=3600)
goals["sur00"] = freeze(a, *p_sur00, 0.0, bearing_to(p_sur00, p_sur01))
agents.append(a)

a = VirtualAgent("sur01", domain="surface", duration_s=3600)
goals["sur01"] = freeze(a, *p_sur01, 0.0, bearing_to(p_sur01, p_sur00))
agents.append(a)

# --- main cluster: aerial pair hovering above it (white) -----------------
a = VirtualAgent("aer00", domain="aerial", duration_s=3600)
lat, lon = offset(MAIN_E, MAIN_N + 18)
goals["aer00"] = freeze(a, lat, lon, 15.0, math.pi)
agents.append(a)

a = VirtualAgent("aer01", domain="aerial", duration_s=3600)
lat, lon = offset(MAIN_E + 18, MAIN_N + 18)
goals["aer01"] = freeze(a, lat, lon, 15.0, -math.pi / 2)
agents.append(a)

# --- bridge pair: near-collision surface duo, PERPENDICULAR headings -----
# ~2 m apart (safely under the 3 m red threshold); one heading east, one
# heading north, so the crossing reads as two distinct hulls (a "T"),
# not one fused shape.
a = VirtualAgent("sur03", domain="surface", duration_s=3600)
lat, lon = offset(BRIDGE_E - 1.0, BRIDGE_N)
goals["sur03"] = freeze(a, lat, lon, 0.0, math.pi / 2)        # facing east
agents.append(a)

a = VirtualAgent("sur04", domain="surface", duration_s=3600)
lat, lon = offset(BRIDGE_E + 1.0, BRIDGE_N)
goals["sur04"] = freeze(a, lat, lon, 0.0, 0.0)                # facing north
agents.append(a)

# --- calm, normal-operation agents beyond the bridge pair -----------------
a = VirtualAgent("sur02", domain="surface", duration_s=3600)
lat, lon = offset(CALM_E, CALM_N)
goals["sur02"] = freeze(a, lat, lon, 0.0, math.pi / 4)
agents.append(a)

a = VirtualAgent("aer02", domain="aerial", duration_s=3600)
lat, lon = offset(CALM_E - 15, CALM_N + 15)
goals["aer02"] = freeze(a, lat, lon, 15.0, 0.0)
agents.append(a)

# --- sur05: a REAL, moving boat patrolling a loop around everything ------
# Kept ~10-15 m clear of every cluster's own footprint, so its path
# never crosses through the frozen vehicles, while staying close enough
# to read as part of the same mission.
LOOP = [offset(-16, 26), offset(70, 26), offset(70, -40), offset(-16, -40)]
sur05 = VirtualAgent("sur05", domain="surface", duration_s=3600)
sur05.lat, sur05.lon = LOOP[0]
sur05.yaw = bearing_to(LOOP[0], LOOP[1])
agents.append(sur05)
loop_idx = [0]
goals["sur05"] = (*LOOP[0], 0.0)


def patrol_monitor():
    while True:
        time.sleep(0.25)
        rec = dt.agents.get("sur05")
        if rec is None:
            continue
        tgt = LOOP[loop_idx[0] % len(LOOP)]
        if math.hypot((tgt[0] - rec.state.lat) * M_PER_DEG_LAT,
                      (tgt[1] - rec.state.lon) * M_PER_DEG_LON) < ARRIVE_M:
            loop_idx[0] += 1
            nxt = LOOP[loop_idx[0] % len(LOOP)]
            goals["sur05"] = (*nxt, 0.0)


time.sleep(1.0)
for ag in agents:
    ag.start()

dt = MissionDT(swarm=False)
dt.cli.publish("missiondt/mission/checkpoints", json.dumps(checkpoints),
               qos=1, retain=True)
dt.cli.publish("missiondt/mission/routes",
               json.dumps({"sur05": [(*p, 0.0) for p in LOOP]}),
               qos=1, retain=True)
threading.Thread(target=patrol_monitor, daemon=True).start()

print("Staged scene running -- eight agents frozen, sur05 patrolling "
      "the perimeter. Open the 3D view and screenshot whenever you "
      "like. Ctrl+C to stop.")
dt.run(3600.0, goals=goals)
