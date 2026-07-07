# Mission-DT — Reproducible artifact

Mission-level Digital Twin for hybrid fleets (physical + virtual, aerial + surface).
All numbers in the paper come from `results/*.json` (raw measurements included).

## Reproduce the experiments (any Linux box)
    apt install mosquitto && pip install paho-mqtt matplotlib
    mosquitto -d
    python3 experiments/run_experiments.py all     # ~6 min; writes results/*.json
    python3 experiments/make_figures.py            # regenerates paper figures

## 3D mission view (needs display/GPU — run locally)
    pip install ursina paho-mqtt
    # terminal 1: mosquitto ; terminal 2: launch a demo mission
    python3 - << 'PY'
import sys, time; sys.path.insert(0,'.')
from mission_dt.core import MissionDT
from mission_dt.agents import VirtualAgent, BASE_LAT, BASE_LON
ags=[VirtualAgent(f"ag{i}", domain=("aerial" if i%2 else "surface"), duration_s=600) for i in range(8)]
[a.start() for a in ags]; time.sleep(1)
goals={a.aid:(BASE_LAT+0.002*(i%4-2), BASE_LON+0.002*(i//4-1), 15.0 if a.domain=="aerial" else 0.0) for i,a in enumerate(ags)}
MissionDT().run(600, goals=goals)
PY
    # terminal 3:
    python3 viz/mission_viz.py
    # Screenshot + FPS from the HUD -> insert into the paper (Sec. IV / new Fig.)

## Connecting the real Jundiá boats (field campaign)
Implement `mavlink_bridge.py` on the RP4: pymavlink ATTITUDE/GLOBAL_POSITION_INT/
SYS_STATUS -> telemetry JSON contract (see agents.py `_telemetry`) on
missiondt/agents/<id>/telemetry; actuation -> RC_OVERRIDE. Local broker already
bridges to the ground station (Fleet-DT infrastructure).

Environment used for the paper's measurements: 1 vCPU Intel Xeon @2.10 GHz,
4 GiB RAM, Linux (Ubuntu 24.04), Python 3.12.3, Mosquitto 2.x, paho-mqtt 2.1.0.
