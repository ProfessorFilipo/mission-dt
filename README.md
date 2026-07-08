# Mission-DT — Reproducible artifact (SBESC 2026)

Mission-level Digital Twin for hybrid fleets (physical + virtual, aerial + surface).
All numbers in the paper come from `results/*.json` (raw measurements included).

## Reproduce the experiments (any Linux box)
    apt install mosquitto && pip install paho-mqtt matplotlib
    mosquitto -d
    python3 experiments/run_experiments.py all     # ~6 min; writes results/*.json
    python3 experiments/make_figures.py            # regenerates paper figures

## E3: swarm-reaction propagation experiment
    python experiments/run_e3.py           # all sizes (10, 25, 50)
    python experiments/run_e3.py 25        # single size
    python experiments/make_figures.py     # includes fig_swarm CDF

## 3D mission view (needs display/GPU — run locally)
    pip install ursina paho-mqtt
    # terminal 1: mosquitto ; terminal 2: swarm demo mission
    python experiments/demo_mission.py
    # terminal 3:
    python viz/mission_viz.py
    # Screenshot + FPS from the HUD -> insert into the paper (Sec. IV, fig_viz3d)

## Connecting the real Jundiá boats (field campaign)
Implement `mavlink_bridge.py` on the RP4: pymavlink ATTITUDE/GLOBAL_POSITION_INT/
SYS_STATUS -> telemetry JSON contract (see agents.py `_telemetry`) on
missiondt/agents/<id>/telemetry; actuation -> RC_OVERRIDE. Local broker already
bridges to the ground station (Fleet-DT infrastructure).

Environment used for the paper's measurements: 1 vCPU Intel Xeon @2.10 GHz,
4 GiB RAM, Linux (Ubuntu 24.04), Python 3.12.3, Mosquitto 2.x, paho-mqtt 2.1.0.
