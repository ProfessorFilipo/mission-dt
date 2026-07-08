# Mission-DT

**Mission-level Digital Twin for hybrid fleets of physical and virtual
unmanned vehicles** (aerial + surface), built on MQTT with a lightweight
Python core, a decoupled 3D mission view, and a live agent panel.
<br>by Filipo Novo Mór

Companion code for the paper *"Mission-DT: A Mission-Level Digital Twin
Architecture for Hybrid Fleets of Physical and Virtual Unmanned
Vehicles"*. All numbers in the paper come
from `results/*.json` (raw measurements included).

## Key ideas
- The **mission** is the twinned entity: `M = <states, transitions, goals, context φ>`.
- **Hybrid agents**: each agent is a physical vehicle (ArduPilot bridged
  to MQTT) or a **virtual agent — an independent digital twin** with its
  own model, state and MQTT connection. The mission core cannot tell
  them apart; agents can move to other processes/machines unchanged.
- **Swarm coordination**: the mission context φ (pairwise distances)
  triggers corrective actuation; propagation to neighbours is bounded
  by two DT frames (250 ms) — measured, not assumed.
- **Bandwidth regulators** decimate 50 Hz sensing to the 8 Hz frame
  rate (~6x uplink reduction).

## Requirements
- Python 3.10+ · Mosquitto MQTT broker (localhost)
- `pip install -r requirements.txt`
- 3D view (GPU machine): `pip install ursina imageio imageio-ffmpeg`
- Panel (optional pretty mode): `pip install rich`

## Quick start
```bash
mosquitto -v                                   # terminal 1 (broker)
python experiments/demo_mission.py             # terminal 2 (mission)
python viz/mission_viz.py                      # terminal 3 (3D view)
python experiments/panel.py                    # terminal 4 (dashboard)
```

## Mission configuration
`configs/mission_default.json` controls fleet size and routes:
```json
{ "profile": "orbitas",
  "drones": { "aerial": 5, "surface": 5 },
  "radius_m": 45.0, "arrive_m": 3.0, "aerial_alt_m": 15.0,
  "checkpoints": {}, "assignments": {} }
```
**Profiles** (auto-generated routes): `orbitas` (antipodal patrol),
`paralelo` (parallel lanes), `explorador` (aerial drones scout a moving
point ahead of their paired vessel), `aleatorio` (random waypoints
converging on a common destination, regenerated every lap).
**Explicit routes**: define labelled `checkpoints` (P01…, `[lat, lon,
alt_m]`; alt>0 air corridor, 0 surface, <0 submerged) and per-drone
`assignments` — checkpoints may be shared. See
`configs/mission_checkpoints_example.json`. Checkpoints and planned
routes are published retained on MQTT, so the 3D view labels them with
no config file.

## 3D view controls
| Button / key | Action |
|---|---|
| FOTO / `P` | screenshot → `captures/` |
| REC / `R` | record MP4 (H.264, YouTube/Vimeo-ready) → `captures/` |
| HQ / `H` | toggle 30 fps high / 15 fps low quality |
| ID / `I` | drone name labels |
| ROTA / `K` | planned-route lines (green; grey dots = past trail) |
| LEG / `L` | translucent legend |
| PANEL / `O` | floating agent panel inside the 3D window |
| `G` / `ESC` | sky grid / quit |

Status cues: agent turns **grey** after 1.5 s without telemetry;
pulsing **red** marker = low battery; safety spheres (12 m true-scale)
white on approach, orange in conflict, red on near-collision.

## Experiments (reproduce the paper)
```bash
python experiments/run_experiments.py all   # E1 scalability + E2 regulators (~6 min)
python experiments/run_e3.py                # E3 swarm propagation latency
python experiments/run_e4.py                # E4 twin fidelity at 0/5/10% loss
python experiments/make_figures.py          # figures into results/
```
Measured on 1 vCPU (Xeon 2.10 GHz): zero 125 ms frame overruns up to
100 agents; regulators cut uplink ~6x; swarm corrective actuation
median 118 ms, max 215 ms (bound: 250 ms). Independently reproduced on
Windows 10 (i7-9700) and macOS (Apple Silicon).

## Repository layout
```
mission_dt/    core (MissionDT) and virtual agents
experiments/   demo_mission, run_experiments (E1/E2), run_e3, panel, figures
viz/           3D mission view (Ursina)
configs/       mission configuration files
results/       raw measurements (JSON) and figures
```

## Connecting real vehicles
Run a MAVLink→MQTT bridge on the companion computer (e.g. Raspberry
Pi 4 + ArduPilot): publish telemetry on
`missiondt/agents/<id>/telemetry`, consume
`missiondt/agents/<id>/actuation`. The local broker bridges to the
ground station over Wi-Fi (see the paper, Section III).

## License
MIT (see `LICENSE`).
