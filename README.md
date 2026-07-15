# Mission-DT

**Mission-level Digital Twin for hybrid fleets of physical and virtual
unmanned vehicles** (aerial + surface), built on MQTT with a lightweight
Python core, a decoupled 3D mission view, and a live agent panel.
<br>by Filipo Novo Mór

Companion code for the paper *"A Mission-Level Digital Twin for Hybrid
Fleets of Physical and Virtual Unmanned Vehicles"*. All numbers in the
paper come from `results/*.json` (raw measurements included).

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
The launcher checks your virtual environment, starts the broker if it
isn't already running, clears any leftover retained MQTT state, and
opens the mission plus the 3D view in one go:
```bash
./run_mission.sh                              # default mission
./run_mission.sh configs/mission_photo.json   # any other mission file
```
Press **ESC** in the 3D view to stop everything cleanly.

Or, step by step, in separate terminals (useful if you want the live
terminal panel running alongside):
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
`configs/mission_checkpoints_example.json` for a small hand-written
example, or `configs/mission_photo.json` for a denser 6-agent, 10-
checkpoint mission (this is the one used to compose the paper's Figure
7). Checkpoints and planned routes are published retained on MQTT, so
the 3D view labels them with no config file of its own.

## Staging a screenshot deterministically
`experiments/staged_photo.py` is a different kind of script: instead of
generating a route and waiting for an interesting moment to occur
live, it places every agent directly at a chosen pose and holds it
there (goal = current position), so the safety-sphere states you want
(white/orange/red) are present and stable from the very first frame —
no timing, no luck, no waiting. One agent still patrols normally in
the background, to also show the ROTA route-line feature against an
otherwise-frozen scene:
```bash
python experiments/staged_photo.py     # terminal 1 — scene freezes instantly
python viz/mission_viz.py              # terminal 2 — screenshot whenever
```
Edit the coordinate constants near the top of the file to change the
composition (cluster spacing, which pairs are in conflict, etc.).

## 3D view controls
| Button / key | Action |
|---|---|
| FOTO / `P` | screenshot → `captures/` |
| REC / `R` | record MP4 (H.264, YouTube/Vimeo-ready) → `captures/` |
| HQ / `H` | toggle 30 fps high / 15 fps low quality |
| ID / `I` | drone name labels |
| ROTA / `K` | planned-route lines (green; grey dots = past trail) |
| LEG / `L` | translucent legend |
| PANEL / `O` | floating agent panel inside the 3D window (reuses `panel.py`) |
| `G` / `ESC` | sky grid / quit |

Status cues: agent turns **grey** after 1.5 s without telemetry;
pulsing **red** marker = battery below 17.6 V; safety spheres (12 m
true-scale) — white on approach (<24 m), orange in conflict (<12 m),
red on near-collision (<3 m).

## Live monitoring
Besides the in-view floating panel, `experiments/panel.py` runs as a
standalone terminal dashboard — a plain, independent MQTT client, so it
works on any machine that can reach the broker, with or without the 3D
view running:
```bash
python experiments/panel.py
```
Shows one row per agent (position, altitude, speed, battery, telemetry
rate, status) twice a second. Renders as a colour-coded live table if
`rich` is installed, otherwise falls back to plain printed rows.

## Experiments (reproduce the paper)
The whole battery, one command:
```bash
bash run_all.sh
```
Checks your virtual environment (by locating its python interpreter
directly, rather than relying on `source activate` — more robust
across Git Bash on Windows), starts the broker if needed, clears
retained state, then runs E1 through E4 and regenerates the figures in
sequence.

Or step by step:
```bash
python experiments/run_experiments.py all   # E1 scalability + E2 regulators (~6 min)
python experiments/run_e3.py                # E3 swarm propagation latency
python experiments/run_e4.py                # E4 twin fidelity at 0/5/10% loss
python experiments/make_figures.py          # figures into results/
```
Measured on 1 vCPU (Xeon 2.10 GHz): zero 125 ms frame overruns up to
100 agents; regulators cut uplink ~6x and redundant samples ~71x;
swarm corrective actuation median 118 ms, max 215 ms (bound: 250 ms);
twin position RMSE stays under 1 m through 10% injected packet loss.
Independently reproduced on Windows 10 (i7-9700) and macOS (Apple
Silicon) — see the paper's cross-platform table.

`experiments/clear_retained.py` clears leftover retained MQTT state
(ghost agent registrations, stale checkpoints/routes) — run it before
experiments if you've been poking around the broker manually;
`run_mission.sh` already calls it automatically.

## Maintainers: checking your checkout
```bash
bash check_uptodate.sh
```
Greps your local files for markers of every major feature (swarm
separation, retained registration, the four experiment scripts, the 3D
view's checkpoints/routes/panel support, …) and reports what's missing
— a quick sanity check after pulling or before reporting an issue.

## Repository layout
```
mission_dt/       core (MissionDT) and virtual agents
experiments/      demo_mission, staged_photo, run_experiments (E1/E2),
                  run_e3, run_e4, panel, make_figures, clear_retained
viz/              3D mission view (Ursina)
configs/          mission configuration files
results/          raw measurements (JSON) and figures
run_mission.sh    one-command launcher (broker + mission + 3D view)
run_all.sh        one-command experiment battery (E1-E4 + figures)
check_uptodate.sh maintainer script: verifies a checkout is current
requirements.txt  Python dependencies
```

## Connecting real vehicles
Run a MAVLink→MQTT bridge on the companion computer (e.g. Raspberry
Pi 4 + ArduPilot): publish telemetry on
`missiondt/agents/<id>/telemetry`, consume
`missiondt/agents/<id>/actuation`. The local broker bridges to the
ground station over Wi-Fi (see the paper, Section III).

## License
MIT (see `LICENSE`).