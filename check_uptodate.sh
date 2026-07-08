#!/bin/bash
# Mission-DT: verify local files against the latest canonical versions.
# Run from the repository root:  bash check_uptodate.sh
ok=0; bad=0
chk () {  # chk <file> <marker> <description>
  if [ -f "$1" ] && grep -q "$2" "$1"; then
    echo "  OK        $1  ($3)"; ok=$((ok+1))
  else
    echo "  OUTDATED  $1  -- missing: $3"; bad=$((bad+1))
  fi
}
echo "== Mission-DT up-to-date check =="
chk mission_dt/core.py        "_separation"          "swarm separation rule (E3)"
chk mission_dt/core.py        "avoid_events"         "E3 metrics"
chk mission_dt/agents.py      "retain=True"          "retained MQTT registration"
chk mission_dt/agents.py      "swarm_latencies"      "E3 propagation measurement"
chk experiments/run_e3.py     "antipodal"            "E3 experiment script"
chk experiments/demo_mission.py "WAYPOINTS"          "waypoint patrol demo v2"
chk experiments/make_figures.py "fig_swarm"          "E3 CDF figure"
chk experiments/run_experiments.py "pathlib"         "relative paths fix"
chk viz/mission_viz.py        "SAFE_M"               "safety spheres (viz v3)"
chk viz/mission_viz.py        "class Recorder"       "video recording (viz v3)"
chk viz/mission_viz.py        "build_aerial"         "quadcopter/vessel models"
chk results/e3_swarm.json     "swarm_lat_ms"         "E3 canonical results"
chk paper/mission_dt_sbesc2026.tex "Swarm-Reaction"  "paper with E3 section"
chk paper/mission_dt_sbesc2026.tex "ProfessorFilipo" "paper with repo footnote"
chk README.md                 "run_e3"               "README with E3 instructions"
echo "== $ok up-to-date, $bad outdated =="
