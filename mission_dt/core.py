"""
Mission-DT: Mission-level Digital Twin core.

Extends the fleet-level DT model (Fleet-DT) to the mission level:
  M^t = < A^t, Delta^e, g^t, phi^t >
where A^t is the set of hybrid agents (physical or virtual, aerial or
surface), Delta^e the set of extended transition functions, g^t the
mission goals and phi^t the mission context.

Each agent k has:
  I_k^t  : input model (sensor readings, received via MQTT)
  B_k^t  : state model (pose, attitude, velocities)
  A_k^t  : actuation model (throttle, steering), computed by lambda
The DT runs at a fixed frame period T_f (default 125 ms, as in Fleet-DT).
"""
import json
import math
import time
import threading
from collections import deque, defaultdict
from dataclasses import dataclass, field

import paho.mqtt.client as mqtt

FRAME_MS = 125.0  # DT frame period (ms)


# ----------------------------------------------------------------------
# State models
# ----------------------------------------------------------------------
@dataclass
class AgentState:
    """B_k^t : lat, lon, alt, attitude, body velocities."""
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    u: float = 0.0   # surge (m/s)
    v: float = 0.0   # sway  (m/s)
    w: float = 0.0   # heave (m/s)
    battery_v: float = 18.5
    t: float = 0.0   # timestamp of last update


@dataclass
class AgentRecord:
    agent_id: str
    domain: str                 # "aerial" | "surface"
    kind: str                   # "physical" | "virtual"
    state: AgentState = field(default_factory=AgentState)
    history: deque = field(default_factory=lambda: deque(maxlen=8))  # [B^i;B^j]
    last_seq: int = -1
    goal: tuple = (0.0, 0.0, 0.0)   # g_k^t : target lat, lon, alt
    stale: bool = True


# ----------------------------------------------------------------------
# Mission Digital Twin
# ----------------------------------------------------------------------
class MissionDT:
    """
    Subscribes to  missiondt/agents/+/telemetry
    Publishes  to  missiondt/agents/<id>/actuation
    Runs delta^e once per frame for every registered agent and
    lambda (goal-seeking controller) to produce actuation A_k^t.
    Collects per-frame and per-message metrics.
    """

    def __init__(self, host="127.0.0.1", frame_ms=FRAME_MS, viz_hook=None,
                 swarm=False, sep_m=12.0):
        self.frame_s = frame_ms / 1000.0
        self.swarm = swarm          # enable inter-agent separation (phi context)
        self.sep_m = sep_m          # separation threshold (meters)
        self.avoid_events = 0       # frames in which avoidance overrode lambda
        self.agents: dict[str, AgentRecord] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.viz_hook = viz_hook  # callable(agents_dict) -> None (3D frontend)

        # metrics
        self.msg_latencies = []          # publish -> DT ingestion (s)
        self.frame_compute = []          # delta+lambda compute time per frame (s)
        self.frame_overruns = 0          # frames whose work exceeded T_f
        self.frames = 0
        self.stale_updates = 0           # frames where an agent had no fresh telemetry
        self.dup_updates = 0             # >1 telemetry msg consumed in one frame
        self.bytes_in = 0
        self._pending = defaultdict(list)  # agent_id -> [telemetry dicts]

        self.cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                               client_id="mission-dt", protocol=mqtt.MQTTv5)
        self.cli.on_message = self._on_msg
        self.cli.connect(host, 1883)
        self.cli.subscribe("missiondt/agents/+/telemetry", qos=0)
        self.cli.subscribe("missiondt/agents/+/register", qos=1)
        self.cli.loop_start()

    # ------------------------------------------------------------------
    def _on_msg(self, cli, ud, msg):
        now = time.time()
        try:
            payload = json.loads(msg.payload)
        except json.JSONDecodeError:
            return
        self.bytes_in += len(msg.payload)
        parts = msg.topic.split("/")
        aid, kind_topic = parts[2], parts[3]
        if kind_topic == "register":
            with self._lock:
                self.agents[aid] = AgentRecord(
                    agent_id=aid, domain=payload["domain"], kind=payload["kind"])
            return
        # telemetry: I_k^t
        self.msg_latencies.append(now - payload["t_pub"])
        with self._lock:
            self._pending[aid].append(payload)

    # ------------------------------------------------------------------
    # delta^e : state transition using latest input + state history
    # ------------------------------------------------------------------
    def _delta(self, rec: AgentRecord, telems: list) -> None:
        if not telems:
            rec.stale = True
            self.stale_updates += 1
            # dead-reckon from history (proactive operation, MPC-like);
            # only after the first real telemetry has seeded the state
            if rec.last_seq >= 0 and len(rec.history) >= 2:
                b1, b0 = rec.history[-1], rec.history[-2]
                dt = self.frame_s
                rec.state.lat += (b1.lat - b0.lat)
                rec.state.lon += (b1.lon - b0.lon)
                rec.state.alt += (b1.alt - b0.alt) if rec.domain == "aerial" else 0
        else:
            if len(telems) > 1:
                self.dup_updates += len(telems) - 1
            m = max(telems, key=lambda p: p["seq"])  # newest by seq (ordering guard)
            if m["seq"] <= rec.last_seq:
                return  # outdated packet: drop (real-time rule)
            rec.last_seq = m["seq"]
            s = rec.state
            s.lat, s.lon, s.alt = m["gps"]
            s.roll, s.pitch, s.yaw = m["att"]
            s.u, s.v, s.w = m["vel"]
            s.battery_v = m["vb"]
            s.t = m["t_pub"]
            rec.stale = False
        if rec.last_seq >= 0:
            rec.history.append(AgentState(**vars(rec.state)))

    # ------------------------------------------------------------------
    # lambda : goal-seeking decision function -> A_k^t
    # ------------------------------------------------------------------
    def _lambda(self, rec: AgentRecord) -> dict:
        s, g = rec.state, rec.goal
        dy = (g[0] - s.lat) * 111_320.0
        dx = (g[1] - s.lon) * 111_320.0 * math.cos(math.radians(s.lat))
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dx, dy)
        err = (bearing - s.yaw + math.pi) % (2 * math.pi) - math.pi
        throttle = max(0.0, min(1.0, dist / 20.0)) if dist > 1.5 else 0.0
        steer = max(-1.0, min(1.0, 1.2 * err))
        act = {"tau": throttle, "alpha": steer, "t_pub": time.time()}
        if rec.domain == "aerial":
            act["climb"] = max(-1.0, min(1.0, (g[2] - s.alt) / 5.0))
        return act


    # ------------------------------------------------------------------
    # phi + swarm rule : mission context (inter-agent distances) feeding
    # a separation behavior. Returns (neighbor, dist, away_bearing) when
    # the nearest same-domain neighbor is within sep_m, else None.
    # ------------------------------------------------------------------
    def _separation(self, rec: AgentRecord, agents: list):
        best, bd, bdx, bdy = None, 1e12, 0.0, 0.0
        for other in agents:
            if other is rec or other.domain != rec.domain:
                continue
            dy = (other.state.lat - rec.state.lat) * 111_320.0
            dx = (other.state.lon - rec.state.lon) * 111_320.0 * \
                math.cos(math.radians(rec.state.lat))
            d = math.hypot(dx, dy)
            if d < bd:
                best, bd, bdx, bdy = other, d, dx, dy
        if best is not None and bd < self.sep_m:
            away = math.atan2(-bdx, -bdy)   # bearing pointing away from neighbor
            return best, bd, away
        return None

    # ------------------------------------------------------------------
    def run(self, duration_s: float, goals: dict | None = None):
        """Main DT loop: one Delta^e evaluation per frame for the mission."""
        t_next = time.monotonic()
        t_end = t_next + duration_s
        while not self._stop.is_set() and time.monotonic() < t_end:
            t0 = time.monotonic()
            with self._lock:
                pending, self._pending = self._pending, defaultdict(list)
                agents = list(self.agents.values())
            for rec in agents:
                if goals and rec.agent_id in goals:
                    rec.goal = goals[rec.agent_id]
                self._delta(rec, pending.get(rec.agent_id, []))
            for rec in agents:
                act = self._lambda(rec)
                if self.swarm:
                    hit = self._separation(rec, agents)
                    if hit is not None:
                        nb, dist, away = hit
                        err = (away - rec.state.yaw + math.pi) % (2 * math.pi) - math.pi
                        act["alpha"] = max(-1.0, min(1.0, 1.5 * err))
                        act["tau"] = min(act["tau"], 0.5)
                        act["avoid"] = True
                        act["trig_t"] = nb.state.t     # neighbor telemetry timestamp
                        act["trig_id"] = nb.agent_id
                        self.avoid_events += 1
                self.cli.publish(f"missiondt/agents/{rec.agent_id}/actuation",
                                 json.dumps(act), qos=0)
            if self.viz_hook:
                self.viz_hook(agents)
            work = time.monotonic() - t0
            self.frame_compute.append(work)
            self.frames += 1
            if work > self.frame_s:
                self.frame_overruns += 1
            t_next += self.frame_s
            sleep = t_next - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                t_next = time.monotonic()  # resync after overrun
        self.cli.loop_stop()
        self.cli.disconnect()

    def stop(self):
        self._stop.set()
