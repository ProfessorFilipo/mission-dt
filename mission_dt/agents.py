"""
Virtual agents: the "second digital twin" that emulates a physical
drone (aerial) or vessel (surface). From the Mission-DT viewpoint a
virtual agent is indistinguishable from a physical one: both publish
I_k^t on missiondt/agents/<id>/telemetry and consume A_k^t from
missiondt/agents/<id>/actuation.

Sensors are sampled at their native rate (SENSOR_HZ); the bandwidth
regulator forwards only every k-th sample so that the network sees
PUBLISH_HZ, matching the DT frame rate (Fleet-DT regulator scheme).
"""
import json
import math
import random
import time
import threading

import paho.mqtt.client as mqtt

SENSOR_HZ = 50.0    # native IMU/estimator sampling rate
PUBLISH_HZ = 8.0    # regulated network rate (= 1/125 ms)
BASE_LAT, BASE_LON = -30.0577, -51.1729  # Porto Alegre test area


class VirtualAgent(threading.Thread):
    def __init__(self, agent_id, domain="surface", host="127.0.0.1",
                 regulator=True, duration_s=30.0, jitter=True, loss=0.0):
        super().__init__(daemon=True)
        self.aid, self.domain = agent_id, domain
        self.regulator = regulator
        self.loss = loss              # simulated network loss probability
        self.lost_msgs = 0
        self.truth_log = []           # (t, lat, lon, yaw) ground truth
        self.duration = duration_s
        self.jitter = jitter
        # physical state (ground truth of the emulated vehicle)
        self.lat = BASE_LAT + random.uniform(-1e-3, 1e-3)
        self.lon = BASE_LON + random.uniform(-1e-3, 1e-3)
        self.alt = 0.0 if domain == "surface" else 10.0
        self.yaw = random.uniform(-math.pi, math.pi)
        self.speed = 0.0
        self.vb = 18.5
        self.tau, self.alpha, self.climb = 0.0, 0.0, 0.0
        self.seq = 0
        self.bytes_out = 0
        self.msgs_out = 0
        self.act_latencies = []   # DT actuation publish -> agent apply (s)
        self.swarm_latencies = []  # neighbor telemetry pub -> corrective actuation here (s)

        self.cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                               client_id=agent_id, protocol=mqtt.MQTTv5)
        self.cli.on_message = self._on_act
        self.cli.connect(host, 1883)
        self.cli.subscribe(f"missiondt/agents/{agent_id}/actuation", qos=0)
        self.cli.publish(f"missiondt/agents/{agent_id}/register",
                         json.dumps({"domain": domain, "kind": "virtual"}),
                         qos=1, retain=True)
        self.cli.loop_start()

    # actuation A_k^t from the Mission-DT (two-way channel: DT -> twin)
    def _on_act(self, cli, ud, msg):
        a = json.loads(msg.payload)
        now = time.time()
        self.act_latencies.append(now - a["t_pub"])
        if a.get("avoid") and a.get("trig_t"):
            self.swarm_latencies.append(now - a["trig_t"])
        self.tau, self.alpha = a["tau"], a["alpha"]
        self.climb = a.get("climb", 0.0)

    # simple kinematic model (differential-thrust vessel / quadplane)
    def _step(self, dt):
        vmax = 2.0 if self.domain == "surface" else 12.0
        self.speed += (self.tau * vmax - self.speed) * min(1.0, 1.5 * dt)
        self.yaw += self.alpha * (0.6 if self.domain == "surface" else 1.5) * dt
        dist = self.speed * dt
        self.lat += dist * math.cos(self.yaw) / 111_320.0
        self.lon += dist * math.sin(self.yaw) / (111_320.0 * math.cos(math.radians(self.lat)))
        if self.domain == "aerial":
            self.alt = max(0.0, self.alt + self.climb * 2.0 * dt)
        self.vb -= 0.00005 * (1 + self.tau) * dt * 60

    def _telemetry(self):
        n = (lambda s: random.gauss(0, s)) if self.jitter else (lambda s: 0)
        return {
            "t_pub": time.time(), "seq": self.seq,
            "gps": [self.lat + n(1e-6), self.lon + n(1e-6), self.alt + n(0.1)],
            "att": [n(0.01), n(0.01), self.yaw + n(0.005)],
            "vel": [self.speed + n(0.02), n(0.02), self.climb if self.domain == "aerial" else n(0.01)],
            "imu": [n(0.05), n(0.05), 9.81 + n(0.05), n(0.01), n(0.01), n(0.01)],
            "mag": [22.0 + n(0.2), -8.0 + n(0.2), -12.0 + n(0.2)],
            "baro": [101_325 + n(5), 24.0 + n(0.05)],
            "vb": self.vb, "ib": 1.2 + 6.0 * self.tau + n(0.05),
        }

    def run(self):
        period = 1.0 / SENSOR_HZ
        decim = int(SENSOR_HZ / PUBLISH_HZ)
        tick, t_next = 0, time.monotonic()
        t_end = t_next + self.duration
        while time.monotonic() < t_end:
            self._step(period)
            self.truth_log.append((time.time(), self.lat, self.lon, self.yaw))
            if not self.regulator or tick % decim == 0:
                self.seq += 1
                if self.loss > 0.0 and random.random() < self.loss:
                    self.lost_msgs += 1          # dropped by the "network"
                else:
                    p = json.dumps(self._telemetry())
                    self.cli.publish(f"missiondt/agents/{self.aid}/telemetry",
                                     p, qos=0)
                    self.bytes_out += len(p)
                    self.msgs_out += 1
            tick += 1
            t_next += period
            s = t_next - time.monotonic()
            if s > 0:
                time.sleep(s)
            else:
                t_next = time.monotonic()
        # deregister: clear our retained registration so no ghost remains
        self.cli.publish(f"missiondt/agents/{self.aid}/register",
                         payload=b"", qos=1, retain=True)
        time.sleep(0.1)
        self.cli.loop_stop()
        self.cli.disconnect()
