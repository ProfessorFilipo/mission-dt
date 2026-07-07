"""
Mission-DT 3D visualizer v2 (run LOCALLY on a machine with GPU/display).

    pip install ursina paho-mqtt imageio imageio-ffmpeg
    python viz/mission_viz.py --host <broker-ip>

Renders every hybrid agent from missiondt/agents/+/telemetry:
  aerial  = orange quadcopter (white nose = front) + altitude line
  surface = blue vessel hull (white bow marker = front)
Status: no telemetry for >1.5 s -> agent turns gray; battery <17.6 V ->
pulsing red marker above the vehicle.

Controls: right-drag orbit / scroll zoom / middle-drag pan
  P or [FOTO] : screenshot -> captures/
  R or [REC]  : start/stop video -> captures/*.mp4 (YouTube/Vimeo ready)
  H or [HQ]   : toggle high (30 fps) / low (15 fps) quality
  G           : toggle sky grid    |  ESC: quit
"""
import argparse
import datetime
import glob
import json
import math
import os
import threading
import time as pytime

import paho.mqtt.client as mqtt
from ursina import (Ursina, Entity, EditorCamera, Text, Button, color,
                    window, held_keys, application, destroy)

BASE_LAT, BASE_LON = -30.0577, -51.1729
SCALE = 111_320.0 / 5.0      # 1 scene unit = 5 m
TRAIL_LEN = 60
STALE_S = 1.5
LOW_BATT_V = 17.6
SAFE_M = 12.0    # separation threshold (matches mission core sep_m)
COLL_M = 3.0     # near-collision distance
SHOW_M = 24.0    # sphere becomes visible below this neighbor distance
CAPT = "captures"


# ------------------------------------------------------------------ models
def build_aerial():
    root = Entity()
    parts = []
    body = Entity(parent=root, model="sphere", scale=(0.55, 0.28, 0.75),
                  color=color.orange)
    parts.append((body, color.orange))
    for dx, dz in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
        arm = Entity(parent=root, model="cube", position=(dx * 0.55, 0, dz * 0.55),
                     scale=(0.12, 0.07, 0.9), rotation_y=45 * dx * dz,
                     color=color.rgb(0.75, 0.35, 0.05))
        rotor = Entity(parent=root, model="sphere",
                       position=(dx * 0.85, 0.1, dz * 0.85),
                       scale=(0.45, 0.07, 0.45), color=color.rgb(0.35, 0.35, 0.4))
        parts += [(arm, color.rgb(0.75, 0.35, 0.05)),
                  (rotor, color.rgb(0.35, 0.35, 0.4))]
    nose = Entity(parent=root, model="cube", position=(0, 0, 0.75),
                  scale=(0.12, 0.12, 0.4), color=color.white)
    parts.append((nose, color.white))
    return root, parts


def build_surface():
    root = Entity()
    parts = []
    hull = Entity(parent=root, model="cube", scale=(0.9, 0.35, 2.2),
                  color=color.azure)
    bow = Entity(parent=root, model="cube", position=(0, 0, 1.25),
                 scale=(0.64, 0.34, 0.64), rotation_y=45, color=color.azure)
    cabin = Entity(parent=root, model="cube", position=(0, 0.3, -0.45),
                   scale=(0.55, 0.3, 0.7), color=color.rgb(0.55, 0.75, 0.95))
    tip = Entity(parent=root, model="cube", position=(0, 0.12, 1.62),
                 scale=(0.14, 0.14, 0.3), color=color.white)
    for e, c in [(hull, color.azure), (bow, color.azure),
                 (cabin, color.rgb(0.55, 0.75, 0.95)), (tip, color.white)]:
        parts.append((e, c))
    return root, parts


# ------------------------------------------------------------------ MQTT
class Store:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = {}   # aid -> (x, y, z, yaw, domain, vb, t_seen)

    def update(self, aid, p):
        lat, lon, alt = p["gps"]
        x = (lon - BASE_LON) * SCALE * math.cos(math.radians(lat))
        z = (lat - BASE_LAT) * SCALE
        dom = "aerial" if alt > 1.0 else "surface"
        with self.lock:
            self.state[aid] = (x, alt / 5.0, z, p["att"][2], dom,
                               p.get("vb", 18.5), pytime.time())


# ------------------------------------------------------------------ video
class Recorder:
    def __init__(self):
        self.on, self.hq, self.prefix, self.t0 = False, True, None, 0

    def start(self):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        d = os.path.join(CAPT, f"rec_{ts}")
        os.makedirs(d, exist_ok=True)
        self.prefix = os.path.join(d, "f").replace("\\", "/")
        fps = 30 if self.hq else 15
        application.base.movie(namePrefix=self.prefix, duration=3600,
                               fps=fps, format="jpg", sd=6)
        self.on, self.t0 = True, pytime.time()
        print(f"[REC] recording at {fps} fps -> {d}")

    def stop(self):
        self.on = False
        try:
            application.base.taskMgr.remove(self.prefix + "_task")
        except Exception:
            pass
        d = os.path.dirname(self.prefix)
        fps = 30 if self.hq else 15
        out = d + (".mp4")
        frames = sorted(glob.glob(self.prefix + "_*.jpg"))
        if not frames:
            print("[REC] no frames captured"); return
        print(f"[REC] encoding {len(frames)} frames -> {out} (window may "
              "freeze for a moment)")
        try:
            import imageio.v2 as imageio
            w = imageio.get_writer(out, fps=fps, codec="libx264",
                                   quality=8 if self.hq else 4)
            for f in frames:
                w.append_data(imageio.imread(f))
            w.close()
            for f in frames:
                os.remove(f)
            os.rmdir(d)
            print(f"[REC] done: {out} (H.264 MP4, YouTube/Vimeo ready)")
        except ImportError:
            print("[REC] imageio not installed; frames kept in", d)
            print("      pip install imageio imageio-ffmpeg   or encode with:")
            print(f"      ffmpeg -framerate {fps} -i {self.prefix}_%06d.jpg "
                  f"-c:v libx264 -pix_fmt yuv420p {out}")


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args()
    os.makedirs(CAPT, exist_ok=True)

    store = Store()
    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="viz")
    cli.on_message = lambda c, u, m: store.update(
        m.topic.split("/")[2], json.loads(m.payload))
    cli.connect(args.host, 1883)
    cli.subscribe("missiondt/agents/+/telemetry")
    cli.loop_start()

    app = Ursina(title="Mission-DT")
    window.color = color.rgb(10 / 255, 20 / 255, 40 / 255)
    window.fps_counter.enabled = True
    Entity(model="plane", scale=600,
           color=color.rgb(20 / 255, 60 / 255, 90 / 255),
           texture="white_cube", texture_scale=(120, 120))
    sky = Entity(model="plane", scale=600, y=12, rotation_x=180,
                 color=color.rgb(14 / 255, 35 / 255, 60 / 255),
                 texture="white_cube", texture_scale=(120, 120),
                 enabled=False)
    cam = EditorCamera()
    cam.rotation_x = 40
    cam.target_z = -55
    hud = Text(text="agents: 0", position=(-0.85, 0.47), scale=0.8)
    rec = Recorder()

    def take_shot():
        application.base.screenshot(namePrefix=os.path.join(CAPT, "shot"))
        print("[FOTO] saved in", CAPT)

    def toggle_rec():
        rec.stop() if rec.on else rec.start()
        b_rec.text = "STOP" if rec.on else "REC"
        b_rec.color = color.red if rec.on else color.rgb(0.2, 0.3, 0.45)

    def toggle_q():
        if rec.on:
            print("[REC] stop recording before changing quality"); return
        rec.hq = not rec.hq
        b_q.text = "HQ" if rec.hq else "LQ"

    b_shot = Button(text="FOTO", scale=(0.09, 0.045), position=(-0.78, -0.44),
                    color=color.rgb(0.2, 0.3, 0.45), on_click=take_shot)
    b_rec = Button(text="REC", scale=(0.09, 0.045), position=(-0.67, -0.44),
                   color=color.rgb(0.2, 0.3, 0.45), on_click=toggle_rec)
    b_q = Button(text="HQ", scale=(0.09, 0.045), position=(-0.56, -0.44),
                 color=color.rgb(0.2, 0.3, 0.45), on_click=toggle_q)

    ents = {}      # aid -> dict(root, parts, alt_line, batt, domain)
    trails, tick = {}, [0]
    GRAY = color.rgb(0.45, 0.45, 0.45)

    def update():
        now = pytime.time()
        with store.lock:
            snap = dict(store.state)
        for aid, (x, y, z, yaw, dom, vb, t_seen) in snap.items():
            if aid not in ents:
                root, parts = build_aerial() if dom == "aerial" \
                    else build_surface()
                alt_line = Entity(model="cube",
                                  color=color.rgba(1, 1, 1, 0.25),
                                  scale=(0.05, 1, 0.05)) \
                    if dom == "aerial" else None
                batt = Entity(model="sphere", color=color.red,
                              scale=0.25, enabled=False)
                safe = Entity(model="sphere", enabled=False,
                              scale=2 * SAFE_M / 5.0,
                              color=color.rgba(1, 1, 1, 0.10))
                ents[aid] = dict(root=root, parts=parts, alt=alt_line,
                                 batt=batt, safe=safe, dom=dom)
                trails[aid] = []
            a = ents[aid]
            a["root"].position = (x, max(0.3, y), z)
            a["root"].rotation_y = math.degrees(yaw)
            if a["alt"]:
                a["alt"].position = (x, y / 2, z)
                a["alt"].scale_y = max(0.01, y)
            stale = (now - t_seen) > STALE_S
            for e, c in a["parts"]:
                e.color = GRAY if stale else c
            low = (not stale) and vb < LOW_BATT_V
            a["batt"].enabled = low
            if low:
                a["batt"].position = (x, max(0.3, y) + 1.5, z)
                a["batt"].scale = 0.22 + 0.1 * math.sin(tick[0] * 0.35)
            if tick[0] % 4 == 0 and not stale:
                trails[aid].append(Entity(model="sphere", scale=0.12,
                                          color=color.light_gray,
                                          position=a["root"].position))
                if len(trails[aid]) > TRAIL_LEN:
                    destroy(trails[aid].pop(0))
        # safety spheres: nearest same-domain neighbor distance (meters, 3D)
        for aid, (x, y, z, yaw, dom, vb, t_seen) in snap.items():
            a = ents.get(aid)
            if not a:
                continue
            dmin = 1e9
            for oid, (ox, oy, oz, _, odom, _, _) in snap.items():
                if oid == aid or odom != dom:
                    continue
                dmin = min(dmin, 5.0 * math.sqrt((ox - x) ** 2 +
                                                 (oy - y) ** 2 +
                                                 (oz - z) ** 2))
            s = a["safe"]
            if dmin > SHOW_M:
                s.enabled = False
            else:
                s.enabled = True
                s.position = a["root"].position
                if dmin < COLL_M:
                    s.color = color.rgba(1.0, 0.15, 0.15, 0.40)
                elif dmin < SAFE_M:
                    s.color = color.rgba(1.0, 0.55, 0.0, 0.28)
                else:
                    s.color = color.rgba(1.0, 1.0, 1.0, 0.10)
        tick[0] += 1
        hud.text = f"agents: {len(ents)}" + \
            (f"   REC {int(now - rec.t0)}s" if rec.on else "")

    def on_input(key):
        if key == "p":
            take_shot()
        elif key == "r":
            toggle_rec()
        elif key == "h":
            toggle_q()
        elif key == "g":
            sky.enabled = not sky.enabled
        elif key == "escape":
            if rec.on:
                rec.stop()
            application.quit()

    Entity(update=update, input=on_input)
    app.run()


if __name__ == "__main__":
    main()
