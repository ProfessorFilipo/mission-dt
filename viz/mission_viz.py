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
  P or [FOTO]  : screenshot -> captures/
  R or [REC]   : start/stop video -> captures/*.mp4 (YouTube/Vimeo ready)
  H or [HQ]    : toggle high (30 fps) / low (15 fps) quality
  I or [ID]    : toggle drone name labels
  K or [ROTA]  : toggle planned-route lines (green; gray dots = past trail)
  L or [LEG]   : toggle translucent legend
  O or [PANEL] : toggle floating agent panel (reuses panel.py's Watch)
  G : sky grid   |   ESC : quit
"""
import argparse
import datetime
import glob
import json
import math
import os
import threading
import time as pytime

import importlib.util
from pathlib import Path

import paho.mqtt.client as mqtt
from panda3d.core import ClockObject
from ursina import (Ursina, Entity, EditorCamera, Text, Button, Mesh,
                    color, window, held_keys, application, destroy)

# reuse the Watch class from the terminal panel (panel.py stays untouched)
_pp = Path(__file__).resolve().parent.parent / "experiments" / "panel.py"
_spec = importlib.util.spec_from_file_location("mdt_panel", _pp)
_panel_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_panel_mod)
Watch, PANEL_HEAD = _panel_mod.Watch, _panel_mod.HEAD

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
    watch = Watch()

    def on_telemetry(c, u, m):
        aid = m.topic.split("/")[2]
        p = json.loads(m.payload)
        store.update(aid, p)
        watch.tel(aid, p)

    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="viz")
    cli.on_message = on_telemetry
    cli.connect(args.host, 1883)
    cli.subscribe("missiondt/agents/+/telemetry")
    def on_register(c, u, m):
        if not m.payload:
            return                     # deregistration (cleared retained)
        try:
            watch.reg(m.topic.split("/")[2], json.loads(m.payload))
        except json.JSONDecodeError:
            pass

    cli.message_callback_add("missiondt/agents/+/register", on_register)
    cli.subscribe("missiondt/agents/+/register", qos=1)
    routes_msg, routes_dirty = {}, [False]

    def on_routes(c, u, m):
        try:
            data = json.loads(m.payload or b"{}")
        except json.JSONDecodeError:
            return
        routes_msg.clear()
        routes_msg.update(data)
        routes_dirty[0] = True

    cli.message_callback_add("missiondt/mission/routes", on_routes)
    cli.subscribe("missiondt/mission/routes", qos=1)
    cps = {}

    def on_checkpoints(c, u, m):
        try:
            data = json.loads(m.payload or b"{}")
        except json.JSONDecodeError:
            return
        cps.clear()
        cps.update(data)

    cli.message_callback_add("missiondt/mission/checkpoints", on_checkpoints)
    cli.subscribe("missiondt/mission/checkpoints", qos=1)
    cli.loop_start()

    app = Ursina(title="Mission-DT")
    window.borderless = False          # normal window (fixes macOS cropping)
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
    hud = Text(text="agents: 0", position=(-0.45, 0.46), scale=0.8)
    rec = Recorder()
    show = {"ids": False, "route": False, "leg": False, "panel": False}

    LEGEND = (
        "LEGENDA\n"
        "laranja (4 bracos) = drone aereo\n"
        "azul (casco) = embarcacao\n"
        "linha vertical = altitude do aereo\n"
        "pontos cinza = trilha percorrida\n"
        "linha verde = rota planejada\n"
        "esfera branca = aproximacao (<24 m)\n"
        "esfera laranja = conflito (<12 m)\n"
        "esfera vermelha = quase-colisao (<3 m)\n"
        "pino ciano/laranja/roxo =\n"
        "  checkpoint superficie/aereo/submerso\n"
        "vermelho pulsante = bateria baixa\n"
        "drone cinza = sem comunicacao")
    from ursina import camera as _cam
    leg_bg = Entity(parent=_cam.ui, model="quad", scale=(0.36, 0.34),
                    position=(0.52, 0.18), color=color.rgba(0, 0, 0, 0.45),
                    enabled=False)
    leg_txt = Text(parent=_cam.ui, text=LEGEND, position=(0.36, 0.34),
                   scale=0.62, color=color.white, enabled=False)

    pan_bg = Entity(parent=_cam.ui, model="quad", scale=(0.9, 0.36),
                    position=(0.0, 0.24), color=color.rgba(0, 0, 0, 0.55),
                    enabled=False)
    pan_txt = Text(parent=_cam.ui, text="", position=(-0.43, 0.40),
                   scale=0.55, color=color.white, enabled=False)

    route_ents = []

    def rebuild_routes():
        for e in route_ents:
            destroy(e)
        route_ents.clear()
        for aid, wps in routes_msg.items():
            pts = []
            for rlat, rlon, ralt in wps:
                rx = (rlon - BASE_LON) * SCALE * math.cos(math.radians(rlat))
                rz = (rlat - BASE_LAT) * SCALE
                pts.append((rx, max(0.15, ralt / 5.0), rz))
            if len(pts) >= 2:
                pts.append(pts[0])            # close the loop
                route_ents.append(Entity(
                    model=Mesh(vertices=pts, mode="line", thickness=2),
                    color=color.rgba(0.35, 1.0, 0.45, 0.9),
                    enabled=show["route"]))

    def refresh_panel():
        w = (7, 8, 8, 10, 11, 6, 5, 6, 5, 6, 7)
        lines = ["  ".join(h.ljust(x) for h, x in zip(PANEL_HEAD, w))]
        for r in watch.rows():
            lines.append("  ".join(str(v).ljust(x) for v, x in zip(r, w)))
        pan_txt.text = "\n".join(lines[:14])
    cp_ents = {}

    def draw_checkpoints():
        for name, (clat, clon, calt) in dict(cps).items():
            if name in cp_ents or name == "clear":
                continue
            x = (clon - BASE_LON) * SCALE * math.cos(math.radians(clat))
            z = (clat - BASE_LAT) * SCALE
            y = calt / 5.0
            if calt > 1.0:
                col, label = color.orange, name              # air corridor
            elif calt < -0.5:
                col, label = color.violet, f"{name} ({calt:.0f}m)"  # submerged
                y = 0.05
            else:
                col, label = color.cyan, name                # surface
            pin = Entity(model="cube", scale=(0.06, max(0.05, y), 0.06),
                         position=(x, max(0.05, y) / 2, z),
                         color=color.rgba(1, 1, 1, 0.35))
            mark = Entity(model="sphere", scale=0.5, color=col,
                          position=(x, max(0.25, y), z))
            txt = Text(parent=mark, text=label, scale=18, y=1.2,
                       color=color.white, billboard=True)
            cp_ents[name] = (pin, mark, txt)

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

    BTN_C = color.rgb(0.2, 0.3, 0.45)
    BTN_ON = color.rgb(0.15, 0.5, 0.35)

    def toggle_ids():
        show["ids"] = not show["ids"]
        b_ids.color = BTN_ON if show["ids"] else BTN_C
        for a in ents.values():
            a["lbl"].enabled = show["ids"]

    def toggle_route():
        show["route"] = not show["route"]
        b_route.color = BTN_ON if show["route"] else BTN_C
        for e in route_ents:
            e.enabled = show["route"]

    def toggle_leg():
        show["leg"] = not show["leg"]
        b_leg.color = BTN_ON if show["leg"] else BTN_C
        leg_bg.enabled = leg_txt.enabled = show["leg"]

    def toggle_panel():
        show["panel"] = not show["panel"]
        b_pan.color = BTN_ON if show["panel"] else BTN_C
        pan_bg.enabled = pan_txt.enabled = show["panel"]

    xs = [-0.36, -0.24, -0.12, 0.0, 0.12, 0.24, 0.36]
    b_shot = Button(text="FOTO", scale=(0.10, 0.045), position=(xs[0], -0.43),
                    color=BTN_C, on_click=take_shot)
    b_rec = Button(text="REC", scale=(0.10, 0.045), position=(xs[1], -0.43),
                   color=BTN_C, on_click=toggle_rec)
    b_q = Button(text="HQ", scale=(0.10, 0.045), position=(xs[2], -0.43),
                 color=BTN_C, on_click=toggle_q)
    b_ids = Button(text="ID", scale=(0.10, 0.045), position=(xs[3], -0.43),
                   color=BTN_C, on_click=toggle_ids)
    b_route = Button(text="ROTA", scale=(0.10, 0.045), position=(xs[4], -0.43),
                     color=BTN_C, on_click=toggle_route)
    b_leg = Button(text="LEG", scale=(0.10, 0.045), position=(xs[5], -0.43),
                   color=BTN_C, on_click=toggle_leg)
    b_pan = Button(text="PANEL", scale=(0.10, 0.045), position=(xs[6], -0.43),
                   color=BTN_C, on_click=toggle_panel)

    ents = {}      # aid -> dict(root, parts, alt_line, batt, domain)
    trails, tick = {}, [0]
    GRAY = color.rgb(0.45, 0.45, 0.45)

    def update():
        now = pytime.time()
        draw_checkpoints()
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
                lbl_anchor = Entity(enabled=show["ids"])
                Text(parent=lbl_anchor, text=aid, scale=12,
                     billboard=True, color=color.white)
                safe = Entity(model="sphere", enabled=False,
                              scale=2 * SAFE_M / 5.0,
                              color=color.rgba(1, 1, 1, 0.10))
                ents[aid] = dict(root=root, parts=parts, alt=alt_line,
                                 batt=batt, safe=safe, dom=dom,
                                 lbl=lbl_anchor)
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
        for aid2, a2 in ents.items():
            if aid2 in snap:
                x2, y2 = snap[aid2][0], snap[aid2][1]
                a2["lbl"].position = (x2, max(0.3, y2) + 2.0, snap[aid2][2])
        if routes_dirty[0]:
            routes_dirty[0] = False
            rebuild_routes()
        if show["panel"] and tick[0] % 30 == 0:
            refresh_panel()
        tick[0] += 1
        fps = ClockObject.getGlobalClock().getAverageFrameRate()
        hud.text = f"agents: {len(ents)}   FPS: {fps:.0f}" + \
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
        elif key == "i":
            toggle_ids()
        elif key == "k":
            toggle_route()
        elif key == "l":
            toggle_leg()
        elif key == "o":
            toggle_panel()
        elif key == "escape":
            if rec.on:
                rec.stop()
            application.quit()

    Entity(update=update, input=on_input)
    app.run()


if __name__ == "__main__":
    main()
