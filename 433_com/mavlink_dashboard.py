#!/usr/bin/env python3
"""
Live web dashboard for Pixhawk telemetry (MAVLink over a SiK radio).

Runs a MAVLink reader thread plus a tiny web server (standard library only).
Open http://localhost:8000 in any browser (works from WSL -> Windows browser).

Needs pixhawk_receiver.py in the same folder (reuses its connect/request code).

    python pixhawk_dashboard.py --conn /dev/ttyUSB0 --baud 57600
"""

import argparse
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from pymavlink import mavutil
from pixhawk_receiver import BASE_MESSAGES, EXTRA_MESSAGES, connect, request_messages

HIST = 300  # samples kept for the sparklines
lock = threading.Lock()
state = {
    "status": "connecting", "mode": None, "armed": None,
    "roll": None, "pitch": None, "yaw": None,
    "lat": None, "lon": None, "alt_msl": None, "rel_alt": None, "hdg": None,
    "fix": None, "sats": None, "hdop": None,
    "airspeed": None, "groundspeed": None, "climb": None, "throttle": None,
    "volt": None, "amp": None, "batt": None,
    "rc": [], "servo": [], "radio": None, "loss": None,
    "texts": [], "last_rx": None, "hb": None,
}
hist = {"alt": deque(maxlen=HIST), "gs": deque(maxlen=HIST), "volt": deque(maxlen=HIST)}


def update(msg):
    t = msg.get_type()
    with lock:
        if t == "HEARTBEAT" and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
            state["mode"] = mavutil.mode_string_v10(msg)
            state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            state["hb"] = time.time()
        elif t == "ATTITUDE":
            state["roll"], state["pitch"], state["yaw"] = (
                math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw) % 360)
        elif t == "GLOBAL_POSITION_INT":
            state.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, alt_msl=msg.alt / 1000,
                         rel_alt=msg.relative_alt / 1000, hdg=msg.hdg / 100)
            hist["alt"].append(msg.relative_alt / 1000)
        elif t == "GPS_RAW_INT":
            state.update(fix=msg.fix_type, sats=msg.satellites_visible, hdop=msg.eph / 100)
        elif t == "VFR_HUD":
            state.update(airspeed=msg.airspeed, groundspeed=msg.groundspeed,
                         climb=msg.climb, throttle=msg.throttle)
            hist["gs"].append(msg.groundspeed)
        elif t == "SYS_STATUS":
            v = msg.voltage_battery / 1000 if msg.voltage_battery != 65535 else None
            state.update(volt=v,
                         amp=msg.current_battery / 100 if msg.current_battery != -1 else None,
                         batt=msg.battery_remaining if msg.battery_remaining != -1 else None)
            if v is not None:
                hist["volt"].append(v)
        elif t == "RC_CHANNELS":
            state["rc"] = [getattr(msg, f"chan{i}_raw") for i in range(1, 9)]
        elif t == "SERVO_OUTPUT_RAW":
            state["servo"] = [getattr(msg, f"servo{i}_raw") for i in range(1, 9)]
        elif t == "RADIO_STATUS":
            state["radio"] = dict(rssi=msg.rssi, remrssi=msg.remrssi, noise=msg.noise,
                                  remnoise=msg.remnoise, txbuf=msg.txbuf, rxerrors=msg.rxerrors)
        elif t == "STATUSTEXT":
            state["texts"].append(f"{time.strftime('%H:%M:%S')}  {msg.text}")
            del state["texts"][:-30]
        state["last_rx"] = time.time()


def reader(args):
    wanted = dict(BASE_MESSAGES)
    if args.extra:
        wanted.update(EXTRA_MESSAGES)
    master = connect(args.conn, args.baud, 255)
    request_messages(master, wanted)
    with lock:
        state["status"] = "connected"
    last_hb = last_req = time.time()
    got = False
    while True:
        now = time.time()
        if now - last_hb >= 1:
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = now
        if now - last_req >= (60 if got else 5):
            request_messages(master, wanted)
            last_req = now
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None or msg.get_type() == "BAD_DATA":
            continue
        if msg.get_type() in wanted:
            got = True
        update(msg)
        with lock:
            state["loss"] = master.packet_loss()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/data":
            with lock:
                out = dict(state)
                out["hist"] = {k: list(v) for k, v in hist.items()}
                out["age"] = None if state["last_rx"] is None else time.time() - state["last_rx"]
                out["hb_age"] = None if state["hb"] is None else time.time() - state["hb"]
            body, ctype = json.dumps(out).encode(), "application/json"
        else:
            body, ctype = PAGE.encode(), "text/html; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Pixhawk Monitor</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#12151a;--card:#1c2128;--fg:#e6edf3;--mut:#8b949e;--ok:#3fb950;--bad:#f85149;--warn:#d29922;--acc:#58a6ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
header{display:flex;gap:16px;align-items:center;padding:12px 18px;background:var(--card);border-bottom:1px solid #30363d}
h1{font-size:16px;margin:0;flex:1}.pill{padding:3px 10px;border-radius:12px;font-weight:600;background:#30363d}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px;padding:14px}
.card{background:var(--card);border-radius:8px;padding:12px 14px}.card h2{margin:0 0 8px;font-size:12px;color:var(--mut);text-transform:uppercase;letter-spacing:.06em}
.row{display:flex;justify-content:space-between;padding:2px 0}.row span:first-child{color:var(--mut)}
.big{font-size:26px;font-weight:600}canvas{width:100%;display:block}
.bar{height:10px;background:#30363d;border-radius:5px;overflow:hidden;margin:3px 0}.bar i{display:block;height:100%;background:var(--acc)}
#log{font:12px ui-monospace,monospace;max-height:150px;overflow:auto;white-space:pre-wrap;color:var(--mut)}
</style></head><body>
<header><h1>Pixhawk Monitor</h1><span class="pill" id="conn">connecting</span>
<span class="pill" id="mode">–</span><span class="pill" id="armed">–</span></header>
<div class="grid">
 <div class="card"><h2>Attitude</h2><canvas id="hor" width="240" height="240"></canvas>
  <div class="row"><span>Roll / Pitch</span><span id="rp">–</span></div><div class="row"><span>Yaw</span><span id="yaw">–</span></div></div>
 <div class="card"><h2>Flight</h2>
  <div class="row"><span>Rel. altitude</span><span class="big" id="alt">–</span></div>
  <div class="row"><span>Ground speed</span><span id="gs">–</span></div><div class="row"><span>Airspeed</span><span id="as">–</span></div>
  <div class="row"><span>Climb</span><span id="climb">–</span></div><div class="row"><span>Throttle</span><span id="thr">–</span></div>
  <canvas id="sAlt" width="300" height="60"></canvas></div>
 <div class="card"><h2>Battery</h2>
  <div class="row"><span>Voltage</span><span class="big" id="volt">–</span></div>
  <div class="row"><span>Current</span><span id="amp">–</span></div><div class="row"><span>Remaining</span><span id="batt">–</span></div>
  <div class="bar"><i id="battbar" style="width:0"></i></div><canvas id="sVolt" width="300" height="60"></canvas></div>
 <div class="card"><h2>GPS</h2>
  <div class="row"><span>Fix / Sats</span><span id="fix">–</span></div><div class="row"><span>HDOP</span><span id="hdop">–</span></div>
  <div class="row"><span>Lat</span><span id="lat">–</span></div><div class="row"><span>Lon</span><span id="lon">–</span></div>
  <div class="row"><span>Alt MSL</span><span id="msl">–</span></div><div class="row"><span>Heading</span><span id="hdg">–</span></div></div>
 <div class="card"><h2>Radio link</h2>
  <div class="row"><span>RSSI local / remote</span><span id="rssi">–</span></div><div class="row"><span>Noise local / remote</span><span id="noise">–</span></div>
  <div class="row"><span>TX buffer</span><span id="txbuf">–</span></div><div class="row"><span>RX errors</span><span id="rxe">–</span></div>
  <div class="row"><span>Packet loss</span><span id="loss">–</span></div><div class="row"><span>Last message</span><span id="age">–</span></div></div>
 <div class="card"><h2>RC inputs</h2><div id="rc"></div></div>
 <div class="card" style="grid-column:1/-1"><h2>Status messages</h2><div id="log"></div></div>
</div>
<script>
const $=id=>document.getElementById(id);
const f=(v,d=1,u='')=>(v===null||v===undefined)?'–':Number(v).toFixed(d)+u;
function horizon(roll,pitch){
  const c=$('hor'),x=c.getContext('2d'),w=c.width,h=c.height,r=w/2-4;
  x.clearRect(0,0,w,h);x.save();x.translate(w/2,h/2);
  x.beginPath();x.arc(0,0,r,0,7);x.clip();
  x.rotate(-(roll||0)*Math.PI/180);
  const py=(pitch||0)*3;
  x.fillStyle='#3b7dd8';x.fillRect(-w,-h*2+py,w*2,h*2);
  x.fillStyle='#8a5a2b';x.fillRect(-w,py,w*2,h*2);
  x.strokeStyle='#fff';x.lineWidth=2;x.beginPath();x.moveTo(-w,py);x.lineTo(w,py);x.stroke();
  x.lineWidth=1;for(let p=-30;p<=30;p+=10){if(!p)continue;const y=py-p*3,l=p%20?15:28;x.beginPath();x.moveTo(-l,y);x.lineTo(l,y);x.stroke()}
  x.restore();
  x.strokeStyle='#ffcc00';x.lineWidth=3;x.beginPath();
  x.moveTo(w/2-45,h/2);x.lineTo(w/2-12,h/2);x.moveTo(w/2+12,h/2);x.lineTo(w/2+45,h/2);x.stroke();
  x.beginPath();x.arc(w/2,h/2,r,0,7);x.strokeStyle='#30363d';x.lineWidth=2;x.stroke();
}
function spark(id,a,color){
  const c=$(id),x=c.getContext('2d'),w=c.width,h=c.height;x.clearRect(0,0,w,h);
  if(!a||a.length<2)return;let lo=Math.min(...a),hi=Math.max(...a);if(hi-lo<.5){hi+=.25;lo-=.25}
  x.strokeStyle=color;x.lineWidth=1.5;x.beginPath();
  a.forEach((v,i)=>{const px=i/(a.length-1)*w,py=h-4-(v-lo)/(hi-lo)*(h-8);i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke();
}
function render(s){
  const live=s.age!==null&&s.age<3;
  const cn=$('conn');cn.textContent=live?'LINK OK':(s.status==='connecting'?'connecting…':'NO DATA');
  cn.style.background=live?'var(--ok)':'var(--bad)';
  $('mode').textContent=s.mode||'–';
  const a=$('armed');a.textContent=s.armed===null?'–':(s.armed?'ARMED':'DISARMED');
  a.style.background=s.armed?'var(--bad)':'#30363d';
  horizon(s.roll,s.pitch);
  $('rp').textContent=f(s.roll,1,'°')+' / '+f(s.pitch,1,'°');$('yaw').textContent=f(s.yaw,0,'°');
  $('alt').textContent=f(s.rel_alt,1,' m');$('gs').textContent=f(s.groundspeed,1,' m/s');
  $('as').textContent=f(s.airspeed,1,' m/s');$('climb').textContent=f(s.climb,2,' m/s');$('thr').textContent=f(s.throttle,0,'%');
  $('volt').textContent=f(s.volt,2,' V');$('amp').textContent=f(s.amp,1,' A');$('batt').textContent=f(s.batt,0,'%');
  $('battbar').style.width=(s.batt||0)+'%';$('battbar').style.background=(s.batt!==null&&s.batt<25)?'var(--bad)':'var(--ok)';
  $('fix').textContent=(s.fix??'–')+' / '+(s.sats??'–');$('hdop').textContent=f(s.hdop,2);
  $('lat').textContent=f(s.lat,7);$('lon').textContent=f(s.lon,7);$('msl').textContent=f(s.alt_msl,1,' m');$('hdg').textContent=f(s.hdg,0,'°');
  const r=s.radio;
  $('rssi').textContent=r?r.rssi+' / '+r.remrssi:'–';$('noise').textContent=r?r.noise+' / '+r.remnoise:'–';
  $('txbuf').textContent=r?r.txbuf+'%':'–';$('rxe').textContent=r?r.rxerrors:'–';
  $('loss').textContent=f(s.loss,1,'%');$('age').textContent=s.age===null?'–':f(s.age,1,' s ago');
  $('rc').innerHTML=(s.rc||[]).map((v,i)=>`<div class="row"><span>CH${i+1}</span><span>${v}</span></div><div class="bar"><i style="width:${Math.max(0,Math.min(100,(v-1000)/10))}%"></i></div>`).join('')||'<span style="color:var(--mut)">no data</span>';
  $('log').textContent=(s.texts||[]).join('\n');
  spark('sAlt',s.hist.alt,'#58a6ff');spark('sVolt',s.hist.volt,'#3fb950');
}
async function tick(){try{render(await (await fetch('/data')).json())}catch(e){$('conn').textContent='server unreachable';$('conn').style.background='var(--bad)'}}
setInterval(tick,250);tick();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser(description="Pixhawk web dashboard")
    ap.add_argument("--conn", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--extra", action="store_true")
    args = ap.parse_args()

    threading.Thread(target=reader, args=(args,), daemon=True).start()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Dashboard: http://localhost:{args.port}  (Ctrl+C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()