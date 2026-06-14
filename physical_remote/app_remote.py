"""
实体摇杆遥控器 WebUI

- 轮询遥控器 ESP32 的 /status，显示摇杆位置与小车连接状态
- 可代理读取小车 /status、发送停车
- 与主项目 app.py 独立，默认端口 8002

用法：
  python app_remote.py

环境变量：
  REMOTE_ESP32_URL   遥控器 ESP32，例如 http://192.168.152.176
  ROBOT_ESP32_URL    小车 ESP32，例如 http://192.168.152.11
  PORT               本服务端口，默认 8002
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlparse
from urllib.request import Request, urlopen
import json
import os
import time

HOST = "0.0.0.0"
PORT = int(os.environ.get("PORT", "8002"))
REMOTE_ESP32_URL = os.environ.get("REMOTE_ESP32_URL", "http://192.168.152.176").rstrip("/")
ROBOT_ESP32_URL = os.environ.get("ROBOT_ESP32_URL", "http://192.168.152.11").rstrip("/")
POLL_MS = 50
ROBOT_POLL_ENABLED = os.environ.get("ROBOT_POLL_ENABLED", "0") == "1"

HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>实体摇杆遥控器</title>
  <style>
    :root{color-scheme:dark;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
    body{margin:0;background:#0b1220;color:#e5e7eb}
    .wrap{max-width:1040px;margin:0 auto;padding:20px}
    h1{margin:0 0 8px;font-size:1.6rem}
    p{color:#94a3b8;line-height:1.5}
    .grid{display:grid;grid-template-columns:1.2fr .8fr;gap:16px}
    @media(max-width:860px){.grid{grid-template-columns:1fr}}
    .card{background:#111827;border:1px solid #334155;border-radius:14px;padding:16px}
    .card h2{margin:0 0 12px;font-size:1.05rem}
    .twin-wrap{display:flex;gap:20px;align-items:flex-start;flex-wrap:wrap;justify-content:center}
    .joystick{position:relative;width:260px;height:260px;border-radius:50%;
      background:radial-gradient(circle at 50% 50%,#334155 0 10%,#111827 11% 100%);
      border:2px solid #475569;transition:border-color .15s,box-shadow .15s}
    .joystick.active{border-color:#38bdf8;box-shadow:0 0 24px rgba(56,189,248,.25)}
    .joystick:before,.joystick:after{content:"";position:absolute;background:#475569;z-index:1}
    .joystick:before{left:50%;top:14px;width:2px;height:232px;transform:translateX(-50%)}
    .joystick:after{top:50%;left:14px;height:2px;width:232px;transform:translateY(-50%)}
    .deadzone{position:absolute;left:50%;top:50%;width:36px;height:36px;margin:-18px 0 0 -18px;border-radius:50%;
      border:1px dashed #64748b;opacity:.7;z-index:2}
    .vector{position:absolute;left:50%;top:50%;height:3px;transform-origin:left center;background:linear-gradient(90deg,#22d3ee,#38bdf8);
      border-radius:2px;opacity:0;z-index:3;pointer-events:none}
    .stick{position:absolute;left:50%;top:50%;width:78px;height:78px;margin:-39px 0 0 -39px;
      border-radius:50%;background:linear-gradient(145deg,#38bdf8,#0ea5e9);box-shadow:0 8px 20px rgba(14,165,233,.35);
      z-index:4;will-change:transform}
    .stick.active{box-shadow:0 0 28px rgba(56,189,248,.65)}
    .legend{display:grid;grid-template-columns:1fr 1fr;gap:8px;min-width:220px;flex:1}
    .val{background:#0f172a;border-radius:10px;padding:10px}
    .val span{display:block;color:#94a3b8;font-size:.78rem}
    .val strong{font-size:1.15rem;font-variant-numeric:tabular-nums}
    .val.highlight strong{color:#38bdf8}
    .badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:.75rem;margin-left:8px;vertical-align:middle}
    .badge.idle{background:#334155;color:#94a3b8}
    .badge.live{background:#14532d;color:#4ade80}
    .badge.stop{background:#7f1d1d;color:#fca5a5}
    .ok{color:#4ade80}.bad{color:#f87171}.warn{color:#fbbf24}
    button{background:#1d4ed8;color:#fff;border:0;border-radius:10px;padding:10px 14px;cursor:pointer}
    button.secondary{background:#334155}
    button.danger{background:#b91c1c}
    .row{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
    pre{background:#0f172a;border-radius:10px;padding:12px;overflow:auto;font-size:.82rem;max-height:240px}
    code{background:#1e293b;padding:2px 6px;border-radius:6px}
    .fps{color:#64748b;font-size:.82rem;margin-top:8px}
  </style>
</head>
<body>
  <div class="wrap">
    <h1>实体摇杆数字孪生</h1>
    <p>左侧虚拟摇杆实时镜像 <code>__REMOTE_ESP32_URL__</code> 的物理位置；当前为摇杆测试模式，不请求小车。轮询间隔 <code>__POLL_MS__ms</code>。</p>

    <div class="grid">
      <div class="card">
        <h2>物理孪生 <span id="twinBadge" class="badge idle">待机</span></h2>
        <div class="twin-wrap">
          <div id="joystick" class="joystick">
            <div class="deadzone"></div>
            <div id="vector" class="vector"></div>
            <div id="stick" class="stick"></div>
          </div>
          <div class="legend">
            <div class="val highlight"><span>物理 X (normX)</span><strong id="normX">0.0</strong></div>
            <div class="val highlight"><span>物理 Y (normY)</span><strong id="normY">0.0</strong></div>
            <div class="val"><span>向量幅度</span><strong id="vectorMag">0</strong></div>
            <div class="val"><span>ADC 原始值</span><strong id="rawXY">- , -</strong></div>
            <div class="val"><span>校准中心</span><strong id="centerXY">- , -</strong></div>
            <div class="val"><span>死区</span><strong id="deadZone">8</strong></div>
          </div>
        </div>
        <div class="legend" style="margin-top:12px">
          <div class="val"><span>驱动 Throttle</span><strong id="throttle">0</strong></div>
          <div class="val"><span>驱动 Steering</span><strong id="steering">0</strong></div>
          <div class="val"><span>原始 Throttle</span><strong id="throttleRaw">0</strong></div>
          <div class="val"><span>原始 Steering</span><strong id="steeringRaw">0</strong></div>
        </div>
        <div class="fps" id="fps">等待首帧...</div>
        <div class="row">
          <button class="danger" id="stopRemote">遥控器停车</button>
          <button class="secondary" id="recalibrate">重新校准中心</button>
        </div>
        <div class="card" style="margin-top:12px;background:#0f172a">
          <h2>满量程 remap 校准</h2>
          <p style="margin-top:0">依次把摇杆推到对应位置并点击记录。校准会保存到 ESP32，之后速度输出会按四个端点重新映射到完整 -100~100。</p>
          <div class="row">
            <button class="secondary" data-cal-edge="center">记录中心</button>
            <button class="secondary" data-cal-edge="left">左推到底</button>
            <button class="secondary" data-cal-edge="right">右推到底</button>
            <button class="secondary" data-cal-edge="up">上推到底</button>
            <button class="secondary" data-cal-edge="down">下推到底</button>
            <button class="danger" id="resetCal">恢复默认端点</button>
          </div>
          <div class="fps" id="calInfo">默认端点：X 146 / 2746 / 4095 · Y 292 / 2814 / 4095</div>
        </div>
      </div>

      <div class="card">
        <h2>连接状态</h2>
        <div class="legend">
          <div class="val"><span>遥控器 IP</span><strong id="remoteIp">-</strong></div>
          <div class="val"><span>小车转发</span><strong id="robotForward" class="warn">关闭</strong></div>
          <div class="val"><span>按键 SW</span><strong id="buttonPressed">-</strong></div>
          <div class="val"><span>最近 HTTP</span><strong id="lastHttp">-</strong></div>
          <div class="val"><span>方向旋转</span><strong id="rotationDeg">0°</strong></div>
          <div class="val"><span>控制模式</span><strong id="assistMode">相对</strong></div>
        </div>
        <div class="row">
          <button id="refresh">立即刷新</button>
          <button class="secondary" id="toggleForward">开启小车转发</button>
          <button class="danger" id="stopRobot">小车停车</button>
        </div>
        <div class="card" style="margin-top:12px;background:#0f172a">
          <h2>方向对齐</h2>
          <p style="margin-top:0">如果实体手柄方向和小车方向不一致，在这里旋转控制向量。数字孪生和发给小车的速度都会同步旋转。</p>
          <div class="row">
            <button class="secondary" data-rotate="0">0°</button>
            <button class="secondary" data-rotate="90">90°</button>
            <button class="secondary" data-rotate="180">180°</button>
            <button class="secondary" data-rotate="270">270°</button>
          </div>
        </div>
        <div class="card" style="margin-top:12px;background:#0f172a">
          <h2>控制模式</h2>
          <p style="margin-top:0">短按实体摇杆 SW 会把当前 IMU 方向设为绝对前方；按住 1 秒以上重新校准中心。</p>
          <div class="row">
            <button class="secondary" data-mode="1">相对方向矫正</button>
            <button class="secondary" data-mode="2">绝对方向模式</button>
            <button class="secondary" id="toggleMode">切换模式</button>
          </div>
        </div>
        <p id="message" class="warn" style="margin-top:12px">正在连接遥控器...</p>
        <h2 style="margin-top:18px">小车状态</h2>
        <pre id="robotStatus">测试阶段不轮询小车。需要启用时设置 ROBOT_POLL_ENABLED=1，并打开遥控器 /forward?enabled=1。</pre>
      </div>
    </div>
  </div>

<script>
const POLL_MS=__POLL_MS__;
const ROBOT_POLL_ENABLED=__ROBOT_POLL_ENABLED__;
const STICK_MAX=88;
let lastFrameAt=0, frameCount=0, fpsTimer=performance.now();

function setText(id,text){const el=document.querySelector("#"+id);if(el)el.textContent=text}
function setClass(id,cls){const el=document.querySelector("#"+id);if(el)el.className=cls}

async function remoteApi(path,opt={}){
  const r=await fetch("/api/remote"+path,{cache:"no-store",...opt});
  const p=await r.json();
  if(!r.ok)throw new Error(p.error||"遥控器请求失败");
  return p;
}

async function robotApi(path,opt={}){
  const r=await fetch("/api/robot"+path,{cache:"no-store",...opt});
  const p=await r.json();
  if(!r.ok)throw new Error(p.error||"小车请求失败");
  return p;
}

function fmt(v,d=1){return Number(v||0).toFixed(d)}

function renderRemote(s){
  const nx=Number(s.normX||0), ny=Number(s.normY||0);
  const t=Number(s.throttle||0), st=Number(s.steering||0);
  const mag=Number(s.vectorMag||0);
  const active=!!s.driveActive;
  const pressed=!!s.buttonPressed;

  setText("normX",fmt(nx));
  setText("normY",fmt(ny));
  setText("vectorMag",fmt(mag,0));
  setText("rawXY",`${s.rawX??"-"} , ${s.rawY??"-"}`);
  setText("centerXY",`${s.centerX??"-"} , ${s.centerY??"-"}`);
  setText("calInfo",`X ${s.calXMin??"-"} / ${s.centerX??"-"} / ${s.calXMax??"-"} · Y ${s.calYMin??"-"} / ${s.centerY??"-"} / ${s.calYMax??"-"}`);
  setText("deadZone",s.deadZone??8);
  setText("throttle",t);
  setText("steering",st);
  setText("throttleRaw",s.throttleRaw??0);
  setText("steeringRaw",s.steeringRaw??0);
  setText("remoteIp",s.ip||"-");
  setText("buttonPressed",pressed?"按下：松开后设当前绝对前方；按住1秒校准中心":"松开");
  setText("lastHttp",s.lastHttpCode??"-");
  setClass("robotForward",s.robotForwardEnabled?"ok":"warn");
  setText("robotForward",s.robotForwardEnabled?"开启":"关闭");
  setText("rotationDeg",`${s.controlRotationDeg??0}°`);
  setText("assistMode",Number(s.remoteAssistMode||1)===2?"绝对":"相对");
  const forwardBtn=document.querySelector("#toggleForward");
  if(forwardBtn){
    forwardBtn.textContent=s.robotForwardEnabled?"关闭小车转发":"开启小车转发";
    forwardBtn.className=s.robotForwardEnabled?"danger":"secondary";
    forwardBtn.dataset.enabled=s.robotForwardEnabled?"1":"0";
  }

  const joy=document.querySelector("#joystick");
  const stick=document.querySelector("#stick");
  const vector=document.querySelector("#vector");
  const badge=document.querySelector("#twinBadge");
  const dx=nx/100*STICK_MAX, dy=-ny/100*STICK_MAX;
  if(stick){
    stick.style.transform=`translate(${dx}px,${dy}px)`;
    stick.classList.toggle("active",active||mag>3);
  }
  if(joy) joy.classList.toggle("active",active);
  if(vector){
    const len=Math.hypot(dx,dy);
    if(len>2){
      const ang=Math.atan2(dy,dx)*180/Math.PI;
      vector.style.width=`${len}px`;
      vector.style.transform=`rotate(${ang}deg)`;
      vector.style.opacity=String(Math.min(1,0.35+len/STICK_MAX));
    }else{
      vector.style.opacity="0";
    }
  }
  if(badge){
    if(pressed){badge.textContent="设前方";badge.className="badge stop"}
    else if(active){badge.textContent="操控中";badge.className="badge live"}
    else{badge.textContent="待机";badge.className="badge idle"}
  }

  const msg=document.querySelector("#message");
  if(msg){
    msg.textContent=s.robotForwardEnabled?(s.lastRobotError?`小车通信：${s.lastRobotError}`:"遥控器在线，小车转发已开启"):"摇杆测试模式：只显示孪生，不发送小车";
    msg.className=s.robotForwardEnabled?(s.robotReachable?"ok":"warn"):"ok";
  }

  frameCount++;
  const now=performance.now();
  if(now-fpsTimer>=1000){
    setText("fps",`刷新 ${frameCount} fps · 延迟约 ${Math.max(0,Math.round(now-lastFrameAt))} ms`);
    frameCount=0; fpsTimer=now;
  }
  lastFrameAt=now;
}

let remotePolling=false, robotPolling=false;

async function refreshRemote(){
  if(remotePolling)return;
  remotePolling=true;
  try{
    const remote=await remoteApi("/status");
    renderRemote(remote);
  }finally{
    remotePolling=false;
  }
}

async function refreshRobot(){
  if(!ROBOT_POLL_ENABLED){
    document.querySelector("#robotStatus").textContent="测试阶段不轮询小车。摇杆调好后再启用小车转发。";
    return;
  }
  if(robotPolling)return;
  robotPolling=true;
  try{
    const robot=await robotApi("/status");
    document.querySelector("#robotStatus").textContent=JSON.stringify(robot,null,2);
  }catch(e){
    document.querySelector("#robotStatus").textContent="读取小车失败: "+e.message;
  }finally{
    robotPolling=false;
  }
}

async function refreshAll(){
  try{await refreshRemote()}catch(e){
    const msg=document.querySelector("#message");
    if(msg){msg.textContent=e.message;msg.className="bad"}
  }
  if(ROBOT_POLL_ENABLED) refreshRobot();
}

document.querySelector("#refresh").onclick=()=>refreshAll();
document.querySelector("#stopRemote").onclick=()=>remoteApi("/stop",{method:"POST"}).then(refreshAll).catch(e=>alert(e.message));
document.querySelector("#recalibrate").onclick=()=>remoteApi("/recalibrate",{method:"POST"}).then(refreshAll).catch(e=>alert(e.message));
document.querySelector("#stopRobot").onclick=()=>robotApi("/stop",{method:"POST"}).then(refreshAll).catch(e=>alert(e.message));
document.querySelector("#toggleForward").onclick=()=>{
  const enabled=document.querySelector("#toggleForward").dataset.enabled==="1";
  remoteApi(`/forward?enabled=${enabled?0:1}`,{method:"POST"}).then(refreshAll).catch(e=>alert(e.message));
};
document.querySelector("#toggleMode").onclick=()=>remoteApi("/mode?toggle=1",{method:"POST"}).then(refreshAll).catch(e=>alert(e.message));
document.querySelector("#resetCal").onclick=()=>remoteApi("/calibration-reset",{method:"POST"}).then(refreshAll).catch(e=>alert(e.message));
document.querySelectorAll("[data-cal-edge]").forEach(btn=>{
  btn.onclick=()=>remoteApi(`/calibrate-edge?edge=${btn.dataset.calEdge}`,{method:"POST"}).then(refreshAll).catch(e=>alert(e.message));
});
document.querySelectorAll("[data-rotate]").forEach(btn=>{
  btn.onclick=()=>remoteApi(`/orientation?rotate=${btn.dataset.rotate}`,{method:"POST"}).then(refreshAll).catch(e=>alert(e.message));
});
document.querySelectorAll("[data-mode]").forEach(btn=>{
  btn.onclick=()=>remoteApi(`/mode?assistMode=${btn.dataset.mode}`,{method:"POST"}).then(refreshAll).catch(e=>alert(e.message));
});

refreshAll();
setInterval(refreshRemote,POLL_MS);
if(ROBOT_POLL_ENABLED)setInterval(refreshRobot,500);
</script>
</body>
</html>"""


def json_response(handler: BaseHTTPRequestHandler, code: int, payload: dict) -> None:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def request_device(base_url: str, path: str, method: str = "GET", timeout: float = 0.6) -> dict:
    request = Request(f"{base_url}{path}", data=(b"" if method == "POST" else None), method=method)
    with urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            body = (
                HTML.replace("__REMOTE_ESP32_URL__", REMOTE_ESP32_URL)
                .replace("__ROBOT_ESP32_URL__", ROBOT_ESP32_URL)
                .replace("__POLL_MS__", str(POLL_MS))
                .replace("__ROBOT_POLL_ENABLED__", "true" if ROBOT_POLL_ENABLED else "false")
                .encode("utf-8")
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path.startswith("/api/remote"):
            sub = parsed.path[len("/api/remote") :]
            if parsed.query:
                sub = f"{sub}?{parsed.query}"
            try:
                json_response(self, 200, request_device(REMOTE_ESP32_URL, sub or "/status", method="GET"))
            except HTTPError as error:
                detail = error.read().decode("utf-8", errors="replace")
                json_response(self, 502, {"error": f"遥控器 HTTP {error.code}", "detail": detail})
            except URLError as error:
                json_response(self, 502, {"error": f"连接不到遥控器：{error.reason}", "remote": REMOTE_ESP32_URL})
            except Exception as error:
                json_response(self, 502, {"error": str(error), "remote": REMOTE_ESP32_URL})
            return

        if parsed.path.startswith("/api/robot"):
            sub = parsed.path[len("/api/robot") :]
            if parsed.query:
                sub = f"{sub}?{parsed.query}"
            try:
                json_response(self, 200, request_device(ROBOT_ESP32_URL, sub or "/status", method="GET"))
            except HTTPError as error:
                detail = error.read().decode("utf-8", errors="replace")
                json_response(self, 502, {"error": f"小车 HTTP {error.code}", "detail": detail})
            except URLError as error:
                json_response(self, 502, {"error": f"连接不到小车：{error.reason}", "robot": ROBOT_ESP32_URL})
            except Exception as error:
                json_response(self, 502, {"error": str(error), "robot": ROBOT_ESP32_URL})
            return

        self.send_error(404)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path.startswith("/api/remote"):
            sub = parsed.path[len("/api/remote") :]
            if parsed.query:
                sub = f"{sub}?{parsed.query}"
            try:
                json_response(self, 200, request_device(REMOTE_ESP32_URL, sub or "/status", method="POST"))
            except Exception as error:
                json_response(self, 502, {"error": str(error), "remote": REMOTE_ESP32_URL})
            return

        if parsed.path.startswith("/api/robot"):
            sub = parsed.path[len("/api/robot") :]
            if parsed.query:
                sub = f"{sub}?{parsed.query}"
            try:
                json_response(self, 200, request_device(ROBOT_ESP32_URL, sub or "/status", method="POST"))
            except Exception as error:
                json_response(self, 502, {"error": str(error), "robot": ROBOT_ESP32_URL})
            return

        self.send_error(404)


def main() -> None:
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"实体遥控器 WebUI: http://127.0.0.1:{PORT}", flush=True)
    print(f"遥控器 ESP32: {REMOTE_ESP32_URL}", flush=True)
    print(f"小车 ESP32:   {ROBOT_ESP32_URL}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("退出", flush=True)


if __name__ == "__main__":
    main()
