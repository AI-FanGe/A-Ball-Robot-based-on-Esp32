from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import base64
from dataclasses import dataclass, field
import hashlib
import json
import os
import socket
import struct
import threading
import time
from typing import List, Optional, Tuple
from urllib.parse import urlparse

import cv2
import numpy as np
import torch
from ultralytics import YOLO, YOLOE


HOST = os.environ.get("HOST", "0.0.0.0")
PORT = int(os.environ.get("PORT", "8001"))
PUBLIC_IP = os.environ.get("PUBLIC_IP", "192.168.152.216")

CAMERA_WS_HOST = os.environ.get("CAMERA_WS_HOST", "0.0.0.0")
CAMERA_WS_PORT = int(os.environ.get("CAMERA_WS_PORT", "8081"))
CAMERA_WS_PATH = "/ws/camera"
CAMERA_ALLOWED_IP = os.environ.get("CAMERA_ALLOWED_IP", "192.168.152.71").strip()

YOLO_MODEL = os.environ.get("YOLO_MODEL", "yoloe-11l-seg.pt")
IS_YOLOE = "yoloe" in YOLO_MODEL.lower()
DEFAULT_TARGET_CLASSES = os.environ.get("TARGET_CLASSES", "football" if IS_YOLOE else "")
YOLO_CONF = float(os.environ.get("YOLO_CONF", "0.35"))
YOLO_IMGSZ = int(os.environ.get("YOLO_IMGSZ", "640"))
YOLO_DEVICE = os.environ.get("YOLO_DEVICE", "0" if torch.cuda.is_available() else "cpu")
DETECT_EVERY_N_FRAMES = max(1, int(os.environ.get("DETECT_EVERY_N_FRAMES", "1")))
DETECTION_OVERLAY_TTL = float(os.environ.get("DETECTION_OVERLAY_TTL", "1.0"))
MASK_ALPHA = float(os.environ.get("MASK_ALPHA", "0.45"))


HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ESP32 Camera YOLOE</title>
  <style>
    body{margin:0;padding:24px;background:#10141c;color:#edf4ff;font-family:"Microsoft YaHei",system-ui,sans-serif}
    main{max-width:1180px;margin:auto;padding:24px;border:1px solid #334155;border-radius:22px;background:#192231}
    h1{margin:0 0 8px;font-size:34px} p{color:#9fb0c4;line-height:1.7}
    .grid{display:grid;grid-template-columns:minmax(0,1fr)330px;gap:18px;align-items:start}
    .video-wrap{position:relative;width:100%;line-height:0}
    #cameraLive{display:block;width:100%;max-height:650px;object-fit:contain;border:1px solid #334155;border-radius:16px;background:#05070a}
    #maskOverlay{position:absolute;inset:0;width:100%;height:100%;pointer-events:none;border-radius:16px}
    .card{margin-bottom:12px;padding:14px;border:1px solid #334155;border-radius:14px;background:#222d3d}
    .label{display:block;color:#9fb0c4;font-size:13px;margin-bottom:6px}.value{font-size:20px;font-weight:800}
    input{box-sizing:border-box;width:100%;padding:12px;border:1px solid #3a4a60;border-radius:10px;background:#101722;color:#edf4ff;font-size:15px}
    button{margin-top:10px;width:100%;border:0;border-radius:10px;padding:12px;background:#2f80ed;color:white;font-size:15px;font-weight:800;cursor:pointer}
    button:hover{background:#3b8cff}
    .ok{color:#6cff9d}.warn{color:#ffcf5a}.err{color:#ff7b7b}
    code{color:#9bdcff}.small{color:#9fb0c4;font-size:13px;line-height:1.6;word-break:break-all}
    @media(max-width:860px){.grid{grid-template-columns:1fr}}
  </style>
</head>
<body>
<main>
  <h1>ESP32 Camera YOLOE 实时识别</h1>
  <p>ESP32 摄像头通过 <code>ws://__PUBLIC_IP__:__CAMERA_WS_PORT____CAMERA_WS_PATH__</code> 上传 JPEG；本页面显示 YOLOE 标注后的实时画面。</p>
  <div class="grid">
    <div class="video-wrap">
      <img id="cameraLive" src="/api/detected.mjpg" alt="ESP32 live camera stream" />
      <canvas id="maskOverlay"></canvas>
    </div>
    <aside>
      <div class="card"><span class="label">摄像头连接</span><span id="connected" class="value warn">starting</span></div>
      <div class="card">
        <span class="label">检测类型</span>
        <input id="targetClasses" placeholder="默认 football，例如：football, goal, person" />
        <button id="saveClasses">应用检测类型</button>
        <div id="classesHelp" class="small">YOLOE 支持开放词汇提示词，多个类型用逗号分隔。</div>
      </div>
      <div class="card"><span class="label">状态</span><div id="status" class="small">-</div></div>
      <div class="card"><span class="label">接收帧 / 识别帧</span><span id="frames" class="value">-</span></div>
      <div class="card"><span class="label">最新识别耗时</span><span id="latency" class="value">-</span></div>
      <div class="card"><span class="label">最新目标</span><div id="objects" class="small">-</div></div>
      <div class="card"><span class="label">最近错误</span><div id="error" class="small">-</div></div>
    </aside>
  </div>
</main>
<script>
const MASK_ALPHA=__MASK_ALPHA__;
const MASK_COLORS=[`rgba(0,255,0,${MASK_ALPHA})`,`rgba(255,128,0,${MASK_ALPHA})`,`rgba(0,200,255,${MASK_ALPHA})`,`rgba(255,0,200,${MASK_ALPHA})`,`rgba(128,255,0,${MASK_ALPHA})`,`rgba(0,128,255,${MASK_ALPHA})`,`rgba(255,255,0,${MASK_ALPHA})`,`rgba(180,0,255,${MASK_ALPHA})`];
const LABEL_COLORS=["rgb(0,255,0)","rgb(255,128,0)","rgb(0,200,255)","rgb(255,0,200)","rgb(128,255,0)","rgb(0,128,255)","rgb(255,255,0)","rgb(180,0,255)"];
function drawMasks(data){
  const img=document.querySelector("#cameraLive");
  const canvas=document.querySelector("#maskOverlay");
  const rect=img.getBoundingClientRect();
  const dpr=window.devicePixelRatio||1;
  canvas.style.width=rect.width+"px";
  canvas.style.height=rect.height+"px";
  canvas.width=Math.max(1,Math.round(rect.width*dpr));
  canvas.height=Math.max(1,Math.round(rect.height*dpr));
  const ctx=canvas.getContext("2d");
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,rect.width,rect.height);
  if(!data.detection_boxes || !data.detection_source_size || (data.overlay_age_ms ?? 99999)>1000){return;}
  const sourceW=data.detection_source_size[0]||1;
  const sourceH=data.detection_source_size[1]||1;
  const scaleX=rect.width/sourceW;
  const scaleY=rect.height/sourceH;
  for(const box of data.detection_boxes){
    const polygon=box.polygon||[];
    if(polygon.length<3){continue;}
    const idx=(box.color_index||0)%MASK_COLORS.length;
    ctx.beginPath();
    polygon.forEach((point,i)=>{
      const x=point[0]*scaleX;
      const y=point[1]*scaleY;
      if(i===0){ctx.moveTo(x,y);}else{ctx.lineTo(x,y);}
    });
    ctx.closePath();
    ctx.fillStyle=MASK_COLORS[idx];
    ctx.fill();
    if(box.label){
      const first=polygon[0];
      ctx.font="16px Microsoft YaHei, sans-serif";
      ctx.fillStyle=LABEL_COLORS[idx];
      ctx.fillText(box.label, first[0]*scaleX, Math.max(18, first[1]*scaleY-6));
    }
  }
}
async function refresh(){
  try{
    const res=await fetch("/api/state",{cache:"no-store"});
    const data=await res.json();
    const c=document.querySelector("#connected");
    c.textContent=data.connected && (data.age_ms ?? 99999) < 3000 ? "connected" : "waiting";
    c.className="value "+(data.connected && (data.age_ms ?? 99999) < 3000 ? "ok" : "warn");
    document.querySelector("#status").textContent=data.status || "-";
    document.querySelector("#frames").textContent=`${data.frame_count ?? 0} / ${data.detect_count ?? 0}`;
    document.querySelector("#latency").textContent=data.inference_ms == null ? "-" : `${data.inference_ms} ms`;
    document.querySelector("#objects").textContent=(data.objects || []).join(", ") || "未识别到目标";
    document.querySelector("#error").textContent=data.last_error || "-";
    const input=document.querySelector("#targetClasses");
    if(document.activeElement!==input){input.value=data.target_classes || "";}
    drawMasks(data);
  }catch(e){
    document.querySelector("#connected").textContent=e.message;
    document.querySelector("#connected").className="value err";
  }
}
document.querySelector("#saveClasses").addEventListener("click", async ()=>{
  const target_classes=document.querySelector("#targetClasses").value.trim();
  const res=await fetch("/api/config",{
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:JSON.stringify({target_classes})
  });
  const data=await res.json();
  if(!res.ok){alert(data.error || "设置失败");}
  refresh();
});
refresh(); setInterval(refresh,400);
window.addEventListener("resize", refresh);
</script>
</body>
</html>
"""


@dataclass
class CameraYoloState:
    connected: bool = False
    status: str = "starting"
    last_error: str = ""
    frame_count: int = 0
    detect_count: int = 0
    byte_count: int = 0
    frame_size: int = 0
    last_frame_at: float = 0.0
    inference_ms: Optional[int] = None
    objects: List[str] = field(default_factory=list)
    latest_jpeg: Optional[bytes] = None
    detection_boxes: List[dict] = field(default_factory=list)
    detection_source_size: Optional[Tuple[int, int]] = None
    last_detection_at: float = 0.0
    target_classes: str = DEFAULT_TARGET_CLASSES
    target_class_ids: Optional[List[int]] = None

    def __post_init__(self) -> None:
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)

    def update(self, **kwargs) -> None:
        with self.condition:
            for key, value in kwargs.items():
                setattr(self, key, value)
            self.condition.notify_all()

    def update_frame(self, frame: bytes) -> None:
        with self.condition:
            self.frame_count += 1
            self.byte_count += len(frame)
            self.frame_size = len(frame)
            self.last_frame_at = time.time()
            self.latest_jpeg = frame
            self.status = "frame received"
            self.condition.notify_all()

    def update_detection(
        self,
        inference_ms: int,
        objects: List[str],
        boxes: List[dict],
        source_size: Tuple[int, int],
    ) -> None:
        with self.condition:
            self.detect_count += 1
            self.inference_ms = inference_ms
            self.objects = objects
            self.detection_boxes = boxes
            self.detection_source_size = source_size
            self.last_detection_at = time.time()
            self.status = "detecting"
            self.condition.notify_all()

    def update_detection_filter(self, target_classes: str, target_class_ids: Optional[List[int]]) -> None:
        with self.condition:
            self.target_classes = target_classes
            self.target_class_ids = target_class_ids
            self.objects = []
            self.detection_boxes = []
            self.detection_source_size = None
            self.status = "detection filter updated"
            self.condition.notify_all()

    def wait_raw_frame(self, last_seen: int, timeout: float = 5.0) -> Tuple[int, Optional[bytes]]:
        deadline = time.time() + timeout
        with self.condition:
            while self.frame_count == last_seen:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self.condition.wait(remaining)
            return self.frame_count, self.latest_jpeg

    def snapshot(self) -> dict:
        with self.lock:
            age_ms = None
            if self.last_frame_at:
                age_ms = int((time.time() - self.last_frame_at) * 1000)
            overlay_age_ms = None
            if self.last_detection_at:
                overlay_age_ms = int((time.time() - self.last_detection_at) * 1000)
            detection_boxes = []
            detection_source_size = None
            if overlay_age_ms is not None and overlay_age_ms <= int(DETECTION_OVERLAY_TTL * 1000):
                detection_boxes = [dict(box) for box in self.detection_boxes if len(box.get("polygon") or []) >= 3]
                detection_source_size = list(self.detection_source_size) if self.detection_source_size else None
            return {
                "connected": self.connected,
                "status": self.status,
                "last_error": self.last_error,
                "frame_count": self.frame_count,
                "detect_count": self.detect_count,
                "byte_count": self.byte_count,
                "frame_size": self.frame_size,
                "age_ms": age_ms,
                "inference_ms": self.inference_ms,
                "objects": list(self.objects),
                "overlay_age_ms": overlay_age_ms,
                "detection_boxes": detection_boxes,
                "detection_source_size": detection_source_size,
                "target_classes": self.target_classes,
                "allowed_camera_ip": CAMERA_ALLOWED_IP or "any",
                "model": YOLO_MODEL,
                "device": YOLO_DEVICE,
                "conf": YOLO_CONF,
                "imgsz": YOLO_IMGSZ,
            }


STATE = CameraYoloState()
MODEL = YOLOE(YOLO_MODEL) if IS_YOLOE else YOLO(YOLO_MODEL)
MODEL_CLASS_LOCK = threading.Lock()
MODEL_CLASS_TEXT: Optional[str] = None

CLASS_ALIASES = {
    "人": "person",
    "人物": "person",
    "球": "ball",
    "足球": "football",
    "车": "car",
    "汽车": "car",
    "巴士": "bus",
    "公交车": "bus",
    "卡车": "truck",
    "摩托": "motorcycle",
    "摩托车": "motorcycle",
    "自行车": "bicycle",
    "狗": "dog",
    "猫": "cat",
    "瓶子": "bottle",
    "椅子": "chair",
    "电视": "tv",
    "手机": "cell phone",
}

def parse_target_classes(text: str) -> Tuple[str, Optional[List[int]]]:
    text = text.strip()
    if IS_YOLOE and not text:
        text = DEFAULT_TARGET_CLASSES
    elif not text:
        return "", None

    raw_items = [
        item.strip()
        for item in text.replace("，", ",").replace("、", ",").replace("；", ",").replace(";", ",").split(",")
    ]
    normalized_prompts = []
    for item in raw_items:
        if not item:
            continue
        normalized_prompts.append(CLASS_ALIASES.get(item, item))

    if IS_YOLOE:
        return ", ".join(normalized_prompts), None

    names = MODEL.names
    name_items = names.items() if hasattr(names, "items") else enumerate(names)
    id_to_name = {int(class_id): str(name) for class_id, name in name_items}
    name_to_id = {name.lower(): class_id for class_id, name in id_to_name.items()}
    class_ids = []
    normalized_names = []
    unknown = []

    for item in normalized_prompts:
        mapped = item.lower()
        if mapped in name_to_id:
            class_id = name_to_id[mapped]
            if class_id not in class_ids:
                class_ids.append(class_id)
                normalized_names.append(id_to_name[class_id])
        else:
            unknown.append(item)

    if unknown:
        raise ValueError(f"未知检测类型：{', '.join(unknown)}。请使用 YOLO COCO 类名，例如 person, sports ball, car。")
    if not class_ids:
        return "", None
    return ", ".join(normalized_names), class_ids


def yoloe_prompts_from_text(text: str) -> List[str]:
    target_classes, _ = parse_target_classes(text)
    return [item.strip() for item in target_classes.split(",") if item.strip()]


def apply_yoloe_classes(text: str) -> None:
    global MODEL_CLASS_TEXT
    if not IS_YOLOE:
        return

    prompts = yoloe_prompts_from_text(text)
    prompt_text = ", ".join(prompts)
    with MODEL_CLASS_LOCK:
        if MODEL_CLASS_TEXT == prompt_text:
            return
        MODEL.set_classes(prompts)
        MODEL_CLASS_TEXT = prompt_text
        STATE.update(status=f"YOLOE prompts: {prompt_text}")


def recv_exact(conn: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            raise ConnectionError("socket closed")
        data.extend(chunk)
    return bytes(data)


def read_http_headers(conn: socket.socket) -> str:
    data = bytearray()
    while b"\r\n\r\n" not in data:
        chunk = conn.recv(1024)
        if not chunk:
            raise ConnectionError("socket closed before websocket handshake")
        data.extend(chunk)
        if len(data) > 8192:
            raise ValueError("websocket handshake too large")
    return data.decode("iso-8859-1", errors="replace")


def websocket_accept_key(client_key: str) -> str:
    magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
    digest = hashlib.sha1((client_key + magic).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def perform_websocket_handshake(conn: socket.socket) -> None:
    headers = read_http_headers(conn)
    lines = headers.split("\r\n")
    request = lines[0].split()
    if len(request) < 2:
        raise ValueError("bad websocket request")
    path = request[1].split("?", 1)[0]
    if path != CAMERA_WS_PATH:
        body = b"Not found"
        conn.sendall(
            b"HTTP/1.1 404 Not Found\r\n"
            b"Content-Type: text/plain\r\n"
            + f"Content-Length: {len(body)}\r\n\r\n".encode("ascii")
            + body
        )
        raise ValueError(f"unsupported websocket path: {path}")

    header_map = {}
    for line in lines[1:]:
        if ":" in line:
            key, value = line.split(":", 1)
            header_map[key.strip().lower()] = value.strip()
    client_key = header_map.get("sec-websocket-key")
    if not client_key:
        raise ValueError("missing Sec-WebSocket-Key")

    response = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {websocket_accept_key(client_key)}\r\n"
        "\r\n"
    )
    conn.sendall(response.encode("ascii"))


def read_websocket_frame(conn: socket.socket) -> Tuple[int, bytes]:
    first, second = recv_exact(conn, 2)
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", recv_exact(conn, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", recv_exact(conn, 8))[0]
    mask = recv_exact(conn, 4) if masked else b""
    payload = bytearray(recv_exact(conn, length))
    if masked:
        for index in range(length):
            payload[index] ^= mask[index % 4]
    return opcode, bytes(payload)


def send_websocket_frame(conn: socket.socket, opcode: int, payload: bytes = b"") -> None:
    header = bytearray([0x80 | (opcode & 0x0F)])
    length = len(payload)
    if length < 126:
        header.append(length)
    elif length <= 0xFFFF:
        header.append(126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(127)
        header.extend(struct.pack("!Q", length))
    conn.sendall(bytes(header) + payload)


def detect_frame(frame: bytes) -> None:
    image_array = np.frombuffer(frame, dtype=np.uint8)
    image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("failed to decode JPEG frame")

    with STATE.lock:
        target_class_ids = list(STATE.target_class_ids) if STATE.target_class_ids is not None else None
        target_classes = STATE.target_classes

    apply_yoloe_classes(target_classes)
    started = time.perf_counter()
    predict_kwargs = {
        "imgsz": YOLO_IMGSZ,
        "conf": YOLO_CONF,
        "device": YOLO_DEVICE,
        "verbose": False,
    }
    if not IS_YOLOE:
        predict_kwargs["classes"] = target_class_ids
    results = MODEL.predict(image, **predict_kwargs)
    inference_ms = int((time.perf_counter() - started) * 1000)

    result = results[0]
    names = result.names
    name_items = names.items() if hasattr(names, "items") else enumerate(names)
    id_to_name = {int(class_id): str(name) for class_id, name in name_items}
    objects = []
    boxes = []
    if result.boxes is not None:
        xyxy_list = result.boxes.xyxy.tolist()
        cls_list = result.boxes.cls.tolist()
        conf_list = result.boxes.conf.tolist()
        mask_polygons = []
        if result.masks is not None and result.masks.xy is not None:
            mask_polygons = result.masks.xy
        for index, (xyxy, cls_id, conf) in enumerate(zip(xyxy_list, cls_list, conf_list)):
            class_name = id_to_name.get(int(cls_id), str(int(cls_id)))
            label = f"{class_name} {conf:.2f}"
            objects.append(label)
            polygon = []
            if index < len(mask_polygons):
                polygon = mask_polygons[index].tolist()
            boxes.append(
                {
                    "xyxy": [float(value) for value in xyxy],
                    "label": label,
                    "conf": float(conf),
                    "polygon": [[float(x), float(y)] for x, y in polygon],
                    "color_index": index,
                }
            )
    height, width = image.shape[:2]
    STATE.update_detection(inference_ms, objects[:20], boxes, (width, height))


def handle_camera_ws_client(conn: socket.socket, address) -> None:
    conn.settimeout(10.0)
    client_ip = address[0]
    if CAMERA_ALLOWED_IP and client_ip != CAMERA_ALLOWED_IP:
        try:
            conn.close()
        except Exception:
            pass
        STATE.update(last_error=f"ignored camera ip {client_ip}, only allow {CAMERA_ALLOWED_IP}")
        return

    try:
        perform_websocket_handshake(conn)
        STATE.update(connected=True, status=f"esp32 connected: {client_ip}:{address[1]}", last_error="")
        while True:
            opcode, payload = read_websocket_frame(conn)
            if opcode == 0x8:
                break
            if opcode == 0x9:
                send_websocket_frame(conn, 0xA, payload)
                continue
            if opcode == 0x2 and payload:
                STATE.update_frame(payload)
            elif opcode == 0x1:
                text = payload.decode("utf-8", errors="replace")
                STATE.update(status=f"camera text: {text[:80]}")
    except Exception as exc:
        STATE.update(last_error=str(exc), status=f"camera websocket error: {exc}")
    finally:
        try:
            conn.close()
        except Exception:
            pass
        STATE.update(connected=False, status=f"esp32 disconnected: {client_ip}:{address[1]}")


def camera_websocket_server(host: str, port: int) -> None:
    while True:
        try:
            STATE.update(status=f"listening ws://{host}:{port}{CAMERA_WS_PATH}")
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((host, port))
                server.listen(4)
                while True:
                    conn, address = server.accept()
                    threading.Thread(target=handle_camera_ws_client, args=(conn, address), daemon=True).start()
        except Exception as exc:
            STATE.update(connected=False, status=f"camera websocket server error: {exc}", last_error=str(exc))
            time.sleep(1.0)


def detector_loop() -> None:
    last_seen = 0
    while True:
        frame_no, frame = STATE.wait_raw_frame(last_seen, timeout=1.0)
        if frame_no == last_seen or not frame:
            continue
        last_seen = frame_no
        if frame_no % DETECT_EVERY_N_FRAMES != 0:
            continue
        try:
            detect_frame(frame)
        except Exception as exc:
            STATE.update(last_error=str(exc), status=f"detection error: {exc}")
            time.sleep(0.1)


def json_response(handler: BaseHTTPRequestHandler, status: int, payload: dict) -> None:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def read_json(handler: BaseHTTPRequestHandler) -> dict:
    length = int(handler.headers.get("Content-Length", "0") or "0")
    if length <= 0:
        return {}
    raw = handler.rfile.read(length)
    return json.loads(raw.decode("utf-8"))


def send_detected_mjpeg(handler: BaseHTTPRequestHandler) -> None:
    boundary = "frame"
    handler.send_response(200)
    handler.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
    handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
    handler.send_header("Pragma", "no-cache")
    handler.end_headers()

    last_seen = -1
    while True:
        frame_no, frame = STATE.wait_raw_frame(last_seen, timeout=5.0)
        if frame_no == last_seen or not frame:
            continue
        last_seen = frame_no
        try:
            handler.wfile.write(f"--{boundary}\r\n".encode("ascii"))
            handler.wfile.write(b"Content-Type: image/jpeg\r\n")
            handler.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
            handler.wfile.write(frame)
            handler.wfile.write(b"\r\n")
            handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            break


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            body = (
                HTML.replace("__PUBLIC_IP__", PUBLIC_IP)
                .replace("__CAMERA_WS_PORT__", str(CAMERA_WS_PORT))
                .replace("__CAMERA_WS_PATH__", CAMERA_WS_PATH)
                .replace("__MASK_ALPHA__", str(MASK_ALPHA))
                .encode("utf-8")
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif parsed.path == "/api/state":
            json_response(self, 200, STATE.snapshot())
        elif parsed.path == "/api/detected.mjpg":
            send_detected_mjpeg(self)
        else:
            json_response(self, 404, {"error": "Not found"})

    def do_POST(self):
        parsed = urlparse(self.path)
        try:
            body = read_json(self)
            if parsed.path == "/api/config":
                target_classes, class_ids = parse_target_classes(str(body.get("target_classes", "")))
                STATE.update_detection_filter(target_classes, class_ids)
                json_response(self, 200, STATE.snapshot())
            else:
                json_response(self, 404, {"error": "Not found"})
        except ValueError as error:
            json_response(self, 400, {"error": str(error)})
        except Exception as error:
            json_response(self, 500, {"error": str(error)})


if __name__ == "__main__":
    print(f"Loading YOLO model: {YOLO_MODEL}")
    print(f"YOLO device: {YOLO_DEVICE}")
    if torch.cuda.is_available():
        print(f"CUDA GPU: {torch.cuda.get_device_name(0)}")
    threading.Thread(target=camera_websocket_server, args=(CAMERA_WS_HOST, CAMERA_WS_PORT), daemon=True).start()
    threading.Thread(target=detector_loop, daemon=True).start()
    print(f"WebUI: http://{PUBLIC_IP}:{PORT}")
    print(f"本机: http://127.0.0.1:{PORT}")
    print(f"摄像头WS监听: {CAMERA_WS_HOST}:{CAMERA_WS_PORT}{CAMERA_WS_PATH}")
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()

