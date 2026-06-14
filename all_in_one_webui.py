# -*- coding: utf-8 -*-
"""

三合一 WebUI：小车控制 + 摄像头 YOLO/YOLOE + 实体摇杆遥控器。



运行：

  python all_in_one_webui.py



这个文件不依赖 app.py、esp32_camera_yolo8_project/app_camera_yolo.py、

physical_remote/app_remote.py。删除那三个文件后，只运行本文件也能提供三者合并后的功能。

"""



from dataclasses import dataclass, field

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from typing import Dict, List, Optional, Tuple

from urllib.error import HTTPError, URLError

from urllib.parse import urlencode, urlparse

from urllib.request import Request, urlopen

import base64

import hashlib

import json

import math

import os

import re

import socket

import struct

import sys

import threading

import time


def load_local_env(path: str = ".env.local") -> None:

    try:

        env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), path)

        with open(env_path, "r", encoding="utf-8") as handle:

            for raw_line in handle:

                line = raw_line.strip()

                if not line or line.startswith("#") or "=" not in line:

                    continue

                key, value = line.split("=", 1)

                key = key.strip()

                value = value.strip().strip('"').strip("'")

                if key and key not in os.environ:

                    os.environ[key] = value

    except FileNotFoundError:

        return


load_local_env()





HOST = "0.0.0.0"

PORT = int(os.environ.get("PORT", "8000"))

PUBLIC_IP = os.environ.get("PUBLIC_IP", "192.168.152.216")

ESP32_BASE_URL = os.environ.get("ESP32_BASE_URL", "http://192.168.152.11").rstrip("/")

REMOTE_ESP32_URL = os.environ.get("REMOTE_ESP32_URL", "http://192.168.152.176").rstrip("/")

ROBOT_ESP32_URL = os.environ.get("ROBOT_ESP32_URL", ESP32_BASE_URL).rstrip("/")

ROBOT_POLL_ENABLED = os.environ.get("ROBOT_POLL_ENABLED", "0") == "1"

REMOTE_POLL_MS = int(os.environ.get("REMOTE_POLL_MS", "50"))

REMOTE_STEERING_SIGN = -1 if os.environ.get("REMOTE_STEERING_SIGN", "-1").strip() == "-1" else 1

VOICE_DRIVE_KEEPALIVE_MS = int(os.environ.get("VOICE_DRIVE_KEEPALIVE_MS", "120"))

VOICE_DRIVE_DEFAULT_HOLD_MS = int(os.environ.get("VOICE_DRIVE_HOLD_MS", "1200"))

VOICE_DRIVE_FULL_SPEED = int(os.environ.get("VOICE_DRIVE_FULL_SPEED", "100"))

VOICE_DRIVE_SLOW_SPEED = int(os.environ.get("VOICE_DRIVE_SLOW_SPEED", "50"))

VOICE_WS_ENABLED = os.environ.get("VOICE_WS_ENABLED", "1") != "0"



IMU_TCP_HOST = os.environ.get("IMU_TCP_HOST", "0.0.0.0")

IMU_TCP_PORT = int(os.environ.get("IMU_TCP_PORT", "9000"))

IMU_TCP_STALE_TIMEOUT = float(os.environ.get("IMU_TCP_STALE_TIMEOUT", "30.0"))

CAMERA_WS_HOST = os.environ.get("CAMERA_WS_HOST", "0.0.0.0")

CAMERA_WS_PORT = int(os.environ.get("CAMERA_WS_PORT", "8081"))

CAMERA_WS_PATH = "/ws/camera"

YOLO_CAMERA_ALLOWED_IP = os.environ.get(
    "YOLO_CAMERA_ALLOWED_IP",
    os.environ.get("CAMERA_ALLOWED_IP", "192.168.152.71"),
).strip()

ROBOT_CAMERA_ALLOWED_IP = os.environ.get("ROBOT_CAMERA_ALLOWED_IP", "192.168.152.11").strip()

CAMERA_WS_PING_INTERVAL = float(os.environ.get("CAMERA_WS_PING_INTERVAL", "10.0"))

SERVER_RESTART_DELAY = float(os.environ.get("SERVER_RESTART_DELAY", "2.0"))



YOLO_MODEL = os.environ.get("YOLO_MODEL", "yoloe-11l-seg.pt")

IS_YOLOE = "yoloe" in YOLO_MODEL.lower()

DEFAULT_TARGET_CLASSES = os.environ.get("TARGET_CLASSES", "football, football gate" if IS_YOLOE else "")

YOLO_CONF = float(os.environ.get("YOLO_CONF", "0.35"))

YOLO_IMGSZ = int(os.environ.get("YOLO_IMGSZ", "640"))

YOLO_DEVICE_ENV = os.environ.get("YOLO_DEVICE", "")

DETECT_EVERY_N_FRAMES = max(1, int(os.environ.get("DETECT_EVERY_N_FRAMES", "1")))

DETECTION_OVERLAY_TTL = float(os.environ.get("DETECTION_OVERLAY_TTL", "1.0"))

MASK_ALPHA = float(os.environ.get("MASK_ALPHA", "0.45"))

AUTO_PATH_ENABLED_ON_START = os.environ.get("AUTO_PATH_ENABLED", "0") == "1"

AUTO_PATH_BALL_CLASSES = os.environ.get("AUTO_PATH_BALL_CLASSES", "football, sports ball, ball")

AUTO_PATH_GATE_CLASSES = os.environ.get("AUTO_PATH_GATE_CLASSES", "football gate, gate, goal")

AUTO_PATH_INTERVAL_MS = int(os.environ.get("AUTO_PATH_INTERVAL_MS", "160"))

AUTO_PATH_STALE_MS = int(os.environ.get("AUTO_PATH_STALE_MS", "900"))

AUTO_PATH_MIN_SPEED = int(os.environ.get("AUTO_PATH_MIN_SPEED", "18"))

AUTO_PATH_MAX_SPEED = int(os.environ.get("AUTO_PATH_MAX_SPEED", "65"))

AUTO_PATH_STEERING_LIMIT = int(os.environ.get("AUTO_PATH_STEERING_LIMIT", "80"))

AUTO_PATH_DEADZONE_RATIO = float(os.environ.get("AUTO_PATH_DEADZONE_RATIO", "0.04"))

AUTO_PATH_ARRIVAL_RATIO = float(os.environ.get("AUTO_PATH_ARRIVAL_RATIO", "0.06"))



HEADER = b"\x59\x53"

DATA_ID_TEMP = 0x01

DATA_ID_ACCEL = 0x10

DATA_ID_GYRO = 0x20

DATA_ID_MAG_NORM = 0x30

DATA_ID_MAG_FIELD = 0x31

DATA_ID_EULER = 0x40

DATA_ID_QUATERNION = 0x41





def finite_json(value):

    if isinstance(value, float):

        return value if math.isfinite(value) else None

    if isinstance(value, list):

        return [finite_json(item) for item in value]

    if isinstance(value, dict):

        return {key: finite_json(item) for key, item in value.items()}

    return value





@dataclass

class ImuState:

    frame_no: int = 0

    temperature: Optional[float] = None

    accel: Optional[list] = None

    gyro: Optional[list] = None

    mag_norm: Optional[list] = None

    mag_field: Optional[list] = None

    euler_deg: Optional[list] = None

    quaternion: Optional[list] = None

    last_update: float = 0.0

    last_raw_update: float = 0.0

    raw_bytes: int = 0

    buffered_bytes: int = 0

    header_hits: int = 0

    wit_header_hits: int = 0

    aa55_header_hits: int = 0

    last_raw_hex: str = ""

    rolling_raw: bytes = b""

    raw_packets: int = 0

    raw_groups: int = 0

    parse_errors: int = 0

    last_parse_error: str = ""

    status: str = "starting"

    lock: threading.Lock = field(default_factory=threading.Lock)



    def update(self, **kwargs) -> None:

        with self.lock:

            for key, value in kwargs.items():

                setattr(self, key, value)



    def note_raw_chunk(self, chunk: bytes, buffered_bytes: int) -> None:

        with self.lock:

            self.raw_bytes += len(chunk)

            self.buffered_bytes = buffered_bytes

            self.last_raw_update = time.time()

            self.header_hits += chunk.count(HEADER)

            self.wit_header_hits += chunk.count(b"\x55")

            self.aa55_header_hits += chunk.count(b"\xaa\x55")

            self.rolling_raw = (self.rolling_raw + chunk)[-96:]

            self.last_raw_hex = self.rolling_raw.hex(" ")



    def snapshot(self) -> Dict:

        with self.lock:

            return finite_json(

                {

                    "frame_no": self.frame_no,

                    "temperature": self.temperature,

                    "accel": self.accel,

                    "gyro": self.gyro,

                    "mag_norm": self.mag_norm,

                    "mag_field": self.mag_field,

                    "euler_deg": self.euler_deg,

                    "quaternion": self.quaternion,

                    "raw_bytes": self.raw_bytes,

                    "buffered_bytes": self.buffered_bytes,

                    "header_hits": self.header_hits,

                    "wit_header_hits": self.wit_header_hits,

                    "aa55_header_hits": self.aa55_header_hits,

                    "last_raw_hex": self.last_raw_hex,

                    "raw_packets": self.raw_packets,

                    "raw_groups": self.raw_groups,

                    "parse_errors": self.parse_errors,

                    "last_parse_error": self.last_parse_error,

                    "status": self.status,

                    "age_ms": int((time.time() - self.last_update) * 1000) if self.last_update else None,

                    "raw_age_ms": int((time.time() - self.last_raw_update) * 1000) if self.last_raw_update else None,

                }

            )





@dataclass

class CameraState:

    status: str = "starting"

    connected: bool = False

    frame_count: int = 0

    byte_count: int = 0

    last_update: float = 0.0

    last_frame: Optional[bytes] = None

    last_error: str = ""

    lock: threading.Lock = field(default_factory=threading.Lock)

    condition: threading.Condition = field(init=False)



    def __post_init__(self) -> None:

        self.condition = threading.Condition(self.lock)



    def update(self, **kwargs) -> None:

        self.condition.acquire()

        try:

            for key, value in kwargs.items():

                setattr(self, key, value)

            self.condition.notify_all()

        finally:

            self.condition.release()



    def update_frame(self, frame: bytes) -> None:

        self.condition.acquire()

        try:

            self.last_frame = frame

            self.frame_count += 1

            self.byte_count += len(frame)

            self.last_update = time.time()

            self.status = "receiving"

            self.condition.notify_all()

        finally:

            self.condition.release()



    def wait_frame(self, last_seen_count: int, timeout: float = 2.0):

        self.condition.acquire()

        try:

            self.condition.wait_for(lambda: self.frame_count != last_seen_count, timeout)

            return self.frame_count, self.last_frame

        finally:

            self.condition.release()



    def snapshot(self) -> Dict:

        with self.lock:

            return {

                "status": self.status,

                "connected": self.connected,

                "frame_count": self.frame_count,

                "byte_count": self.byte_count,

                "last_error": self.last_error,

                "age_ms": int((time.time() - self.last_update) * 1000) if self.last_update else None,

                "frame_size": len(self.last_frame) if self.last_frame else 0,

                "robot_camera_allowed_ip": ROBOT_CAMERA_ALLOWED_IP or "any",
                "yolo_camera_allowed_ip": YOLO_CAMERA_ALLOWED_IP or "not configured",

            }





@dataclass

class YoloState:

    status: str = "starting"

    last_error: str = ""

    detect_count: int = 0

    inference_ms: Optional[int] = None

    objects: List[str] = field(default_factory=list)

    detection_boxes: List[dict] = field(default_factory=list)

    detection_source_size: Optional[Tuple[int, int]] = None

    last_detection_at: float = 0.0

    target_classes: str = DEFAULT_TARGET_CLASSES

    target_class_ids: Optional[List[int]] = None

    lock: threading.Lock = field(default_factory=threading.Lock)



    def update(self, **kwargs) -> None:

        with self.lock:

            for key, value in kwargs.items():

                setattr(self, key, value)



    def update_detection(self, inference_ms: int, objects: List[str], boxes: List[dict], source_size: Tuple[int, int]) -> None:

        with self.lock:

            self.detect_count += 1

            self.inference_ms = inference_ms

            self.objects = objects

            self.detection_boxes = boxes

            self.detection_source_size = source_size

            self.last_detection_at = time.time()

            self.status = "detecting"



    def snapshot(self) -> Dict:

        with self.lock:

            overlay_age_ms = int((time.time() - self.last_detection_at) * 1000) if self.last_detection_at else None

            boxes = []

            source_size = None

            if overlay_age_ms is not None and overlay_age_ms <= int(DETECTION_OVERLAY_TTL * 1000):

                boxes = [dict(box) for box in self.detection_boxes]

                source_size = list(self.detection_source_size) if self.detection_source_size else None

            return {

                "status": self.status,

                "last_error": self.last_error,

                "detect_count": self.detect_count,

                "inference_ms": self.inference_ms,

                "objects": list(self.objects),

                "overlay_age_ms": overlay_age_ms,

                "detection_boxes": boxes,

                "detection_source_size": source_size,

                "target_classes": self.target_classes,

                "model": YOLO_MODEL,

                "device": yolo_device(),

                "conf": YOLO_CONF,

                "imgsz": YOLO_IMGSZ,

            }



@dataclass

class AutoPathState:

    enabled: bool = AUTO_PATH_ENABLED_ON_START

    status: str = "waiting for football and gate"

    last_error: str = ""

    ball_classes: str = AUTO_PATH_BALL_CLASSES

    gate_classes: str = AUTO_PATH_GATE_CLASSES

    interval_ms: int = AUTO_PATH_INTERVAL_MS

    stale_ms: int = AUTO_PATH_STALE_MS

    min_speed: int = AUTO_PATH_MIN_SPEED

    max_speed: int = AUTO_PATH_MAX_SPEED

    steering_limit: int = AUTO_PATH_STEERING_LIMIT

    deadzone_ratio: float = AUTO_PATH_DEADZONE_RATIO

    arrival_ratio: float = AUTO_PATH_ARRIVAL_RATIO

    last_plan: Optional[dict] = None

    last_plan_at: float = 0.0

    last_sent_at: float = 0.0

    last_sent: Optional[dict] = None

    command_count: int = 0

    lock: threading.Lock = field(default_factory=threading.Lock)



    def update(self, **kwargs) -> None:

        with self.lock:

            for key, value in kwargs.items():

                setattr(self, key, value)



    def update_plan(self, plan: dict) -> None:

        with self.lock:

            self.last_plan = plan

            self.last_plan_at = time.time()

            self.status = plan.get("status") or plan.get("reason") or self.status

            if plan.get("ok"):

                self.last_error = ""



    def mark_sent(self, throttle: int, steering: int, error: str = "") -> None:

        with self.lock:

            self.last_sent_at = time.time()

            self.last_sent = {"throttle": throttle, "steering": steering}

            if error:

                self.last_error = error

                self.status = f"send error: {error}"

            else:

                self.last_error = ""

                self.command_count += 1



    def snapshot(self) -> Dict:

        now = time.time()

        with self.lock:

            plan = dict(self.last_plan) if isinstance(self.last_plan, dict) else None

            plan_age_ms = int((now - self.last_plan_at) * 1000) if self.last_plan_at else None

            return {

                "enabled": self.enabled,

                "status": self.status,

                "last_error": self.last_error,

                "ball_classes": self.ball_classes,

                "gate_classes": self.gate_classes,

                "interval_ms": self.interval_ms,

                "stale_ms": self.stale_ms,

                "min_speed": self.min_speed,

                "max_speed": self.max_speed,

                "steering_limit": self.steering_limit,

                "deadzone_ratio": self.deadzone_ratio,

                "arrival_ratio": self.arrival_ratio,

                "plan": plan,

                "plan_age_ms": plan_age_ms,

                "last_sent": dict(self.last_sent) if isinstance(self.last_sent, dict) else None,

                "last_sent_age_ms": int((now - self.last_sent_at) * 1000) if self.last_sent_at else None,

                "command_count": self.command_count,

            }





IMU_STATE = ImuState()

ROBOT_CAMERA_STATE = CameraState()

YOLO_CAMERA_STATE = CameraState()

CAMERA_STATE = ROBOT_CAMERA_STATE

YOLO_STATE = YoloState()

AUTO_PATH_STATE = AutoPathState()

_ACTIVE_CAMERA_CONNS: Dict[str, Optional[socket.socket]] = {"robot": None, "yolo": None}

_CAMERA_CONN_LOCK = threading.Lock()

_MODEL = None

_MODEL_CLASS_TEXT: Optional[str] = None

_MODEL_CLASS_LOCK = threading.Lock()

_YOLO_IMPORTS = None

REMOTE_FORWARD_LOCK = threading.Lock()

LOCAL_REMOTE_FORWARD_ENABLED = False

LOCAL_REMOTE_FORWARD_ERROR = ""

LOCAL_REMOTE_FORWARD_LAST_SENT = 0.0

LOCAL_REMOTE_ASSIST_MODE = 1

REMOTE_BUTTON_LONG_PRESS_SECONDS = 1.0

REMOTE_BUTTON_LAST_PRESSED = False

REMOTE_BUTTON_PRESSED_AT = 0.0

REMOTE_BUTTON_LONG_PRESS_HANDLED = False

REMOTE_BUTTON_LAST_ACTION = ""

VOICE_DRIVE_LOCK = threading.Lock()

VOICE_DRIVE_STATE = {

    "active": False,

    "throttle": 0,

    "steering": 0,

    "keyword": "",

    "transcript": "",

    "source": "",

    "speed": VOICE_DRIVE_FULL_SPEED,

    "last_update": 0.0,

    "last_sent": 0.0,

    "expires_at": 0.0,

    "last_error": "",

    "command_count": 0,

}

VOICE_STOP_WORDS = (

    "停止",

    "停下",

    "停车",

    "刹车",

    "别动",

    "不要动",

    "停",

)

VOICE_DIRECTION_KEYWORDS = (

    ("left_forward", ("左上", "上左", "左前", "前左", "向左上", "往左上", "左前方"), 1, 1),

    ("right_forward", ("右上", "上右", "右前", "前右", "向右上", "往右上", "右前方"), 1, -1),

    ("left_backward", ("左下", "下左", "左后", "后左", "向左下", "往左下", "左后方"), -1, 1),

    ("right_backward", ("右下", "下右", "右后", "后右", "向右下", "往右下", "右后方"), -1, -1),

    ("forward", ("前进", "向前", "往前", "前冲", "向前冲", "冲", "上"), 1, 0),

    ("backward", ("后退", "向后", "往后", "倒车", "后"), -1, 0),

    ("left", ("向左", "往左", "左移", "左转", "左"), 0, 1),

    ("right", ("向右", "往右", "右移", "右转", "右"), 0, -1),

)



CLASS_ALIASES = {

    "人": "person",

    "人物": "person",

    "球": "ball",

    "足球": "football",

    "足球门": "football gate",

    "球门": "football gate",

    "球门框": "football gate",

    "门框": "football gate",

    "门": "football gate",

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





def yolo_device() -> str:

    if YOLO_DEVICE_ENV:

        return YOLO_DEVICE_ENV

    try:

        import torch



        return "0" if torch.cuda.is_available() else "cpu"

    except Exception:

        return "cpu"





def load_yolo_model():

    global _MODEL, _YOLO_IMPORTS

    if _MODEL is not None:

        return _MODEL

    try:

        import cv2

        import numpy as np

        from ultralytics import YOLO, YOLOE



        _YOLO_IMPORTS = {"cv2": cv2, "np": np}

        _MODEL = YOLOE(YOLO_MODEL) if IS_YOLOE else YOLO(YOLO_MODEL)

        YOLO_STATE.update(status=f"YOLO model loaded: {YOLO_MODEL}")

        return _MODEL

    except Exception as exc:

        YOLO_STATE.update(status=f"YOLO load error: {exc}", last_error=str(exc))

        raise





def configure_socket_keepalive(sock: socket.socket) -> None:

    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

    for option, value in (("TCP_KEEPIDLE", 10), ("TCP_KEEPINTVL", 3), ("TCP_KEEPCNT", 3)):

        if hasattr(socket, option):

            sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, option), value)





def read_scaled_int32_triplet(data: bytes, scale: float = 0.000001) -> list:

    values = [round(v * scale, 6) for v in struct.unpack("<iii", data)]

    if any(not math.isfinite(v) for v in values):

        raise ValueError("non-finite float in frame")

    return values





def normalize_quaternion(q: list) -> list:

    norm = math.sqrt(sum(v * v for v in q))

    if norm <= 0:

        return [1.0, 0.0, 0.0, 0.0]

    return [round(v / norm, 6) for v in q]





def euler_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list:

    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))

    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)

    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    return normalize_quaternion(

        [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]

    )





def parse_data_group(data_id: int, data: bytes) -> Dict:

    if data_id == DATA_ID_TEMP and len(data) == 2:

        temp = round(struct.unpack("<h", data)[0] / 100.0, 2)

        if -80.0 <= temp <= 125.0:

            return {"temperature": temp}

    if data_id == DATA_ID_ACCEL and len(data) == 12:

        return {"accel": read_scaled_int32_triplet(data)}

    if data_id == DATA_ID_GYRO and len(data) == 12:

        return {"gyro": read_scaled_int32_triplet(data)}

    if data_id == DATA_ID_MAG_NORM and len(data) == 12:

        return {"mag_norm": read_scaled_int32_triplet(data)}

    if data_id == DATA_ID_MAG_FIELD and len(data) == 12:

        return {"mag_field": read_scaled_int32_triplet(data, 0.001)}

    if data_id == DATA_ID_EULER and len(data) == 12:

        pitch, roll, yaw = read_scaled_int32_triplet(data)

        euler = [roll, pitch, yaw]

        return {"euler_deg": euler, "quaternion": euler_to_quaternion(*euler)}

    if data_id == DATA_ID_QUATERNION and len(data) == 16:

        q = [round(v * 0.000001, 6) for v in struct.unpack("<iiii", data)]

        return {"quaternion": normalize_quaternion(q)}

    raise ValueError(f"unsupported data group id=0x{data_id:02x} len={len(data)}")





def verify_checksum(frame: bytes, payload_len: int) -> None:

    ck1 = ck2 = 0

    for byte in frame[2 : 5 + payload_len]:

        ck1 = (ck1 + byte) & 0xFF

        ck2 = (ck2 + ck1) & 0xFF

    if ck1 != frame[5 + payload_len] or ck2 != frame[6 + payload_len]:

        raise ValueError("checksum mismatch")





def parse_frame(frame: bytes) -> Dict:

    if len(frame) < 7 or frame[:2] != HEADER:

        raise ValueError("bad frame header")

    frame_no = struct.unpack("<H", frame[2:4])[0]

    payload_len = frame[4]

    if len(frame) != 2 + 2 + 1 + payload_len + 2:

        raise ValueError("frame length mismatch")

    verify_checksum(frame, payload_len)

    result = {}

    payload = frame[5 : 5 + payload_len]

    index = 0

    while index + 2 <= len(payload):

        data_id, length = payload[index], payload[index + 1]

        index += 2

        data = payload[index : index + length]

        index += length

        if len(data) == length:

            try:

                result.update(parse_data_group(data_id, data))

            except ValueError:

                pass

    if not result:

        raise ValueError("frame contains no recognized data groups")

    result["frame_no"] = frame_no

    return result





def scan_data_groups(buffer: bytearray, state: ImuState) -> None:

    known_lengths = {DATA_ID_TEMP: 2, DATA_ID_ACCEL: 12, DATA_ID_GYRO: 12, DATA_ID_MAG_NORM: 12, DATA_ID_MAG_FIELD: 12, DATA_ID_EULER: 12, DATA_ID_QUATERNION: 16}

    index = groups_found = 0

    updates = {}

    while index + 2 <= len(buffer):

        data_id, length = buffer[index], buffer[index + 1]

        if known_lengths.get(data_id) != length:

            index += 1

            continue

        end = index + 2 + length

        if end > len(buffer):

            break

        try:

            updates.update(parse_data_group(data_id, bytes(buffer[index + 2 : end])))

            groups_found += 1

            index = end

        except ValueError as exc:

            state.update(last_parse_error=str(exc))

            index += 1

    if groups_found:

        with state.lock:

            state.raw_groups += groups_found

        updates.update(last_update=time.time())

        state.update(**updates)





def consume_imu_buffer(buffer: bytearray, state: ImuState) -> None:

    while True:

        start = buffer.find(HEADER)

        if start < 0:

            del buffer[:-1]

            state.update(buffered_bytes=len(buffer))

            return

        if start > 0:

            del buffer[:start]

            state.update(buffered_bytes=len(buffer))

        if len(buffer) < 7:

            state.update(buffered_bytes=len(buffer))

            return

        payload_len = buffer[4]

        frame_len = 2 + 2 + 1 + payload_len + 2

        if len(buffer) < frame_len:

            state.update(buffered_bytes=len(buffer))

            return

        frame = bytes(buffer[:frame_len])

        del buffer[:frame_len]

        state.update(buffered_bytes=len(buffer))

        try:

            parsed = parse_frame(frame)

            with state.lock:

                state.raw_packets += 1

            parsed.update(last_update=time.time())

            state.update(**parsed)

        except Exception as exc:

            with state.lock:

                state.parse_errors += 1

            state.update(last_parse_error=str(exc))





def frame_stream_from_tcp_server(host: str, port: int, state: ImuState) -> None:

    while True:

        try:

            state.update(status=f"listening tcp://{host}:{port}")

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:

                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                configure_socket_keepalive(server)

                server.bind((host, port))

                server.listen(4)

                print(f"[imu-tcp] listening on {host}:{port}", flush=True)

                while True:

                    client, address = server.accept()

                    buffer = bytearray()

                    with client:

                        configure_socket_keepalive(client)

                        client.settimeout(1.0)

                        last_chunk_at = time.time()

                        state.update(status=f"esp32 connected: {address[0]}:{address[1]}")

                        while True:

                            try:

                                chunk = client.recv(4096)

                            except socket.timeout:

                                if time.time() - last_chunk_at > IMU_TCP_STALE_TIMEOUT:

                                    state.update(status=f"esp32 data timeout: {address[0]}:{address[1]}")

                                    break

                                continue

                            except OSError as exc:

                                state.update(status=f"esp32 socket error: {exc}")

                                break

                            if not chunk:

                                state.update(status=f"esp32 disconnected: {address[0]}:{address[1]}")

                                break

                            last_chunk_at = time.time()

                            buffer.extend(chunk)

                            state.note_raw_chunk(chunk, len(buffer))

                            scan_data_groups(buffer, state)

                            consume_imu_buffer(buffer, state)

                    state.update(status=f"waiting esp32 on tcp://{host}:{port}")

        except Exception as exc:

            state.update(status=f"tcp server error: {exc}")

            print(f"[imu-tcp] server error: {exc}, retry in {SERVER_RESTART_DELAY}s", flush=True)

            time.sleep(SERVER_RESTART_DELAY)





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

    digest = hashlib.sha1((client_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()

    return base64.b64encode(digest).decode("ascii")





def perform_websocket_handshake(conn: socket.socket) -> None:

    headers = read_http_headers(conn)

    lines = headers.split("\r\n")

    request = lines[0].split()

    if len(request) < 2:

        raise ValueError("bad websocket request")

    path = request[1].split("?", 1)[0]

    header_map = {}

    for line in lines[1:]:

        if ":" in line:

            key, value = line.split(":", 1)

            header_map[key.strip().lower()] = value.strip()

    client_key = header_map.get("sec-websocket-key")

    if not client_key:

        raise ValueError("missing Sec-WebSocket-Key")

    if path != CAMERA_WS_PATH:

        body = b"Not found"

        conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\n" + f"Content-Length: {len(body)}\r\n\r\n".encode("ascii") + body)

        raise ValueError(f"unsupported websocket path: {path}")

    conn.sendall(

        (

            "HTTP/1.1 101 Switching Protocols\r\n"

            "Upgrade: websocket\r\n"

            "Connection: Upgrade\r\n"

            f"Sec-WebSocket-Accept: {websocket_accept_key(client_key)}\r\n\r\n"

        ).encode("ascii")

    )





def read_websocket_frame(conn: socket.socket):

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





def select_camera_state(client_ip: str) -> Tuple[Optional[str], Optional[CameraState]]:

    if YOLO_CAMERA_ALLOWED_IP and client_ip == YOLO_CAMERA_ALLOWED_IP:

        return "yolo", YOLO_CAMERA_STATE

    if ROBOT_CAMERA_ALLOWED_IP and client_ip != ROBOT_CAMERA_ALLOWED_IP:

        return None, None

    return "robot", ROBOT_CAMERA_STATE



def handle_camera_ws_client(conn: socket.socket, address) -> None:

    client_ip = address[0]

    bucket, state = select_camera_state(client_ip)

    if state is None or bucket is None:

        conn.close()

        ROBOT_CAMERA_STATE.update(last_error=f"ignored camera ip {client_ip}; robot={ROBOT_CAMERA_ALLOWED_IP or 'any'}, yolo={YOLO_CAMERA_ALLOWED_IP or 'not configured'}")

        YOLO_CAMERA_STATE.update(last_error=f"ignored camera ip {client_ip}; robot={ROBOT_CAMERA_ALLOWED_IP or 'any'}, yolo={YOLO_CAMERA_ALLOWED_IP or 'not configured'}")

        return

    is_current = False

    conn.settimeout(CAMERA_WS_PING_INTERVAL)

    try:

        perform_websocket_handshake(conn)

        with _CAMERA_CONN_LOCK:

            old_conn = _ACTIVE_CAMERA_CONNS.get(bucket)

            if old_conn is not None:

                try:

                    old_conn.close()

                except Exception:

                    pass

            _ACTIVE_CAMERA_CONNS[bucket] = conn

            is_current = True

        configure_socket_keepalive(conn)

        state.update(connected=True, status=f"{bucket} camera connected: {address[0]}:{address[1]}", last_error="")

        while True:

            try:

                opcode, payload = read_websocket_frame(conn)

            except socket.timeout:

                send_websocket_frame(conn, 0x9, b"ping")

                continue

            if opcode == 0x8:

                break

            if opcode == 0x9:

                send_websocket_frame(conn, 0xA, payload)

            elif opcode == 0x2 and payload:

                state.update_frame(payload)

            elif opcode == 0x1:

                state.update(status=f"{bucket} camera text: {payload.decode('utf-8', errors='replace')[:80]}")

    except Exception as exc:

        if is_current:

            state.update(last_error=str(exc), status=f"{bucket} camera websocket error: {exc}")

    finally:

        with _CAMERA_CONN_LOCK:

            if _ACTIVE_CAMERA_CONNS.get(bucket) is conn:

                _ACTIVE_CAMERA_CONNS[bucket] = None

                is_current = True

        try:

            conn.close()

        except Exception:

            pass

        if is_current:

            state.update(connected=False, status=f"{bucket} camera disconnected: {address[0]}:{address[1]}")





def camera_websocket_server(host: str, port: int) -> None:

    while True:

        try:

            ROBOT_CAMERA_STATE.update(status=f"listening ws://{host}:{port}{CAMERA_WS_PATH}")

            YOLO_CAMERA_STATE.update(status=f"listening ws://{host}:{port}{CAMERA_WS_PATH}")

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:

                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                configure_socket_keepalive(server)

                server.bind((host, port))

                server.listen(8)

                print(f"[camera-ws] listening on {host}:{port}{CAMERA_WS_PATH}", flush=True)

                while True:

                    conn, address = server.accept()

                    threading.Thread(target=handle_camera_ws_client, args=(conn, address), daemon=True).start()

        except Exception as exc:

            ROBOT_CAMERA_STATE.update(connected=False, status=f"camera websocket server error: {exc}", last_error=str(exc))

            YOLO_CAMERA_STATE.update(connected=False, status=f"camera websocket server error: {exc}", last_error=str(exc))

            time.sleep(SERVER_RESTART_DELAY)





def parse_target_classes(text: str) -> Tuple[str, Optional[List[int]]]:

    model = load_yolo_model()

    text = text.strip()

    if IS_YOLOE and not text:

        text = DEFAULT_TARGET_CLASSES

    elif not text:

        return "", None

    items = [CLASS_ALIASES.get(item.strip(), item.strip()) for item in re.split(r"[,，、;；]", text) if item.strip()]

    if IS_YOLOE:

        return ", ".join(items), None

    name_items = model.names.items() if hasattr(model.names, "items") else enumerate(model.names)

    id_to_name = {int(class_id): str(name) for class_id, name in name_items}

    name_to_id = {name.lower(): class_id for class_id, name in id_to_name.items()}

    class_ids, names, unknown = [], [], []

    for item in items:

        class_id = name_to_id.get(item.lower())

        if class_id is None:

            unknown.append(item)

        elif class_id not in class_ids:

            class_ids.append(class_id)

            names.append(id_to_name[class_id])

    if unknown:

        raise ValueError(f"未知检测类型：{', '.join(unknown)}")

    return ", ".join(names), class_ids or None





def apply_yoloe_classes(text: str) -> None:

    global _MODEL_CLASS_TEXT

    if not IS_YOLOE:

        return

    model = load_yolo_model()

    target_classes, _ = parse_target_classes(text)

    prompts = [item.strip() for item in target_classes.split(",") if item.strip()]

    prompt_text = ", ".join(prompts)

    with _MODEL_CLASS_LOCK:

        if _MODEL_CLASS_TEXT != prompt_text:

            model.set_classes(prompts)

            _MODEL_CLASS_TEXT = prompt_text

            YOLO_STATE.update(status=f"YOLOE prompts: {prompt_text}")



def parse_detection_class_set(text: str) -> set:

    items = []

    for raw in re.split(r"[,，、;；]", text or ""):

        item = raw.strip()

        if not item:

            continue

        items.append(CLASS_ALIASES.get(item, item).strip().lower())

    return set(items)



def clean_detection_name(value: str) -> str:

    name = str(value or "").strip()

    name = re.sub(r"\s+\d(?:\.\d+)?$", "", name)

    return CLASS_ALIASES.get(name, name).strip().lower()



def detection_center(box: dict) -> Optional[Tuple[float, float]]:

    xyxy = box.get("xyxy") or []

    if len(xyxy) >= 4:

        return (float(xyxy[0]) + float(xyxy[2])) / 2.0, (float(xyxy[1]) + float(xyxy[3])) / 2.0

    polygon = box.get("polygon") or []

    if polygon:

        xs = [float(point[0]) for point in polygon if len(point) >= 2]

        ys = [float(point[1]) for point in polygon if len(point) >= 2]

        if xs and ys:

            return sum(xs) / len(xs), sum(ys) / len(ys)

    return None



def detection_area(box: dict) -> float:

    xyxy = box.get("xyxy") or []

    if len(xyxy) >= 4:

        return max(0.0, float(xyxy[2]) - float(xyxy[0])) * max(0.0, float(xyxy[3]) - float(xyxy[1]))

    polygon = box.get("polygon") or []

    if len(polygon) >= 3:

        area = 0.0

        points = [(float(point[0]), float(point[1])) for point in polygon if len(point) >= 2]

        for index, (x1, y1) in enumerate(points):

            x2, y2 = points[(index + 1) % len(points)]

            area += x1 * y2 - x2 * y1

        return abs(area) / 2.0

    return 0.0



def summarize_detection(box: dict) -> dict:

    center = detection_center(box) or (0.0, 0.0)

    return {

        "name": box.get("name") or clean_detection_name(box.get("label", "")),

        "label": box.get("label", ""),

        "conf": box.get("conf"),

        "center": [round(center[0], 1), round(center[1], 1)],

        "xyxy": [round(float(v), 1) for v in (box.get("xyxy") or [])[:4]],

    }



def axis_direction(dx: float, dy: float, deadzone_px: float) -> str:

    parts = []

    if abs(dy) > deadzone_px:

        parts.append("前进" if dy < 0 else "后退")

    if abs(dx) > deadzone_px:

        parts.append("向右" if dx > 0 else "向左")

    return " + ".join(parts) if parts else "保持"



def build_auto_path_plan(boxes: List[dict], source_size: Tuple[int, int]) -> dict:

    with AUTO_PATH_STATE.lock:

        ball_classes = parse_detection_class_set(AUTO_PATH_STATE.ball_classes)

        gate_classes = parse_detection_class_set(AUTO_PATH_STATE.gate_classes)

        min_speed = max(1, min(100, int(AUTO_PATH_STATE.min_speed)))

        max_speed = max(min_speed, min(100, int(AUTO_PATH_STATE.max_speed)))

        steering_limit = max(1, min(100, int(AUTO_PATH_STATE.steering_limit)))

        deadzone_ratio = max(0.0, min(0.5, float(AUTO_PATH_STATE.deadzone_ratio)))

        arrival_ratio = max(deadzone_ratio, min(0.8, float(AUTO_PATH_STATE.arrival_ratio)))

    balls, gates = [], []

    for box in boxes:

        name = clean_detection_name(box.get("name") or box.get("label", ""))

        center = detection_center(box)

        if not name or center is None:

            continue

        enriched = dict(box)

        enriched["name"] = name

        enriched["center"] = [center[0], center[1]]

        enriched["area"] = detection_area(box)

        if name in gate_classes:

            gates.append(enriched)

        elif name in ball_classes:

            balls.append(enriched)

    if not balls or not gates:

        missing = []

        if not balls:

            missing.append("football")

        if not gates:

            missing.append("football gate")

        return {"ok": False, "status": f"waiting for {', '.join(missing)}", "reason": f"未同时识别到 {', '.join(missing)}", "balls": len(balls), "gates": len(gates), "source_size": list(source_size)}

    best = None

    for ball in balls:

        bx, by = ball["center"]

        for gate in gates:

            gx, gy = gate["center"]

            distance = math.hypot(gx - bx, gy - by)

            score = (distance, -float(ball.get("conf") or 0), -float(gate.get("conf") or 0), -float(ball.get("area") or 0))

            if best is None or score < best[0]:

                best = (score, ball, gate, gx - bx, gy - by, distance)

    _, ball, gate, dx, dy, distance = best

    width, height = source_size

    diag = max(1.0, math.hypot(float(width), float(height)))

    deadzone_px = max(8.0, diag * deadzone_ratio)

    arrival_px = max(deadzone_px, diag * arrival_ratio)

    arrived = distance <= arrival_px

    if arrived:

        throttle = steering = 0

    else:

        progress = max(0.0, min(1.0, (distance - deadzone_px) / max(1.0, diag * 0.45)))

        speed = min_speed + (max_speed - min_speed) * progress

        throttle = clamp_drive((-dy / distance) * speed) if distance else 0

        steering = clamp_drive((-dx / distance) * speed) if distance else 0

        steering = max(-steering_limit, min(steering_limit, steering))

        if abs(dx) <= deadzone_px:

            steering = 0

        if abs(dy) <= deadzone_px:

            throttle = 0

        if throttle == 0 and steering == 0:

            if abs(dx) >= abs(dy):

                steering = -min_speed if dx > 0 else min_speed

            else:

                throttle = -min_speed if dy > 0 else min_speed

    direction = "到达球门中心" if arrived else axis_direction(dx, dy, deadzone_px)

    return {

        "ok": True,

        "status": f"{direction} · throttle {throttle} / steering {steering}",

        "direction": direction,

        "football": summarize_detection(ball),

        "gate": summarize_detection(gate),

        "vector_px": [round(dx, 1), round(dy, 1)],

        "distance_px": round(distance, 1),

        "deadzone_px": round(deadzone_px, 1),

        "arrival_radius_px": round(arrival_px, 1),

        "arrived": arrived,

        "throttle": throttle,

        "steering": steering,

        "source_size": [width, height],

        "balls": len(balls),

        "gates": len(gates),

    }



def update_auto_path_plan(boxes: List[dict], source_size: Tuple[int, int]) -> None:

    AUTO_PATH_STATE.update_plan(build_auto_path_plan(boxes, source_size))



def apply_auto_path_config(body: dict) -> dict:

    updates = {}

    if "enabled" in body:

        updates["enabled"] = bool(body.get("enabled"))

    if "ball_classes" in body:

        updates["ball_classes"] = str(body.get("ball_classes", "")).strip() or AUTO_PATH_BALL_CLASSES

    if "gate_classes" in body:

        updates["gate_classes"] = str(body.get("gate_classes", "")).strip() or AUTO_PATH_GATE_CLASSES

    int_fields = {

        "interval_ms": (60, 2000),

        "stale_ms": (200, 5000),

        "min_speed": (1, 100),

        "max_speed": (1, 100),

        "steering_limit": (1, 100),

    }

    for key, (low, high) in int_fields.items():

        if key in body:

            updates[key] = max(low, min(high, int(body[key])))

    float_fields = {

        "deadzone_ratio": (0.0, 0.5),

        "arrival_ratio": (0.0, 0.8),

    }

    for key, (low, high) in float_fields.items():

        if key in body:

            updates[key] = max(low, min(high, float(body[key])))

    if "max_speed" in updates and "min_speed" not in updates:

        with AUTO_PATH_STATE.lock:

            updates["max_speed"] = max(int(AUTO_PATH_STATE.min_speed), updates["max_speed"])

    if "min_speed" in updates and "max_speed" not in updates:

        with AUTO_PATH_STATE.lock:

            updates["min_speed"] = min(int(AUTO_PATH_STATE.max_speed), updates["min_speed"])

    AUTO_PATH_STATE.update(**updates)

    if updates.get("enabled") is False:

        try:

            request_esp32_quick("/drive?throttle=0&steering=0", method="POST", timeout=0.35)

        except Exception as exc:

            AUTO_PATH_STATE.mark_sent(0, 0, str(exc))

    return AUTO_PATH_STATE.snapshot()



def auto_path_loop() -> None:

    last_command = (0, 0)

    while True:

        state = AUTO_PATH_STATE.snapshot()

        interval = max(0.06, int(state.get("interval_ms") or AUTO_PATH_INTERVAL_MS) / 1000.0)

        if not state.get("enabled"):

            last_command = (0, 0)

            time.sleep(interval)

            continue

        with REMOTE_FORWARD_LOCK:

            remote_forward_enabled = LOCAL_REMOTE_FORWARD_ENABLED

        if remote_forward_enabled:

            AUTO_PATH_STATE.update(status="实体遥控器转发已开启，自动路径暂停", last_error="remote forward enabled")

            time.sleep(interval)

            continue

        plan = state.get("plan") or {}

        plan_age_ms = state.get("plan_age_ms")

        if not plan.get("ok") or plan_age_ms is None or plan_age_ms > int(state.get("stale_ms") or AUTO_PATH_STALE_MS):

            throttle = steering = 0

            if last_command != (0, 0):

                try:

                    request_esp32_quick("/drive?throttle=0&steering=0", method="POST", timeout=0.35)

                    AUTO_PATH_STATE.mark_sent(0, 0)

                except Exception as exc:

                    AUTO_PATH_STATE.mark_sent(0, 0, str(exc))

                last_command = (0, 0)

            if plan_age_ms is not None and plan_age_ms > int(state.get("stale_ms") or AUTO_PATH_STALE_MS):

                AUTO_PATH_STATE.update(status="识别结果超时，自动路径停车")

            time.sleep(interval)

            continue

        throttle = int(plan.get("throttle", 0))

        steering = int(plan.get("steering", 0))

        try:

            request_esp32_quick(f"/drive?{urlencode({'throttle': throttle, 'steering': steering})}", method="POST", timeout=0.35)

            AUTO_PATH_STATE.mark_sent(throttle, steering)

            last_command = (throttle, steering)

        except Exception as exc:

            AUTO_PATH_STATE.mark_sent(throttle, steering, str(exc))

        time.sleep(interval)





def detect_frame(frame: bytes) -> None:

    model = load_yolo_model()

    cv2, np = _YOLO_IMPORTS["cv2"], _YOLO_IMPORTS["np"]

    image = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_COLOR)

    if image is None:

        raise ValueError("failed to decode JPEG frame")

    with YOLO_STATE.lock:

        target_classes = YOLO_STATE.target_classes

        target_class_ids = list(YOLO_STATE.target_class_ids) if YOLO_STATE.target_class_ids is not None else None

    apply_yoloe_classes(target_classes)

    started = time.perf_counter()

    kwargs = {"imgsz": YOLO_IMGSZ, "conf": YOLO_CONF, "device": yolo_device(), "verbose": False}

    if not IS_YOLOE:

        kwargs["classes"] = target_class_ids

    result = model.predict(image, **kwargs)[0]

    inference_ms = int((time.perf_counter() - started) * 1000)

    name_items = result.names.items() if hasattr(result.names, "items") else enumerate(result.names)

    id_to_name = {int(class_id): str(name) for class_id, name in name_items}

    objects, boxes = [], []

    if result.boxes is not None:

        mask_polygons = result.masks.xy if result.masks is not None and result.masks.xy is not None else []

        for index, (xyxy, cls_id, conf) in enumerate(zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist(), result.boxes.conf.tolist())):

            name = id_to_name.get(int(cls_id), str(int(cls_id)))

            label = f"{name} {conf:.2f}"

            polygon = mask_polygons[index].tolist() if index < len(mask_polygons) else []

            objects.append(label)

            center = [(float(xyxy[0]) + float(xyxy[2])) / 2.0, (float(xyxy[1]) + float(xyxy[3])) / 2.0]

            boxes.append({"xyxy": [float(v) for v in xyxy], "label": label, "name": str(name), "conf": float(conf), "center": center, "polygon": [[float(x), float(y)] for x, y in polygon], "color_index": index})

    height, width = image.shape[:2]

    YOLO_STATE.update_detection(inference_ms, objects[:20], boxes, (width, height))

    update_auto_path_plan(boxes, (width, height))





def detector_loop() -> None:

    last_seen = 0

    while True:

        frame_no, frame = YOLO_CAMERA_STATE.wait_frame(last_seen, timeout=1.0)

        if frame_no == last_seen or not frame:

            continue

        last_seen = frame_no

        if frame_no % DETECT_EVERY_N_FRAMES:

            continue

        try:

            detect_frame(frame)

        except Exception as exc:

            YOLO_STATE.update(last_error=str(exc), status=f"detection error: {exc}")

            time.sleep(0.2)





def request_json(base_url: str, path: str, method: str = "GET", timeout: float = 2.0) -> dict:

    request = Request(f"{base_url}{path}", data=(b"" if method == "POST" else None), method=method)

    with urlopen(request, timeout=timeout) as response:

        return json.loads(response.read().decode("utf-8"))





def request_esp32(path: str, method: str = "GET") -> dict:

    try:

        return request_json(ESP32_BASE_URL, path, method, 2.0)

    except HTTPError as error:

        detail = error.read().decode("utf-8", errors="replace")

        raise RuntimeError(f"ESP32 返回错误 {error.code}: {detail}") from error

    except URLError as error:

        raise RuntimeError(f"连接不到 ESP32：{error.reason}") from error

    except TimeoutError as error:

        raise RuntimeError("连接 ESP32 超时") from error


def request_esp32_quick(path: str, method: str = "GET", timeout: float = 0.45) -> dict:

    return request_json(ESP32_BASE_URL, path, method, timeout)


def clamp_drive(value: float) -> int:

    return max(-100, min(100, int(round(value))))


def normalize_voice_text(text: str) -> str:

    return re.sub(r"[\s，。！？、,.!?；;：:]+", "", text.strip().lower())


def voice_speed_from_text(text: str, speed_value=None) -> int:

    if isinstance(speed_value, (int, float)):

        return max(1, min(100, int(round(speed_value))))

    speed_text = str(speed_value or "").strip().lower()

    if speed_text in {"slow", "low", "慢", "慢速", "低速"}:

        return max(1, min(100, VOICE_DRIVE_SLOW_SPEED))

    normalized = normalize_voice_text(text)

    if any(word in normalized for word in ("慢", "慢点", "慢速", "低速", "小点")):

        return max(1, min(100, VOICE_DRIVE_SLOW_SPEED))

    return max(1, min(100, VOICE_DRIVE_FULL_SPEED))


def parse_voice_drive_command(text: str, speed_value=None) -> Optional[dict]:

    normalized = normalize_voice_text(text)

    if not normalized:

        return None

    if any(word in normalized for word in VOICE_STOP_WORDS):

        return {"action": "stop", "keyword": "stop", "throttle": 0, "steering": 0, "speed": 0}

    speed = voice_speed_from_text(text, speed_value)

    for key, words, forward, left in VOICE_DIRECTION_KEYWORDS:

        if not any(word in normalized for word in words):

            continue

        if forward and left:

            component = speed / math.sqrt(2.0)

            throttle = clamp_drive(forward * component)

            steering = clamp_drive(left * component)

        else:

            throttle = clamp_drive(forward * speed)

            steering = clamp_drive(left * speed)

        return {

            "action": "drive",

            "keyword": key,

            "throttle": throttle,

            "steering": steering,

            "speed": speed,

        }

    return None


def voice_drive_snapshot() -> dict:

    now = time.time()

    with VOICE_DRIVE_LOCK:

        state = dict(VOICE_DRIVE_STATE)

    state["age_ms"] = int((now - state["last_update"]) * 1000) if state["last_update"] else None

    state["expires_in_ms"] = max(0, int((state["expires_at"] - now) * 1000)) if state["expires_at"] else 0

    state["keepalive_ms"] = VOICE_DRIVE_KEEPALIVE_MS

    state["default_hold_ms"] = VOICE_DRIVE_DEFAULT_HOLD_MS

    return state


def stop_voice_drive(reason: str = "stop") -> dict:

    now = time.time()

    with VOICE_DRIVE_LOCK:

        VOICE_DRIVE_STATE.update(

            active=False,

            throttle=0,

            steering=0,

            keyword=reason,

            last_update=now,

            expires_at=0.0,

        )

    try:

        response = request_esp32_quick("/drive?throttle=0&steering=0", method="POST", timeout=0.5)

        with VOICE_DRIVE_LOCK:

            VOICE_DRIVE_STATE["last_error"] = ""

            VOICE_DRIVE_STATE["last_sent"] = now

        return response

    except Exception as exc:

        with VOICE_DRIVE_LOCK:

            VOICE_DRIVE_STATE["last_error"] = str(exc)

        raise


def apply_voice_drive_command(text: str, speed_value=None, hold_ms=None, source: str = "voice") -> dict:

    parsed = parse_voice_drive_command(text, speed_value)

    if parsed is None:

        raise ValueError("没有识别到方向词")

    if parsed["action"] == "stop":

        stop_voice_drive("stop")

        out = voice_drive_snapshot()

        out.update({"ok": True, "matched": parsed, "transcript": text})

        return out

    hold = VOICE_DRIVE_DEFAULT_HOLD_MS if hold_ms is None else int(hold_ms)

    hold = max(250, min(10000, hold))

    throttle = parsed["throttle"]

    steering = parsed["steering"]

    now = time.time()

    try:

        request_esp32_quick("/config?assistMode=2&headingHold=1", method="POST", timeout=0.6)

        response = request_esp32_quick(

            f"/drive?{urlencode({'throttle': throttle, 'steering': steering})}",

            method="POST",

            timeout=0.45,

        )

        with VOICE_DRIVE_LOCK:

            VOICE_DRIVE_STATE.update(

                active=True,

                throttle=throttle,

                steering=steering,

                keyword=parsed["keyword"],

                transcript=text,

                source=source,

                speed=parsed["speed"],

                last_update=now,

                last_sent=now,

                expires_at=now + hold / 1000.0,

                last_error="",

                command_count=int(VOICE_DRIVE_STATE.get("command_count", 0)) + 1,

            )

        out = voice_drive_snapshot()

        out.update({"ok": True, "matched": parsed, "robot": response})

        return out

    except Exception as exc:

        with VOICE_DRIVE_LOCK:

            VOICE_DRIVE_STATE.update(last_error=str(exc), last_update=now)

        raise


def voice_drive_keepalive_loop() -> None:

    interval = max(40, VOICE_DRIVE_KEEPALIVE_MS) / 1000.0

    while True:

        time.sleep(interval)

        now = time.time()

        with VOICE_DRIVE_LOCK:

            active = bool(VOICE_DRIVE_STATE.get("active"))

            throttle = int(VOICE_DRIVE_STATE.get("throttle", 0))

            steering = int(VOICE_DRIVE_STATE.get("steering", 0))

            expires_at = float(VOICE_DRIVE_STATE.get("expires_at", 0.0) or 0.0)

            last_sent = float(VOICE_DRIVE_STATE.get("last_sent", 0.0) or 0.0)

        if not active:

            continue

        if expires_at and now >= expires_at:

            try:

                stop_voice_drive("timeout")

            except Exception:

                pass

            continue

        if now - last_sent < interval * 0.8:

            continue

        try:

            request_esp32_quick(

                f"/drive?{urlencode({'throttle': throttle, 'steering': steering})}",

                method="POST",

                timeout=0.35,

            )

            with VOICE_DRIVE_LOCK:

                VOICE_DRIVE_STATE["last_sent"] = time.time()

                VOICE_DRIVE_STATE["last_error"] = ""

        except Exception as exc:

            with VOICE_DRIVE_LOCK:

                VOICE_DRIVE_STATE["last_error"] = str(exc)


def free_voice_listen_port(port: int) -> None:

    """启动前释放被旧语音进程占用的端口，避免单独跑过 voice_drive_realtime 后冲突。"""

    if os.environ.get("VOICE_WS_AUTO_FREE_PORT", "1") == "0":

        return

    try:

        import subprocess

        result = subprocess.run(

            ["lsof", "-ti", f":{port}"],

            capture_output=True,

            text=True,

            timeout=2,

            check=False,

        )

        my_pid = os.getpid()

        for raw_pid in result.stdout.splitlines():

            pid = raw_pid.strip()

            if not pid or not pid.isdigit():

                continue

            if int(pid) == my_pid:

                continue

            print(f"[voice] 结束占用 {port} 的旧进程 PID={pid}", flush=True)

            os.kill(int(pid), 15)

        time.sleep(0.4)

    except FileNotFoundError:

        return

    except Exception as exc:

        print(f"[voice] 自动释放端口 {port} 失败：{exc}", flush=True)


def voice_realtime_server_loop() -> None:

    if not VOICE_WS_ENABLED:

        print("[voice] 内置语音服务已关闭：VOICE_WS_ENABLED=0", flush=True)

        return

    try:

        import voice_drive_realtime

        # all_in_one 可能通过 PORT 改端口；内置启动时让语音服务回调当前 WebUI。
        voice_drive_realtime.WEBUI_URL = os.environ.get("VOICE_WEBUI_URL", f"http://127.0.0.1:{PORT}").rstrip("/")

        free_voice_listen_port(voice_drive_realtime.PORT)

        import asyncio

        asyncio.run(voice_drive_realtime.main())

    except ModuleNotFoundError as exc:

        if exc.name == "websockets":

            print("[voice] 缺少 websockets：请运行 pip install -r voice_requirements.txt", flush=True)

        else:

            print(f"[voice] 语音模块导入失败：{exc}", flush=True)

    except OSError as exc:

        print(f"[voice] 语音 WebSocket 启动失败：{exc}", flush=True)

    except Exception as exc:

        print(f"[voice] 语音服务异常退出：{exc}", flush=True)


def remote_drive_for_webui(status: dict) -> Tuple[int, int]:

    throttle = int(round(float(status.get("throttle", 0))))

    steering = int(round(float(status.get("steering", 0))))

    if str(status.get("driveMapping", "")).lower() != "webui":

        steering *= REMOTE_STEERING_SIGN

    throttle = max(-100, min(100, throttle))

    steering = max(-100, min(100, steering))

    return throttle, steering





def set_local_remote_forward(enabled: bool) -> dict:

    global LOCAL_REMOTE_FORWARD_ENABLED, LOCAL_REMOTE_FORWARD_ERROR, LOCAL_REMOTE_ASSIST_MODE

    with REMOTE_FORWARD_LOCK:

        LOCAL_REMOTE_FORWARD_ENABLED = False

        LOCAL_REMOTE_FORWARD_ERROR = ""

    try:

        if enabled:

            request_esp32("/drive?throttle=0&steering=0", method="POST")

        request_json(REMOTE_ESP32_URL, f"/forward?enabled={1 if enabled else 0}", method="POST", timeout=0.8)

    except Exception as exc:

        with REMOTE_FORWARD_LOCK:

            LOCAL_REMOTE_FORWARD_ERROR = str(exc)

    try:

        remote_status = request_json(REMOTE_ESP32_URL, "/status", method="GET", timeout=0.8)

        LOCAL_REMOTE_ASSIST_MODE = int(remote_status.get("remoteAssistMode", LOCAL_REMOTE_ASSIST_MODE))

    except Exception:

        pass

    return get_remote_status_for_ui()


def get_remote_status_for_ui() -> dict:

    try:

        status = request_json(REMOTE_ESP32_URL, "/status", method="GET", timeout=0.8)

    except Exception as exc:

        with REMOTE_FORWARD_LOCK:

            enabled = LOCAL_REMOTE_FORWARD_ENABLED

            error = LOCAL_REMOTE_FORWARD_ERROR or str(exc)

        return {"error": str(exc), "robotForwardEnabled": enabled, "robotReachable": False, "lastRobotError": error}

    with REMOTE_FORWARD_LOCK:

        local_enabled = LOCAL_REMOTE_FORWARD_ENABLED

        error = LOCAL_REMOTE_FORWARD_ERROR

        last_sent = LOCAL_REMOTE_FORWARD_LAST_SENT

    status["allInOneForward"] = local_enabled

    status["allInOneForwardAgeMs"] = int((time.time() - last_sent) * 1000) if last_sent else None

    status["swAction"] = REMOTE_BUTTON_LAST_ACTION

    status["localAssistMode"] = status.get("remoteAssistMode", LOCAL_REMOTE_ASSIST_MODE)

    status["remoteSteeringSign"] = REMOTE_STEERING_SIGN

    if error:

        status["lastRobotError"] = error

    try:

        _, effective_steering = remote_drive_for_webui(status)

        status["effectiveSteering"] = effective_steering

    except Exception:

        status["effectiveSteering"] = None

    return status


def apply_remote_sw_single_click() -> None:

    global REMOTE_BUTTON_LAST_ACTION, LOCAL_REMOTE_FORWARD_ERROR, LOCAL_REMOTE_ASSIST_MODE

    try:

        request_json(REMOTE_ESP32_URL, "/mode?assistMode=2", method="POST", timeout=0.8)

    except Exception:

        pass

    try:

        request_esp32("/config?assistMode=2&headingHold=1&setAbsForward=1", method="POST")

        with REMOTE_FORWARD_LOCK:

            LOCAL_REMOTE_ASSIST_MODE = 2

            REMOTE_BUTTON_LAST_ACTION = "SW 单击：已把当前 IMU 方向设为绝对前方"

            LOCAL_REMOTE_FORWARD_ERROR = ""

    except Exception as exc:

        with REMOTE_FORWARD_LOCK:

            REMOTE_BUTTON_LAST_ACTION = f"SW 单击失败：{exc}"

            LOCAL_REMOTE_FORWARD_ERROR = str(exc)


def apply_remote_sw_long_press() -> None:

    global REMOTE_BUTTON_LAST_ACTION, LOCAL_REMOTE_FORWARD_ERROR, LOCAL_REMOTE_ASSIST_MODE

    current_mode = LOCAL_REMOTE_ASSIST_MODE

    try:

        robot_status = request_esp32("/status", method="GET")

        current_mode = int(robot_status.get("control", {}).get("assistMode", current_mode))

    except Exception:

        pass

    if current_mode != 2:

        return

    try:

        request_esp32("/config?assistMode=2&headingHold=1&setAbsForward=1", method="POST")

        with REMOTE_FORWARD_LOCK:

            LOCAL_REMOTE_ASSIST_MODE = 2

            REMOTE_BUTTON_LAST_ACTION = "SW 长按：已把当前 IMU 方向设为绝对前方"

            LOCAL_REMOTE_FORWARD_ERROR = ""

    except Exception as exc:

        with REMOTE_FORWARD_LOCK:

            REMOTE_BUTTON_LAST_ACTION = f"SW 长按失败：{exc}"

            LOCAL_REMOTE_FORWARD_ERROR = str(exc)


def handle_remote_sw_button(status: dict) -> None:

    global REMOTE_BUTTON_LAST_PRESSED, REMOTE_BUTTON_PRESSED_AT, REMOTE_BUTTON_LONG_PRESS_HANDLED

    pressed = bool(status.get("buttonPressed"))

    now = time.time()

    if pressed and not REMOTE_BUTTON_LAST_PRESSED:

        REMOTE_BUTTON_PRESSED_AT = now

        REMOTE_BUTTON_LONG_PRESS_HANDLED = False

    if pressed and not REMOTE_BUTTON_LONG_PRESS_HANDLED and (now - REMOTE_BUTTON_PRESSED_AT) >= REMOTE_BUTTON_LONG_PRESS_SECONDS:

        REMOTE_BUTTON_LONG_PRESS_HANDLED = True

        apply_remote_sw_long_press()

    if not pressed and REMOTE_BUTTON_LAST_PRESSED and not REMOTE_BUTTON_LONG_PRESS_HANDLED and (now - REMOTE_BUTTON_PRESSED_AT) < REMOTE_BUTTON_LONG_PRESS_SECONDS:

        apply_remote_sw_single_click()

    REMOTE_BUTTON_LAST_PRESSED = pressed


def remote_forward_loop() -> None:

    global LOCAL_REMOTE_FORWARD_ERROR, LOCAL_REMOTE_FORWARD_LAST_SENT

    interval = max(0.04, REMOTE_POLL_MS / 1000.0)

    while True:

        with REMOTE_FORWARD_LOCK:

            enabled = LOCAL_REMOTE_FORWARD_ENABLED

        if not enabled:

            time.sleep(interval)

            continue

        try:

            status = request_json(REMOTE_ESP32_URL, "/status", method="GET", timeout=0.6)

            handle_remote_sw_button(status)

            throttle, steering = remote_drive_for_webui(status)

            request_esp32(f"/drive?throttle={throttle}&steering={steering}", method="POST")

            with REMOTE_FORWARD_LOCK:

                LOCAL_REMOTE_FORWARD_ERROR = ""

                LOCAL_REMOTE_FORWARD_LAST_SENT = time.time()

        except Exception as exc:

            with REMOTE_FORWARD_LOCK:

                LOCAL_REMOTE_FORWARD_ERROR = str(exc)

        time.sleep(interval)


def parse_text_command(text: str):

    normalized = re.sub(r"\s+", "", text.strip().lower())

    if normalized in {"停", "停止", "stop", "s", "全部停止", "全停"}:

        return None, 0

    motor = 1

    match = re.search(r"(?:电机|motor|m)([12])", normalized)

    command_text = normalized

    if match:

        motor = int(match.group(1))

        command_text = normalized[: match.start()] + normalized[match.end() :]

    if "停止" in command_text or command_text in {"停", "stop", "s"}:

        return motor, 0

    if "正转" in command_text or "前进" in command_text or "forward" in command_text:

        sign = 1

    elif "反转" in command_text or "后退" in command_text or "reverse" in command_text or "back" in command_text:

        sign = -1

    else:

        raise ValueError("方向只能是正转、反转或停止")

    percent_match = re.search(r"(\d+(?:\.\d+)?)", command_text)

    if not percent_match:

        raise ValueError("请加占空比，例如：电机2正转80")

    percent = float(percent_match.group(1))

    if not 0 <= percent <= 100:

        raise ValueError("占空比必须是 0 到 100")

    return motor, int(round(sign * percent))





def pwm_command(motor, percent):

    motor = int(motor)

    percent = int(round(float(percent)))

    if motor not in {1, 2}:

        raise ValueError("motor 必须是 1 或 2")

    if not -100 <= percent <= 100:

        raise ValueError("percent 必须是 -100 到 100")

    if percent == 0:

        return request_esp32(f"/stop?{urlencode({'motor': motor})}", method="POST")

    return request_esp32(f"/pwm?{urlencode({'motor': motor, 'value': percent})}", method="POST")





def json_response(handler, code: int, payload: dict) -> None:

    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")

    try:

        handler.send_response(code)

        handler.send_header("Content-Type", "application/json; charset=utf-8")

        handler.send_header("Content-Length", str(len(body)))

        handler.end_headers()

        handler.wfile.write(body)

    except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):

        return





def read_json(handler) -> dict:

    length = int(handler.headers.get("Content-Length", "0") or "0")

    return json.loads(handler.rfile.read(length).decode("utf-8")) if length else {}




def _imu_vec3(value, keys=("x", "y", "z")):

    if not isinstance(value, dict):

        return None

    out = [value.get(k) for k in keys]

    return out if all(isinstance(v, (int, float)) for v in out) else None




def ingest_bno080_sample(sample: dict) -> None:
    """接收小车 ESP32 上报的 BNO080 姿态 JSON（HTTP POST /imu），填充 IMU_STATE 供可视化使用。"""

    euler = sample.get("euler_deg") or {}

    roll, pitch, yaw = euler.get("roll"), euler.get("pitch"), euler.get("yaw")

    euler_list = [roll, pitch, yaw] if all(isinstance(v, (int, float)) for v in (roll, pitch, yaw)) else None

    quaternion = None

    quat = sample.get("quat") or {}

    if quat:

        q = [quat.get("real"), quat.get("i"), quat.get("j"), quat.get("k")]

        if all(isinstance(v, (int, float)) for v in q):

            quaternion = normalize_quaternion(q)

    now = time.time()

    IMU_STATE.update(

        euler_deg=euler_list,

        quaternion=quaternion,

        accel=_imu_vec3(sample.get("accel_ms2")),

        gyro=_imu_vec3(sample.get("gyro_dps")) or _imu_vec3(sample.get("gyro_rads")),

        mag_field=_imu_vec3(sample.get("mag_ut")),

        frame_no=int(sample.get("seq", 0) or 0),

        last_update=now,

        last_raw_update=now,

        status=f"BNO080 {sample.get('i2c_address', '?')} via HTTP",

    )

    with IMU_STATE.lock:

        IMU_STATE.raw_packets += 1

        IMU_STATE.raw_groups += 1

        IMU_STATE.raw_bytes += len(json.dumps(sample))





def send_camera_mjpeg(handler, state: CameraState = ROBOT_CAMERA_STATE) -> None:

    boundary = "frame"

    try:

        handler.send_response(200)

        handler.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")

        handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")

        handler.send_header("Pragma", "no-cache")

        handler.end_headers()

    except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):

        return

    last_seen = -1

    while True:

        try:

            frame_no, frame = state.wait_frame(last_seen, timeout=5.0)

        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError, SystemError):

            break

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





HTML = r"""<!doctype html>

<html lang="zh-CN">

<head>

<meta charset="utf-8" />

<meta name="viewport" content="width=device-width, initial-scale=1" />

<title>ESP32 三合一控制台</title>

<style>

body{margin:0;padding:22px;background:#10141c;color:#edf4ff;font-family:"Microsoft YaHei",system-ui,sans-serif}main{max-width:1440px;margin:auto;padding:22px;border:1px solid #334155;border-radius:22px;background:#192231}h1{margin:0 0 8px;font-size:34px}h2{margin:0 0 10px}p{color:#9fb0c4;line-height:1.6}.section{margin-top:18px;padding:18px;border:1px solid #334155;border-radius:18px;background:#222d3d}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:18px}.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}.card{min-height:58px;padding:12px;border:1px solid #334155;border-radius:12px;background:rgba(255,255,255,.045)}.label{color:#9fb0c4;font-size:13px}.value{display:block;margin-top:6px;font-size:20px;font-weight:900}.small{color:#9fb0c4;font-size:13px;line-height:1.7;word-break:break-all}.ok{color:#6cff9d}.warn{color:#ffcf5a}.bad{color:#ff7b7b}code{color:#9bdcff}button{border:0;border-radius:12px;padding:12px;color:white;font-weight:800;cursor:pointer;background:#2f80ed}.forward{background:#27ae60}.reverse{background:#2f80ed}.stop,.danger{background:#eb5757}.send{background:#7c3aed}.secondary{background:#334155}input{box-sizing:border-box;width:100%;padding:12px;border:1px solid #3a4a60;border-radius:10px;background:#101722;color:#edf4ff}input[type=range]{padding:0;accent-color:#2f80ed}.command{display:grid;grid-template-columns:1fr auto auto;gap:12px}.motors{display:grid;grid-template-columns:repeat(2,1fr);gap:18px}.buttons,.status{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:12px}.row{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-top:12px}.drive-wrap{display:grid;grid-template-columns:260px 1fr;gap:18px;align-items:center}.joystick{position:relative;width:240px;height:240px;border-radius:50%;background:radial-gradient(circle at 50% 50%,#334155 0 12%,#111827 13% 100%);border:2px solid #475569;touch-action:none;user-select:none}.joystick:before,.joystick:after{content:"";position:absolute;background:#475569}.joystick:before{left:50%;top:12px;width:2px;height:216px}.joystick:after{top:50%;left:12px;height:2px;width:216px}.stick{position:absolute;left:50%;top:50%;width:74px;height:74px;margin:-37px 0 0 -37px;border-radius:50%;background:#2f80ed;box-shadow:0 8px 24px rgba(47,128,237,.45);pointer-events:none}.quick-drive{display:grid;grid-template-columns:repeat(3,84px);grid-template-rows:repeat(3,52px);gap:8px;margin-top:14px}.quick-drive button{padding:8px}.empty{visibility:hidden}.toggle{display:inline-flex;align-items:center;gap:8px;margin:4px 16px 8px 0}.toggle input{width:20px;height:20px}.tune{display:flex;flex-direction:column;gap:6px}.tlabel{display:flex;justify-content:space-between;gap:8px}.numbox{width:74px;padding:4px 6px;text-align:right}.camera-wrap{display:grid;grid-template-columns:1fr 340px;gap:18px}.video-wrap{position:relative;line-height:0}.video-wrap img{display:block;width:100%;max-height:620px;object-fit:contain;border:1px solid #334155;border-radius:16px;background:#05070a}.video-wrap canvas{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}.imu-wrap{display:grid;grid-template-columns:1fr 360px;gap:18px}.imu-plane{height:420px;border:1px solid #334155;border-radius:16px;background:#101820;display:grid;place-items:center;overflow:hidden}.imu-box{width:190px;height:92px;border-radius:18px;background:#42d9ff;box-shadow:0 18px 50px rgba(66,217,255,.25);display:grid;place-items:center;color:#07131c;font-weight:900;transform-style:preserve-3d}.remote-wrap{display:grid;grid-template-columns:300px 1fr;gap:18px}.remote-joy{width:260px;height:260px}.deadzone{position:absolute;left:50%;top:50%;width:36px;height:36px;margin:-18px 0 0 -18px;border-radius:50%;border:1px dashed #64748b;z-index:2}.vector{position:absolute;left:50%;top:50%;height:3px;transform-origin:left center;background:#38bdf8;border-radius:2px;opacity:0;z-index:3}.badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:.75rem;margin-left:8px}.idle{background:#334155;color:#94a3b8}.live{background:#14532d;color:#4ade80}.pre{background:#0f172a;border-radius:10px;padding:12px;overflow:auto;max-height:220px;white-space:pre-wrap}@media(max-width:960px){.grid2,.grid3,.grid4,.motors,.command,.drive-wrap,.camera-wrap,.imu-wrap,.remote-wrap{grid-template-columns:1fr}}

</style>

<style>
.pid-group{border:1px solid #334155;border-radius:14px;padding:12px 14px;margin-top:12px;background:#1c2636}
.assist-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px}
.pid-actions{display:flex;gap:10px;flex-wrap:wrap;margin-top:8px}
.control-lock{margin-top:10px;padding:10px;border:1px solid #8a6a22;border-radius:12px;background:#2b2414;color:#ffcf5a}
.control-locked .web-control button,.control-locked .web-control input{opacity:.45;pointer-events:none}
.camera-duo{display:grid;grid-template-columns:1fr 1fr;gap:18px}
.camera-duo .camera-wrap{grid-template-columns:1fr 320px}
.video-wrap img{cursor:zoom-in}
.camera-modal{position:fixed;inset:0;z-index:9999;display:none;align-items:center;justify-content:center;background:rgba(2,6,23,.86);backdrop-filter:blur(4px);padding:4vh 4vw}
.camera-modal.open{display:flex}
.camera-modal-panel{position:relative;width:80vw;height:80vh;display:flex;flex-direction:column;gap:10px}
.camera-modal-title{color:#edf4ff;font-weight:900;font-size:18px;text-align:center}
.camera-modal img{width:100%;height:100%;object-fit:contain;border:1px solid #64748b;border-radius:18px;background:#05070a;box-shadow:0 24px 80px rgba(0,0,0,.55)}
.camera-modal-close{position:absolute;right:-14px;top:-14px;width:42px;height:42px;border-radius:50%;padding:0;background:#eb5757;font-size:24px;line-height:42px}
.imu-canvas-wrap{position:relative;min-height:520px}
#imuScene{width:100%;height:520px;display:block;border:1px solid #334155;border-radius:16px;background:#101820}
.compass{position:absolute;left:16px;top:16px;width:96px;height:96px;border:1px solid #facc15;border-radius:50%;background:rgba(15,23,42,.82);color:#9fb0c4;font-weight:900;font-size:12px;pointer-events:none}
.compass .tick{position:absolute;inset:0}.compass .tick b{position:absolute;left:50%;transform:translateX(-50%);top:4px}.compass .tick i{position:absolute;left:50%;transform:translateX(-50%);bottom:4px;font-style:normal}.compass .tick em{position:absolute;top:50%;transform:translateY(-50%);font-style:normal}.compass .tick em.w{left:6px}.compass .tick em.e{right:6px}
.compass-needle{position:absolute;inset:0;transition:transform .08s linear}.compass-needle:before{content:"";position:absolute;left:50%;top:8px;margin-left:-6px;border-left:6px solid transparent;border-right:6px solid transparent;border-bottom:34px solid #ef4444}.compass-needle:after{content:"";position:absolute;left:50%;bottom:8px;margin-left:-6px;border-left:6px solid transparent;border-right:6px solid transparent;border-top:34px solid #64748b}
.auto-path-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}
.auto-path-controls{display:flex;gap:10px;flex-wrap:wrap;margin-top:12px}
@media(max-width:960px){.camera-duo,.assist-grid{grid-template-columns:1fr}}
</style>

</head>

<body>

<main>

<h1>ESP32 三合一控制台</h1>

<p>一个页面合并小车 PWM/差速控制、摄像头 YOLO/YOLOE 识别、IMU 姿态、实体摇杆遥控器。摄像头上传地址：<code>ws://__PUBLIC_IP__:__CAMERA_WS_PORT____CAMERA_WS_PATH__</code>。</p>



<section class="section web-control">

<h2>小车控制</h2>

<div id="controlLock" class="control-lock" style="display:none">实体遥控器转发已开启：小车控制由实体手柄独占，网页摇杆和按钮暂时锁定。</div>

<div class="command"><input id="command" placeholder="例如：电机1正转50 / 电机2反转80 / 全部停止" /><button class="send" id="sendCommand">发送</button><button class="stop" id="stopAll">全部停止</button></div>

<div class="grid2">

<div class="section"><h2>赛车式差速控制</h2><div class="drive-wrap"><div><div id="driveJoystick" class="joystick"><div id="driveStick" class="stick"></div></div><div class="quick-drive"><span class="empty"></span><button class="forward" data-drive="100,0">前进</button><span class="empty"></span><button class="reverse" data-drive="100,80">左转</button><button class="stop" data-drive="0,0">停止</button><button class="forward" data-drive="100,-80">右转</button><span class="empty"></span><button class="reverse" data-drive="-100,0">后退</button><span class="empty"></span></div></div><div class="grid4"><div class="card"><span class="label">油门</span><span id="driveThrottle" class="value">0</span></div><div class="card"><span class="label">转向</span><span id="driveSteering" class="value">0</span></div><div class="card"><span class="label">左轮</span><span id="driveLeft" class="value">0</span></div><div class="card"><span class="label">右轮</span><span id="driveRight" class="value">0</span></div></div></div></div>

<div class="section"><h2>语音摇杆控制</h2><p>joystick 麦克风流式上传后，识别到“前进/后退/左/右/左上/右下/停止”等词会转成 IMU 绝对模式摇杆向量。</p><div class="grid4"><div class="card"><span class="label">语音状态</span><span id="voiceActive" class="value warn">待机</span></div><div class="card"><span class="label">方向词</span><span id="voiceKeyword" class="value">-</span></div><div class="card"><span class="label">Throttle / Steering</span><span id="voiceDrive" class="value">0 / 0</span></div><div class="card"><span class="label">剩余保活</span><span id="voiceTtl" class="value">0 ms</span></div></div><div id="voiceTranscript" class="small">等待语音识别服务...</div></div>

<div class="section"><h2>IMU 闭环辅助</h2><label class="toggle"><input type="checkbox" id="headingHold" checked />航向锁定 / 直行纠偏</label><label class="toggle"><input type="checkbox" id="headSign" />纠偏方向反转</label><label class="toggle"><input type="checkbox" id="absMode" />IMU 绝对模式</label><label class="toggle"><input type="checkbox" id="absTurnSign" checked />绝对旋转镜像</label><div class="pid-actions"><button class="secondary" id="setAbsForward">设当前 IMU 方向为绝对前方</button></div><div class="pid-group"><h3>航向 PID / 转向</h3><div class="assist-grid"><label class="tune"><span class="tlabel">航向比例 Kp<input type="number" class="numbox" id="headKpNum" min="0" max="20" step="0.1" value="1.6" /></span><input type="range" id="headKp" min="0" max="6" step="0.1" value="1.6" /></label><label class="tune"><span class="tlabel">航向微分 Kd<input type="number" class="numbox" id="headKdNum" min="0" max="5" step="0.01" value="0.06" /></span><input type="range" id="headKd" min="0" max="1" step="0.01" value="0.06" /></label><label class="tune"><span class="tlabel">转向灵敏度<input type="number" class="numbox" id="steerGainNum" min="0.1" max="1" step="0.05" value="0.45" /></span><input type="range" id="steerGain" min="0.1" max="1" step="0.05" value="0.45" /></label></div></div><div class="pid-group"><h3>绝对模式校正</h3><div class="assist-grid"><label class="tune"><span class="tlabel">绝对比例 Kp<input type="number" class="numbox" id="absHeadKpNum" min="0" max="20" step="0.1" value="2.2" /></span><input type="range" id="absHeadKp" min="0" max="8" step="0.1" value="2.2" /></label><label class="tune"><span class="tlabel">绝对微分 Kd<input type="number" class="numbox" id="absHeadKdNum" min="0" max="5" step="0.01" value="0.08" /></span><input type="range" id="absHeadKd" min="0" max="1" step="0.01" value="0.08" /></label><label class="tune"><span class="tlabel">原地校正上限<input type="number" class="numbox" id="absPivotLimitNum" min="0" max="100" step="1" value="85" /></span><input type="range" id="absPivotLimit" min="0" max="100" step="1" value="85" /></label><label class="tune"><span class="tlabel">开始原地校正角<input type="number" class="numbox" id="absPivotStartDegNum" min="0" max="90" step="1" value="10" /></span><input type="range" id="absPivotStartDeg" min="0" max="90" step="1" value="10" /></label><label class="tune"><span class="tlabel">完全原地校正角<input type="number" class="numbox" id="absPivotFullDegNum" min="1" max="180" step="1" value="45" /></span><input type="range" id="absPivotFullDeg" min="1" max="120" step="1" value="45" /></label></div></div><div class="pid-group"><h3>斜坡 / 看门狗</h3><div class="assist-grid"><label class="tune"><span class="tlabel">加速斜率 %/s<input type="number" class="numbox" id="rampUpNum" min="10" max="2000" step="10" value="450" /></span><input type="range" id="rampUp" min="30" max="800" step="10" value="450" /></label><label class="tune"><span class="tlabel">减速斜率 %/s<input type="number" class="numbox" id="rampDownNum" min="10" max="2000" step="10" value="600" /></span><input type="range" id="rampDown" min="30" max="1200" step="10" value="600" /></label><label class="tune"><span class="tlabel">看门狗超时 ms<input type="number" class="numbox" id="timeoutMsNum" min="100" max="5000" step="50" value="350" /></span><input type="range" id="timeoutMs" min="150" max="1500" step="50" value="350" /></label></div><div class="pid-actions"><button type="button" id="pidReset" class="secondary">恢复默认参数</button></div></div><div class="grid4" style="margin-top:10px"><div class="card"><span class="label">模式</span><span id="ctrlMode" class="value">-</span></div><div class="card"><span class="label">航向纠偏量</span><span id="ctrlSteerCorr" class="value">-</span></div><div class="card"><span class="label">目标航向</span><span id="ctrlSetpoint" class="value">-</span></div><div class="card"><span class="label">当前航向</span><span id="ctrlYaw" class="value">-</span></div><div class="card"><span class="label">航向偏差</span><span id="ctrlErr" class="value">-</span></div><div class="card"><span class="label">当前俯仰</span><span id="ctrlPitch" class="value">-</span></div><div class="card"><span class="label">绝对前方</span><span id="ctrlAbsForward" class="value">-</span></div><div class="card"><span class="label">绝对偏差</span><span id="ctrlAbsErr" class="value">-</span></div></div></div>

</div>

<div class="motors"><article class="section motor" data-motor="1"><h2>电机1</h2><p>D0/D1 编码器，D2/D3 驱动</p><div class="row"><strong>PWM 占空比</strong><strong><span data-role="speedText">50</span>%</strong></div><input data-role="speed" type="range" min="0" max="100" value="50" /><div class="buttons"><button class="forward" data-direction="forward">正转</button><button class="reverse" data-direction="reverse">反转</button><button class="stop" data-direction="stop">停止</button></div><div class="status"><div class="card"><span class="label">方向</span><span class="value" data-role="direction">-</span></div><div class="card"><span class="label">占空比</span><span class="value" data-role="percent">-</span></div><div class="card"><span class="label">PWM</span><span class="value" data-role="pwm">-</span></div><div class="card"><span class="label">实测RPM</span><span class="value" data-role="rpm">-</span></div><div class="card"><span class="label">编码器</span><span class="value" data-role="encoder">-</span></div><div class="card"><span class="label">模式</span><span class="value" data-role="mode">-</span></div></div></article><article class="section motor" data-motor="2"><h2>电机2</h2><p>D4/D5 编码器，D9/D10 驱动</p><div class="row"><strong>PWM 占空比</strong><strong><span data-role="speedText">50</span>%</strong></div><input data-role="speed" type="range" min="0" max="100" value="50" /><div class="buttons"><button class="forward" data-direction="forward">正转</button><button class="reverse" data-direction="reverse">反转</button><button class="stop" data-direction="stop">停止</button></div><div class="status"><div class="card"><span class="label">方向</span><span class="value" data-role="direction">-</span></div><div class="card"><span class="label">占空比</span><span class="value" data-role="percent">-</span></div><div class="card"><span class="label">PWM</span><span class="value" data-role="pwm">-</span></div><div class="card"><span class="label">实测RPM</span><span class="value" data-role="rpm">-</span></div><div class="card"><span class="label">编码器</span><span class="value" data-role="encoder">-</span></div><div class="card"><span class="label">模式</span><span class="value" data-role="mode">-</span></div></div></article></div>

<div id="message" class="small">等待连接 ESP32...</div>

</section>



<section class="section">

<h2>两路摄像头</h2>

<div class="row"><button class="danger" id="robotCameraToggle">暂停小车摄像头传输</button><span id="robotCameraStreamState" class="small">小车摄像头传输状态：读取中...</span></div>

<div class="camera-duo"><div><h2>小车 ESP32 摄像头</h2><p>普通画面，默认接收非 YOLOE 摄像头 IP。</p><div class="camera-wrap"><div class="video-wrap"><img id="robotCameraLive" src="/api/camera.mjpg" alt="robot camera" /></div><aside><div class="card"><span class="label">小车摄像头连接</span><span id="robotCameraStatus" class="value warn">starting</span><div id="robotCameraAge" class="small"></div></div><div class="card"><span class="label">接收帧</span><span id="robotCameraFrames" class="value">-</span></div><div class="card"><span class="label">最近帧大小</span><span id="robotCameraFrameSize" class="value">-</span></div><div class="card"><span class="label">总字节</span><span id="robotCameraBytes" class="value">-</span></div><div class="card"><span class="label">最近错误</span><div id="robotCameraError" class="small">-</div></div></aside></div></div><div><h2>独立 YOLOE 识别摄像头</h2><p>默认只接收 <code>__YOLO_CAMERA_ALLOWED_IP__</code>，不会和小车摄像头混流。</p><div class="camera-wrap"><div class="video-wrap"><img id="cameraLive" src="/api/detected.mjpg" alt="yolo camera" /><canvas id="maskOverlay"></canvas></div><aside><div class="card"><span class="label">YOLOE 摄像头连接</span><span id="cameraStatus" class="value warn">starting</span><div id="cameraAge" class="small"></div></div><div class="card"><span class="label">接收帧 / 识别帧</span><span id="cameraFrames" class="value">-</span></div><div class="card"><span class="label">检测类型</span><input id="targetClasses" placeholder="football, person, sports ball" /><button id="saveClasses">应用检测类型</button></div><div class="card"><span class="label">YOLO 状态</span><div id="yoloStatus" class="small">-</div></div><div class="card"><span class="label">最新识别耗时</span><span id="yoloLatency" class="value">-</span></div><div class="card"><span class="label">最新目标</span><div id="yoloObjects" class="small">-</div></div><div class="card"><span class="label">最近错误</span><div id="cameraError" class="small">-</div></div></aside></div></div></div>

</section>



<section class="section">
<h2>无人机追球路径</h2>
<p>使用 YOLOE 摄像头从高处识别 <code>football</code> 和 <code>football gate</code>，匹配两个框的中心点，连续换算成小球的 throttle / steering 指令。无需改动 stick 或小车固件。</p>
<div class="auto-path-grid">
<div class="card"><span class="label">自动路径</span><span id="autoPathEnabled" class="value warn">关闭</span></div>
<div class="card"><span class="label">方向判断</span><span id="autoPathDirection" class="value">-</span></div>
<div class="card"><span class="label">Throttle / Steering</span><span id="autoPathCommand" class="value">0 / 0</span></div>
<div class="card"><span class="label">中心距离</span><span id="autoPathDistance" class="value">-</span></div>
</div>
<div class="auto-path-controls">
<button id="autoPathToggle" class="secondary">开启自动路径</button>
<button id="autoPathStop" class="danger">停止自动路径并停车</button>
</div>
<div class="grid2">
<div class="card"><span class="label">匹配详情</span><div id="autoPathMatch" class="small">等待识别 football 和 football gate...</div></div>
<div class="card"><span class="label">发送状态</span><div id="autoPathStatus" class="small">-</div></div>
</div>
</section>

<section class="section"><h2>IMU 实体姿态可视化</h2><p>小车 ESP32 通过 HTTP POST <code>/imu</code> 上报 BNO080 姿态（同时兼容旧的 TCP <code>__IMU_TCP_PORT__</code> 原始帧）。</p><div class="imu-wrap"><div class="imu-canvas-wrap"><canvas id="imuScene"></canvas><div class="compass"><div class="tick"><b>N</b><i>S</i><em class="w">W</em><em class="e">E</em></div><div id="compassNeedle" class="compass-needle"></div></div><div id="imuBox" style="display:none"></div></div><aside><div class="card"><span class="label">IMU 连接状态</span><span id="imuStatus" class="value warn">starting</span><div id="imuAge" class="small"></div></div><div class="card"><span class="label">航向角</span><span id="imuHeading" class="value">-</span></div><div class="card"><span class="label">欧拉角 Roll / Pitch / Yaw</span><span id="imuEuler" class="value">-</span></div><div class="card"><span class="label">四元数 w / x / y / z</span><span id="imuQuat" class="value">-</span></div><div class="card"><span class="label">传感器数据</span><div id="imuSensor" class="small">-</div></div><div class="card"><span class="label">数据包</span><div id="imuPacket" class="small">-</div></div><div class="card"><span class="label">显示朝向校准（把红色轴转到车头方向）</span><div style="display:flex;flex-wrap:wrap;gap:6px;margin-top:8px"><button class="secondary" data-imuoff="-90">↺ -90°</button><button class="secondary" data-imuoff="-15">-15°</button><button class="secondary" data-imuoff="-5">-5°</button><button class="secondary" data-imuoff="5">+5°</button><button class="secondary" data-imuoff="15">+15°</button><button class="secondary" data-imuoff="90">↻ +90°</button><button class="secondary" data-imuoff="180">翻转180°</button><button class="danger" id="imuOffReset">归零</button></div><div id="imuOffsetText" class="small" style="margin-top:6px">当前显示偏移：0°</div></div></aside></div></section>



<section class="section"><h2>实体摇杆遥控器 <span id="twinBadge" class="badge idle">待机</span></h2><p>镜像 <code>__REMOTE_ESP32_URL__</code> 的物理摇杆。SW 单击=把当前 IMU 方向设为绝对前方；按住 1 秒以上=重新校准中心。</p><div class="remote-wrap"><div><div id="remoteJoystick" class="joystick remote-joy"><div class="deadzone"></div><div id="remoteVector" class="vector"></div><div id="remoteStick" class="stick"></div></div><div class="row"><button class="danger" id="stopRemote">遥控器停车</button><button class="secondary" id="recalibrate">重新校准中心</button></div></div><div><div class="grid4"><div class="card"><span class="label">物理 X/Y</span><span id="normXY" class="value">0 / 0</span></div><div class="card"><span class="label">驱动 Throttle / Steering</span><span id="remoteDrive" class="value">0 / 0</span></div><div class="card"><span class="label">ADC 原始值</span><span id="rawXY" class="value">-</span></div><div class="card"><span class="label">遥控器 IP</span><span id="remoteIp" class="value">-</span></div><div class="card"><span class="label">小车转发</span><span id="robotForward" class="value warn">关闭</span></div><div class="card"><span class="label">按键 SW</span><span id="buttonPressed" class="value">-</span></div><div class="card"><span class="label">方向旋转</span><span id="rotationDeg" class="value">0°</span></div><div class="card"><span class="label">控制模式</span><span id="assistMode" class="value">相对</span></div></div><div class="row"><button id="refreshRemote">立即刷新</button><button class="secondary" id="toggleForward">开启小车转发</button><button class="danger" id="stopRobot">小车停车</button></div><div class="row"><button class="secondary" data-rotate="0">0°</button><button class="secondary" data-rotate="90">90°</button><button class="secondary" data-rotate="180">180°</button><button class="secondary" data-rotate="270">270°</button><button class="secondary" data-mode="1">相对方向矫正</button><button class="secondary" data-mode="2">绝对方向模式</button><button class="secondary" id="toggleMode">切换模式</button></div><div class="row"><button class="secondary" data-cal-edge="center">记录中心</button><button class="secondary" data-cal-edge="left">左推到底</button><button class="secondary" data-cal-edge="right">右推到底</button><button class="secondary" data-cal-edge="up">上推到底</button><button class="secondary" data-cal-edge="down">下推到底</button><button class="danger" id="resetCal">恢复默认端点</button></div><p id="remoteMessage" class="small warn">正在连接遥控器...</p><div id="robotStatus" class="pre">测试阶段不轮询小车。</div></div></div></section>

</main>

<div id="cameraModal" class="camera-modal" aria-hidden="true"><div class="camera-modal-panel"><button id="cameraModalClose" class="camera-modal-close" type="button">×</button><div id="cameraModalTitle" class="camera-modal-title">摄像头预览</div><img id="cameraModalImage" alt="camera preview" /></div></div>

<script>

const ESP32_BASE_URL="__ESP32_BASE_URL__",POLL_MS=__REMOTE_POLL_MS__,ROBOT_POLL_ENABLED=__ROBOT_POLL_ENABLED__,MASK_ALPHA=__MASK_ALPHA__;

const msg=document.querySelector("#message"),command=document.querySelector("#command"),running={1:"stop",2:"stop"},timers={},driveState={throttle:0,steering:0,lastSent:0,pending:null,active:false};

let remoteForwardEnabled=false,autoPathEnabled=false;

const MASK_COLORS=[`rgba(0,255,0,${MASK_ALPHA})`,`rgba(255,128,0,${MASK_ALPHA})`,`rgba(0,200,255,${MASK_ALPHA})`,`rgba(255,0,200,${MASK_ALPHA})`,`rgba(255,255,0,${MASK_ALPHA})`];

function text(id,v){const e=document.querySelector("#"+id);if(e)e.textContent=v}

function info(t,e=false){msg.textContent=t;msg.classList.toggle("bad",e)}

function setupCameraModal(){const modal=document.querySelector("#cameraModal"),img=document.querySelector("#cameraModalImage"),title=document.querySelector("#cameraModalTitle"),close=document.querySelector("#cameraModalClose");if(!modal||!img||!title||!close)return;const open=(src,label)=>{img.src=src;title.textContent=label;modal.classList.add("open");modal.setAttribute("aria-hidden","false")};const hide=()=>{modal.classList.remove("open");modal.setAttribute("aria-hidden","true");img.removeAttribute("src")};document.querySelector("#robotCameraLive")?.addEventListener("click",()=>open("/api/camera.mjpg","小车 ESP32 摄像头"));document.querySelector("#cameraLive")?.addEventListener("click",()=>open("/api/detected.mjpg","独立 YOLOE 识别摄像头"));close.addEventListener("click",hide);modal.addEventListener("click",e=>{if(e.target===modal)hide()});window.addEventListener("keydown",e=>{if(e.key==="Escape"&&modal.classList.contains("open"))hide()})}

async function api(url,opt={}){const r=await fetch(url,{headers:{"Content-Type":"application/json"},cache:"no-store",...opt});const p=await r.json();if(!r.ok)throw new Error(p.error||p.detail||"请求失败");return p}

async function esp32Api(path,opt={}){const r=await fetch(ESP32_BASE_URL+path,{cache:"no-store",...opt});const p=await r.json();if(!r.ok)throw new Error(p.error||p.detail||"ESP32请求失败");return p}

function card(id){return document.querySelector(`.motor[data-motor="${id}"]`)}function setv(c,k,v){c.querySelector(`[data-role="${k}"]`).textContent=v}

let configTouched=false;
const sliderCfg={steerGain:{el:"steerGain",d:2,def:0.45},headKp:{el:"headKp",d:2,def:1.6},headKd:{el:"headKd",d:2,def:0.06},absHeadKp:{el:"absHeadKp",d:2,def:6},absHeadKd:{el:"absHeadKd",d:2,def:0.8},absPivotLimit:{el:"absPivotLimit",d:0,def:85},absPivotStartDeg:{el:"absPivotStartDeg",d:0,def:10},absPivotFullDeg:{el:"absPivotFullDeg",d:0,def:45},rampUp:{el:"rampUp",d:0,def:450},rampDown:{el:"rampDown",d:0,def:600},timeoutMs:{el:"timeoutMs",d:0,def:350}};
function setCtl(key,v){const cfg=sliderCfg[key];if(!cfg)return;const sl=document.querySelector("#"+cfg.el),nb=document.querySelector("#"+cfg.el+"Num");if(sl)sl.value=v;if(nb&&document.activeElement!==nb)nb.value=Number(v).toFixed(cfg.d)}
setCtl("absHeadKp",sliderCfg.absHeadKp.def);setCtl("absHeadKd",sliderCfg.absHeadKd.def);
function syncConfigUi(c){const hh=document.querySelector("#headingHold"),hs=document.querySelector("#headSign"),am=document.querySelector("#absMode"),ats=document.querySelector("#absTurnSign");if(hh&&typeof c.headingHold==="boolean")hh.checked=c.headingHold;if(hs&&c.headSign!=null)hs.checked=Number(c.headSign)<0;if(am&&c.assistMode!=null)am.checked=Number(c.assistMode)===2;if(ats&&c.absTurnSign!=null)ats.checked=Number(c.absTurnSign)<0;for(const key in sliderCfg){if(c[key]!=null)setCtl(key,c[key])}}

function render(status){(status.motors||[]).forEach(m=>{const c=card(m.id);if(!c)return;running[m.id]=m.direction||"stop";setv(c,"direction",m.direction||"-");setv(c,"percent",`${Math.abs(m.percent??0)}%`);setv(c,"pwm",m.pwmDuty??"-");setv(c,"rpm",m.rpm??"-");setv(c,"encoder",m.encoderCount??"-");setv(c,"mode",m.mode||"pwm")});if(status.drive){text("driveThrottle",status.drive.throttle??0);text("driveSteering",status.drive.steering??0);text("driveLeft",status.drive.left??0);text("driveRight",status.drive.right??0)}if(status.control){const c=status.control,mode={0:"停止",1:"驾驶",2:"手动"}[c.mode]??c.mode;const assist={1:"相对IMU",2:"绝对IMU"}[c.assistMode]??"相对IMU";text("ctrlMode",c.mode===1?`${mode} / ${assist}`:mode);text("ctrlSteerCorr",`${Number(c.steerCorr??0).toFixed(1)}`);text("ctrlSetpoint",c.headingValid?`${Number(c.headingSetpoint??0).toFixed(1)}°`:"未锁定");text("ctrlYaw",`${Number(c.imuYaw??0).toFixed(1)}°`);let err=Number(c.headingSetpoint??0)-Number(c.imuYaw??0);err=((err+180)%360+360)%360-180;text("ctrlErr",c.headingValid?`${err.toFixed(1)}°`:"-");text("ctrlPitch",`${Number(c.imuPitch??0).toFixed(1)}°`);text("ctrlAbsForward",c.absForwardValid?`${Number(c.absForwardYaw??0).toFixed(1)}°`:"未设定");text("ctrlAbsErr",c.headingValid?`${Number(c.absYawErr??0).toFixed(1)}°`:"-");if(!configTouched)syncConfigUi(c)}info(`ESP32：${status.ip||"未知"}，状态已更新`)}

async function refresh(){try{render(await esp32Api("/status"))}catch(e){info(e.message,true)}}

async function refreshVoice(){try{const s=await api("/api/voice-status");text("voiceActive",s.active?"控制中":"待机");const va=document.querySelector("#voiceActive");if(va)va.className="value "+(s.active?"ok":"warn");text("voiceKeyword",s.keyword||"-");text("voiceDrive",`${s.throttle??0} / ${s.steering??0}`);text("voiceTtl",`${s.expires_in_ms??0} ms`);text("voiceTranscript",(s.transcript||"等待语音识别服务...")+(s.last_error?`；错误：${s.last_error}`:""))}catch(e){text("voiceTranscript","语音状态读取失败："+e.message)}}

async function sendPercent(motor,dir,percent){if(remoteForwardEnabled){info("实体遥控器转发已开启，网页电机按钮已锁定",true);return}if(autoPathEnabled){info("无人机自动路径已开启，网页电机按钮已锁定",true);return}const value=dir==="reverse"?-Number(percent):dir==="stop"?0:Number(percent);render(await esp32Api(value===0?`/stop?motor=${motor}`:`/pwm?motor=${motor}&value=${value}`,{method:"POST"}))}

function sliderChanged(motor,percent){const dir=running[motor];if(!dir||dir==="stop")return;clearTimeout(timers[motor]);timers[motor]=setTimeout(()=>sendPercent(motor,dir,percent).catch(e=>info(e.message,true)),25)}

async function sendDrive(throttle,steering){if(remoteForwardEnabled){driveState.active=false;clearTimeout(driveState.pending);info("实体遥控器转发已开启，网页摇杆已锁定",true);return}if(autoPathEnabled){driveState.active=false;clearTimeout(driveState.pending);info("无人机自动路径已开启，网页摇杆已锁定",true);return}throttle=Math.max(-100,Math.min(100,Math.round(throttle)));steering=Math.max(-100,Math.min(100,Math.round(steering)));driveState.throttle=throttle;driveState.steering=steering;driveState.active=!(throttle===0&&steering===0);const now=Date.now(),run=()=>esp32Api(`/drive?throttle=${throttle}&steering=${steering}`,{method:"POST"}).then(render).catch(e=>info(e.message,true));clearTimeout(driveState.pending);if(now-driveState.lastSent>45){driveState.lastSent=now;run()}else driveState.pending=setTimeout(()=>{driveState.lastSent=Date.now();run()},45-(now-driveState.lastSent))}

setInterval(()=>{if(remoteForwardEnabled||autoPathEnabled){driveState.active=false;return}if(driveState.active)esp32Api(`/drive?throttle=${driveState.throttle}&steering=${driveState.steering}`,{method:"POST"}).catch(()=>{})},120);

function joystickDriveFromOffset(dx,dy,max){return{throttle:-dy/max*100,steering:-dx/max*100}}
function previewDrive(throttle,steering){text("driveThrottle",Math.round(throttle));text("driveSteering",Math.round(steering))}
function setupJoystick(){const joy=document.querySelector("#driveJoystick"),stick=document.querySelector("#driveStick");let active=false;const reset=()=>{active=false;stick.style.transform="translate(0,0)";previewDrive(0,0);sendDrive(0,0)};const move=e=>{if(!active)return;e.preventDefault();const r=joy.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2,max=r.width/2-38;let dx=e.clientX-cx,dy=e.clientY-cy,len=Math.hypot(dx,dy);if(len>max){dx=dx/len*max;dy=dy/len*max}stick.style.transform=`translate(${dx}px,${dy}px)`;const drive=joystickDriveFromOffset(dx,dy,max);previewDrive(drive.throttle,drive.steering);sendDrive(drive.throttle,drive.steering)};joy.addEventListener("pointerdown",e=>{active=true;joy.setPointerCapture(e.pointerId);move(e)});joy.addEventListener("pointermove",move);joy.addEventListener("pointerup",reset);joy.addEventListener("pointercancel",reset)}

document.querySelectorAll("[data-drive]").forEach(b=>{b.addEventListener("pointerdown",()=>{const [t,s]=b.dataset.drive.split(",").map(Number);sendDrive(t,s)});b.addEventListener("pointerup",()=>sendDrive(0,0));b.addEventListener("pointercancel",()=>sendDrive(0,0));b.addEventListener("click",e=>e.preventDefault())});

document.querySelectorAll(".motor").forEach(c=>{const m=Number(c.dataset.motor),speed=c.querySelector('[data-role="speed"]'),label=c.querySelector('[data-role="speedText"]');speed.addEventListener("input",()=>{label.textContent=speed.value;sliderChanged(m,speed.value)});speed.addEventListener("change",()=>sliderChanged(m,speed.value));c.querySelectorAll("button[data-direction]").forEach(b=>b.addEventListener("click",()=>sendPercent(m,b.dataset.direction,b.dataset.direction==="stop"?0:speed.value).catch(e=>info(e.message,true))))});

document.querySelector("#sendCommand").onclick=()=>{if(remoteForwardEnabled){info("实体遥控器转发已开启，网页文字命令已锁定",true);return}if(autoPathEnabled){info("无人机自动路径已开启，网页文字命令已锁定",true);return}api("/api/command",{method:"POST",body:JSON.stringify({command:command.value})}).then(render).catch(e=>info(e.message,true))};document.querySelector("#stopAll").onclick=()=>{api("/api/auto-path-config",{method:"POST",body:JSON.stringify({enabled:false})}).catch(()=>{});if(remoteForwardEnabled){info("实体遥控器转发已开启，请在遥控器区关闭转发或使用小车停车",true);return}esp32Api("/stop",{method:"POST"}).then(render).catch(e=>info(e.message,true))};command.addEventListener("keydown",e=>{if(e.key==="Enter")document.querySelector("#sendCommand").click()});["headingHold","headSign","absMode","absTurnSign"].forEach(id=>document.querySelector("#"+id).addEventListener("change",()=>pushConfig({headingHold:headingHold.checked?1:0,headSign:headSign.checked?-1:1,assistMode:absMode.checked?2:1,absTurnSign:absTurnSign.checked?-1:1})));document.querySelector("#setAbsForward").onclick=()=>pushConfig({setAbsForward:1,assistMode:2});function pushConfig(params){configTouched=true;const qs=Object.entries(params).map(([k,v])=>`${k}=${encodeURIComponent(v)}`).join("&");esp32Api(`/config?${qs}`,{method:"POST"}).then(render).catch(e=>info(e.message,true))}
for(const key in sliderCfg){const cfg=sliderCfg[key],el=document.querySelector("#"+cfg.el),nb=document.querySelector("#"+cfg.el+"Num");if(el){el.addEventListener("input",()=>{if(nb)nb.value=Number(el.value).toFixed(cfg.d);configTouched=true});el.addEventListener("change",()=>pushConfig({[key]:el.value}))}if(nb&&el){const commitNum=()=>{let v=parseFloat(nb.value);if(isNaN(v))return;el.value=v;configTouched=true;pushConfig({[key]:v})};nb.addEventListener("input",()=>{const v=parseFloat(nb.value);if(!isNaN(v)){el.value=v;configTouched=true}});nb.addEventListener("change",commitNum);nb.addEventListener("keydown",e=>{if(e.key==="Enter"){e.preventDefault();commitNum();nb.blur()}})}}
const pidResetBtn=document.querySelector("#pidReset");if(pidResetBtn)pidResetBtn.addEventListener("click",()=>{const defs={headSign:1,absTurnSign:-1,assistMode:1};for(const key in sliderCfg){defs[key]=sliderCfg[key].def;setCtl(key,sliderCfg[key].def)}const hs=document.querySelector("#headSign"),ats=document.querySelector("#absTurnSign"),am=document.querySelector("#absMode");if(hs)hs.checked=false;if(ats)ats.checked=true;if(am)am.checked=false;pushConfig(defs);info("已恢复默认参数")});

setupCameraModal();setupJoystick();refresh();setInterval(refresh,1000);refreshVoice();setInterval(refreshVoice,250);

function drawMasks(data,autoPath={}){const img=document.querySelector("#cameraLive"),canvas=document.querySelector("#maskOverlay"),rect=img.getBoundingClientRect(),dpr=window.devicePixelRatio||1;canvas.width=Math.max(1,Math.round(rect.width*dpr));canvas.height=Math.max(1,Math.round(rect.height*dpr));canvas.style.width=rect.width+"px";canvas.style.height=rect.height+"px";const ctx=canvas.getContext("2d");ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,rect.width,rect.height);if(!data.detection_boxes||!data.detection_source_size||(data.overlay_age_ms??99999)>1000)return;const sx=rect.width/(data.detection_source_size[0]||1),sy=rect.height/(data.detection_source_size[1]||1);data.detection_boxes.forEach((box,i)=>{const p=box.polygon||[],xy=box.xyxy||[],center=box.center||null;ctx.strokeStyle=MASK_COLORS[i%MASK_COLORS.length].replace(/[\d.]+\)$/,"1)");ctx.fillStyle=MASK_COLORS[i%MASK_COLORS.length];ctx.lineWidth=2;if(p.length>=3){ctx.beginPath();p.forEach((pt,j)=>j?ctx.lineTo(pt[0]*sx,pt[1]*sy):ctx.moveTo(pt[0]*sx,pt[1]*sy));ctx.closePath();ctx.fill();ctx.stroke()}else if(xy.length>=4){ctx.strokeRect(xy[0]*sx,xy[1]*sy,(xy[2]-xy[0])*sx,(xy[3]-xy[1])*sy)}const cx=center?center[0]*sx:(xy.length>=4?(xy[0]+xy[2])*0.5*sx:null),cy=center?center[1]*sy:(xy.length>=4?(xy[1]+xy[3])*0.5*sy:null);if(cx!=null&&cy!=null){ctx.beginPath();ctx.arc(cx,cy,4,0,Math.PI*2);ctx.fillStyle="#fff";ctx.fill();ctx.strokeStyle="#111827";ctx.stroke()}const tx=p.length>=3?p[0][0]*sx:(xy[0]||0)*sx,ty=p.length>=3?p[0][1]*sy:(xy[1]||18)*sy;ctx.fillStyle="#e5ff8a";ctx.font="16px Microsoft YaHei";ctx.fillText(box.label||"",tx,Math.max(18,ty-6))});const plan=autoPath.plan;if(plan&&plan.ok&&plan.football&&plan.gate){const b=plan.football.center,g=plan.gate.center;if(b&&g){const bx=b[0]*sx,by=b[1]*sy,gx=g[0]*sx,gy=g[1]*sy;ctx.save();ctx.strokeStyle="#ff4d6d";ctx.fillStyle="#ff4d6d";ctx.lineWidth=4;ctx.beginPath();ctx.moveTo(bx,by);ctx.lineTo(gx,gy);ctx.stroke();const angle=Math.atan2(gy-by,gx-bx);ctx.beginPath();ctx.moveTo(gx,gy);ctx.lineTo(gx-Math.cos(angle-0.45)*16,gy-Math.sin(angle-0.45)*16);ctx.lineTo(gx-Math.cos(angle+0.45)*16,gy-Math.sin(angle+0.45)*16);ctx.closePath();ctx.fill();ctx.fillStyle="#fff";ctx.font="bold 16px Microsoft YaHei";ctx.fillText(plan.direction||"",Math.min(rect.width-180,(bx+gx)/2+8),Math.max(22,(by+gy)/2-8));ctx.restore()}}}

let robotCameraStreamEnabled=true;
function renderRobotCameraControl(camera={}){if(typeof camera.streamEnabled==="boolean")robotCameraStreamEnabled=camera.streamEnabled;const btn=document.querySelector("#robotCameraToggle");if(btn){btn.textContent=robotCameraStreamEnabled?"暂停小车摄像头传输":"开启小车摄像头传输";btn.className=robotCameraStreamEnabled?"danger":"secondary"}text("robotCameraStreamState",robotCameraStreamEnabled?"小车摄像头正在传输，占用 WiFi 带宽":"小车摄像头传输已暂停，优先保证 IMU 和遥控数据")}
async function refreshRobotCameraControl(){try{const s=await robotApi("/status");renderRobotCameraControl(s.camera||{})}catch(e){text("robotCameraStreamState","小车摄像头传输状态读取失败："+e.message)}}
async function setRobotCameraStream(enabled){const btn=document.querySelector("#robotCameraToggle");if(btn)btn.disabled=true;try{const s=await robotApi(`/camera-stream?enabled=${enabled?1:0}`,{method:"POST"});renderRobotCameraControl(s.camera||{});refreshCamera()}catch(e){alert(e.message)}finally{if(btn)btn.disabled=false}}
document.querySelector("#robotCameraToggle").onclick=()=>setRobotCameraStream(!robotCameraStreamEnabled);
refreshRobotCameraControl();setInterval(refreshRobotCameraControl,1500);

function renderAutoPath(s={}){autoPathEnabled=!!s.enabled;const plan=s.plan||{},sent=s.last_sent||{};text("autoPathEnabled",autoPathEnabled?"开启":"关闭");const en=document.querySelector("#autoPathEnabled");if(en)en.className="value "+(autoPathEnabled?"ok":"warn");text("autoPathDirection",plan.ok?(plan.direction||"-"):"-");text("autoPathCommand",`${plan.throttle??0} / ${plan.steering??0}`);text("autoPathDistance",plan.ok?`${plan.distance_px} px`:"-");const btn=document.querySelector("#autoPathToggle");if(btn){btn.textContent=autoPathEnabled?"关闭自动路径":"开启自动路径";btn.className=autoPathEnabled?"danger":"secondary"}const f=plan.football,g=plan.gate;const match=plan.ok&&f&&g?`football 中心：${(f.center||[]).join(", ")}；football gate 中心：${(g.center||[]).join(", ")}；向量：${(plan.vector_px||[]).join(", ")} px；候选：球 ${plan.balls} / 门 ${plan.gates}`:(plan.reason||"等待识别 football 和 football gate...");text("autoPathMatch",match);text("autoPathStatus",`${s.status||"-"}；识别年龄：${s.plan_age_ms??"-"} ms；最近发送：${sent.throttle??0} / ${sent.steering??0}；发送次数：${s.command_count??0}${s.last_error?`；错误：${s.last_error}`:""}`)}
async function setAutoPathEnabled(enabled){try{renderAutoPath(await api("/api/auto-path-config",{method:"POST",body:JSON.stringify({enabled})}));refreshCamera()}catch(e){alert(e.message)}}
document.querySelector("#autoPathToggle").onclick=()=>setAutoPathEnabled(!autoPathEnabled);
document.querySelector("#autoPathStop").onclick=()=>setAutoPathEnabled(false).then(()=>esp32Api("/drive?throttle=0&steering=0",{method:"POST"}).catch(()=>{}));

async function refreshCamera(){try{const robot=await api("/api/camera-state"),c=await api("/api/yolo-camera-state"),y=await api("/api/yolo-state"),auto=await api("/api/auto-path-state");const robotFresh=robot.connected&&((robot.age_ms??99999)<3000);text("robotCameraStatus",robotFresh?"connected":robot.status||"waiting");document.querySelector("#robotCameraStatus").className="value "+(robotFresh?"ok":"warn");text("robotCameraAge",robot.age_ms==null?"尚未收到画面":`距离上帧：${robot.age_ms} ms`);text("robotCameraFrames",robot.frame_count??0);text("robotCameraFrameSize",(robot.frame_size??0)+" bytes");text("robotCameraBytes",robot.byte_count??"-");text("robotCameraError",robot.last_error||"-");const fresh=c.connected&&((c.age_ms??99999)<3000);text("cameraStatus",fresh?"connected":c.status||"waiting");document.querySelector("#cameraStatus").className="value "+(fresh?"ok":"warn");text("cameraAge",c.age_ms==null?"尚未收到画面":`距离上帧：${c.age_ms} ms`);text("cameraFrames",`${c.frame_count??0} / ${y.detect_count??0}`);text("yoloStatus",`${y.status||"-"} · ${y.model} · ${y.device}`);text("yoloLatency",y.inference_ms==null?"-":`${y.inference_ms} ms`);text("yoloObjects",(y.objects||[]).join(", ")||"未识别到目标");text("cameraError",c.last_error||y.last_error||"-");const input=document.querySelector("#targetClasses");if(document.activeElement!==input)input.value=y.target_classes||"";renderAutoPath(auto);drawMasks(y,auto)}catch(e){text("cameraStatus",e.message);document.querySelector("#cameraStatus").className="value bad"}}

document.querySelector("#saveClasses").onclick=()=>api("/api/yolo-config",{method:"POST",body:JSON.stringify({target_classes:document.querySelector("#targetClasses").value.trim()})}).then(refreshCamera).catch(e=>alert(e.message));refreshCamera();setInterval(refreshCamera,400);window.addEventListener("resize",refreshCamera);

function fmtVec(v,u=""){return v&&v.length?v.map(x=>`${Number(x).toFixed(3)}${u}`).join(" / "):"-"}let imuYawOffset=Number(localStorage.getItem("imu_yaw_offset_deg")||"0");if(!Number.isFinite(imuYawOffset))imuYawOffset=0;function normDeg(d){return ((Number(d)%360)+360)%360}function applyYawOffset(yaw){return normDeg(Number(yaw)+imuYawOffset)}function updateImuOffsetText(){const el=document.querySelector("#imuOffsetText");if(el)el.textContent=`当前显示偏移：${normDeg(imuYawOffset).toFixed(0)}°（仅旋转显示，不影响小车纠偏）`}function setImuOffset(v){imuYawOffset=normDeg(v);localStorage.setItem("imu_yaw_offset_deg",String(imuYawOffset));updateImuOffsetText()}document.querySelectorAll("[data-imuoff]").forEach(b=>b.addEventListener("click",()=>setImuOffset(imuYawOffset+Number(b.dataset.imuoff))));const _imuOffReset=document.querySelector("#imuOffReset");if(_imuOffReset)_imuOffReset.addEventListener("click",()=>setImuOffset(0));updateImuOffsetText();function heading(data){const e=data.euler_deg;return e&&e.length>=3?applyYawOffset(Number(e[2])):null}

async function pollImu(){try{const d=await api("/api/imu-state"),fresh=(d.age_ms??99999)<1500&&(d.raw_age_ms??99999)<1500,h=heading(d);text("imuStatus",(d.status||"-")+(fresh?"":"（数据超时）"));document.querySelector("#imuStatus").className="value "+(fresh?"ok":"warn");text("imuAge",d.age_ms==null?"尚未收到姿态数据":`距离上次更新：${d.age_ms} ms`);text("imuHeading",h==null?"-":`${h.toFixed(1)}°`);text("imuEuler",fmtVec(d.euler_deg,"°"));text("imuQuat",fmtVec(d.quaternion));text("imuSensor",`温度：${d.temperature==null?"-":Number(d.temperature).toFixed(2)+" ℃"} | 加速度：${fmtVec(d.accel)} | 角速度：${fmtVec(d.gyro)}`);text("imuPacket",`帧序号：${d.frame_no} | TCP原始字节：${d.raw_bytes} | 缓冲区：${d.buffered_bytes} | 已解析包：${d.raw_packets} | 数据组：${d.raw_groups} | 错误：${d.parse_errors} | 最近错误：${d.last_parse_error||"-"}`);if(d.euler_deg){const [roll,pitch,yaw]=d.euler_deg.map(Number);document.querySelector("#imuBox").style.transform=`perspective(700px) rotateZ(${yaw}deg) rotateX(${pitch}deg) rotateY(${roll}deg)`}}catch(e){text("imuStatus","IMU poll error: "+e.message);document.querySelector("#imuStatus").className="value warn"}}setInterval(pollImu,100);pollImu();

async function remoteApi(path,opt={}){return api("/api/remote"+path,opt)}
async function robotApi(path,opt={}){return api("/api/robot"+path,opt)}
function renderRemote(s){
  const nx=Number(s.normX||0),ny=Number(s.normY||0),t=Number(s.throttle||0),st=Number(s.effectiveSteering??s.steering??0),active=!!s.driveActive,pressed=!!s.buttonPressed;
  remoteForwardEnabled=!!s.robotForwardEnabled;
  document.body.classList.toggle("control-locked",remoteForwardEnabled);
  const lock=document.querySelector("#controlLock");
  if(lock)lock.style.display=remoteForwardEnabled?"block":"none";
  if(remoteForwardEnabled){driveState.active=false;clearTimeout(driveState.pending)}
  const max=88,dx=nx/100*max,dy=-ny/100*max;
  document.querySelector("#remoteStick").style.transform=`translate(${dx}px,${dy}px)`;
  const v=document.querySelector("#remoteVector"),len=Math.hypot(dx,dy);
  if(len>2){v.style.width=`${len}px`;v.style.transform=`rotate(${Math.atan2(dy,dx)*180/Math.PI}deg)`;v.style.opacity="1"}else v.style.opacity="0";
  text("normXY",`${nx.toFixed(1)} / ${ny.toFixed(1)}`);
  text("remoteDrive",`${t} / ${st}`);
  text("rawXY",`${s.rawX??"-"} / ${s.rawY??"-"}`);
  text("remoteIp",s.ip||"-");
  text("buttonPressed",pressed?"按下：松开后设当前绝对前方；按住1秒校准中心":"松开");
  text("rotationDeg",`${s.controlRotationDeg??0}°`);
  text("assistMode",Number(s.localAssistMode??s.remoteAssistMode??1)===2?"绝对":"相对");
  text("robotForward",s.robotForwardEnabled?"开启":"关闭");
  document.querySelector("#robotForward").className="value "+(s.robotForwardEnabled?"ok":"warn");
  const btn=document.querySelector("#toggleForward");
  btn.textContent=s.robotForwardEnabled?"关闭小车转发":"开启小车转发";
  btn.className=s.robotForwardEnabled?"danger":"secondary";
  btn.dataset.enabled=s.robotForwardEnabled?"1":"0";
  const badge=document.querySelector("#twinBadge");
  if(pressed){badge.textContent="SW按下";badge.className="badge stop"}else if(active){badge.textContent="操控中";badge.className="badge live"}else{badge.textContent="待机";badge.className="badge idle"}
  const base=s.robotForwardEnabled?(s.lastRobotError?`小车通信：${s.lastRobotError}`:"遥控器在线，小车转发已开启，网页控制已锁定"):"摇杆测试模式：只显示孪生，不发送小车";
  text("remoteMessage",s.swAction?`${base}；${s.swAction}`:base);
}

let remotePolling=false;async function refreshRemote(){if(remotePolling)return;remotePolling=true;try{renderRemote(await remoteApi("/status"))}catch(e){text("remoteMessage",e.message)}finally{remotePolling=false}}async function refreshRobot(){if(!ROBOT_POLL_ENABLED)return;text("robotStatus",JSON.stringify(await robotApi("/status"),null,2))}

document.querySelector("#refreshRemote").onclick=()=>refreshRemote();document.querySelector("#stopRemote").onclick=()=>remoteApi("/stop",{method:"POST"}).then(refreshRemote).catch(e=>alert(e.message));document.querySelector("#recalibrate").onclick=()=>remoteApi("/recalibrate",{method:"POST"}).then(refreshRemote).catch(e=>alert(e.message));document.querySelector("#stopRobot").onclick=()=>robotApi("/stop",{method:"POST"}).then(refreshRemote).catch(e=>alert(e.message));document.querySelector("#toggleForward").onclick=()=>{const en=document.querySelector("#toggleForward").dataset.enabled==="1";const before=en?Promise.resolve():esp32Api("/drive?throttle=0&steering=0",{method:"POST"}).catch(()=>{});before.then(()=>remoteApi(`/forward?enabled=${en?0:1}`,{method:"POST"})).then(refreshRemote).catch(e=>alert(e.message))};document.querySelector("#toggleMode").onclick=()=>remoteApi("/mode?toggle=1",{method:"POST"}).then(refreshRemote).catch(e=>alert(e.message));document.querySelector("#resetCal").onclick=()=>remoteApi("/calibration-reset",{method:"POST"}).then(refreshRemote).catch(e=>alert(e.message));document.querySelectorAll("[data-cal-edge]").forEach(b=>b.onclick=()=>remoteApi(`/calibrate-edge?edge=${b.dataset.calEdge}`,{method:"POST"}).then(refreshRemote).catch(e=>alert(e.message)));document.querySelectorAll("[data-rotate]").forEach(b=>b.onclick=()=>remoteApi(`/orientation?rotate=${b.dataset.rotate}`,{method:"POST"}).then(refreshRemote).catch(e=>alert(e.message)));document.querySelectorAll("[data-mode]").forEach(b=>b.onclick=()=>remoteApi(`/mode?assistMode=${b.dataset.mode}`,{method:"POST"}).then(refreshRemote).catch(e=>alert(e.message)));refreshRemote();setInterval(refreshRemote,POLL_MS);if(ROBOT_POLL_ENABLED)setInterval(refreshRobot,500);

</script>

<script type="importmap">
  { "imports": { "three": "https://esm.sh/three@0.160.0" } }
</script>
<script type="module">
import * as THREE from "three";
const canvas=document.querySelector("#imuScene");
if(canvas){
  const renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));
  const scene=new THREE.Scene();
  const camera=new THREE.PerspectiveCamera(45,1,0.1,100);
  camera.position.set(4,3,6);
  camera.lookAt(0,0,0);
  scene.add(new THREE.AmbientLight(0xffffff,0.7));
  const light=new THREE.DirectionalLight(0xffffff,1.7);
  light.position.set(3,5,4);
  scene.add(light);
  const grid=new THREE.GridHelper(7,14,0x3b5368,0x233746);
  grid.position.y=-1.4;
  scene.add(grid);
  const body=new THREE.Group();
  scene.add(body);
  function createTextSprite(text,color="#fef08a"){
    const c=document.createElement("canvas");c.width=256;c.height=96;
    const ctx=c.getContext("2d");ctx.font="bold 34px sans-serif";ctx.fillStyle=color;ctx.textAlign="center";ctx.textBaseline="middle";ctx.fillText(text,128,48);
    const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(c),transparent:true,depthWrite:false}));
    sprite.scale.set(1.2,0.45,1);
    return sprite;
  }
  const northGroup=new THREE.Group();
  northGroup.position.set(-3.0,-1.18,-2.4);
  northGroup.add(new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(0,0,0),1.55,0xfacc15,0.28,0.16));
  const northLabel=createTextSprite("航向 0°");
  northLabel.position.set(1.72,0.18,0);
  northGroup.add(northLabel);
  scene.add(northGroup);
  const modelGroup=new THREE.Group();
  modelGroup.rotation.y=Math.PI/2;
  body.add(modelGroup);
  const box=new THREE.Mesh(new THREE.BoxGeometry(2.4,0.45,1.35),new THREE.MeshStandardMaterial({color:0x42d9ff,metalness:0.18,roughness:0.35,emissive:0x062433}));
  modelGroup.add(box);
  const nose=new THREE.Mesh(new THREE.ConeGeometry(0.42,0.9,4),new THREE.MeshStandardMaterial({color:0xffcf5a,metalness:0.1,roughness:0.38}));
  nose.rotation.z=-Math.PI/2;
  nose.position.x=1.65;
  modelGroup.add(nose);
  modelGroup.add(new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(0,0,0),2.2,0x39d0ff,0.32,0.18));
  modelGroup.add(new THREE.ArrowHelper(new THREE.Vector3(0,1,0),new THREE.Vector3(0,0,0),1.5,0x6cff9d,0.28,0.16));
  modelGroup.add(new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(0,0,0),1.7,0xff5b5b,0.28,0.16));
  const compassNeedleEl=document.querySelector("#compassNeedle");
  function resizeImuScene(){
    const w=canvas.clientWidth,h=canvas.clientHeight;
    if(canvas.width!==w||canvas.height!==h){renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}
  }
  function resolveHeading(data){
    const e=data.euler_deg;
    if(!e||e.length<3||!Number.isFinite(Number(e[2])))return null;
    return applyYawOffset(Number(e[2]));
  }
  function applyImuPose(euler,heading){
    if(heading==null&&(!euler||euler.length<3))return;
    const headingDeg=heading??((Number(euler[2])%360)+360)%360;
    const roll=THREE.MathUtils.degToRad(euler&&euler.length>=3?Number(euler[0]):0);
    const pitch=THREE.MathUtils.degToRad(euler&&euler.length>=3?Number(euler[1]):0);
    const headingRad=THREE.MathUtils.degToRad(headingDeg);
    const yawQ=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-headingRad);
    const pitchQ=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,0,1),-pitch);
    const rollQ=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),roll);
    body.quaternion.copy(yawQ).multiply(pitchQ).multiply(rollQ).invert();
  }
  async function pollImu3d(){
    try{
      const r=await fetch("/api/imu-state",{cache:"no-store"});
      const data=await r.json();
      const fresh=(data.age_ms??99999)<1500&&(data.raw_age_ms??99999)<1500;
      const heading=resolveHeading(data);
      if(fresh){applyImuPose(data.euler_deg,heading);if(compassNeedleEl&&heading!=null)compassNeedleEl.style.transform=`rotate(${-heading}deg)`;}
    }catch(e){}
  }
  function animate(){
    resizeImuScene();
    renderer.render(scene,camera);
    requestAnimationFrame(animate);
  }
  setInterval(pollImu3d,100);
  pollImu3d();
  animate();
}
</script>

</body>

</html>"""





def page_html() -> bytes:

    return (

        HTML.replace("__PUBLIC_IP__", PUBLIC_IP)

        .replace("__CAMERA_WS_PORT__", str(CAMERA_WS_PORT))

        .replace("__CAMERA_WS_PATH__", CAMERA_WS_PATH)

        .replace("__YOLO_CAMERA_ALLOWED_IP__", YOLO_CAMERA_ALLOWED_IP or "未设置")

        .replace("__IMU_TCP_PORT__", str(IMU_TCP_PORT))

        .replace("__ESP32_BASE_URL__", ESP32_BASE_URL)

        .replace("__REMOTE_ESP32_URL__", REMOTE_ESP32_URL)

        .replace("__REMOTE_POLL_MS__", str(REMOTE_POLL_MS))

        .replace("__ROBOT_POLL_ENABLED__", "true" if ROBOT_POLL_ENABLED else "false")

        .replace("__MASK_ALPHA__", str(MASK_ALPHA))

        .encode("utf-8")

    )





class Handler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):

        pass



    def do_GET(self):

        parsed = urlparse(self.path)

        if parsed.path == "/":

            body = page_html()

            self.send_response(200)

            self.send_header("Content-Type", "text/html; charset=utf-8")

            self.send_header("Content-Length", str(len(body)))

            self.end_headers()

            self.wfile.write(body)

        elif parsed.path == "/api/status":

            try:

                json_response(self, 200, request_esp32("/status"))

            except Exception as error:

                json_response(self, 502, {"error": str(error), "esp32": ESP32_BASE_URL})

        elif parsed.path == "/api/imu-state":

            json_response(self, 200, IMU_STATE.snapshot())

        elif parsed.path == "/api/camera-state":

            json_response(self, 200, ROBOT_CAMERA_STATE.snapshot())

        elif parsed.path == "/api/yolo-camera-state":

            json_response(self, 200, YOLO_CAMERA_STATE.snapshot())

        elif parsed.path == "/api/yolo-state":

            json_response(self, 200, YOLO_STATE.snapshot())

        elif parsed.path == "/api/auto-path-state":

            json_response(self, 200, AUTO_PATH_STATE.snapshot())

        elif parsed.path == "/api/voice-status":

            json_response(self, 200, voice_drive_snapshot())

        elif parsed.path == "/api/camera.mjpg":

            send_camera_mjpeg(self, ROBOT_CAMERA_STATE)

        elif parsed.path == "/api/detected.mjpg":

            send_camera_mjpeg(self, YOLO_CAMERA_STATE)

        elif parsed.path == "/api/remote/status":

            json_response(self, 200, get_remote_status_for_ui())

        elif parsed.path == "/api/remote/forward":

            enabled = parsed.query.split("enabled=", 1)[1].split("&", 1)[0] if "enabled=" in parsed.query else "0"

            json_response(self, 200, set_local_remote_forward(enabled == "1"))

        elif parsed.path.startswith("/api/remote"):

            self.proxy_device(parsed, REMOTE_ESP32_URL, "遥控器", "GET")

        elif parsed.path.startswith("/api/robot"):

            self.proxy_device(parsed, ROBOT_ESP32_URL, "小车", "GET")

        else:

            json_response(self, 404, {"error": "Not found"})



    def do_POST(self):

        parsed = urlparse(self.path)

        try:

            body = read_json(self)

            if parsed.path == "/api/pwm":

                json_response(self, 200, pwm_command(body.get("motor", 1), body.get("percent", 0)))

            elif parsed.path == "/api/command":

                motor, percent = parse_text_command(str(body.get("command", "")))

                json_response(self, 200, request_esp32("/stop", method="POST") if motor is None else pwm_command(motor, percent))

            elif parsed.path == "/api/stop":

                json_response(self, 200, request_esp32("/stop", method="POST"))

            elif parsed.path == "/api/voice-drive":

                json_response(

                    self,

                    200,

                    apply_voice_drive_command(

                        str(body.get("text", body.get("command", ""))),

                        body.get("speed"),

                        body.get("hold_ms"),

                        str(body.get("source", "api")),

                    ),

                )

            elif parsed.path == "/api/yolo-config":

                target_classes, class_ids = parse_target_classes(str(body.get("target_classes", "")))

                YOLO_STATE.update(target_classes=target_classes, target_class_ids=class_ids, objects=[], detection_boxes=[], detection_source_size=None, status="detection filter updated")

                json_response(self, 200, YOLO_STATE.snapshot())

            elif parsed.path == "/api/auto-path-config":

                json_response(self, 200, apply_auto_path_config(body))

            elif parsed.path == "/api/remote/status":

                json_response(self, 200, get_remote_status_for_ui())

            elif parsed.path == "/api/remote/forward":

                enabled = parsed.query.split("enabled=", 1)[1].split("&", 1)[0] if "enabled=" in parsed.query else "0"

                json_response(self, 200, set_local_remote_forward(enabled == "1"))

            elif parsed.path.startswith("/api/remote"):

                self.proxy_device(parsed, REMOTE_ESP32_URL, "遥控器", "POST")

            elif parsed.path.startswith("/api/robot"):

                self.proxy_device(parsed, ROBOT_ESP32_URL, "小车", "POST")

            elif parsed.path == "/imu":

                ingest_bno080_sample(body)

                json_response(self, 200, {"ok": True})

            else:

                json_response(self, 404, {"error": "Not found"})

        except ValueError as error:

            json_response(self, 400, {"error": str(error)})

        except Exception as error:

            json_response(self, 502, {"error": str(error)})



    def proxy_device(self, parsed, base_url: str, label: str, method: str) -> None:

        prefix = "/api/remote" if parsed.path.startswith("/api/remote") else "/api/robot"

        sub = parsed.path[len(prefix) :] or "/status"

        if parsed.query:

            sub = f"{sub}?{parsed.query}"

        try:

            json_response(self, 200, request_json(base_url, sub, method=method, timeout=0.8))

        except HTTPError as error:

            detail = error.read().decode("utf-8", errors="replace")

            json_response(self, 502, {"error": f"{label} HTTP {error.code}", "detail": detail})

        except URLError as error:

            json_response(self, 502, {"error": f"连接不到{label}：{error.reason}", "url": base_url})

        except Exception as error:

            json_response(self, 502, {"error": str(error), "url": base_url})





class QuietThreadingHTTPServer(ThreadingHTTPServer):

    def handle_error(self, request, client_address):

        exc_type, exc, _ = sys.exc_info()

        if exc_type in {BrokenPipeError, ConnectionAbortedError, ConnectionResetError}:

            return

        if exc_type is OSError and getattr(exc, "winerror", None) in {10053, 10054}:

            return

        if exc_type is SystemError and "unknown opcode" in str(exc):

            print(f"[http] request from {client_address} ended with transient Python runtime error: {exc}", flush=True)

            return

        super().handle_error(request, client_address)


def start_background_services() -> None:

    threading.Thread(target=frame_stream_from_tcp_server, args=(IMU_TCP_HOST, IMU_TCP_PORT, IMU_STATE), daemon=True, name="imu-tcp-server").start()

    threading.Thread(target=camera_websocket_server, args=(CAMERA_WS_HOST, CAMERA_WS_PORT), daemon=True, name="camera-ws-server").start()

    threading.Thread(target=detector_loop, daemon=True, name="yolo-detector").start()

    threading.Thread(target=auto_path_loop, daemon=True, name="auto-path-loop").start()

    threading.Thread(target=remote_forward_loop, daemon=True, name="remote-forward-loop").start()

    threading.Thread(target=voice_drive_keepalive_loop, daemon=True, name="voice-drive-keepalive").start()

    threading.Thread(target=voice_realtime_server_loop, daemon=True, name="voice-realtime-server").start()





def run_http_server() -> None:

    while True:

        try:

            httpd = QuietThreadingHTTPServer((HOST, PORT), Handler)

            httpd.allow_reuse_address = True

            print(f"三合一 WebUI: http://{PUBLIC_IP}:{PORT}", flush=True)

            print(f"本机: http://127.0.0.1:{PORT}", flush=True)

            print(f"小车 ESP32: {ESP32_BASE_URL}", flush=True)

            print(f"遥控器 ESP32: {REMOTE_ESP32_URL}", flush=True)

            print(f"IMU TCP监听: {IMU_TCP_HOST}:{IMU_TCP_PORT}", flush=True)

            print(f"摄像头WS监听: {CAMERA_WS_HOST}:{CAMERA_WS_PORT}{CAMERA_WS_PATH}", flush=True)

            print(f"YOLOE摄像头IP过滤: {YOLO_CAMERA_ALLOWED_IP or '未设置'}", flush=True)

            print(f"小车摄像头IP过滤: {ROBOT_CAMERA_ALLOWED_IP or 'any'}", flush=True)

            httpd.serve_forever()

        except KeyboardInterrupt:

            print("HTTP 服务已停止", flush=True)

            break

        except OSError as exc:

            print(f"[http] 端口 {PORT} 启动失败: {exc}，{SERVER_RESTART_DELAY}s 后重试", flush=True)

            time.sleep(SERVER_RESTART_DELAY)





if __name__ == "__main__":

    threading.excepthook = lambda args: print(f"[thread-error] {args.thread.name}: {args.exc_value}", flush=True)

    start_background_services()

    run_http_server()

