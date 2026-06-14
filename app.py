from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlparse
from urllib.request import Request, urlopen
from dataclasses import dataclass, field
import base64
import hashlib
import json
import math
import os
import re
import socket
import struct
import threading
import time
from typing import Dict, Optional

HOST = "0.0.0.0"
PORT = int(os.environ.get("PORT", "8000"))
PUBLIC_IP = os.environ.get("PUBLIC_IP", "192.168.152.216")
ESP32_BASE_URL = os.environ.get("ESP32_BASE_URL", "http://192.168.152.11")
IMU_TCP_HOST = os.environ.get("IMU_TCP_HOST", "0.0.0.0")
IMU_TCP_PORT = int(os.environ.get("IMU_TCP_PORT", "9000"))
IMU_TCP_STALE_TIMEOUT = float(os.environ.get("IMU_TCP_STALE_TIMEOUT", "30.0"))
CAMERA_WS_PING_INTERVAL = float(os.environ.get("CAMERA_WS_PING_INTERVAL", "10.0"))
SERVER_RESTART_DELAY = float(os.environ.get("SERVER_RESTART_DELAY", "2.0"))
CAMERA_WS_HOST = os.environ.get("CAMERA_WS_HOST", "0.0.0.0")
CAMERA_WS_PORT = int(os.environ.get("CAMERA_WS_PORT", "8081"))
CAMERA_WS_PATH = "/ws/camera"

HEADER = b"\x59\x53"
DATA_ID_TEMP = 0x01
DATA_ID_ACCEL = 0x10
DATA_ID_GYRO = 0x20
DATA_ID_MAG_NORM = 0x30
DATA_ID_MAG_FIELD = 0x31
DATA_ID_EULER = 0x40
DATA_ID_QUATERNION = 0x41


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

    def snapshot(self) -> Dict:
        with self.lock:
            return sanitize_json_value(
                {
                    "frame_no": self.frame_no,
                    "temperature": self.temperature,
                    "accel": self.accel,
                    "gyro": self.gyro,
                    "mag_norm": self.mag_norm,
                    "mag_field": self.mag_field,
                    "euler_deg": self.euler_deg,
                    "quaternion": self.quaternion,
                    "last_update": self.last_update,
                    "last_raw_update": self.last_raw_update,
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


IMU_STATE = ImuState()


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
            }

    def update(self, **kwargs) -> None:
        with self.condition:
            for key, value in kwargs.items():
                setattr(self, key, value)
            self.condition.notify_all()

    def update_frame(self, frame: bytes) -> None:
        with self.condition:
            self.last_frame = frame
            self.frame_count += 1
            self.byte_count += len(frame)
            self.last_update = time.time()
            self.status = "receiving"
            self.condition.notify_all()

    def wait_frame(self, last_seen_count: int, timeout: float = 2.0):
        with self.condition:
            self.condition.wait_for(lambda: self.frame_count != last_seen_count, timeout)
            return self.frame_count, self.last_frame


CAMERA_STATE = CameraState()
_CAMERA_CONN_LOCK = threading.Lock()
_ACTIVE_CAMERA_CONN: Optional[socket.socket] = None


def configure_socket_keepalive(sock: socket.socket, idle: int = 10, interval: int = 3, count: int = 3) -> None:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    for option, value in (
        ("TCP_KEEPIDLE", idle),
        ("TCP_KEEPINTVL", interval),
        ("TCP_KEEPCNT", count),
    ):
        if hasattr(socket, option):
            sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, option), value)


def install_thread_exception_logger() -> None:
    def hook(args) -> None:
        print(f"[thread-error] {args.thread.name}: {args.exc_type.__name__}: {args.exc_value}", flush=True)

    threading.excepthook = hook


def sanitize_json_value(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, list):
        return [sanitize_json_value(item) for item in value]
    if isinstance(value, dict):
        return {key: sanitize_json_value(item) for key, item in value.items()}
    return value


def require_finite(values: list) -> None:
    if any(not math.isfinite(v) for v in values):
        raise ValueError("non-finite float in frame")


def require_range(values: list, limit: float, name: str) -> None:
    if any(abs(v) > limit for v in values):
        raise ValueError(f"{name} out of expected range: {values}")


def require_euler_range(pitch: float, roll: float, yaw: float) -> None:
    if not (-95.0 <= pitch <= 95.0 and -185.0 <= roll <= 185.0 and -365.0 <= yaw <= 365.0):
        raise ValueError(f"euler out of expected range: pitch={pitch}, roll={roll}, yaw={yaw}")


def read_scaled_int32_triplet(data: bytes, scale: float = 0.000001) -> list:
    values = [round(v * scale, 6) for v in struct.unpack("<iii", data)]
    require_finite(values)
    return values


def read_scaled_int32_quad(data: bytes, scale: float = 0.000001) -> list:
    values = [round(v * scale, 6) for v in struct.unpack("<iiii", data)]
    require_finite(values)
    return values


def normalize_quaternion(q: list) -> list:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0:
        return [1.0, 0.0, 0.0, 0.0]
    return [round(v / norm, 6) for v in q]


def euler_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return normalize_quaternion(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def parse_data_group(data_id: int, data: bytes) -> Dict:
    if data_id == DATA_ID_TEMP and len(data) == 2:
        temperature = round(struct.unpack("<h", data)[0] / 100.0, 2)
        if -80.0 <= temperature <= 125.0:
            return {"temperature": temperature}
        raise ValueError(f"temperature out of expected range: {temperature}")

    if data_id == DATA_ID_ACCEL and len(data) == 12:
        accel = read_scaled_int32_triplet(data)
        require_range(accel, 200.0, "accel")
        return {"accel": accel}

    if data_id == DATA_ID_GYRO and len(data) == 12:
        gyro = read_scaled_int32_triplet(data)
        require_range(gyro, 10000.0, "gyro")
        return {"gyro": gyro}

    if data_id == DATA_ID_MAG_NORM and len(data) == 12:
        mag_norm = read_scaled_int32_triplet(data)
        require_range(mag_norm, 1000.0, "mag_norm")
        return {"mag_norm": mag_norm}

    if data_id == DATA_ID_MAG_FIELD and len(data) == 12:
        mag_field = read_scaled_int32_triplet(data, 0.001)
        require_range(mag_field, 8000.0, "mag_field")
        return {"mag_field": mag_field}

    if data_id == DATA_ID_EULER and len(data) == 12:
        pitch, roll, yaw = read_scaled_int32_triplet(data)
        require_euler_range(pitch, roll, yaw)
        euler = [roll, pitch, yaw]
        return {"euler_deg": euler, "quaternion": euler_to_quaternion(*euler)}

    if data_id == DATA_ID_QUATERNION and len(data) == 16:
        q = read_scaled_int32_quad(data)
        require_range(q, 2.0, "quaternion")
        return {"quaternion": normalize_quaternion(q)}

    raise ValueError(f"unsupported data group id=0x{data_id:02x} len={len(data)}")


def parse_payload(payload: bytes) -> Dict:
    result = {}
    index = 0
    while index + 2 <= len(payload):
        data_id = payload[index]
        length = payload[index + 1]
        index += 2
        data = payload[index : index + length]
        if len(data) != length:
            raise ValueError("payload ended before data group was complete")
        index += length
        try:
            result.update(parse_data_group(data_id, data))
        except ValueError:
            continue
    if not result:
        raise ValueError("frame contains no recognized data groups")
    return result


def verify_checksum(frame: bytes, payload_len: int) -> None:
    ck1 = 0
    ck2 = 0
    for byte in frame[2 : 5 + payload_len]:
        ck1 = (ck1 + byte) & 0xFF
        ck2 = (ck2 + ck1) & 0xFF
    expected_ck1 = frame[5 + payload_len]
    expected_ck2 = frame[6 + payload_len]
    if ck1 != expected_ck1 or ck2 != expected_ck2:
        raise ValueError(
            f"checksum mismatch: got {expected_ck1:02x} {expected_ck2:02x}, expected {ck1:02x} {ck2:02x}"
        )


def parse_frame(frame: bytes) -> Dict:
    if len(frame) < 7 or frame[:2] != HEADER:
        raise ValueError("bad frame header")
    frame_no = struct.unpack("<H", frame[2:4])[0]
    payload_len = frame[4]
    expected_len = 2 + 2 + 1 + payload_len + 2
    if len(frame) != expected_len:
        raise ValueError("frame length mismatch")
    verify_checksum(frame, payload_len)
    parsed = parse_payload(frame[5 : 5 + payload_len])
    parsed["frame_no"] = frame_no
    return parsed


def scan_data_groups(buffer: bytearray, state: ImuState) -> None:
    index = 0
    updates = {}
    groups_found = 0
    known_lengths = {
        DATA_ID_TEMP: 2,
        DATA_ID_ACCEL: 12,
        DATA_ID_GYRO: 12,
        DATA_ID_MAG_NORM: 12,
        DATA_ID_MAG_FIELD: 12,
        DATA_ID_EULER: 12,
        DATA_ID_QUATERNION: 16,
    }

    while index + 2 <= len(buffer):
        data_id = buffer[index]
        length = buffer[index + 1]
        expected_length = known_lengths.get(data_id)
        if expected_length is None or length != expected_length:
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
                        print(f"[imu-tcp] connected {address[0]}:{address[1]}", flush=True)
                        while True:
                            try:
                                chunk = client.recv(4096)
                            except socket.timeout:
                                if time.time() - last_chunk_at > IMU_TCP_STALE_TIMEOUT:
                                    state.update(status=f"esp32 data timeout: {address[0]}:{address[1]}")
                                    print(f"[imu-tcp] stale timeout {address[0]}:{address[1]}", flush=True)
                                    break
                                continue
                            except (ConnectionResetError, ConnectionAbortedError, OSError) as exc:
                                state.update(status=f"esp32 socket error: {exc}")
                                print(f"[imu-tcp] socket error {address[0]}:{address[1]}: {exc}", flush=True)
                                break
                            if not chunk:
                                state.update(status=f"esp32 disconnected: {address[0]}:{address[1]}")
                                print(f"[imu-tcp] disconnected {address[0]}:{address[1]}", flush=True)
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
    magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
    digest = hashlib.sha1((client_key + magic).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def perform_websocket_handshake(conn: socket.socket) -> str:
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
        conn.sendall(
            b"HTTP/1.1 404 Not Found\r\n"
            b"Content-Type: text/plain\r\n"
            + f"Content-Length: {len(body)}\r\n\r\n".encode("ascii")
            + body
        )
        raise ValueError(f"unsupported websocket path: {path}")

    response = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {websocket_accept_key(client_key)}\r\n"
        "\r\n"
    )
    conn.sendall(response.encode("ascii"))
    return path


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


def handle_camera_ws_client(conn: socket.socket, address) -> None:
    global _ACTIVE_CAMERA_CONN
    conn.settimeout(CAMERA_WS_PING_INTERVAL)
    is_current_conn = False
    try:
        perform_websocket_handshake(conn)
        with _CAMERA_CONN_LOCK:
            if _ACTIVE_CAMERA_CONN is not None:
                try:
                    _ACTIVE_CAMERA_CONN.close()
                except Exception:
                    pass
            _ACTIVE_CAMERA_CONN = conn
            is_current_conn = True
        configure_socket_keepalive(conn)
        CAMERA_STATE.update(connected=True, status=f"esp32 connected: {address[0]}:{address[1]}", last_error="")
        print(f"[camera-ws] connected {address[0]}:{address[1]}", flush=True)
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
                continue
            if opcode == 0x2 and payload:
                CAMERA_STATE.update_frame(payload)
            elif opcode == 0x1:
                text = payload.decode("utf-8", errors="replace")
                CAMERA_STATE.update(status=f"camera text: {text[:80]}")
    except Exception as exc:
        with _CAMERA_CONN_LOCK:
            is_current_conn = _ACTIVE_CAMERA_CONN is conn
        if is_current_conn:
            CAMERA_STATE.update(last_error=str(exc), status=f"camera websocket error: {exc}")
            print(f"[camera-ws] error {address[0]}:{address[1]}: {exc}", flush=True)
    finally:
        with _CAMERA_CONN_LOCK:
            if _ACTIVE_CAMERA_CONN is conn:
                _ACTIVE_CAMERA_CONN = None
                is_current_conn = True
            else:
                is_current_conn = False
        try:
            conn.close()
        except Exception:
            pass
        if is_current_conn:
            CAMERA_STATE.update(connected=False, status=f"esp32 disconnected: {address[0]}:{address[1]}")
            print(f"[camera-ws] disconnected {address[0]}:{address[1]}", flush=True)


def camera_websocket_server(host: str, port: int) -> None:
    while True:
        try:
            CAMERA_STATE.update(status=f"listening ws://{host}:{port}{CAMERA_WS_PATH}")
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                configure_socket_keepalive(server)
                server.bind((host, port))
                server.listen(8)
                print(f"[camera-ws] listening on {host}:{port}{CAMERA_WS_PATH}", flush=True)
                while True:
                    conn, address = server.accept()
                    threading.Thread(
                        target=handle_camera_ws_client,
                        args=(conn, address),
                        daemon=True,
                        name=f"camera-ws-{address[0]}",
                    ).start()
        except Exception as exc:
            CAMERA_STATE.update(connected=False, status=f"camera websocket server error: {exc}", last_error=str(exc))
            print(f"[camera-ws] server error: {exc}, retry in {SERVER_RESTART_DELAY}s", flush=True)
            time.sleep(SERVER_RESTART_DELAY)

HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ESP32 双电机 PWM 控制</title>
  <style>
    body{margin:0;padding:24px;background:#10141c;color:#edf4ff;font-family:"Microsoft YaHei",system-ui,sans-serif}
    main{max-width:1180px;margin:auto;padding:24px;border:1px solid #334155;border-radius:22px;background:#192231}
    h1{margin:0 0 8px;font-size:36px} h2{margin:0} p{color:#9fb0c4}
    .panel,.motor{margin-top:18px;padding:18px;border:1px solid #334155;border-radius:18px;background:#222d3d}
    .command{display:grid;grid-template-columns:1fr auto auto;gap:12px}
    input[type=text]{width:100%;padding:14px;border:1px solid #334155;border-radius:12px;background:#101722;color:#edf4ff;font-size:18px}
    input[type=range]{width:100%;accent-color:#2f80ed}
    button{border:0;border-radius:12px;padding:14px;color:white;font-size:16px;font-weight:800;cursor:pointer}
    .forward{background:#27ae60}.reverse{background:#2f80ed}.stop{background:#eb5757}.send{background:#7c3aed}
    .motors{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px}
    .row{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-top:14px}
    .buttons,.status{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:12px}
    .card{min-height:72px;padding:12px;border:1px solid #334155;border-radius:12px;background:rgba(255,255,255,.045)}
    .label{color:#9fb0c4;font-size:13px}.value{display:block;margin-top:6px;font-size:20px;font-weight:900}
    #message{min-height:24px;margin-top:16px;color:#9fb0c4}.error{color:#ffaaaa} code{color:#b8d7ff}
    .drive-section{margin-top:18px;padding:18px;border:1px solid #334155;border-radius:18px;background:#222d3d}
    .drive-wrap{display:grid;grid-template-columns:260px minmax(0,1fr);gap:18px;align-items:center}
    .joystick{position:relative;width:240px;height:240px;border-radius:50%;background:radial-gradient(circle at 50% 50%,#334155 0 12%,#111827 13% 100%);border:2px solid #475569;touch-action:none;user-select:none}
    .joystick:before,.joystick:after{content:"";position:absolute;background:#475569}
    .joystick:before{left:50%;top:12px;width:2px;height:216px}
    .joystick:after{top:50%;left:12px;height:2px;width:216px}
    .stick{position:absolute;left:50%;top:50%;width:74px;height:74px;margin:-37px 0 0 -37px;border-radius:50%;background:#2f80ed;box-shadow:0 8px 24px rgba(47,128,237,.45);pointer-events:none}
    .drive-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
    .drive-grid .card{min-height:58px}
    .quick-drive{display:grid;grid-template-columns:repeat(3,84px);grid-template-rows:repeat(3,52px);gap:8px;margin-top:14px}
    .quick-drive button{padding:8px}.quick-drive .empty{visibility:hidden}
    .assist-toggles{display:flex;flex-wrap:wrap;gap:18px;margin:6px 0 14px}
    .toggle{display:flex;align-items:center;gap:8px;font-size:15px;color:#edf4ff;cursor:pointer}
    .toggle input{width:20px;height:20px;accent-color:#27ae60}
    .assist-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px;margin-bottom:14px}
    .tune{display:flex;flex-direction:column;gap:6px;font-size:13px;color:#9fb0c4}
    .tune span{color:#edf4ff;font-weight:800}
    @media(max-width:860px){.assist-grid{grid-template-columns:1fr 1fr}}
    .pid-group{border:1px solid #334155;border-radius:14px;padding:12px 14px;margin-bottom:14px;background:#1c2636}
    .pid-group>h3{margin:0 0 10px;font-size:14px;color:#7fd1ff;letter-spacing:.5px;display:flex;align-items:center;gap:8px}
    .pid-group>h3 .hint{font-size:11px;color:#8595a8;font-weight:500}
    .tlabel{display:flex;justify-content:space-between;align-items:center;gap:8px}
    .numbox{width:74px;padding:4px 6px;border:1px solid #3a4a60;border-radius:8px;background:#0f1722;color:#edf4ff;font-weight:800;font-size:13px;text-align:right}
    .numbox:focus{outline:none;border-color:#27ae60}
    .pid-actions{display:flex;gap:10px;flex-wrap:wrap;margin-top:4px}
    .pid-actions button{padding:8px 14px;border:1px solid #3a4a60;border-radius:10px;background:#26334a;color:#edf4ff;font-weight:700;cursor:pointer}
    .pid-actions button:hover{background:#30425f}
    .camera-section{margin-top:18px;padding:18px;border:1px solid #334155;border-radius:18px;background:#222d3d}
    .camera-wrap{display:grid;grid-template-columns:minmax(0,1fr)320px;gap:18px;align-items:start}
    #cameraLive{width:100%;max-height:520px;object-fit:contain;border:1px solid #334155;border-radius:16px;background:#05070a}
    .imu-section{margin-top:18px;padding:18px;border:1px solid #334155;border-radius:18px;background:#222d3d}
    .imu-wrap{display:grid;grid-template-columns:minmax(0,1fr)340px;gap:18px;min-height:520px}
    .imu-canvas-wrap{position:relative;min-height:520px}
    #imuScene{width:100%;height:520px;display:block;border:1px solid #334155;border-radius:16px;background:#101820}
    .compass{position:absolute;left:16px;top:16px;width:96px;height:96px;border:1px solid #facc15;border-radius:50%;background:rgba(15,23,42,.82);color:#9fb0c4;font-weight:900;font-size:12px;pointer-events:none}
    .compass .tick{position:absolute;inset:0}
    .compass .tick b{position:absolute;left:50%;transform:translateX(-50%);top:4px}
    .compass .tick i{position:absolute;left:50%;transform:translateX(-50%);bottom:4px;font-style:normal}
    .compass .tick em{position:absolute;top:50%;transform:translateY(-50%);font-style:normal}
    .compass .tick em.w{left:6px}.compass .tick em.e{right:6px}
    .compass-needle{position:absolute;inset:0;transition:transform .08s linear}
    .compass-needle:before{content:"";position:absolute;left:50%;top:8px;margin-left:-6px;border-left:6px solid transparent;border-right:6px solid transparent;border-bottom:34px solid #ef4444}
    .compass-needle:after{content:"";position:absolute;left:50%;bottom:8px;margin-left:-6px;border-left:6px solid transparent;border-right:6px solid transparent;border-top:34px solid #64748b}
    .imu-side{display:grid;gap:12px;align-content:start}
    .small{color:#9fb0c4;font-size:13px;line-height:1.7;word-break:break-all}
    .status-ok{color:#6cff9d}.status-warn{color:#ffcf5a}
    @media(max-width:860px){.motors,.command,.drive-wrap,.camera-wrap,.imu-wrap{grid-template-columns:1fr}.status,.drive-grid{grid-template-columns:1fr 1fr}.imu-canvas-wrap{min-height:360px}#imuScene{height:360px}}
  </style>
</head>
<body>
<main>
  <h1>ESP32 双电机 PWM 控制</h1>
  <p>输入 <code>电机1正转50</code>、<code>电机2反转80</code>、<code>全部停止</code>。slider 是占空比百分比，运行时拖动会直接调速。</p>
  <section class="panel">
    <div class="command">
      <input id="command" placeholder="例如：电机1正转50 / 电机2反转80 / 全部停止" />
      <button class="send" id="sendCommand">发送</button>
      <button class="stop" id="stopAll">全部停止</button>
    </div>
  </section>
  <section class="drive-section">
    <h2>赛车式差速控制</h2>
    <p>向上前进、向下后退、左右控制差速；0° 到约 80° 按行进转弯处理，最后接近纯左/纯右的 10° 平滑过渡到左右轮反向原地转。</p>
    <div class="drive-wrap">
      <div>
        <div id="joystick" class="joystick"><div id="stick" class="stick"></div></div>
        <div class="quick-drive">
          <span class="empty"></span><button class="forward" data-drive="100,0">前进</button><span class="empty"></span>
          <button class="reverse" data-drive="100,-80">左转</button><button class="stop" data-drive="0,0">停止</button><button class="forward" data-drive="100,80">右转</button>
          <span class="empty"></span><button class="reverse" data-drive="-100,0">后退</button><span class="empty"></span>
        </div>
      </div>
      <div class="drive-grid">
        <div class="card"><span class="label">油门</span><span id="driveThrottle" class="value">0</span></div>
        <div class="card"><span class="label">转向</span><span id="driveSteering" class="value">0</span></div>
        <div class="card"><span class="label">左轮</span><span id="driveLeft" class="value">0</span></div>
        <div class="card"><span class="label">右轮</span><span id="driveRight" class="value">0</span></div>
      </div>
    </div>
  </section>
  <section class="drive-section">
    <h2>IMU 闭环辅助（航向纠偏）</h2>
    <p>模式1为当前相对航向保持；绝对模式可设定一个 IMU 绝对前方，摇杆前后左右会映射到这个绝对坐标系。</p>
    <div class="assist-toggles">
      <label class="toggle"><input type="checkbox" id="headingHold" checked /> 航向锁定 / 直行纠偏</label>
      <label class="toggle"><input type="checkbox" id="headSign" /> 纠偏方向反转</label>
      <label class="toggle"><input type="checkbox" id="absMode" /> IMU 绝对模式</label>
      <label class="toggle"><input type="checkbox" id="absTurnSign" checked /> 绝对旋转校正镜像</label>
    </div>
    <div class="pid-actions">
      <button type="button" id="setAbsForward">设当前 IMU 方向为绝对前方</button>
    </div>
    <div class="pid-group">
      <h3>航向 PID / 转向 <span class="hint">（直行纠偏与转向手感）</span></h3>
      <div class="assist-grid">
        <label class="tune"><span class="tlabel">航向比例 Kp<input type="number" class="numbox" id="headKpNum" min="0" max="20" step="0.1" value="1.6" /></span><input type="range" id="headKp" min="0" max="6" step="0.1" value="1.6" /></label>
        <label class="tune"><span class="tlabel">航向微分 Kd<input type="number" class="numbox" id="headKdNum" min="0" max="5" step="0.01" value="0.06" /></span><input type="range" id="headKd" min="0" max="1" step="0.01" value="0.06" /></label>
        <label class="tune"><span class="tlabel">转向灵敏度<input type="number" class="numbox" id="steerGainNum" min="0.1" max="1" step="0.05" value="0.45" /></span><input type="range" id="steerGain" min="0.1" max="1" step="0.05" value="0.45" /></label>
      </div>
    </div>
    <div class="pid-group">
      <h3>绝对模式校正 <span class="hint">（大偏差时左右轮反转快速对准目标方向）</span></h3>
      <div class="assist-grid">
        <label class="tune"><span class="tlabel">绝对比例 Kp<input type="number" class="numbox" id="absHeadKpNum" min="0" max="20" step="0.1" value="2.2" /></span><input type="range" id="absHeadKp" min="0" max="8" step="0.1" value="2.2" /></label>
        <label class="tune"><span class="tlabel">绝对微分 Kd<input type="number" class="numbox" id="absHeadKdNum" min="0" max="5" step="0.01" value="0.08" /></span><input type="range" id="absHeadKd" min="0" max="1" step="0.01" value="0.08" /></label>
        <label class="tune"><span class="tlabel">原地校正上限<input type="number" class="numbox" id="absPivotLimitNum" min="0" max="100" step="1" value="85" /></span><input type="range" id="absPivotLimit" min="0" max="100" step="1" value="85" /></label>
        <label class="tune"><span class="tlabel">开始原地校正角<input type="number" class="numbox" id="absPivotStartDegNum" min="0" max="90" step="1" value="10" /></span><input type="range" id="absPivotStartDeg" min="0" max="90" step="1" value="10" /></label>
        <label class="tune"><span class="tlabel">完全原地校正角<input type="number" class="numbox" id="absPivotFullDegNum" min="1" max="180" step="1" value="45" /></span><input type="range" id="absPivotFullDeg" min="1" max="120" step="1" value="45" /></label>
      </div>
    </div>
    <div class="pid-group">
      <h3>斜坡 / 看门狗 <span class="hint">（启停手感与失联保护）</span></h3>
      <div class="assist-grid">
        <label class="tune"><span class="tlabel">加速斜率 %/s<input type="number" class="numbox" id="rampUpNum" min="10" max="2000" step="10" value="450" /></span><input type="range" id="rampUp" min="30" max="800" step="10" value="450" /></label>
        <label class="tune"><span class="tlabel">减速斜率 %/s<input type="number" class="numbox" id="rampDownNum" min="10" max="2000" step="10" value="600" /></span><input type="range" id="rampDown" min="30" max="1200" step="10" value="600" /></label>
        <label class="tune"><span class="tlabel">看门狗超时 ms<input type="number" class="numbox" id="timeoutMsNum" min="100" max="5000" step="50" value="350" /></span><input type="range" id="timeoutMs" min="150" max="1500" step="50" value="350" /></label>
      </div>
      <div class="pid-actions">
        <button type="button" id="pidReset">恢复默认参数</button>
      </div>
    </div>
    <div class="drive-grid">
      <div class="card"><span class="label">模式</span><span id="ctrlMode" class="value">-</span></div>
      <div class="card"><span class="label">航向纠偏量</span><span id="ctrlSteerCorr" class="value">-</span></div>
      <div class="card"><span class="label">目标航向</span><span id="ctrlSetpoint" class="value">-</span></div>
      <div class="card"><span class="label">当前航向</span><span id="ctrlYaw" class="value">-</span></div>
      <div class="card"><span class="label">航向偏差</span><span id="ctrlErr" class="value">-</span></div>
      <div class="card"><span class="label">当前俯仰</span><span id="ctrlPitch" class="value">-</span></div>
      <div class="card"><span class="label">绝对前方</span><span id="ctrlAbsForward" class="value">-</span></div>
      <div class="card"><span class="label">绝对偏差</span><span id="ctrlAbsErr" class="value">-</span></div>
    </div>
  </section>
  <section class="motors">
    <article class="motor" data-motor="1">
      <h2>电机1</h2><p>D0/D1 编码器，D2/D3 驱动</p>
      <div class="row"><strong>PWM 占空比</strong><strong><span data-role="speedText">50</span>%</strong></div>
      <input data-role="speed" type="range" min="0" max="100" value="50" />
      <div class="buttons"><button class="forward" data-direction="forward">正转</button><button class="reverse" data-direction="reverse">反转</button><button class="stop" data-direction="stop">停止</button></div>
      <div class="status"><div class="card"><span class="label">方向</span><span class="value" data-role="direction">-</span></div><div class="card"><span class="label">占空比</span><span class="value" data-role="percent">-</span></div><div class="card"><span class="label">PWM</span><span class="value" data-role="pwm">-</span></div><div class="card"><span class="label">实测RPM</span><span class="value" data-role="rpm">-</span></div><div class="card"><span class="label">编码器</span><span class="value" data-role="encoder">-</span></div><div class="card"><span class="label">模式</span><span class="value" data-role="mode">-</span></div></div>
    </article>
    <article class="motor" data-motor="2">
      <h2>电机2</h2><p>D4/D5 编码器，D9/D10 驱动</p>
      <div class="row"><strong>PWM 占空比</strong><strong><span data-role="speedText">50</span>%</strong></div>
      <input data-role="speed" type="range" min="0" max="100" value="50" />
      <div class="buttons"><button class="forward" data-direction="forward">正转</button><button class="reverse" data-direction="reverse">反转</button><button class="stop" data-direction="stop">停止</button></div>
      <div class="status"><div class="card"><span class="label">方向</span><span class="value" data-role="direction">-</span></div><div class="card"><span class="label">占空比</span><span class="value" data-role="percent">-</span></div><div class="card"><span class="label">PWM</span><span class="value" data-role="pwm">-</span></div><div class="card"><span class="label">实测RPM</span><span class="value" data-role="rpm">-</span></div><div class="card"><span class="label">编码器</span><span class="value" data-role="encoder">-</span></div><div class="card"><span class="label">模式</span><span class="value" data-role="mode">-</span></div></div>
    </article>
  </section>
  <section class="camera-section">
    <h2>Live Camera</h2>
    <p>ESP32 摄像头通过 WebSocket 连接本机 <code>8081/ws/camera</code>，这里用 MJPEG 实时显示最新 JPEG 帧。</p>
    <div class="camera-wrap">
      <img id="cameraLive" src="/api/camera.mjpg" alt="Live camera" />
      <div class="camera-side">
        <div class="card"><span class="label">摄像头连接状态</span><span id="cameraStatus" class="value status-warn">starting</span><div id="cameraAge" class="small"></div></div>
        <div class="card"><span class="label">接收帧</span><span id="cameraFrames" class="value">-</span></div>
        <div class="card"><span class="label">最近帧大小</span><span id="cameraFrameSize" class="value">-</span></div>
        <div class="card"><span class="label">总字节</span><span id="cameraBytes" class="value">-</span></div>
        <div class="card"><span class="label">最近错误</span><div id="cameraError" class="small">-</div></div>
      </div>
    </div>
  </section>
  <section class="imu-section">
    <h2>IMU 实体姿态可视化</h2>
    <p>ESP32 同时把 IMU 原始串口帧转发到本机 TCP <code>9000</code> 端口。IMU 旁有电机磁铁，已改用 VRU 陀螺航向（不用地磁），指北针表示相对水平朝向，长时间会有缓慢漂移。</p>
    <div class="imu-wrap">
      <div class="imu-canvas-wrap">
        <canvas id="imuScene"></canvas>
        <div class="compass"><div class="tick"><b>N</b><i>S</i><em class="w">W</em><em class="e">E</em></div><div id="compassNeedle" class="compass-needle"></div></div>
      </div>
      <div class="imu-side">
        <div class="card"><span class="label">IMU 连接状态</span><span id="imuStatus" class="value status-warn">starting</span><div id="imuAge" class="small"></div></div>
        <div class="card"><span class="label">航向角（陀螺）</span><span id="imuHeading" class="value">-</span></div>
        <div class="card"><span class="label">欧拉角 Roll / Pitch / Yaw</span><span id="imuEuler" class="value">-</span></div>
        <div class="card"><span class="label">四元数 w / x / y / z</span><span id="imuQuat" class="value">-</span></div>
        <div class="card"><span class="label">传感器数据</span><div id="imuSensor" class="small">-</div></div>
        <div class="card"><span class="label">数据包</span><div id="imuPacket" class="small">-</div></div>
      </div>
    </div>
  </section>
  <div id="message">等待连接 ESP32...</div>
</main>
<script type="importmap">
  { "imports": { "three": "https://esm.sh/three@0.160.0" } }
</script>
<script>
const ESP32_BASE_URL="__ESP32_BASE_URL__";
const msg=document.querySelector("#message");
const command=document.querySelector("#command");
const running={1:"stop",2:"stop"};
const timers={};
const driveState={throttle:0,steering:0,lastSent:0,pending:null,active:false};
function info(t,e=false){msg.textContent=t;msg.classList.toggle("error",e)}
async function api(url,opt={}){const r=await fetch(url,{headers:{"Content-Type":"application/json"},...opt});const p=await r.json();if(!r.ok)throw new Error(p.error||p.detail||"请求失败");return p}
async function esp32Api(path,opt={}){const r=await fetch(ESP32_BASE_URL+path,{cache:"no-store",...opt});const p=await r.json();if(!r.ok)throw new Error(p.error||p.detail||"ESP32请求失败");return p}
function card(id){return document.querySelector(`.motor[data-motor="${id}"]`)}
function setv(c,k,v){c.querySelector(`[data-role="${k}"]`).textContent=v}
function render(status){
  (status.motors||[]).forEach(m=>{
    const id=m.id,c=card(id); if(!c)return;
    running[id]=m.direction||"stop";
    setv(c,"direction",m.direction||"-");
    setv(c,"percent",`${Math.abs(m.percent??0)}%`);
    setv(c,"pwm",m.pwmDuty??"-");
    setv(c,"rpm",`${m.rpm??"-"}`);
    setv(c,"encoder",m.encoderCount??"-");
    setv(c,"mode",m.mode||"pwm");
  });
  if(status.drive){
    document.querySelector("#driveThrottle").textContent=status.drive.throttle??0;
    document.querySelector("#driveSteering").textContent=status.drive.steering??0;
    document.querySelector("#driveLeft").textContent=status.drive.left??0;
    document.querySelector("#driveRight").textContent=status.drive.right??0;
  }
  if(status.control){
    const c=status.control;
    const driveModeText={0:"停止",1:"驾驶",2:"手动"}[c.mode]??c.mode;
    const assistText={1:"相对IMU",2:"绝对IMU"}[c.assistMode]??"相对IMU";
    const modeText=c.mode===1?`${driveModeText} / ${assistText}`:driveModeText;
    document.querySelector("#ctrlMode").textContent=modeText;
    document.querySelector("#ctrlSteerCorr").textContent=`${Number(c.steerCorr??0).toFixed(1)}`;
    document.querySelector("#ctrlSetpoint").textContent=c.headingValid?`${Number(c.headingSetpoint??0).toFixed(1)}°`:"未锁定";
    document.querySelector("#ctrlYaw").textContent=`${Number(c.imuYaw??0).toFixed(1)}°`;
    let err=Number(c.headingSetpoint??0)-Number(c.imuYaw??0);
    err=((err+180)%360+360)%360-180;
    document.querySelector("#ctrlErr").textContent=c.headingValid?`${err.toFixed(1)}°`:"-";
    document.querySelector("#ctrlPitch").textContent=`${Number(c.imuPitch??0).toFixed(1)}°`;
    document.querySelector("#ctrlAbsForward").textContent=c.absForwardValid?`${Number(c.absForwardYaw??0).toFixed(1)}°`:"未设定";
    document.querySelector("#ctrlAbsErr").textContent=c.headingValid?`${Number(c.absYawErr??0).toFixed(1)}°`:"-";
    if(!configTouched) syncConfigUi(c);
  }
  info(`ESP32：${status.ip||"未知"}，闭环控制模式`);
}
async function sendPercent(motor,dir,percent){
  const value=dir==="reverse"?-Number(percent):dir==="stop"?0:Number(percent);
  const path=value===0?`/stop?motor=${motor}`:`/pwm?motor=${motor}&value=${value}`;
  render(await esp32Api(path,{method:"POST"}));
}
function sliderChanged(motor,percent){
  const dir=running[motor]; if(!dir||dir==="stop")return;
  clearTimeout(timers[motor]);
  timers[motor]=setTimeout(()=>sendPercent(motor,dir,percent).catch(e=>info(e.message,true)),25);
}
async function refresh(){try{render(await esp32Api("/status"))}catch(e){info(e.message,true)}}

async function sendDrive(throttle,steering){
  throttle=Math.max(-100,Math.min(100,Math.round(throttle)));
  steering=Math.max(-100,Math.min(100,Math.round(steering)));
  driveState.throttle=throttle;
  driveState.steering=steering;
  driveState.active=!(throttle===0&&steering===0);
  const now=Date.now();
  const run=()=>esp32Api(`/drive?throttle=${throttle}&steering=${steering}`,{method:"POST"}).then(render).catch(e=>info(e.message,true));
  clearTimeout(driveState.pending);
  if(now-driveState.lastSent>45){
    driveState.lastSent=now;
    run();
  }else{
    driveState.pending=setTimeout(()=>{driveState.lastSent=Date.now();run();},45-(now-driveState.lastSent));
  }
}
// 保活心跳：按住期间每 120ms 重发当前指令，避免固件看门狗误判松手。
setInterval(()=>{
  if(!driveState.active)return;
  esp32Api(`/drive?throttle=${driveState.throttle}&steering=${driveState.steering}`,{method:"POST"}).catch(()=>{});
},120);

function setupJoystick(){
  const joy=document.querySelector("#joystick"),stick=document.querySelector("#stick");
  if(!joy||!stick)return;
  let active=false;
  const reset=()=>{active=false;stick.style.transform="translate(0px,0px)";sendDrive(0,0)};
  const move=e=>{
    if(!active)return;
    const rect=joy.getBoundingClientRect();
    const cx=rect.left+rect.width/2,cy=rect.top+rect.height/2;
    let dx=e.clientX-cx,dy=e.clientY-cy;
    const max=rect.width/2-38;
    const len=Math.hypot(dx,dy);
    if(len>max){dx=dx/len*max;dy=dy/len*max;}
    stick.style.transform=`translate(${dx}px,${dy}px)`;
    sendDrive(-dy/max*100,dx/max*100);
  };
  joy.addEventListener("pointerdown",e=>{active=true;joy.setPointerCapture(e.pointerId);move(e)});
  joy.addEventListener("pointermove",move);
  joy.addEventListener("pointerup",reset);
  joy.addEventListener("pointercancel",reset);
}

document.querySelectorAll("[data-drive]").forEach(button=>{
  button.addEventListener("pointerdown",()=>{
    const [throttle,steering]=button.dataset.drive.split(",").map(Number);
    sendDrive(throttle,steering);
  });
  button.addEventListener("pointerup",()=>sendDrive(0,0));
  button.addEventListener("pointercancel",()=>sendDrive(0,0));
  button.addEventListener("click",e=>e.preventDefault());
});
setupJoystick();
document.querySelectorAll(".motor").forEach(c=>{
  const motor=Number(c.dataset.motor),speed=c.querySelector('[data-role="speed"]'),text=c.querySelector('[data-role="speedText"]');
  speed.addEventListener("input",()=>{text.textContent=speed.value;sliderChanged(motor,speed.value)});
  speed.addEventListener("change",()=>sliderChanged(motor,speed.value));
  c.querySelectorAll("button[data-direction]").forEach(b=>b.addEventListener("click",()=>sendPercent(motor,b.dataset.direction,b.dataset.direction==="stop"?0:speed.value).catch(e=>info(e.message,true))));
});
document.querySelector("#sendCommand").onclick=()=>api("/api/command",{method:"POST",body:JSON.stringify({command:command.value})}).then(render).catch(e=>info(e.message,true));
document.querySelector("#stopAll").onclick=()=>esp32Api("/stop",{method:"POST"}).then(render).catch(e=>info(e.message,true));
command.addEventListener("keydown",e=>{if(e.key==="Enter")document.querySelector("#sendCommand").click()});

// ---- IMU 闭环辅助：开关与调参，调用 ESP32 /config ----
let configTouched=false;
const sliderCfg={steerGain:{el:"steerGain",d:2,def:0.45},headKp:{el:"headKp",d:2,def:1.6},headKd:{el:"headKd",d:2,def:0.06},absHeadKp:{el:"absHeadKp",d:2,def:2.2},absHeadKd:{el:"absHeadKd",d:2,def:0.08},absPivotLimit:{el:"absPivotLimit",d:0,def:85},absPivotStartDeg:{el:"absPivotStartDeg",d:0,def:10},absPivotFullDeg:{el:"absPivotFullDeg",d:0,def:45},rampUp:{el:"rampUp",d:0,def:450},rampDown:{el:"rampDown",d:0,def:600},timeoutMs:{el:"timeoutMs",d:0,def:350}};
function setCtl(key,v){
  const cfg=sliderCfg[key];if(!cfg)return;
  const sl=document.querySelector("#"+cfg.el),nb=document.querySelector("#"+cfg.el+"Num");
  if(sl)sl.value=v;
  if(nb&&document.activeElement!==nb)nb.value=Number(v).toFixed(cfg.d);
}
function syncConfigUi(c){
  const hh=document.querySelector("#headingHold"),hs=document.querySelector("#headSign"),am=document.querySelector("#absMode"),ats=document.querySelector("#absTurnSign");
  if(hh&&typeof c.headingHold==="boolean")hh.checked=c.headingHold;
  if(hs&&c.headSign!=null)hs.checked=Number(c.headSign)<0;
  if(am&&c.assistMode!=null)am.checked=Number(c.assistMode)===2;
  if(ats&&c.absTurnSign!=null)ats.checked=Number(c.absTurnSign)<0;
  for(const key in sliderCfg){if(c[key]!=null)setCtl(key,c[key]);}
}
function pushConfig(params){
  configTouched=true;
  const qs=Object.entries(params).map(([k,v])=>`${k}=${encodeURIComponent(v)}`).join("&");
  esp32Api(`/config?${qs}`,{method:"POST"}).then(render).catch(e=>info(e.message,true));
}
document.querySelector("#headingHold").addEventListener("change",e=>pushConfig({headingHold:e.target.checked?1:0}));
document.querySelector("#headSign").addEventListener("change",e=>pushConfig({headSign:e.target.checked?-1:1}));
document.querySelector("#absMode").addEventListener("change",e=>pushConfig({assistMode:e.target.checked?2:1}));
document.querySelector("#absTurnSign").addEventListener("change",e=>pushConfig({absTurnSign:e.target.checked?-1:1}));
document.querySelector("#setAbsForward").addEventListener("click",()=>{pushConfig({setAbsForward:1,assistMode:2});const am=document.querySelector("#absMode");if(am)am.checked=true;info("已将当前 IMU 航向设为绝对前方");});
for(const key in sliderCfg){
  const cfg=sliderCfg[key],el=document.querySelector("#"+cfg.el),nb=document.querySelector("#"+cfg.el+"Num");
  // 滑杆拖动：实时同步数字框，松手下发。
  el.addEventListener("input",()=>{if(nb)nb.value=Number(el.value).toFixed(cfg.d);configTouched=true;});
  el.addEventListener("change",()=>pushConfig({[key]:el.value}));
  // 数字框：键入精确值，回车或失焦下发，并联动滑杆。
  if(nb){
    const commitNum=()=>{let v=parseFloat(nb.value);if(isNaN(v))return;el.value=v;configTouched=true;pushConfig({[key]:v});};
    nb.addEventListener("input",()=>{const v=parseFloat(nb.value);if(!isNaN(v)){el.value=v;configTouched=true;}});
    nb.addEventListener("change",commitNum);
    nb.addEventListener("keydown",e=>{if(e.key==="Enter"){e.preventDefault();commitNum();nb.blur();}});
  }
}
const pidResetBtn=document.querySelector("#pidReset");
if(pidResetBtn)pidResetBtn.addEventListener("click",()=>{
  const defs={headSign:1,absTurnSign:-1,assistMode:1};
  for(const key in sliderCfg){defs[key]=sliderCfg[key].def;setCtl(key,sliderCfg[key].def);}
  const hs=document.querySelector("#headSign");
  if(hs)hs.checked=false;
  const ats=document.querySelector("#absTurnSign");
  if(ats)ats.checked=true;
  const am=document.querySelector("#absMode");
  if(am)am.checked=false;
  pushConfig(defs);
  info("已恢复默认参数");
});

refresh(); setInterval(refresh,1000);

const cameraStatusEl=document.querySelector("#cameraStatus");
const cameraAgeEl=document.querySelector("#cameraAge");
const cameraFramesEl=document.querySelector("#cameraFrames");
const cameraFrameSizeEl=document.querySelector("#cameraFrameSize");
const cameraBytesEl=document.querySelector("#cameraBytes");
const cameraErrorEl=document.querySelector("#cameraError");
async function refreshCamera(){
  try{
    const data=await api("/api/camera-state");
    cameraStatusEl.textContent=data.status||"-";
    cameraStatusEl.className="value "+(data.connected&&((data.age_ms??99999)<2000)?"status-ok":"status-warn");
    cameraAgeEl.textContent=data.age_ms==null?"尚未收到画面":`距离上帧：${data.age_ms} ms`;
    cameraFramesEl.textContent=data.frame_count??"-";
    cameraFrameSizeEl.textContent=(data.frame_size??0)+" bytes";
    cameraBytesEl.textContent=data.byte_count??"-";
    cameraErrorEl.textContent=data.last_error||"-";
  }catch(e){
    cameraStatusEl.textContent=e.message;
    cameraStatusEl.className="value status-warn";
  }
}
refreshCamera(); setInterval(refreshCamera,1000);
</script>
<script type="module">
import * as THREE from "three";

const canvas = document.querySelector("#imuScene");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
camera.position.set(4, 3, 6);
camera.lookAt(0, 0, 0);

scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const light = new THREE.DirectionalLight(0xffffff, 1.7);
light.position.set(3, 5, 4);
scene.add(light);

const grid = new THREE.GridHelper(7, 14, 0x3b5368, 0x233746);
grid.position.y = -1.4;
scene.add(grid);

const body = new THREE.Group();
scene.add(body);

function createTextSprite(text, color = "#fef08a") {
  const labelCanvas = document.createElement("canvas");
  labelCanvas.width = 256;
  labelCanvas.height = 96;
  const ctx = labelCanvas.getContext("2d");
  ctx.font = "bold 34px sans-serif";
  ctx.fillStyle = color;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, 128, 48);
  const texture = new THREE.CanvasTexture(labelCanvas);
  const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthWrite: false });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(1.2, 0.45, 1);
  return sprite;
}

const northGroup = new THREE.Group();
northGroup.position.set(-3.0, -1.18, -2.4);
northGroup.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 1.55, 0xfacc15, 0.28, 0.16));
const northLabel = createTextSprite("航向 0°");
northLabel.position.set(1.72, 0.18, 0);
northGroup.add(northLabel);
scene.add(northGroup);

// 安装校正层：盒子和三色轴都放在这里，后续要按实物初始摆放微调时会整体一起转。
// 当前校正目标：蓝色 X 前，绿色 Y 上，红色 Z 与蓝色 X 同在水平面。
const modelGroup = new THREE.Group();
// 绕竖直轴（Y）逆时针旋转 90°，对齐实物初始摆放。
modelGroup.rotation.y = Math.PI / 2;
body.add(modelGroup);

const box = new THREE.Mesh(
  new THREE.BoxGeometry(2.4, 0.45, 1.35),
  new THREE.MeshStandardMaterial({ color: 0x42d9ff, metalness: 0.18, roughness: 0.35, emissive: 0x062433 })
);
modelGroup.add(box);

const nose = new THREE.Mesh(
  new THREE.ConeGeometry(0.42, 0.9, 4),
  new THREE.MeshStandardMaterial({ color: 0xffcf5a, metalness: 0.1, roughness: 0.38 })
);
nose.rotation.z = -Math.PI / 2;
nose.position.x = 1.65;
modelGroup.add(nose);

modelGroup.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 2.2, 0x39d0ff, 0.32, 0.18));
modelGroup.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 1.5, 0x6cff9d, 0.28, 0.16));
modelGroup.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 1.7, 0xff5b5b, 0.28, 0.16));

const imuStatusEl = document.querySelector("#imuStatus");
const imuAgeEl = document.querySelector("#imuAge");
const imuHeadingEl = document.querySelector("#imuHeading");
const imuEulerEl = document.querySelector("#imuEuler");
const imuQuatEl = document.querySelector("#imuQuat");
const imuSensorEl = document.querySelector("#imuSensor");
const imuPacketEl = document.querySelector("#imuPacket");
const compassNeedleEl = document.querySelector("#compassNeedle");

function resizeImuScene() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (canvas.width !== width || canvas.height !== height) {
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }
}

function formatVec(v, unit = "") {
  if (!v) return "-";
  return v.map(x => `${Number(x).toFixed(3)}${unit}`).join(" / ");
}

function formatHeadingDeg(heading) {
  return heading == null ? "-" : `${heading.toFixed(1)}°`;
}

// 实测重力在 Z 轴（accel z≈9.8），说明 IMU 是 Z 轴竖直安装。
// 因此左右转就是绕 Z 的航向角，直接用 IMU VRU 模式陀螺积分出来的 yaw=euler[2]。
function resolveHeading(data) {
  const e = data.euler_deg;
  if (!e || e.length < 3 || !Number.isFinite(Number(e[2]))) return null;
  return ((Number(e[2]) % 360) + 360) % 360;
}

function applyImuPose(euler, heading) {
  if (heading == null && (!euler || euler.length < 3)) return;
  const headingDeg = heading ?? ((Number(euler[2]) % 360) + 360) % 360;
  const roll = THREE.MathUtils.degToRad(euler && euler.length >= 3 ? Number(euler[0]) : 0);
  const pitch = THREE.MathUtils.degToRad(euler && euler.length >= 3 ? Number(euler[1]) : 0);
  const headingRad = THREE.MathUtils.degToRad(headingDeg);

  // 航向绕竖直轴（Three.js 的 Y），roll/pitch 作为次要姿态叠加。
  const yawQ = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -headingRad);
  const pitchQ = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -pitch);
  const rollQ = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), roll);
  // 实物与显示三个轴转动方向全部相反，这里整体取逆，让每个轴都镜像过来。
  body.quaternion.copy(yawQ).multiply(pitchQ).multiply(rollQ).invert();
}

function updateCompass(heading) {
  if (!compassNeedleEl) return;
  if (heading == null) return;
  compassNeedleEl.style.transform = `rotate(${-heading}deg)`;
}

async function pollImu() {
  try {
    const response = await fetch("/api/imu-state", { cache: "no-store" });
    const data = await response.json();
    const imuFresh = (data.age_ms ?? 99999) < 1500 && (data.raw_age_ms ?? 99999) < 1500;
    imuStatusEl.textContent = (data.status || "-") + (imuFresh ? "" : "（数据超时）");
    imuStatusEl.className = "value " + (imuFresh ? "status-ok" : "status-warn");
    imuAgeEl.textContent = data.age_ms == null ? "尚未收到姿态数据" : `距离上次更新：${data.age_ms} ms`;
    const magneticHeading = resolveHeading(data);
    imuHeadingEl.textContent = formatHeadingDeg(magneticHeading);
    imuEulerEl.textContent = formatVec(data.euler_deg, "°");
    imuQuatEl.textContent = formatVec(data.quaternion);
    imuSensorEl.innerHTML = `
      温度：${data.temperature == null ? "-" : Number(data.temperature).toFixed(2) + " ℃"}<br>
      加速度：${formatVec(data.accel)}<br>
      角速度：${formatVec(data.gyro)}
    `;
    imuPacketEl.innerHTML = `
      帧序号：${data.frame_no}<br>
      TCP 原始字节：${data.raw_bytes}<br>
      原始数据延迟：${data.raw_age_ms == null ? "-" : data.raw_age_ms + " ms"}<br>
      缓冲区字节：${data.buffered_bytes}<br>
      帧头59 53命中：${data.header_hits}<br>
      帧头55命中：${data.wit_header_hits}<br>
      帧头AA 55命中：${data.aa55_header_hits}<br>
      已解析包：${data.raw_packets}<br>
      已提取数据组：${data.raw_groups}<br>
      解析错误：${data.parse_errors}<br>
      最近解析错误：${data.last_parse_error || "-"}<br>
      最近原始HEX：${data.last_raw_hex || "-"}
    `;
    if (imuFresh) {
      applyImuPose(data.euler_deg, magneticHeading);
      updateCompass(magneticHeading);
    }
  } catch (err) {
    imuStatusEl.textContent = "IMU poll error: " + err;
    imuStatusEl.className = "value status-warn";
  }
}

function animateImuScene() {
  resizeImuScene();
  renderer.render(scene, camera);
  requestAnimationFrame(animateImuScene);
}

setInterval(pollImu, 100);
pollImu();
animateImuScene();
</script>
</body>
</html>
"""


def json_response(handler, status_code, payload):
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(status_code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def read_json(handler):
    length = int(handler.headers.get("Content-Length", "0") or "0")
    return json.loads(handler.rfile.read(length).decode("utf-8")) if length else {}


def request_esp32(path, method="GET"):
    request = Request(f"{ESP32_BASE_URL}{path}", data=(b"" if method == "POST" else None), method=method)
    try:
        with urlopen(request, timeout=2) as response:
            return json.loads(response.read().decode("utf-8"))
    except HTTPError as error:
        detail = error.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"ESP32 返回错误 {error.code}: {detail}") from error
    except URLError as error:
        raise RuntimeError(f"连接不到 ESP32：{error.reason}") from error
    except TimeoutError as error:
        raise RuntimeError("连接 ESP32 超时") from error


def parse_text_command(text):
    normalized = re.sub(r"\s+", "", text.strip().lower())
    if normalized in {"停", "停止", "stop", "s", "全部停止", "全停"}:
        return None, 0

    motor = 1
    motor_match = re.search(r"(?:电机|motor|m)([12])", normalized)
    command_text = normalized
    if motor_match:
        motor = int(motor_match.group(1))
        command_text = normalized[:motor_match.start()] + normalized[motor_match.end():]

    if "停止" in command_text or command_text in {"停", "stop", "s"}:
        return motor, 0
    if "正转" in command_text or "前进" in command_text or "forward" in command_text:
        sign = 1
    elif "反转" in command_text or "后退" in command_text or "reverse" in command_text or "back" in command_text:
        sign = -1
    else:
        raise ValueError("方向只能是正转、反转或停止")

    match = re.search(r"(\d+(?:\.\d+)?)", command_text)
    if not match:
        raise ValueError("请加占空比，例如：电机2正转80")

    percent = float(match.group(1))
    if percent < 0 or percent > 100:
        raise ValueError("占空比必须是 0 到 100")
    return motor, int(round(sign * percent))


def pwm_command(motor, percent):
    motor = int(motor)
    percent = int(round(float(percent)))
    if motor not in {1, 2}:
        raise ValueError("motor 必须是 1 或 2")
    if percent < -100 or percent > 100:
        raise ValueError("percent 必须是 -100 到 100")
    if percent == 0:
        return request_esp32(f"/stop?{urlencode({'motor': motor})}", method="POST")
    return request_esp32(f"/pwm?{urlencode({'motor': motor, 'value': percent})}", method="POST")


def send_camera_mjpeg(handler):
    boundary = "frame"
    handler.send_response(200)
    handler.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
    handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
    handler.send_header("Pragma", "no-cache")
    handler.end_headers()

    last_seen = -1
    while True:
        frame_no, frame = CAMERA_STATE.wait_frame(last_seen, timeout=5.0)
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
            body = HTML.replace("__ESP32_BASE_URL__", ESP32_BASE_URL).encode("utf-8")
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
            json_response(self, 200, CAMERA_STATE.snapshot())
        elif parsed.path == "/api/camera.mjpg":
            send_camera_mjpeg(self)
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
            else:
                json_response(self, 404, {"error": "Not found"})
        except ValueError as error:
            json_response(self, 400, {"error": str(error)})
        except Exception as error:
            json_response(self, 502, {"error": str(error), "esp32": ESP32_BASE_URL})


def start_background_services() -> None:
    threading.Thread(
        target=frame_stream_from_tcp_server,
        args=(IMU_TCP_HOST, IMU_TCP_PORT, IMU_STATE),
        daemon=True,
        name="imu-tcp-server",
    ).start()
    threading.Thread(
        target=camera_websocket_server,
        args=(CAMERA_WS_HOST, CAMERA_WS_PORT),
        daemon=True,
        name="camera-ws-server",
    ).start()


def run_http_server() -> None:
    while True:
        try:
            httpd = ThreadingHTTPServer((HOST, PORT), Handler)
            httpd.allow_reuse_address = True
            print(f"WebUI: http://{PUBLIC_IP}:{PORT}", flush=True)
            print(f"本机: http://127.0.0.1:{PORT}", flush=True)
            print(f"ESP32: {ESP32_BASE_URL}", flush=True)
            print(f"IMU TCP监听: {IMU_TCP_HOST}:{IMU_TCP_PORT}", flush=True)
            print(f"摄像头WS监听: {CAMERA_WS_HOST}:{CAMERA_WS_PORT}{CAMERA_WS_PATH}", flush=True)
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("HTTP 服务已停止", flush=True)
            break
        except OSError as exc:
            print(f"[http] 端口 {PORT} 启动失败: {exc}，{SERVER_RESTART_DELAY}s 后重试", flush=True)
            time.sleep(SERVER_RESTART_DELAY)
        except Exception as exc:
            print(f"[http] 服务异常: {exc}，{SERVER_RESTART_DELAY}s 后重试", flush=True)
            time.sleep(SERVER_RESTART_DELAY)


if __name__ == "__main__":
    install_thread_exception_logger()
    start_background_services()
    run_http_server()
