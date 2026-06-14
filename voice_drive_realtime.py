#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joystick 语音方向控制 sidecar。

数据流：
  joystick ESP32 PDM 麦克风 -> WebSocket 二进制 PCM16/16kHz -> DashScope Realtime ASR
  -> 方向词 -> all_in_one_webui.py /api/voice-drive -> 小车 /drive

运行：
  DASHSCOPE_API_KEY=xxx python voice_drive_realtime.py
"""

from __future__ import annotations

import asyncio
import base64
import inspect
import json
import os
import re
import time
import uuid
from typing import Awaitable, Callable, Optional
from urllib.request import Request, urlopen

import websockets
from websockets.exceptions import ConnectionClosed


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


HOST = os.environ.get("VOICE_WS_HOST", "0.0.0.0")
PORT = int(os.environ.get("VOICE_WS_PORT", "8091"))
WS_PATH = os.environ.get("VOICE_WS_PATH", "/ws_voice_audio")

WEBUI_URL = os.environ.get("VOICE_WEBUI_URL", "http://127.0.0.1:8000").rstrip("/")
VOICE_DRIVE_HOLD_MS = int(os.environ.get("VOICE_DRIVE_HOLD_MS", "1200"))
VOICE_DRIVE_SPEED = os.environ.get("VOICE_DRIVE_SPEED", "full")
VOICE_POST_TIMEOUT_S = float(os.environ.get("VOICE_POST_TIMEOUT_S", "0.5"))

REALTIME_MODEL = os.environ.get("QWEN_REALTIME_MODEL", "qwen3.5-omni-plus-realtime")
REALTIME_WS_URL = os.environ.get("QWEN_REALTIME_WS_URL", "wss://dashscope.aliyuncs.com/api-ws/v1/realtime")
REALTIME_TRANSCRIPTION_MODEL = os.environ.get("QWEN_REALTIME_TRANSCRIPTION_MODEL", "gummy-realtime-v1")
REALTIME_TURN_DETECTION = os.environ.get("QWEN_REALTIME_TURN_DETECTION", "server_vad")
REALTIME_TURN_THRESHOLD = float(os.environ.get("QWEN_REALTIME_TURN_THRESHOLD", "0.45"))
REALTIME_TURN_SILENCE_MS = int(os.environ.get("QWEN_REALTIME_TURN_SILENCE_MS", "550"))
REALTIME_PREFIX_PADDING_MS = int(os.environ.get("QWEN_REALTIME_PREFIX_PADDING_MS", "250"))
REALTIME_READY_TIMEOUT_S = float(os.environ.get("QWEN_REALTIME_READY_TIMEOUT", "10"))
API_KEY = os.environ.get("DASHSCOPE_API_KEY", "")

UPLOAD_QUEUE_MAX = int(os.environ.get("VOICE_UPLOAD_QUEUE_MAX", "120"))
COMMAND_REPEAT_MS = int(os.environ.get("VOICE_COMMAND_REPEAT_MS", "350"))

STOP_WORDS = ("停止", "停下", "停车", "刹车", "别动", "不要动", "停")
DIRECTION_WORDS = (
    ("left_forward", ("左上", "上左", "左前", "前左", "向左上", "往左上", "左前方")),
    ("right_forward", ("右上", "上右", "右前", "前右", "向右上", "往右上", "右前方")),
    ("left_backward", ("左下", "下左", "左后", "后左", "向左下", "往左下", "左后方")),
    ("right_backward", ("右下", "下右", "右后", "后右", "向右下", "往右下", "右后方")),
    ("forward", ("前进", "向前", "往前", "前冲", "向前冲", "冲", "上")),
    ("backward", ("后退", "向后", "往后", "倒车", "后")),
    ("left", ("向左", "往左", "左移", "左转", "左")),
    ("right", ("向右", "往右", "右移", "右转", "右")),
)


def compact_text(text: str) -> str:
    return re.sub(r"[\s，。！？、,.!?；;：:]+", "", (text or "").strip().lower())


def match_command_key(text: str) -> Optional[str]:
    normalized = compact_text(text)
    if not normalized:
        return None
    if any(word in normalized for word in STOP_WORDS):
        return "stop"
    for key, words in DIRECTION_WORDS:
        if any(word in normalized for word in words):
            return key
    return None


def post_json(path: str, payload: dict, timeout: float = VOICE_POST_TIMEOUT_S) -> dict:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    request = Request(
        WEBUI_URL + path,
        data=body,
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with urlopen(request, timeout=timeout) as response:
        raw = response.read().decode("utf-8")
    return json.loads(raw) if raw else {}


async def dispatch_voice_command(text: str, *, partial: bool, state: dict) -> None:
    key = match_command_key(text)
    if key is None:
        return
    now = time.monotonic()
    last_key = state.get("last_key")
    last_at = float(state.get("last_at") or 0.0)
    if partial and key == last_key and (now - last_at) * 1000 < COMMAND_REPEAT_MS:
        return

    payload = {
        "text": text,
        "speed": VOICE_DRIVE_SPEED,
        "hold_ms": VOICE_DRIVE_HOLD_MS,
        "source": "joystick-voice-partial" if partial else "joystick-voice-final",
    }
    try:
        result = await asyncio.to_thread(post_json, "/api/voice-drive", payload)
        state["last_key"] = key
        state["last_at"] = now
        state["last_result"] = result
        print(
            f"[VOICE CMD] {'partial' if partial else 'final'} key={key} text={text} "
            f"drive={result.get('throttle')}/{result.get('steering')}",
            flush=True,
        )
    except Exception as exc:
        state["last_error"] = str(exc)
        print(f"[VOICE CMD] post failed key={key} text={text}: {exc}", flush=True)


class RealtimeAsrSession:
    def __init__(
        self,
        *,
        on_transcript: Callable[[str, bool], Awaitable[None]],
        on_debug: Optional[Callable[[str], Awaitable[None]]] = None,
    ) -> None:
        self.on_transcript = on_transcript
        self.on_debug = on_debug
        self.ws = None
        self.reader_task: Optional[asyncio.Task] = None
        self.ready = asyncio.Event()
        self.connected = False
        self.session_updated = False
        self.send_lock = asyncio.Lock()

    async def connect(self) -> None:
        if not API_KEY:
            raise RuntimeError("未设置 DASHSCOPE_API_KEY")
        if self.ws is not None and self.connected:
            return

        url = f"{REALTIME_WS_URL}?model={REALTIME_MODEL}"
        headers = {"Authorization": f"Bearer {API_KEY}"}
        kwargs = {"open_timeout": 10, "max_size": None, "ping_interval": 20, "ping_timeout": 20}
        sig = inspect.signature(websockets.connect)
        if "additional_headers" in sig.parameters:
            kwargs["additional_headers"] = headers
        else:
            kwargs["extra_headers"] = headers
        await self._debug(f"connecting realtime model={REALTIME_MODEL}")
        self.ws = await websockets.connect(url, **kwargs)
        self.reader_task = asyncio.create_task(self._read_loop())
        await self._send(
            {
                "type": "session.update",
                "session": {
                    "modalities": ["text"],
                    "instructions": "你只负责把中文语音转写成文字，不要闲聊，不要输出额外解释。",
                    "input_audio_format": "pcm",
                    "input_audio_transcription": {"model": REALTIME_TRANSCRIPTION_MODEL},
                    "turn_detection": {
                        "type": REALTIME_TURN_DETECTION,
                        "threshold": REALTIME_TURN_THRESHOLD,
                        "prefix_padding_ms": REALTIME_PREFIX_PADDING_MS,
                        "silence_duration_ms": REALTIME_TURN_SILENCE_MS,
                    },
                },
            }
        )
        await asyncio.wait_for(self.ready.wait(), timeout=REALTIME_READY_TIMEOUT_S)
        if not self.connected or not self.session_updated:
            raise RuntimeError("Realtime 会话未就绪")
        await self._debug("realtime ready")

    async def append_audio(self, pcm: bytes) -> None:
        if not pcm:
            return
        await self.connect()
        await self._send(
            {
                "type": "input_audio_buffer.append",
                "audio": base64.b64encode(pcm).decode("ascii"),
            }
        )

    async def close(self) -> None:
        task = self.reader_task
        self.reader_task = None
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        ws = self.ws
        self.ws = None
        self.connected = False
        if ws is not None:
            try:
                await ws.close()
            except Exception:
                pass

    async def _send(self, event: dict) -> None:
        if self.ws is None:
            raise RuntimeError("Realtime WebSocket 未连接")
        event["event_id"] = "event_" + uuid.uuid4().hex
        async with self.send_lock:
            await self.ws.send(json.dumps(event, ensure_ascii=False))

    async def _read_loop(self) -> None:
        try:
            async for raw in self.ws:
                event = json.loads(raw)
                event_type = event.get("type", "")
                if event_type == "session.created":
                    self.connected = True
                    continue
                if event_type == "session.updated":
                    self.connected = True
                    self.session_updated = True
                    self.ready.set()
                    continue
                if event_type == "error":
                    await self._debug("realtime error: " + json.dumps(event.get("error", event), ensure_ascii=False))
                    self.ready.set()
                    continue
                if event_type == "input_audio_buffer.speech_started":
                    await self._debug("speech started")
                    continue
                if event_type == "input_audio_buffer.speech_stopped":
                    await self._debug("speech stopped")
                    continue
                if "input_audio_transcription" in event_type:
                    if event_type.endswith(".delta"):
                        delta = (event.get("delta") or event.get("transcript") or "").strip()
                        if delta:
                            await self.on_transcript(delta, True)
                    elif event_type.endswith(".completed"):
                        transcript = (event.get("transcript") or "").strip()
                        if transcript:
                            await self.on_transcript(transcript, False)
                    continue
        except ConnectionClosed as exc:
            await self._debug(f"realtime closed code={exc.code} reason={exc.reason}")
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            await self._debug(f"realtime read error: {exc}")
        finally:
            self.connected = False
            self.ready.set()

    async def _debug(self, message: str) -> None:
        if self.on_debug:
            await self.on_debug(message)
        else:
            print(f"[REALTIME] {message}", flush=True)


def websocket_path(ws, path: Optional[str]) -> str:
    if path:
        return path
    request = getattr(ws, "request", None)
    request_path = getattr(request, "path", None)
    if request_path:
        return request_path
    return getattr(ws, "path", "") or "/"


async def audio_ws_handler(ws, path: Optional[str] = None) -> None:
    path = websocket_path(ws, path)
    if path != WS_PATH:
        await ws.close(code=1008, reason="invalid path")
        return

    peer = getattr(ws, "remote_address", None)
    print(f"[WS AUDIO] connected peer={peer}", flush=True)
    upload_queue: asyncio.Queue[Optional[bytes]] = asyncio.Queue(maxsize=UPLOAD_QUEUE_MAX)
    command_state: dict = {}
    session: Optional[RealtimeAsrSession] = None
    streaming = False
    chunk_count = 0
    total_bytes = 0
    started_at = time.monotonic()

    async def debug(message: str) -> None:
        print(f"[REALTIME] {message}", flush=True)

    async def on_transcript(text: str, partial: bool) -> None:
        print(f"[ASR] {'partial' if partial else 'final'}: {text}", flush=True)
        await dispatch_voice_command(text, partial=partial, state=command_state)

    async def start_session() -> None:
        nonlocal session, streaming, chunk_count, total_bytes, started_at
        if session is not None and streaming:
            return
        if session is not None:
            await session.close()
        session = RealtimeAsrSession(on_transcript=on_transcript, on_debug=debug)
        await session.connect()
        streaming = True
        chunk_count = 0
        total_bytes = 0
        started_at = time.monotonic()
        await ws.send("OK:STARTED")

    async def stop_session(notice: str = "OK:STOPPED") -> None:
        nonlocal session, streaming
        streaming = False
        if session is not None:
            await session.close()
            session = None
        try:
            await ws.send(notice)
        except Exception:
            pass

    async def upload_worker() -> None:
        while True:
            data = await upload_queue.get()
            if data is None:
                return
            current = session
            if not streaming or current is None:
                continue
            try:
                await current.append_audio(data)
            except Exception as exc:
                print(f"[WS AUDIO] upload failed: {exc}", flush=True)
                await stop_session("RESTART")

    worker = asyncio.create_task(upload_worker())
    try:
        async for message in ws:
            if isinstance(message, str):
                cmd = message.strip().upper()
                if cmd == "START":
                    try:
                        await start_session()
                    except Exception as exc:
                        print(f"[WS AUDIO] START failed: {exc}", flush=True)
                        await ws.send("ERROR:" + str(exc))
                elif cmd == "STOP":
                    await stop_session()
                elif cmd == "RESET":
                    await stop_session("OK:RESET")
                continue

            if isinstance(message, (bytes, bytearray)):
                if not streaming:
                    try:
                        await start_session()
                    except Exception as exc:
                        print(f"[WS AUDIO] autostart failed: {exc}", flush=True)
                        continue
                chunk = bytes(message)
                chunk_count += 1
                total_bytes += len(chunk)
                if chunk_count == 1 or chunk_count % 50 == 0:
                    elapsed = max(0.001, time.monotonic() - started_at)
                    print(
                        f"[WS AUDIO] chunks={chunk_count} bytes={total_bytes} rate={total_bytes / elapsed:.0f}B/s",
                        flush=True,
                    )
                if upload_queue.full():
                    try:
                        upload_queue.get_nowait()
                    except Exception:
                        pass
                upload_queue.put_nowait(chunk)
    finally:
        print(f"[WS AUDIO] disconnected peer={peer}", flush=True)
        await stop_session()
        try:
            upload_queue.put_nowait(None)
        except Exception:
            pass
        worker.cancel()
        try:
            await worker
        except asyncio.CancelledError:
            pass


async def main() -> None:
    print(f"语音方向 sidecar: ws://{HOST}:{PORT}{WS_PATH}", flush=True)
    print(f"WebUI command endpoint: {WEBUI_URL}/api/voice-drive", flush=True)
    if not API_KEY:
        print("警告：未设置 DASHSCOPE_API_KEY，ESP32 连接后会收到 ERROR。", flush=True)
    async with websockets.serve(audio_ws_handler, HOST, PORT, max_size=None, ping_interval=20, ping_timeout=20):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("语音方向 sidecar 已停止", flush=True)
