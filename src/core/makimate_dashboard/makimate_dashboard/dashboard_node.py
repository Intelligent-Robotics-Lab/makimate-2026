#!/usr/bin/env python3
import asyncio
import json
import socket
import subprocess
import threading
from pathlib import Path
from typing import Dict, Set

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import Log

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse


# ------------------------------------------------------------------ #
# Server-side config — updated via dashboard, propagated to ROS nodes
# ------------------------------------------------------------------ #
SERVER_CONFIG: Dict = {
    "llm_server_url": "http://127.0.0.1:8000",
    "llm_model": "",
    "whisper_server_url": "",
    "whisper_model": "base",
}

# ------------------------------------------------------------------ #
# Parameter registry — which params are visible/editable per node.
# Must mirror the PARAMS object in dashboard.html.
# ------------------------------------------------------------------ #
PARAM_REGISTRY = {
    "maki_dxl_6": ["smoothing_alpha", "update_rate"],
    "maki_behavior": ["no_face_threshold"],
    "speaker_recognition_node": ["threshold"],
    "face_tracker": ["recognition_threshold", "recognition_interval"],
    "respeaker_dsp": ["vad_threshold"],
    "respeaker_whisper_asr": ["vad_aggressiveness", "silence_ms", "no_speech_threshold"],
    "ai_command_router": ["wake_phrase", "sleep_phrase"],
    "llm_bridge": ["laptop_host"],
}


# ------------------------------------------------------------------ #
# ROS 2 node
# ------------------------------------------------------------------ #

class DashboardNode(Node):
    def __init__(self):
        super().__init__('makimate_dashboard')
        self._ws_clients: Set[WebSocket] = set()
        self._loop: asyncio.AbstractEventLoop = None

        rosout_qos = QoSProfile(
            depth=1000,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Log, '/rosout', self._on_log, rosout_qos)
        self.get_logger().info('Dashboard node ready — web UI on :8080')

    def set_event_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    def _on_log(self, msg: Log):
        if self._loop is None:
            return
        payload = json.dumps({
            "type": "log",
            "level": int(msg.level),
            "name": msg.name,
            "msg": msg.msg,
        })
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    async def _broadcast(self, text: str):
        dead = set()
        for ws in list(self._ws_clients):
            try:
                await ws.send_text(text)
            except Exception:
                dead.add(ws)
        self._ws_clients -= dead


# ------------------------------------------------------------------ #
# subprocess helpers
# ------------------------------------------------------------------ #

def ros2_param_get(node_name: str, param: str):
    """Returns parsed value or None on failure."""
    try:
        r = subprocess.run(
            ["ros2", "param", "get", f"/{node_name}", param],
            capture_output=True, text=True, timeout=5,
        )
        if r.returncode != 0:
            return None
        out = r.stdout.strip()
        for prefix, cast in [
            ("Double value is: ",  float),
            ("Integer value is: ", int),
            ("Boolean value is: ", lambda x: x.lower() == "true"),
            ("String value is: ",  str),
        ]:
            if out.startswith(prefix):
                return cast(out[len(prefix):])
        return out
    except Exception:
        return None


def ros2_param_set(node_name: str, param: str, value: str):
    """Returns (success: bool, message: str)."""
    try:
        r = subprocess.run(
            ["ros2", "param", "set", f"/{node_name}", param, str(value)],
            capture_output=True, text=True, timeout=5,
        )
        return r.returncode == 0, (r.stdout.strip() or r.stderr.strip())
    except subprocess.TimeoutExpired:
        return False, "Timeout — node may not be running"
    except Exception as e:
        return False, str(e)


def _whisper_server_set_model(server_url: str, model: str, node) -> None:
    """POST /set_model to the ASR server. Runs in a thread executor."""
    try:
        import urllib.request
        import json as _json
        payload = _json.dumps({"model": model}).encode()
        req = urllib.request.Request(
            f"{server_url}/set_model",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=60) as resp:
            result = _json.loads(resp.read())
        node.get_logger().info(f"ASR server model set: {result}")
    except Exception as e:
        node.get_logger().warn(f"Could not reach ASR server to set model: {e}")


def set_volume(value: int) -> tuple:
    """Set ALSA master volume. Returns (success, message, actual_value)."""
    value = max(0, min(100, value))
    for control in ('Master', 'PCM', 'Speaker'):
        try:
            r = subprocess.run(
                ['amixer', '-q', 'sset', control, f'{value}%'],
                capture_output=True, text=True, timeout=3,
            )
            if r.returncode == 0:
                return True, control, value
        except Exception:
            continue
    return False, 'No suitable ALSA control found (tried Master, PCM, Speaker)', value


def get_volume() -> int:
    """Read current ALSA master volume (0-100). Returns -1 on failure."""
    for control in ('Master', 'PCM', 'Speaker'):
        try:
            r = subprocess.run(
                ['amixer', 'sget', control],
                capture_output=True, text=True, timeout=3,
            )
            if r.returncode == 0:
                import re
                m = re.search(r'\[(\d+)%\]', r.stdout)
                if m:
                    return int(m.group(1))
        except Exception:
            continue
    return -1


def get_local_ip() -> str:
    """Return the Pi's primary LAN IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "unknown"


def fetch_all_params() -> dict:
    """Bulk-read all registered params. Skips nodes that are not running."""
    result = {}
    for node_name, params in PARAM_REGISTRY.items():
        vals = {}
        for p in params:
            v = ros2_param_get(node_name, p)
            if v is not None:
                vals[p] = v
        if vals:
            result[node_name] = vals
    return result


# ------------------------------------------------------------------ #
# FastAPI app
# ------------------------------------------------------------------ #

def create_app(node: DashboardNode) -> FastAPI:
    app = FastAPI()
    html_path = Path(__file__).parent / "dashboard.html"

    @app.on_event("startup")
    async def _startup():
        node.set_event_loop(asyncio.get_event_loop())

    @app.get("/", response_class=HTMLResponse)
    async def get_dashboard():
        return html_path.read_text()

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        node._ws_clients.add(ws)
        node.get_logger().info(
            f"Dashboard client connected ({len(node._ws_clients)} total)"
        )
        asyncio.create_task(_send_initial_state(ws, node))
        try:
            while True:
                raw = await ws.receive_text()
                await _handle(ws, node, json.loads(raw))
        except WebSocketDisconnect:
            pass
        except Exception as e:
            node.get_logger().warn(f"Dashboard WS error: {e}")
        finally:
            node._ws_clients.discard(ws)
            node.get_logger().info(
                f"Dashboard client disconnected ({len(node._ws_clients)} total)"
            )

    return app


async def _send_initial_state(ws: WebSocket, node: DashboardNode):
    loop = asyncio.get_event_loop()
    params = await loop.run_in_executor(None, fetch_all_params)
    try:
        await ws.send_text(json.dumps({"type": "param_all", "params": params}))
        await ws.send_text(json.dumps({"type": "server_config", "config": SERVER_CONFIG}))
        vol = await loop.run_in_executor(None, get_volume)
        if vol >= 0:
            await ws.send_text(json.dumps({"type": "volume_current", "value": vol}))
        ip = await loop.run_in_executor(None, get_local_ip)
        await ws.send_text(json.dumps({"type": "system_info", "ip": ip, "hostname": socket.gethostname()}))
    except Exception:
        pass


async def _handle(ws: WebSocket, node: DashboardNode, msg: dict):
    t = msg.get("type")
    loop = asyncio.get_event_loop()

    if t == "param_set":
        node_name = msg["node"]
        param     = msg["param"]
        value     = msg["value"]
        ok, detail = await loop.run_in_executor(
            None, ros2_param_set, node_name, param, str(value)
        )
        await ws.send_text(json.dumps({
            "type":   "param_set_ok" if ok else "param_set_fail",
            "node":   node_name,
            "param":  param,
            "detail": detail,
        }))
        if ok:
            real = await loop.run_in_executor(None, ros2_param_get, node_name, param)
            if real is not None:
                await node._broadcast(json.dumps({
                    "type":  "param_value",
                    "node":  node_name,
                    "param": param,
                    "value": real,
                }))

    elif t == "param_get":
        val = await loop.run_in_executor(
            None, ros2_param_get, msg["node"], msg["param"]
        )
        await ws.send_text(json.dumps({
            "type":  "param_value",
            "node":  msg["node"],
            "param": msg["param"],
            "value": val,
        }))

    elif t == "volume_set":
        value = int(msg.get("value", 80))
        ok, detail, actual = await loop.run_in_executor(None, set_volume, value)
        if ok:
            await ws.send_text(json.dumps({"type": "volume_ok", "value": actual}))
        else:
            await ws.send_text(json.dumps({"type": "volume_fail", "detail": detail}))

    elif t == "server_config":
        for key in ("llm_server_url", "llm_model", "whisper_server_url", "whisper_model"):
            if key in msg:
                SERVER_CONFIG[key] = msg[key]

        # Propagate to running ROS nodes where applicable
        if "llm_server_url" in msg:
            await loop.run_in_executor(
                None, ros2_param_set, "llm_bridge", "laptop_host", msg["llm_server_url"]
            )
        if "whisper_server_url" in msg:
            await loop.run_in_executor(
                None, ros2_param_set, "respeaker_whisper_asr", "server_url",
                msg["whisper_server_url"]
            )
        if "whisper_model" in msg:
            # Always update the Pi node param (used when server_url is empty = local mode)
            await loop.run_in_executor(
                None, ros2_param_set, "respeaker_whisper_asr", "model_size",
                msg["whisper_model"]
            )
            # If a server URL is configured, also hot-swap the model on the server
            server_url = SERVER_CONFIG.get("whisper_server_url", "").strip()
            if server_url:
                await loop.run_in_executor(
                    None, _whisper_server_set_model, server_url, msg["whisper_model"], node
                )

        await node._broadcast(json.dumps({"type": "server_config", "config": SERVER_CONFIG}))
        node.get_logger().info(f"Server config updated: {SERVER_CONFIG}")


# ------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------ #

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = create_app(node)
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning")

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
