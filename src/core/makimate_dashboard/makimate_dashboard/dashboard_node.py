#!/usr/bin/env python3
import asyncio
import json
import os
import signal
import socket
import subprocess
import threading
from pathlib import Path
from typing import Dict, Optional, Set

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image
from makimate_interfaces.msg import FaceTrackArray

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

        # Publisher for live LLM host updates (more reliable than ros2 param set)
        from std_msgs.msg import String as _String
        self._llm_set_host_pub = self.create_publisher(_String, '/llm/set_host', 10)

        # Robot process tracking
        self._robot_proc: Optional[subprocess.Popen] = None
        self._robot_lock = threading.Lock()
        self._last_robot_running = False
        self.create_timer(2.0, self._check_robot_status)

        # Camera / face-track state
        self._latest_frame: Optional[bytes] = None
        self._last_face_broadcast: float = 0.0
        self._last_image_process: float = 0.0
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Camera feed disabled — too much load on Pi
        # self.create_subscription(Image, '/camera/image_raw', self._on_image, camera_qos)
        # self.create_subscription(FaceTrackArray, '/maki/face_tracks', self._on_face_tracks, 10)

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

    def _check_robot_status(self):
        if self._loop is None:
            return
        running = robot_is_running(self)
        if running != self._last_robot_running:
            self._last_robot_running = running
            payload = json.dumps({"type": "robot_status", "running": running})
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    def _on_image(self, msg: Image):
        """Convert incoming camera frame to JPEG and cache it for the WS stream (5 fps max)."""
        import time as _time
        now = _time.monotonic()
        if now - self._last_image_process < 0.2:
            return
        self._last_image_process = now
        try:
            shape = (msg.height, msg.width, msg.step // msg.width)
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(shape)
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgra8':
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            img = cv2.rotate(img, cv2.ROTATE_180)
            _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 50])
            self._latest_frame = jpeg.tobytes()
        except Exception:
            pass

    def _on_face_tracks(self, msg: FaceTrackArray):
        if self._loop is None:
            return
        # Throttle to 10 fps — at 30 fps this floods the asyncio event loop
        import time as _time
        now = _time.monotonic()
        if now - self._last_face_broadcast < 0.1:
            return
        self._last_face_broadcast = now
        faces = [
            {
                "id": f.id,
                "name": f.name,
                "bbox": list(f.bbox),
                "score": float(f.confidence_score),
                "speaking": f.is_speaking,
            }
            for f in msg.faces[:2]  # top 2 faces by attention score
        ]
        payload = json.dumps({"type": "face_tracks", "faces": faces})
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
    import urllib.request
    import json as _json

    # Quick reachability check — fail fast (5 s) instead of waiting 60 s.
    try:
        with urllib.request.urlopen(f"{server_url}/health", timeout=5):
            pass
    except Exception as e:
        node.get_logger().warn(
            f"ASR server unreachable at {server_url} — skipping set_model ({e})"
        )
        return

    try:
        payload = _json.dumps({"model": model}).encode()
        req = urllib.request.Request(
            f"{server_url}/set_model",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        # Model loading can take 30–90 s on CPU — give it plenty of room.
        with urllib.request.urlopen(req, timeout=120) as resp:
            result = _json.loads(resp.read())
        node.get_logger().info(f"ASR server model set: {result}")
    except Exception as e:
        node.get_logger().warn(f"Could not set ASR server model: {e}")


def _pulse_env() -> dict:
    """Env vars needed to reach PulseAudio from the systemd service."""
    import os
    e = os.environ.copy()
    if 'XDG_RUNTIME_DIR' not in e:
        e['XDG_RUNTIME_DIR'] = '/run/user/1002'
    return e


def set_volume(value: int) -> tuple:
    """Set PulseAudio sink volume. Returns (success, message, actual_value)."""
    value = max(0, min(100, value))
    try:
        r = subprocess.run(
            ['pactl', 'set-sink-volume', '@DEFAULT_SINK@', f'{value}%'],
            capture_output=True, text=True, timeout=3, env=_pulse_env(),
        )
        if r.returncode == 0:
            return True, 'PulseAudio', value
    except Exception as e:
        pass
    # fallback: ALSA PCM
    try:
        r = subprocess.run(
            ['amixer', '-q', 'sset', 'PCM', f'{value}%'],
            capture_output=True, text=True, timeout=3,
        )
        if r.returncode == 0:
            return True, 'ALSA PCM', value
    except Exception:
        pass
    return False, 'No suitable volume control found', value


def get_volume() -> int:
    """Read current PulseAudio sink volume (0-100). Returns -1 on failure."""
    import re
    try:
        r = subprocess.run(
            ['pactl', 'get-sink-volume', '@DEFAULT_SINK@'],
            capture_output=True, text=True, timeout=3, env=_pulse_env(),
        )
        if r.returncode == 0:
            m = re.search(r'(\d+)%', r.stdout)
            if m:
                return int(m.group(1))
    except Exception:
        pass
    # fallback: ALSA PCM
    try:
        r = subprocess.run(
            ['amixer', 'sget', 'PCM'],
            capture_output=True, text=True, timeout=3,
        )
        if r.returncode == 0:
            m = re.search(r'\[(\d+)%\]', r.stdout)
            if m:
                return int(m.group(1))
    except Exception:
        pass
    return -1


def set_mic_gain(value: int) -> tuple:
    """Set mic/capture gain. Returns (success, message, actual_value).
    Tries PulseAudio source first, then ALSA with and without card specifier."""
    import re
    value = max(0, min(100, value))
    # 1. PulseAudio — works regardless of ALSA control naming
    try:
        r = subprocess.run(
            ['pactl', 'set-source-volume', '@DEFAULT_SOURCE@', f'{value}%'],
            capture_output=True, text=True, timeout=3, env=_pulse_env(),
        )
        if r.returncode == 0:
            return True, 'PulseAudio source', value
    except Exception:
        pass
    # 2. ALSA — try ReSpeaker card explicitly, then default card
    _alsa_cards   = [['amixer', '-c', 'ArrayUAC10'], ['amixer', '-c', '2'], ['amixer']]
    _alsa_controls = ('Capture', 'Mic Capture Volume', 'Mic', 'Digital Capture Volume',
                      'Auto Gain Control')
    for card_args in _alsa_cards:
        for control in _alsa_controls:
            try:
                r = subprocess.run(
                    card_args + ['-q', 'sset', control, f'{value}%'],
                    capture_output=True, text=True, timeout=3,
                )
                if r.returncode == 0:
                    return True, f'{" ".join(card_args)} {control}', value
            except Exception:
                continue
    return False, 'No suitable ALSA capture control found', value


def get_mic_gain() -> int:
    """Read current mic/capture gain (0-100). Returns -1 on failure."""
    import re
    # 1. PulseAudio source volume
    try:
        r = subprocess.run(
            ['pactl', 'get-source-volume', '@DEFAULT_SOURCE@'],
            capture_output=True, text=True, timeout=3, env=_pulse_env(),
        )
        if r.returncode == 0:
            m = re.search(r'(\d+)%', r.stdout)
            if m:
                return int(m.group(1))
    except Exception:
        pass
    # 2. ALSA — try ReSpeaker card first, then default
    _alsa_cards    = [['amixer', '-c', 'ArrayUAC10'], ['amixer', '-c', '2'], ['amixer']]
    _alsa_controls = ('Capture', 'Mic Capture Volume', 'Mic', 'Digital Capture Volume')
    for card_args in _alsa_cards:
        for control in _alsa_controls:
            try:
                r = subprocess.run(
                    card_args + ['sget', control],
                    capture_output=True, text=True, timeout=3,
                )
                if r.returncode == 0:
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


def _get_repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


def robot_is_running(node: "DashboardNode") -> bool:
    with node._robot_lock:
        return node._robot_proc is not None and node._robot_proc.poll() is None


ROBOT_LOG = "/tmp/makimate_robot.log"


def robot_start(node: "DashboardNode") -> tuple:
    with node._robot_lock:
        if node._robot_proc is not None and node._robot_proc.poll() is None:
            return False, "Already running"
        try:
            repo = str(_get_repo_root())
            cmd = (
                "export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-} && "
                "source /opt/ros/jazzy/setup.bash && "
                f"source {repo}/install/setup.bash && "
                "if [ -f ~/maki_ws/install/setup.bash ]; then source ~/maki_ws/install/setup.bash; fi && "
                "ros2 launch maki_operational_nodes presentation_mode_v3.launch.py"
            )
            log_fh = open(ROBOT_LOG, "w")
            node._robot_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                start_new_session=True,
                stdout=log_fh,
                stderr=log_fh,
            )
            return True, f"Started (pid {node._robot_proc.pid})"
        except Exception as e:
            return False, str(e)


def robot_stop(node: "DashboardNode") -> tuple:
    import time as _time
    with node._robot_lock:
        proc = node._robot_proc
        if proc is None or proc.poll() is not None:
            node._robot_proc = None
            return False, "Not running"
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        except Exception:
            pass
        # camera_ros holds the libcamera/PISP pipeline even after the top-level
        # process exits — force-kill it and wait for the hardware to fully release
        subprocess.run(["pkill", "-KILL", "-f", "camera_node"], capture_output=True)
        _time.sleep(3.0)
        node._robot_proc = None
        return True, "Stopped"


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

async def _video_broadcast_loop(node: "DashboardNode"):
    """Broadcast camera frames to all WS clients at ~5 fps."""
    import base64
    while True:
        await asyncio.sleep(0.2)
        if node._latest_frame is None or not node._ws_clients:
            continue
        b64 = base64.b64encode(node._latest_frame).decode("ascii")
        await node._broadcast(json.dumps({"type": "video_frame", "data": b64}))


async def _run_cmd_streamed(node: "DashboardNode", cmd: list, cwd: str, label: str) -> bool:
    """Run a shell command and stream its output line-by-line to all WS clients."""
    await node._broadcast(json.dumps({"type": "build_start", "cmd": label}))
    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            cwd=cwd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        async for raw in proc.stdout:
            text = raw.decode("utf-8", errors="ignore").rstrip()
            if text:
                await node._broadcast(json.dumps({"type": "build_line", "text": text}))
        await proc.wait()
        ok = proc.returncode == 0
        await node._broadcast(json.dumps({"type": "build_done", "cmd": label, "ok": ok}))
        return ok
    except Exception as e:
        await node._broadcast(json.dumps({"type": "build_done", "cmd": label, "ok": False, "error": str(e)}))
        return False


async def _pull_and_rebuild(node: "DashboardNode", repo: str):
    ok = await _run_cmd_streamed(node, ["git", "pull"], repo, "git pull")
    if ok:
        ok = await _run_cmd_streamed(node, ["colcon", "build", "--symlink-install"], repo, "colcon build")
    if ok:
        await node._broadcast(json.dumps({
            "type": "build_line",
            "text": "Update complete — use Stop → Start to apply changes.",
        }))


def create_app(node: DashboardNode) -> FastAPI:
    app = FastAPI()
    html_path = Path(__file__).parent / "dashboard.html"

    @app.on_event("startup")
    async def _startup():
        node.set_event_loop(asyncio.get_event_loop())
        # asyncio.create_task(_video_broadcast_loop(node))  # camera disabled

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
        mic = await loop.run_in_executor(None, get_mic_gain)
        if mic >= 0:
            await ws.send_text(json.dumps({"type": "mic_gain_current", "value": mic}))
        ip = await loop.run_in_executor(None, get_local_ip)
        await ws.send_text(json.dumps({"type": "system_info", "ip": ip, "hostname": socket.gethostname()}))
        await ws.send_text(json.dumps({"type": "robot_status", "running": robot_is_running(node)}))
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

    elif t == "mic_gain_set":
        value = int(msg.get("value", 80))
        ok, detail, actual = await loop.run_in_executor(None, set_mic_gain, value)
        if ok:
            await ws.send_text(json.dumps({"type": "mic_gain_ok", "value": actual}))
        else:
            await ws.send_text(json.dumps({"type": "mic_gain_fail", "detail": detail}))

    elif t == "robot_start":
        ok, detail = await loop.run_in_executor(None, robot_start, node)
        await node._broadcast(json.dumps({"type": "robot_status", "running": ok, "detail": detail}))

    elif t == "robot_stop":
        await loop.run_in_executor(None, robot_stop, node)
        await node._broadcast(json.dumps({"type": "robot_status", "running": False}))

    elif t == "git_pull":
        repo = str(_get_repo_root())
        asyncio.create_task(_run_cmd_streamed(node, ["git", "pull"], repo, "git pull"))

    elif t == "rebuild":
        repo = str(_get_repo_root())
        asyncio.create_task(_run_cmd_streamed(node, ["colcon", "build", "--symlink-install"], repo, "colcon build"))

    elif t == "pull_rebuild":
        repo = str(_get_repo_root())
        asyncio.create_task(_pull_and_rebuild(node, repo))

    elif t == "server_config":
        for key in ("llm_server_url", "llm_model", "whisper_server_url", "whisper_model"):
            if key in msg:
                SERVER_CONFIG[key] = msg[key]

        # Propagate to running ROS nodes where applicable
        if "llm_server_url" in msg:
            # Publish directly on a ROS topic — far more reliable than ros2 param set
            from std_msgs.msg import String as _String
            _host_msg = _String()
            _host_msg.data = msg["llm_server_url"]
            node._llm_set_host_pub.publish(_host_msg)
            node.get_logger().info(f"Published LLM host update: {msg['llm_server_url']}")
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
