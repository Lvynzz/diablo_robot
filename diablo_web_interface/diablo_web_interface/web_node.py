#!/usr/bin/env python3
"""FastAPI browser interface for Diablo teleoperation and Nav2."""

import asyncio
import logging
import os
from pathlib import Path
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
import uvicorn

from .ros_node import DiabloWebNode, ros_value_to_bounded_data


logger = logging.getLogger("diablo_web_interface.web")
logging.basicConfig(level=logging.INFO)

app = FastAPI(
    title="Diablo Nav2 Web Interface",
    description="Browser teleoperation, Nav2 goal control and ROS topic echo for Diablo",
    version="0.1.0",
)

cors_env = os.environ.get("DIABLO_WEB_CORS_ORIGINS", "").strip()
cors_origins = [item.strip() for item in cors_env.split(",") if item.strip()] or ["*"]
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=False,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type"],
)

ROS_NODE = None
ROS_THREAD = None
_SHUTTING_DOWN = False


def _static_dir():
    """Resolve source-tree and install-space static asset locations."""
    source_root = Path(__file__).resolve().parent.parent
    built_source = source_root / "dist"
    if (built_source / "index.html").is_file():
        return built_source
    try:
        installed = Path(get_package_share_directory("diablo_web_interface")) / "static"
        if installed.is_dir():
            return installed
    except Exception:
        pass
    return source_root / "diablo_web_interface" / "static"


def _require_node():
    if ROS_NODE is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    return ROS_NODE


def _number(payload, name, default=0.0):
    try:
        value = float(payload.get(name, default))
    except (TypeError, ValueError):
        raise HTTPException(status_code=400, detail=f"Invalid numeric field: {name}")
    if value != value or value in (float("inf"), float("-inf")):
        raise HTTPException(status_code=400, detail=f"Non-finite numeric field: {name}")
    return value


def _motion_fields(payload):
    return {
        "forward": _number(payload, "forward"),
        "left": _number(payload, "left"),
        "roll": _number(payload, "roll"),
        "up": _number(payload, "up", 1.0),
        "pitch": _number(payload, "pitch"),
        "mode_mark": bool(payload.get("mode_mark", False)),
        "height_ctrl_mode": bool(payload.get("height_ctrl_mode", False)),
        "pitch_ctrl_mode": bool(payload.get("pitch_ctrl_mode", False)),
        "roll_ctrl_mode": bool(payload.get("roll_ctrl_mode", False)),
        "stand_mode": bool(payload.get("stand_mode", False)),
        "jump_mode": bool(payload.get("jump_mode", False)),
        "split_mode": bool(payload.get("split_mode", False)),
    }


@app.middleware("http")
async def security_headers(request, call_next):
    response = await call_next(request)
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["Referrer-Policy"] = "same-origin"
    response.headers[
        "Content-Security-Policy"
    ] = (
        "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; "
        "img-src 'self' data:; connect-src 'self' ws: wss:;"
    )
    return response


@app.get("/healthz")
@app.get("/api/healthz")
async def healthz():
    if ROS_NODE is None:
        return JSONResponse(
            {"ok": False, "status": "initializing", "ros_ready": False},
            status_code=503,
        )
    return {"ok": True, "status": "ok", "ros_ready": True, "time": time.time()}


@app.get("/api/config")
async def config():
    node = _require_node()
    return {
        "robot": "Diablo",
        "manual_cmd_topic": node.manual_cmd_topic,
        "control_mode_topic": node.control_mode_topic,
        "map_topic": node.map_topic,
        "odom_topic": node.odom_topic,
        "scan_topic": node.scan_topic,
        "base_frame": node.base_frame,
        "map_frame": node.map_frame,
        "reset_encoder_service": node.reset_encoder_service,
        "lidar_start_service": node.lidar_start_service,
        "diablo_start_command": node.diablo_start_command,
        "lidar_start_command": node.lidar_start_command,
        "dynamixel_start_command": node.dynamixel_start_command,
        "localization_start_command": node.localization_start_command,
        "navigation_start_command": node.navigation_start_command,
        "mapping_start_command": node.mapping_start_command,
        "maps_dir": str(node.maps_dir),
        "limits": {
            "forward": node.max_forward,
            "turn": node.max_turn,
            "roll": node.max_roll,
        },
    }


@app.get("/api/status")
async def status():
    node = _require_node()
    snapshot = node.snapshot()
    return {
        "connected": True,
        "robot": "Diablo",
        "nav2_ready": node.nav2_ready(),
        "control_mode": node._control_mode,
        "pose": snapshot.get("pose"),
        "wheel_pose": snapshot.get("wheel_pose"),
        "wheel_trajectory": snapshot.get("wheel_trajectory"),
        "telemetry": snapshot.get("telemetry"),
        "hardware": snapshot.get("hardware"),
        "navigation": node.get_nav_goal_status(),
    }


@app.get("/api/topics")
@app.get("/api/diagnostics/topics")
async def topics():
    return {"topics": _require_node().list_ros_topics()}


@app.get("/api/maps")
async def maps():
    return _require_node().list_maps()


@app.post("/api/control/mode")
@app.post("/api/drive/mode")
async def set_control_mode(payload: dict):
    mode = str(payload.get("mode", "manual")).strip().lower()
    try:
        mode = _require_node().set_control_mode(mode)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error))
    return {"mode": mode}


@app.post("/api/control/stop")
@app.post("/api/drive/stop")
async def stop_control():
    _require_node().publish_stop()
    return {"status": "stopped", "mode": "manual"}


@app.post("/api/teleop")
@app.post("/api/drive/joystick")
async def teleop(payload: dict):
    node = _require_node()
    try:
        node.publish_manual_command(**_motion_fields(payload))
    except RuntimeError as error:
        raise HTTPException(status_code=409, detail=str(error))
    return {"status": "received", "mode": "manual"}


@app.post("/api/teleop/stand")
async def stand(payload: dict):
    node = _require_node()
    try:
        node.publish_stand_command(bool(payload.get("stand", True)))
    except RuntimeError as error:
        raise HTTPException(status_code=409, detail=str(error))
    return {"status": "received", "stand": bool(payload.get("stand", True))}


@app.post("/api/odom/reset")
@app.post("/api/control/reset_odom")
async def reset_odom():
    return _require_node().reset_odom()


@app.post("/api/encoder/reset")
@app.post("/api/control/reset_encoder")
async def reset_encoder():
    return _require_node().reset_encoder()


@app.post("/api/sensors/lidar/start")
@app.post("/api/lidar/start")
async def start_lidar():
    return _require_node().start_lidar()


@app.post("/api/hardware/start")
@app.post("/api/control/start_hardware")
async def start_hardware():
    return _require_node().start_hardware()


@app.post("/api/navigation/start-localization")
@app.post("/api/localization/start")
async def start_localization():
    return _require_node().start_localization()


@app.post("/api/navigation/start")
async def start_navigation():
    return _require_node().start_navigation()


@app.post("/api/mapping/start")
async def start_mapping():
    return _require_node().start_mapping()


@app.post("/api/goal/nav2")
async def nav_goal(payload: dict):
    node = _require_node()
    try:
        result = node.send_nav_goal(
            _number(payload, "x"),
            _number(payload, "y"),
            _number(payload, "theta"),
        )
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error))
    if not result.get("accepted"):
        raise HTTPException(status_code=503, detail=result.get("message", "Goal rejected"))
    return result


@app.post("/api/goal/cancel")
async def cancel_goal():
    return _require_node().cancel_nav_goal()


@app.post("/api/localization/initialpose")
async def initial_pose(payload: dict):
    node = _require_node()
    try:
        return node.set_initial_pose(
            _number(payload, "x"),
            _number(payload, "y"),
            _number(payload, "theta"),
        )
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error))


@app.post("/api/diagnostics/echo/{topic_name:path}")
async def echo_topic_info(topic_name: str):
    node = _require_node()
    try:
        topic_types = node.resolve_topic_types(topic_name)
    except (ValueError, IndexError):
        raise HTTPException(status_code=404, detail="ROS topic not found")
    return {"topic": topic_name, "types": topic_types}


@app.websocket("/ws")
async def state_websocket(websocket: WebSocket):
    """WebSocket for teleop commands and live navigation/telemetry state."""
    await websocket.accept()
    versions = {}
    try:
        while True:
            try:
                raw_message = await asyncio.wait_for(
                    websocket.receive_text(), timeout=0.05
                )
                await _handle_ws_command(websocket, raw_message)
            except asyncio.TimeoutError:
                pass
            except WebSocketDisconnect:
                raise
            except Exception as error:
                await websocket.send_json({"type": "error", "detail": str(error)})

            node = ROS_NODE
            if node is not None:
                packet = node.snapshot(versions)
                versions = packet["versions"]
                packet["nav2_ready"] = node.nav2_ready()
                await websocket.send_json(packet)
    except WebSocketDisconnect:
        if ROS_NODE is not None:
            ROS_NODE.publish_stop()


async def _handle_ws_command(websocket: WebSocket, raw_message: str):
    import json

    payload = json.loads(raw_message)
    command_type = str(payload.get("type", "")).strip().lower()
    node = _require_node()

    if command_type in ("manual", "motion", "joystick"):
        node.publish_manual_command(**_motion_fields(payload))
    elif command_type == "stop":
        node.publish_stop()
    elif command_type == "stand":
        node.publish_stand_command(bool(payload.get("stand", True)))
    elif command_type == "reset_odom":
        await websocket.send_json({"type": "reset_odom_ack", **node.reset_odom()})
    elif command_type == "reset_encoder":
        await websocket.send_json({"type": "reset_encoder_ack", **node.reset_encoder()})
    elif command_type == "start_lidar":
        await websocket.send_json({"type": "start_lidar_ack", **node.start_lidar()})
    elif command_type in ("start_hardware", "hardware_start"):
        await websocket.send_json({"type": "start_hardware_ack", **node.start_hardware()})
    elif command_type in ("start_localization", "localization_start"):
        await websocket.send_json(
            {"type": "start_localization_ack", **node.start_localization()}
        )
    elif command_type in ("start_navigation", "navigation_start"):
        await websocket.send_json(
            {"type": "start_navigation_ack", **node.start_navigation()}
        )
    elif command_type in ("start_mapping", "mapping_start"):
        await websocket.send_json({"type": "start_mapping_ack", **node.start_mapping()})
    elif command_type == "mode":
        node.set_control_mode(str(payload.get("mode", "manual")))
    elif command_type in ("goal_pose", "goal"):
        result = node.send_nav_goal(
            _number(payload, "x"),
            _number(payload, "y"),
            _number(payload, "theta"),
        )
        await websocket.send_json({"type": "goal_pose_ack", **result})
    elif command_type in ("cancel_goal", "cancel"):
        result = node.cancel_nav_goal()
        await websocket.send_json({"type": "goal_cancel_ack", **result})
    elif command_type == "initial_pose":
        result = node.set_initial_pose(
            _number(payload, "x"),
            _number(payload, "y"),
            _number(payload, "theta"),
        )
        await websocket.send_json({"type": "initial_pose_ack", **result})
    elif command_type == "ping":
        await websocket.send_json({"type": "pong"})
    elif command_type:
        raise ValueError(f"Unknown websocket command: {command_type}")


@app.websocket("/ws/topics")
async def topic_websocket(websocket: WebSocket):
    """Subscribe to up to four dynamically selected ROS topics."""
    await websocket.accept()
    subscriptions = []
    states = {}
    state_lock = threading.Lock()
    next_send = 0.0
    min_interval = 0.2

    def clear_subscriptions():
        for subscription in subscriptions:
            try:
                if ROS_NODE is not None:
                    ROS_NODE.destroy_echo_subscription(subscription)
            except Exception as error:
                logger.warning("Failed to destroy topic echo subscription: %s", error)
        subscriptions.clear()
        with state_lock:
            states.clear()

    def callback_for(slot):
        def callback(message):
            now = time.monotonic()
            with state_lock:
                state = states.get(slot)
                if state is None or now - state["last"] < min_interval:
                    return
                state["last"] = now
            try:
                data = ros_value_to_bounded_data(message)
            except Exception as error:
                data = {"error": str(error)}
            with state_lock:
                state = states.get(slot)
                if state is not None:
                    state["count"] += 1
                    state["stamp"] = time.strftime("%H:%M:%S")
                    state["data"] = data
                    state["dirty"] = True

        return callback

    async def subscribe(topic_names):
        if not isinstance(topic_names, list):
            raise ValueError("topics must be a list")
        clean_names = []
        for item in topic_names:
            name = str(item).strip()
            if name and not name.startswith("/"):
                name = "/" + name
            if name and name not in clean_names:
                clean_names.append(name)
        if len(clean_names) > 4:
            raise ValueError("Topic echo is limited to four topics")

        clear_subscriptions()
        slots = []
        for slot, name in enumerate(clean_names, start=1):
            with state_lock:
                states[slot] = {
                    "topic": name,
                    "msg_type": "",
                    "count": 0,
                    "stamp": "waiting",
                    "data": {"status": "waiting for message"},
                    "dirty": True,
                    "last": 0.0,
                }
            subscription, msg_type = ROS_NODE.create_echo_subscription(
                name, callback_for(slot)
            )
            subscriptions.append(subscription)
            with state_lock:
                states[slot]["msg_type"] = msg_type
            slots.append({"slot": slot, "topic": name, "msg_type": msg_type})
        await websocket.send_json({"type": "subscribed", "slots": slots})

    try:
        while True:
            try:
                raw_message = await asyncio.wait_for(
                    websocket.receive_text(), timeout=0.1
                )
                import json

                payload = json.loads(raw_message)
                command_type = str(payload.get("type", "")).strip().lower()
                if command_type == "subscribe":
                    await subscribe(payload.get("topics", []))
                elif command_type == "clear":
                    clear_subscriptions()
                    await websocket.send_json({"type": "subscribed", "slots": []})
            except asyncio.TimeoutError:
                pass
            except WebSocketDisconnect:
                raise
            except Exception as error:
                await websocket.send_json({"type": "error", "detail": str(error)})

            now = time.monotonic()
            if now >= next_send:
                next_send = now + 0.1
                dirty_packets = []
                with state_lock:
                    for slot, state in states.items():
                        if state["dirty"]:
                            state["dirty"] = False
                            dirty_packets.append(
                                {
                                    "type": "topic",
                                    "slot": slot,
                                    "topic": state["topic"],
                                    "msg_type": state["msg_type"],
                                    "count": state["count"],
                                    "stamp": state["stamp"],
                                    "data": state["data"],
                                }
                            )
                for packet in dirty_packets:
                    await websocket.send_json(packet)
    except WebSocketDisconnect:
        clear_subscriptions()


@app.get("/")
async def index():
    return FileResponse(_static_dir() / "index.html")


@app.get("/static/{asset_path:path}")
async def static_asset(asset_path: str):
    root = _static_dir().resolve()
    candidate = (root / asset_path).resolve()
    if root not in candidate.parents or not candidate.is_file():
        raise HTTPException(status_code=404, detail="Asset not found")
    return FileResponse(candidate)


def _start_ros_node():
    global ROS_NODE, ROS_THREAD
    if ROS_NODE is not None:
        return
    if not rclpy.ok():
        rclpy.init()
    ROS_NODE = DiabloWebNode()

    def spin():
        try:
            rclpy.spin(ROS_NODE)
        except Exception as error:
            if not _SHUTTING_DOWN:
                logger.exception("ROS spin stopped: %s", error)

    ROS_THREAD = threading.Thread(target=spin, name="diablo-web-ros", daemon=True)
    ROS_THREAD.start()


def _stop_ros_node():
    global ROS_NODE, ROS_THREAD, _SHUTTING_DOWN
    _SHUTTING_DOWN = True
    if ROS_NODE is not None:
        try:
            ROS_NODE.publish_stop()
            ROS_NODE.destroy_node()
        except Exception:
            logger.exception("Error while stopping Diablo web ROS node")
        ROS_NODE = None
    if rclpy.ok():
        rclpy.shutdown()
    if ROS_THREAD is not None:
        ROS_THREAD.join(timeout=1.0)
        ROS_THREAD = None


@app.on_event("startup")
async def startup_event():
    _start_ros_node()


@app.on_event("shutdown")
async def shutdown_event():
    _stop_ros_node()


def main():
    host = os.environ.get("DIABLO_WEB_HOST", "0.0.0.0")
    try:
        port = int(os.environ.get("DIABLO_WEB_PORT", "8000"))
    except ValueError:
        port = 8000
    uvicorn.run(app, host=host, port=port, log_level="info")


if __name__ == "__main__":
    main()
