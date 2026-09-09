#!/usr/bin/env python3
"""Small process supervisor used by the web HMI hardware start button.

The robot-specific commands are launch parameters rather than browser input.  This
keeps the web API fixed while allowing the robot image to provide its own LiDAR or
USB2Dynamixel command.
"""

import os
import signal
import subprocess
import threading
import time
from pathlib import Path


class HardwareManager:
    """Start configured hardware commands and expose a JSON-safe status object."""

    COMPONENTS = ("diablo", "lidar", "dynamixel")
    # Arm feedback is useful when the U2D2 bus is present, but it must not
    # prevent the mobile base and LiDAR from being used when an optional hand
    # bus is unplugged or its probe fails.
    OPTIONAL_COMPONENTS = ("dynamixel",)
    OPTIONAL_PROCESSES = ("localization", "navigation", "mapping")
    MAPPING_COMPONENTS = ("diablo", "lidar")
    # Hardware processes can be started manually (or by an older web bridge)
    # and therefore are not always children of this manager.  OFF HARDWARE
    # must still be able to stop those known robot drivers, while avoiding a
    # broad kill of unrelated ROS nodes.
    EXTERNAL_PROCESS_MARKERS = {
        "diablo": ("diablo_ctrl_node",),
        "lidar": ("sllidar_a2m7_launch.py", "sllidar_node"),
        "dynamixel": ("full_body_hardware.launch.py",),
    }
    # Nodes that may survive a crashed/relaunched ROS launch.  These markers
    # are deliberately limited to the Diablo web/Nav2/SLAM stack; the normal
    # web bridge nodes are recreated by web_interface.launch.py afterwards.
    STARTUP_PROCESS_MARKERS = {
        "localization": (
            "localization.launch.py",
            "amcl",
            "map_server",
            "lifecycle_manager_localization",
        ),
        "navigation": (
            "navigation.launch.py",
            "nav2_web.launch.py",
            "motion_cmd_bridge",
            "controller_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
            "waypoint_follower",
            "velocity_smoother",
            "lifecycle_manager_navigation",
        ),
        "mapping": (
            "mapping.launch.py",
            "slam_toolbox",
            "async_slam_toolbox_node",
            "sync_slam_toolbox_node",
        ),
        "web_support": (
            "web_node",
            "motion_cmd_mux",
            "wheel_odom",
            "diablo_lidar_static_tf",
        ),
    }
    _SHELL_AND_QUERY_COMMANDS = {
        "bash",
        "sh",
        "dash",
        "zsh",
        "grep",
        "pgrep",
        "pkill",
        "ssh",
    }

    def __init__(
        self,
        logger,
        diablo_command="",
        lidar_command="",
        dynamixel_command="",
        lidar_topic="/scan",
        dynamixel_topic="/joint_states",
        log_directory="/tmp",
        feedback_timeout=15.0,
    ):
        self._logger = logger
        self._commands = {
            "diablo": str(diablo_command or "").strip(),
            "lidar": str(lidar_command or "").strip(),
            "dynamixel": str(dynamixel_command or "").strip(),
        }
        self._ready_topics = {
            "diablo": {"/diablo/sensor/Motors"},
            "lidar": {self._normalize_topic(lidar_topic)},
            "dynamixel": {self._normalize_topic(dynamixel_topic)},
        }
        self._log_directory = str(log_directory or "/tmp").strip() or "/tmp"
        self._feedback_timeout = max(1.0, float(feedback_timeout))
        self._lock = threading.RLock()
        self._processes = {}
        self._log_handles = {}
        self._started_at = {}
        self._service_state = {}
        self._topic_names = set()
        self._last_messages = {}
        self._status = self._make_initial_status()

    @staticmethod
    def _normalize_topic(topic):
        value = str(topic or "").strip()
        if not value:
            return ""
        return value if value.startswith("/") else "/" + value

    def _make_initial_status(self):
        components = []
        for component in self.COMPONENTS:
            command = self._commands[component]
            if command:
                state = "offline"
                detail = "Ready to start"
            else:
                state = "not_configured"
                detail = "Start command not configured"
            components.append(
                {
                    "id": component,
                    "label": {
                        "diablo": "DIABLO ROS2",
                        "lidar": "LIDAR",
                        "dynamixel": "DYNAMIXEL ARM U2D2",
                    }[component],
                    "state": state,
                    "detail": detail,
                }
            )
        return {
            "ready": False,
            "all_ready": False,
            "starting": False,
            "message": "Press START HARDWARE to enable manual motion",
            "components": components,
            "updated": time.time(),
        }

    def _component(self, component):
        return next(
            item for item in self._status["components"] if item["id"] == component
        )

    def _set_component(self, component, state, detail):
        item = self._component(component)
        item["state"] = state
        item["detail"] = detail

    def _spawn(self, component, command):
        handle = None
        try:
            os.makedirs(self._log_directory, exist_ok=True)
            log_path = os.path.join(
                self._log_directory, f"diablo_web_interface-{component}.log"
            )
            handle = open(log_path, "ab", buffering=0)
            process = subprocess.Popen(
                ["bash", "-lc", command],
                stdin=subprocess.DEVNULL,
                stdout=handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception as error:
            try:
                if handle is not None:
                    handle.close()
            except Exception:
                pass
            self._logger.error(f"Could not start {component} hardware: {error}")
            self._set_component(component, "error", str(error))
            return False

        self._processes[component] = process
        self._log_handles[component] = handle
        self._started_at[component] = time.monotonic()
        self._set_component(component, "starting", f"Command started (PID {process.pid})")
        self._logger.info(f"Started {component} hardware command (PID {process.pid})")
        return True

    def start_component(self, component, command=None):
        """Start one configured component; returns an acknowledgement dictionary."""
        if component not in self.COMPONENTS:
            return {"requested": False, "message": f"Unknown hardware component: {component}"}

        with self._lock:
            selected_command = (
                self._commands[component]
                if command is None
                else str(command or "").strip()
            )
            if self._message_is_recent(component):
                self._set_component(component, "ready", "ROS feedback already detected")
                return {"requested": True, "message": f"{component} is already ready"}
            process = self._processes.get(component)
            if process is not None and process.poll() is None:
                self._set_component(component, "starting", f"Already running (PID {process.pid})")
                return {"requested": True, "message": f"{component} start already in progress"}
            if process is not None:
                # A previous failed/timeout attempt must not leave a stale
                # Popen handle or file descriptor attached to the next try.
                self._processes.pop(component, None)
                self._started_at.pop(component, None)
                self._close_log(component)
            if not selected_command:
                self._set_component(component, "not_configured", "Start command not configured")
                return {"requested": False, "message": f"{component} start command is not configured"}
            requested = self._spawn(component, selected_command)
            return {
                "requested": requested,
                "message": (
                    f"{component} start requested"
                    if requested
                    else f"Could not start {component}"
                ),
            }

    def start_all(self):
        """Start every configured process.  Service-based LiDAR is handled by the ROS node."""
        results = {}
        requested = False
        for component in self.COMPONENTS:
            result = self.start_component(component)
            results[component] = result
            requested = requested or bool(result.get("requested"))
        with self._lock:
            self._refresh_summary_locked()
            self._status["message"] = (
                "Hardware startup requested; waiting for ROS sensor topics"
                if requested
                else "No hardware start command is configured"
            )
            self._status["updated"] = time.time()
        return {"requested": requested, "message": self._status["message"], "results": results}

    def mark_service_start(self, component, requested, message):
        """Record a service-based start request, typically for a LiDAR driver."""
        with self._lock:
            self._service_state[component] = bool(requested)
            if requested:
                self._set_component(component, "starting", message)
            elif not self._commands.get(component):
                self._set_component(component, "offline", message)
            self._status["updated"] = time.time()

    def mark_message(self, component):
        """Mark a component ready from an actual received ROS message."""
        if component not in self.COMPONENTS:
            return
        with self._lock:
            self._last_messages[component] = time.monotonic()

    def _message_is_recent(self, component):
        return time.monotonic() - self._last_messages.get(component, 0.0) <= 3.0

    def update(self, topic_names):
        """Refresh component readiness from received ROS feedback and processes."""
        topic_set = {
            self._normalize_topic(name)
            for name in (topic_names or [])
            if self._normalize_topic(name)
        }
        with self._lock:
            self._topic_names = topic_set
            for component in self.COMPONENTS:
                process = self._processes.get(component)
                ready_topic = self._message_is_recent(component)
                service_started = self._service_state.get(component, False)

                if ready_topic:
                    self._set_component(component, "ready", "ROS feedback detected")
                    continue
                if process is not None:
                    return_code = process.poll()
                    if return_code is not None:
                        self._set_component(
                            component,
                            "offline" if component in self.OPTIONAL_COMPONENTS else "error",
                            (
                                "Optional hardware skipped after startup failure; see hardware log"
                                if component in self.OPTIONAL_COMPONENTS
                                else f"Process exited with code {return_code}; see hardware log"
                            ),
                        )
                        self._close_log(component)
                        self._processes.pop(component, None)
                        self._started_at.pop(component, None)
                    else:
                        age = time.monotonic() - self._started_at.get(component, time.monotonic())
                        if age >= self._feedback_timeout:
                            # A process that remains alive without publishing
                            # its ready topic is usually a blocked hardware
                            # probe (notably an unplugged Dynamixel bus).
                            # Stop that process group and expose an offline
                            # component instead of an endless warning loop.
                            try:
                                os.killpg(process.pid, signal.SIGTERM)
                                process.wait(timeout=2.0)
                            except Exception:
                                try:
                                    os.killpg(process.pid, signal.SIGKILL)
                                except Exception:
                                    pass
                            self._close_log(component)
                            self._processes.pop(component, None)
                            self._started_at.pop(component, None)
                            self._set_component(
                                component,
                                "offline",
                                "No ROS feedback received before startup timeout",
                            )
                            continue
                        self._set_component(
                            component,
                            "starting" if age < 2.0 else "waiting",
                            "Waiting for ROS topic" if age >= 2.0 else "Process is starting",
                        )
                    continue
                if service_started:
                    self._set_component(component, "starting", "Waiting for ROS topic")
                elif self._commands[component]:
                    self._set_component(component, "offline", "Press START HARDWARE")
                else:
                    self._set_component(component, "not_configured", "Start command not configured")

            self._refresh_summary_locked()

    def _refresh_summary_locked(self):
        """Recompute aggregate gates while ``self._lock`` is held."""
        states = {
            item["id"]: item["state"] for item in self._status["components"]
        }
        ready = states.get("diablo") == "ready"
        mapping_ready = all(
            states.get(component) == "ready"
            for component in self.MAPPING_COMPONENTS
        )
        all_ready = all(
            states.get(component) == "ready" for component in self.COMPONENTS
        )
        # Only components required for drive/mapping contribute to the global
        # startup gate. An optional arm bus may remain in ``waiting`` when the
        # robot has no connected Dynamixel, without blocking SLAM readiness.
        starting = any(
            states.get(component) in ("starting", "waiting")
            for component in self.MAPPING_COMPONENTS
        )
        if all_ready:
            message = "All hardware ready: motors, LiDAR and Dynamixel feedback detected"
        elif mapping_ready:
            message = "Diablo and LiDAR ready; mapping enabled (Dynamixel optional)"
        elif ready:
            message = "Diablo driver ready; waiting for LiDAR feedback"
        elif starting:
            message = "Hardware is starting; waiting for Diablo motor feedback"
        else:
            message = "Start Hardware before using Drive Control"
        self._status.update(
            {
                "ready": ready,
                "mapping_ready": mapping_ready,
                "all_ready": all_ready,
                "starting": starting,
                "message": message,
                "updated": time.time(),
            }
        )

    def snapshot(self):
        with self._lock:
            return {
                "ready": bool(self._status["ready"]),
                "mapping_ready": bool(self._status.get("mapping_ready", False)),
                "all_ready": bool(self._status["all_ready"]),
                "starting": bool(self._status["starting"]),
                "message": str(self._status["message"]),
                "components": [dict(item) for item in self._status["components"]],
                "updated": float(self._status["updated"]),
            }

    def process_snapshots(self):
        """Return statuses for optional launch groups owned by the web UI."""
        return {
            name: self.process_status(name)
            for name in self.OPTIONAL_PROCESSES
        }

    def is_ready(self):
        with self._lock:
            return bool(self._status["ready"])

    def is_all_ready(self):
        with self._lock:
            return bool(self._status["all_ready"])

    def is_mapping_ready(self):
        with self._lock:
            return bool(self._status.get("mapping_ready", False))

    def start_process(self, name, command):
        """Start an optional navigation process using a launch-time command."""
        clean_name = str(name or "process").strip().replace(" ", "_")
        clean_command = str(command or "").strip()
        if not clean_command:
            return {
                "requested": False,
                "message": f"{clean_name} command is not configured",
            }
        with self._lock:
            process = self._processes.get(clean_name)
            if process is not None and process.poll() is None:
                return {"requested": True, "message": f"{clean_name} is already running"}
            handle = None
            try:
                os.makedirs(self._log_directory, exist_ok=True)
                log_path = os.path.join(
                    self._log_directory, f"diablo_web_interface-{clean_name}.log"
                )
                handle = open(log_path, "ab", buffering=0)
                process = subprocess.Popen(
                    ["bash", "-lc", clean_command],
                    stdin=subprocess.DEVNULL,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except Exception as error:
                try:
                    if handle is not None:
                        handle.close()
                except Exception:
                    pass
                return {"requested": False, "message": str(error)}
            self._processes[clean_name] = process
            self._log_handles[clean_name] = handle
            self._started_at[clean_name] = time.monotonic()
            return {"requested": True, "message": f"{clean_name} start requested (PID {process.pid})"}

    def process_status(self, name):
        """Return the status of an optional process managed by this supervisor."""
        clean_name = str(name or "process").strip().replace(" ", "_")
        with self._lock:
            process = self._processes.get(clean_name)
            if process is None:
                return {
                    "name": clean_name,
                    "state": "idle",
                    "active": False,
                    "pid": None,
                    "message": "Not running",
                }
            return_code = process.poll()
            if return_code is None:
                return {
                    "name": clean_name,
                    "state": "running",
                    "active": True,
                    "pid": process.pid,
                    "message": f"Running (PID {process.pid})",
                }
            self._close_log(clean_name)
            return {
                "name": clean_name,
                "state": "stopped" if return_code == 0 else "error",
                "active": False,
                "pid": process.pid,
                "message": (
                    "Exited normally"
                    if return_code == 0
                    else f"Process exited with code {return_code}; see hardware log"
                ),
            }

    @staticmethod
    def _parent_pid(pid):
        """Read a process parent PID without invoking shell utilities."""
        try:
            status = Path(f"/proc/{pid}/status").read_text(
                encoding="utf-8", errors="replace"
            )
        except (OSError, ValueError):
            return None
        for line in status.splitlines():
            if line.startswith("PPid:"):
                try:
                    return int(line.split()[1])
                except (IndexError, ValueError):
                    return None
        return None

    def _process_tree_groups(self, root_pid):
        """Return process groups for a launch process and all live descendants."""
        descendants = {int(root_pid)}
        changed = True
        while changed:
            changed = False
            try:
                entries = os.listdir("/proc")
            except OSError:
                break
            for entry in entries:
                if not entry.isdigit():
                    continue
                pid = int(entry)
                if pid in descendants:
                    continue
                parent = self._parent_pid(pid)
                if parent in descendants:
                    descendants.add(pid)
                    changed = True

        groups = set()
        own_group = os.getpgrp()
        for pid in descendants:
            try:
                group = os.getpgid(pid)
            except (ProcessLookupError, PermissionError, OSError):
                continue
            if group not in (0, 1, own_group):
                groups.add(group)
        return groups

    def stop_process(self, name):
        """Stop one optional launch and all of its child process groups."""
        clean_name = str(name or "process").strip().replace(" ", "_")
        with self._lock:
            process = self._processes.get(clean_name)
            if process is None or process.poll() is not None:
                if process is not None:
                    self._close_log(clean_name)
                return {"requested": False, "message": f"{clean_name} is not running"}
            groups = self._process_tree_groups(process.pid)
            if not groups:
                # The launch process normally has its own session/process
                # group, but keep a safe fallback for a short /proc race.
                groups = {process.pid}
            for group in groups:
                try:
                    os.killpg(group, signal.SIGTERM)
                except (ProcessLookupError, PermissionError, OSError):
                    continue
            try:
                process.wait(timeout=2.0)
            except Exception:
                pass
            # Children may use separate process groups.  Escalate only those
            # groups discovered from this launch tree, never the web bridge.
            for group in tuple(groups):
                try:
                    os.killpg(group, 0)
                except (ProcessLookupError, PermissionError, OSError):
                    groups.discard(group)
            for group in groups:
                try:
                    os.killpg(group, signal.SIGKILL)
                except (ProcessLookupError, PermissionError, OSError):
                    pass
            self._close_log(clean_name)
            # SIGTERM requested here is a normal stop. Drop the completed
            # handle so a later poll cannot reinterpret it as a crash.
            self._processes.pop(clean_name, None)
            self._started_at.pop(clean_name, None)
            return {"requested": True, "message": f"{clean_name} stopped"}

    @staticmethod
    def _argument_matches_marker(argument, marker):
        return (
            argument == marker
            or argument.endswith(f"/{marker}")
            or argument.endswith(f":={marker}")
        )

    def _process_pids_for_markers(self, markers):
        """Find processes whose argv contains one of the exact markers.

        Reading ``/proc/*/cmdline`` avoids matching this manager's own grep
        command.  Matching complete argv tokens (or executable/node-name
        suffixes) prevents a query such as ``grep amcl`` from being selected.
        """
        markers = tuple(markers or ())
        if not markers:
            return []
        pids = []
        try:
            entries = os.listdir("/proc")
        except OSError:
            return pids
        for entry in entries:
            if not entry.isdigit():
                continue
            pid = int(entry)
            if pid == os.getpid():
                continue
            try:
                command_line = Path(f"/proc/{pid}/cmdline").read_bytes()
            except (OSError, ValueError):
                continue
            arguments = [
                item.decode("utf-8", "replace")
                for item in command_line.split(b"\0")
                if item
            ]
            if not arguments:
                continue
            # A remote ``grep``/shell command may contain a driver name in
            # its search expression.  It is not a driver and must never be
            # selected for group termination.
            if Path(arguments[0]).name in self._SHELL_AND_QUERY_COMMANDS:
                continue
            if any(
                self._argument_matches_marker(argument, marker)
                for argument in arguments
                for marker in markers
            ):
                pids.append(pid)
        return pids

    def _external_process_pids(self, component):
        """Find known driver processes not owned by this supervisor."""
        return self._process_pids_for_markers(
            self.EXTERNAL_PROCESS_MARKERS.get(component, ())
        )

    def _stop_groups(self, groups, label):
        """Terminate selected process groups and return a JSON-safe result."""
        own_group = os.getpgrp()
        safe_groups = {
            int(group) for group in groups if int(group) not in (0, 1, own_group)
        }
        terminated = []
        for group in safe_groups:
            try:
                os.killpg(group, signal.SIGTERM)
                terminated.append(group)
            except (ProcessLookupError, PermissionError, OSError):
                continue

        deadline = time.monotonic() + 2.0
        remaining = set(terminated)
        while remaining and time.monotonic() < deadline:
            for group in tuple(remaining):
                try:
                    os.killpg(group, 0)
                except (ProcessLookupError, PermissionError, OSError):
                    remaining.discard(group)
            if remaining:
                time.sleep(0.05)
        for group in remaining:
            try:
                os.killpg(group, signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                pass

        if not terminated:
            return {
                "requested": False,
                "message": f"No stale {label} process found",
            }
        return {
            "requested": True,
            "message": f"Stopped stale {label} process group(s)",
            "groups": sorted(terminated),
        }

    def _stop_marker_processes(self, markers, label):
        pids = self._process_pids_for_markers(markers)
        groups = set()
        for pid in pids:
            groups.update(self._process_tree_groups(pid))
        return self._stop_groups(groups, label)

    def _stop_external_component(self, component):
        """Stop process groups belonging to a known external hardware driver."""
        groups = {}
        for pid in self._external_process_pids(component):
            for group in self._process_tree_groups(pid):
                groups[group] = groups.get(group, 0) + 1
        result = self._stop_groups(groups, f"external {component} driver")
        if not result["requested"]:
            result["message"] = f"No external {component} driver process found"
        return result

    def startup_cleanup(self):
        """Make a fresh web instance the sole owner of robot runtime processes.

        This is safe to call repeatedly.  It removes stale Nav2/SLAM and web
        support process groups left by a previous launch, then stops known
        external hardware drivers before the new UI exposes START buttons.
        """
        processes = {}
        for name, markers in self.STARTUP_PROCESS_MARKERS.items():
            processes[name] = self._stop_marker_processes(markers, name)
        hardware = {}
        for component in reversed(self.COMPONENTS):
            hardware[component] = self._stop_external_component(component)

        with self._lock:
            self._processes.clear()
            self._started_at.clear()
            self._service_state = {
                component: False for component in self.COMPONENTS
            }
            self._last_messages = {
                component: 0.0 for component in self.COMPONENTS
            }
            for component in self.COMPONENTS:
                if self._commands[component]:
                    self._set_component(component, "offline", "Startup cleanup complete")
                else:
                    self._set_component(
                        component, "not_configured", "Start command not configured"
                    )
            self._status.update(
                {
                    "ready": False,
                    "mapping_ready": False,
                    "all_ready": False,
                    "starting": False,
                    "message": "Startup cleanup complete; hardware is OFF",
                    "updated": time.time(),
                }
            )
        requested = any(item.get("requested") for item in processes.values()) or any(
            item.get("requested") for item in hardware.values()
        )
        return {
            "requested": requested,
            "message": "Startup cleanup complete; hardware and launches are OFF",
            "processes": processes,
            "hardware": hardware,
        }

    def stop_hardware(self):
        """Stop all known hardware drivers and reset their readiness state."""
        results = {}
        for component in reversed(self.COMPONENTS):
            results[component] = self.stop_process(component)
        external = {}
        for component in reversed(self.COMPONENTS):
            external[component] = self._stop_external_component(component)

        with self._lock:
            # Also clear handles that had already exited before OFF was
            # requested. Their old SIGTERM/error return code must not leak
            # into the next hardware state refresh.
            for component in self.COMPONENTS:
                self._processes.pop(component, None)
                self._started_at.pop(component, None)
            self._service_state = {
                component: False for component in self._service_state
            }
            self._last_messages = {
                component: 0.0 for component in self._last_messages
            }
            for component in self.COMPONENTS:
                if self._commands[component]:
                    self._set_component(component, "offline", "Hardware stopped")
                else:
                    self._set_component(
                        component, "not_configured", "Start command not configured"
                    )
            self._status.update(
                {
                    "ready": False,
                    "mapping_ready": False,
                    "all_ready": False,
                    "starting": False,
                    "message": "Hardware stopped",
                    "updated": time.time(),
                }
            )
        requested = any(item.get("requested") for item in results.values()) or any(
            item.get("requested") for item in external.values()
        )
        return {
            "requested": requested,
            "message": "Hardware stop requested",
            "results": results,
            "external": external,
        }

    def _close_log(self, component):
        handle = self._log_handles.pop(component, None)
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

    def stop(self):
        """Stop only process groups created by this manager."""
        with self._lock:
            processes = list(self._processes.items())
            self._processes.clear()
        for name, process in processes:
            if process.poll() is not None:
                self._close_log(name)
                continue
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=1.0)
            except Exception:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except Exception:
                    pass
            self._close_log(name)
