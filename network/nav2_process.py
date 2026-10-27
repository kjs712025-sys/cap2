"""Lifecycle management for the ROS 2 bridge, Nav2, and SLAM processes."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any


class Nav2Process:
    """Manage fixed ROS launch commands without accepting shell input."""

    def __init__(self, project_root: Path) -> None:
        self.project_root = project_root
        self.workspace = project_root / "ros2_ws"
        self.maps_dir = project_root / "maps"
        self._processes: dict[str, subprocess.Popen[bytes]] = {}
        self._log_files: dict[str, Any] = {}
        self._lock = threading.RLock()
        self._started_at: dict[str, float] = {}
        self._last_exit_code: dict[str, int | None] = {
            "bridge": None,
            "nav2": None,
            "slam": None,
        }

    def state(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_processes_locked()
            return self._state_locked()

    def start(self) -> tuple[bool, str, dict[str, Any]]:
        with self._lock:
            bridge_started, bridge_message = self._ensure_bridge_locked()
            if not bridge_started:
                return False, bridge_message, self._state_locked()
            started, message = self._start_launch_locked(
                "nav2",
                "enable_bridge:=false enable_slam:=false enable_nav2:=true",
            )
            if not started:
                self._stop_bridge_if_unused_locked()
                return False, message, self._state_locked()
            return True, message, self._state_locked()

    def start_slam(self) -> tuple[bool, str, dict[str, Any]]:
        with self._lock:
            bridge_started, bridge_message = self._ensure_bridge_locked()
            if not bridge_started:
                return False, bridge_message, self._state_locked()
            started, message = self._start_launch_locked(
                "slam",
                "enable_bridge:=false enable_slam:=true enable_nav2:=false",
            )
            if not started:
                self._stop_bridge_if_unused_locked()
                return False, message, self._state_locked()
            return True, message, self._state_locked()

    def stop(self) -> tuple[bool, str, dict[str, Any]]:
        with self._lock:
            self._stop_role_locked("nav2")
            self._stop_bridge_if_unused_locked()
            return True, "Nav2 stopped.", self._state_locked()

    def stop_slam(self) -> tuple[bool, str, dict[str, Any]]:
        with self._lock:
            self._stop_role_locked("slam")
            self._stop_bridge_if_unused_locked()
            return True, "SLAM stopped.", self._state_locked()

    def stop_all(self) -> None:
        with self._lock:
            for role in ("nav2", "slam", "bridge"):
                self._stop_role_locked(role)

    def save_map(self) -> tuple[bool, str, dict[str, str]]:
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        map_prefix = self.maps_dir / "nav2_map"
        command = (
            "source /opt/ros/jazzy/setup.bash && "
            "source install/setup.bash && "
            "exec ros2 run nav2_map_server map_saver_cli -f \"$1\""
        )
        try:
            result = subprocess.run(
                ["/bin/bash", "--noprofile", "--norc", "-c", command, "map-saver", str(map_prefix)],
                cwd=self.workspace,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=45.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, f"Could not save ROS map: {exc}", {}

        yaml_path = map_prefix.with_suffix(".yaml")
        image_path = map_prefix.with_suffix(".pgm")
        if result.returncode != 0:
            details = (result.stdout or "").strip()[-800:]
            return False, f"Map saver exited with code {result.returncode}: {details}", {}
        if not yaml_path.is_file() or not image_path.is_file():
            return False, "Map saver completed without producing both map files.", {}
        return True, "ROS map saved.", {
            "yaml": yaml_path.name,
            "image": image_path.name,
        }

    def _ensure_bridge_locked(self) -> tuple[bool, str]:
        self._refresh_processes_locked()
        if self._is_running_locked("bridge"):
            return True, "ROS backend bridge is already running."
        if not self._environment_ready_locked():
            return False, self._environment_error_locked()
        self._close_log_locked("bridge")
        log_file = self._open_log_locked("bridge")
        try:
            process = subprocess.Popen(
                [
                    "/bin/bash", "--noprofile", "--norc", "-c",
                    "source /opt/ros/jazzy/setup.bash && "
                    "source install/setup.bash && "
                    "exec ros2 run omnibot_nav2 backend_bridge",
                ],
                cwd=self.workspace,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                close_fds=True,
            )
        except OSError as exc:
            self._close_log_locked("bridge")
            return False, f"Could not start ROS backend bridge: {exc}"
        self._processes["bridge"] = process
        self._started_at["bridge"] = time.time()
        self._last_exit_code["bridge"] = None
        return True, "ROS backend bridge started."

    def _start_launch_locked(self, role: str, launch_args: str) -> tuple[bool, str]:
        self._refresh_processes_locked()
        if self._is_running_locked(role):
            return True, f"{role.upper()} is already running."
        if not self._environment_ready_locked():
            return False, self._environment_error_locked()
        self._close_log_locked(role)
        log_file = self._open_log_locked(role)
        script = (
            "source /opt/ros/jazzy/setup.bash && "
            "source install/setup.bash && "
            f"exec ros2 launch omnibot_nav2 omnibot_nav2.launch.py {launch_args}"
        )
        try:
            process = subprocess.Popen(
                ["/bin/bash", "--noprofile", "--norc", "-c", script],
                cwd=self.workspace,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                close_fds=True,
            )
        except OSError as exc:
            self._close_log_locked(role)
            return False, f"Could not start {role.upper()}: {exc}"
        self._processes[role] = process
        self._started_at[role] = time.time()
        self._last_exit_code[role] = None
        return True, f"{role.upper()} launch process started."

    def _environment_ready_locked(self) -> bool:
        return (
            Path("/opt/ros/jazzy/setup.bash").is_file()
            and (self.workspace / "install" / "setup.bash").is_file()
        )

    def _environment_error_locked(self) -> str:
        if not Path("/opt/ros/jazzy/setup.bash").is_file():
            return "ROS 2 Jazzy setup was not found at /opt/ros/jazzy."
        return "Nav2 workspace is not built. Build ros2_ws before starting ROS 2."

    def _state_locked(self) -> dict[str, Any]:
        return {
            **self._role_state_locked("nav2"),
            "slam_running": self._is_running_locked("slam"),
            "slam_pid": self._pid_locked("slam"),
            "slam_started_at": self._started_at.get("slam") if self._is_running_locked("slam") else None,
            "slam_last_exit_code": self._last_exit_code["slam"],
            "bridge_running": self._is_running_locked("bridge"),
            "bridge_pid": self._pid_locked("bridge"),
            "workspace_built": (self.workspace / "install" / "setup.bash").is_file(),
            "ros_setup_available": Path("/opt/ros/jazzy/setup.bash").is_file(),
        }

    def _role_state_locked(self, role: str) -> dict[str, Any]:
        return {
            "running": self._is_running_locked(role),
            "pid": self._pid_locked(role),
            "started_at": self._started_at.get(role) if self._is_running_locked(role) else None,
            "last_exit_code": self._last_exit_code[role],
        }

    def _refresh_processes_locked(self) -> None:
        for role, process in tuple(self._processes.items()):
            exit_code = process.poll()
            if exit_code is not None:
                self._last_exit_code[role] = exit_code
                self._processes.pop(role, None)
                self._close_log_locked(role)

    def _is_running_locked(self, role: str) -> bool:
        process = self._processes.get(role)
        return process is not None and process.poll() is None

    def _pid_locked(self, role: str) -> int | None:
        process = self._processes.get(role)
        return process.pid if process is not None and process.poll() is None else None

    def _stop_bridge_if_unused_locked(self) -> None:
        self._refresh_processes_locked()
        if not self._is_running_locked("nav2") and not self._is_running_locked("slam"):
            self._stop_role_locked("bridge")

    def _stop_role_locked(self, role: str) -> None:
        process = self._processes.get(role)
        if process is None or process.poll() is not None:
            if process is not None:
                self._last_exit_code[role] = process.returncode
            self._processes.pop(role, None)
            self._close_log_locked(role)
            return

        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5.0)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=2.0)

        self._last_exit_code[role] = process.returncode
        self._processes.pop(role, None)
        self._close_log_locked(role)

    def _open_log_locked(self, role: str) -> Any:
        logs_dir = self.project_root / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)
        log_file = (logs_dir / f"{role}.log").open("ab", buffering=0)
        self._log_files[role] = log_file
        return log_file

    def _close_log_locked(self, role: str) -> None:
        log_file = self._log_files.pop(role, None)
        if log_file is not None:
            log_file.close()
