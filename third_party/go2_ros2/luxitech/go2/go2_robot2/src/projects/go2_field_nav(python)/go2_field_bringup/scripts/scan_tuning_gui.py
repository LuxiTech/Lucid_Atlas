#!/usr/bin/env python3

import os
import threading
from typing import Dict

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
import tkinter as tk
from tkinter import ttk
import yaml


DEFAULT_CONFIG = {
    "pointcloud_to_laserscan": {
        "ros__parameters": {
            "target_frame": "odom",
            "min_height": -0.03,
            "max_height": 0.03,
            "range_min": 0.60,
            "range_max": 30.0,
        }
    },
    "scan_boundary_filter": {
        "ros__parameters": {
            "min_range": 0.60,
            "max_range": 8.0,
            "median_window": 7,
            "min_cluster_size": 4,
            "outlier_jump_threshold": 0.25,
            "boundary_only": True,
            "edge_jump_threshold": 0.18,
            "edge_keep_neighbors": 1,
        }
    },
    "tuning_gui": {"ros__parameters": {"edge_clarity": 60}},
}


class RosBridge(Node):
    def __init__(self) -> None:
        super().__init__("scan_tuning_gui_bridge")
        self._client = self.create_client(
            SetParameters, "/scan_boundary_filter/set_parameters"
        )
        self._lock = threading.Lock()
        self._pending: Dict[str, object] = {}
        self.create_timer(0.08, self._flush_pending)

    def queue_update(self, param_name: str, value: object) -> None:
        with self._lock:
            self._pending[param_name] = value

    def _flush_pending(self) -> None:
        with self._lock:
            if not self._pending:
                return
            pending = self._pending
            self._pending = {}

        if not self._client.service_is_ready():
            self._client.wait_for_service(timeout_sec=0.01)

        req = SetParameters.Request()
        req.parameters = [
            Parameter(name=name, value=value).to_parameter_msg()
            for name, value in pending.items()
        ]
        fut = self._client.call_async(req)
        fut.add_done_callback(self._on_set_done(pending))

    def _on_set_done(self, pending: Dict[str, object]):
        def _cb(future):
            try:
                result = future.result()
                ok = all(r.successful for r in result.results)
                if ok:
                    self.get_logger().info(f"Updated scan_boundary_filter: {pending}")
                else:
                    reasons = [r.reason for r in result.results if not r.successful]
                    self.get_logger().warning(
                        f"Update rejected: {pending}, reason={reasons}"
                    )
            except Exception as exc:
                self.get_logger().warning(
                    f"Update exception: {pending}, error={exc}"
                )

        return _cb


class ScanTuningGui:
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.config = self._load_config(config_path)

        rclpy.init()
        self.bridge = RosBridge()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        self.root = tk.Tk()
        self.root.title("GO2 Boundary Tuning")
        self.root.geometry("620x260")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._save_after_id = None
        self._status = tk.StringVar(value=f"Config: {self.config_path}")
        self._edge_clarity = tk.IntVar(
            value=int(
                self.config.get("tuning_gui", {})
                .get("ros__parameters", {})
                .get("edge_clarity", 60)
            )
        )

        self._build_ui()
        self._apply_clarity_to_runtime(self._edge_clarity.get())

    def _build_ui(self) -> None:
        pad = {"padx": 10, "pady": 6}
        frame = ttk.Frame(self.root)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(
            frame,
            text="只调边缘明显程度（固定水平切片 Z=0 附近）",
            font=("Sans", 12, "bold"),
        ).grid(row=0, column=0, columnspan=3, sticky="w", **pad)

        ttk.Label(frame, text="Edge Clarity").grid(row=1, column=0, sticky="w", **pad)
        scale = tk.Scale(
            frame,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            resolution=1,
            variable=self._edge_clarity,
            length=430,
            command=self._on_clarity_change,
        )
        scale.grid(row=1, column=1, sticky="we", **pad)

        self._value_label = ttk.Label(frame, text=f"{self._edge_clarity.get()}", width=8)
        self._value_label.grid(row=1, column=2, sticky="e", **pad)

        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=2, column=0, columnspan=3, sticky="w", **pad)
        ttk.Button(btn_frame, text="Save Now", command=self._save_now).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_frame, text="Reload", command=self._reload).pack(side=tk.LEFT, padx=4)

        ttk.Label(
            frame,
            text="说明: 数值越大，边界越干净（噪点更少）",
            foreground="#3f3f3f",
        ).grid(row=3, column=0, columnspan=3, sticky="w", **pad)

        ttk.Label(frame, textvariable=self._status, foreground="#2f5d2f").grid(
            row=4, column=0, columnspan=3, sticky="w", **pad
        )

        frame.columnconfigure(1, weight=1)

    def _on_clarity_change(self, _v=None) -> None:
        c = int(self._edge_clarity.get())
        self._value_label.config(text=str(c))
        self.config.setdefault("tuning_gui", {}).setdefault("ros__parameters", {})[
            "edge_clarity"
        ] = c
        self._apply_clarity_to_runtime(c)
        self._schedule_save()

    def _apply_clarity_to_runtime(self, clarity: int) -> None:
        c = max(0, min(100, int(clarity))) / 100.0

        min_cluster_size = 2 + int(round(c * 4.0))           # 2..6
        median_window = 5 if c < 0.55 else 7
        outlier_jump_threshold = 0.35 - 0.25 * c             # 0.35..0.10
        edge_jump_threshold = 0.10 + 0.22 * c                # 0.10..0.32
        edge_keep_neighbors = 2 - int(round(c * 2.0))        # 2..0
        if edge_keep_neighbors < 0:
            edge_keep_neighbors = 0

        updates = {
            "boundary_only": True,
            "min_cluster_size": int(min_cluster_size),
            "median_window": int(median_window),
            "outlier_jump_threshold": float(outlier_jump_threshold),
            "edge_jump_threshold": float(edge_jump_threshold),
            "edge_keep_neighbors": int(edge_keep_neighbors),
        }

        for k, v in updates.items():
            self.bridge.queue_update(k, v)

        ros_params = self.config.setdefault("scan_boundary_filter", {}).setdefault(
            "ros__parameters", {}
        )
        ros_params.update(updates)

    def _schedule_save(self) -> None:
        if self._save_after_id is not None:
            self.root.after_cancel(self._save_after_id)
        self._save_after_id = self.root.after(250, self._save_now)

    def _save_now(self) -> None:
        self._save_after_id = None
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        tmp_path = self.config_path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.config, f, sort_keys=False, allow_unicode=False)
        os.replace(tmp_path, self.config_path)
        self._status.set(f"Saved: {self.config_path}")

    def _reload(self) -> None:
        self.config = self._load_config(self.config_path)
        c = int(
            self.config.get("tuning_gui", {})
            .get("ros__parameters", {})
            .get("edge_clarity", 60)
        )
        self._edge_clarity.set(c)
        self._value_label.config(text=str(c))
        self._apply_clarity_to_runtime(c)
        self._status.set("Reloaded and applied")

    def _load_config(self, path: str) -> Dict:
        cfg = {
            node: {"ros__parameters": params["ros__parameters"].copy()}
            if isinstance(params, dict) and "ros__parameters" in params
            else params.copy()
            for node, params in DEFAULT_CONFIG.items()
        }
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                loaded = yaml.safe_load(f) or {}
            for key, value in loaded.items():
                if key in ("pointcloud_to_laserscan", "scan_boundary_filter"):
                    ros_params = value.get("ros__parameters", {}) if isinstance(value, dict) else {}
                    if isinstance(ros_params, dict):
                        cfg[key]["ros__parameters"].update(ros_params)
                elif key == "tuning_gui" and isinstance(value, dict):
                    ros_params = value.get("ros__parameters", {})
                    if isinstance(ros_params, dict):
                        cfg["tuning_gui"]["ros__parameters"].update(ros_params)
        return cfg

    def _spin_loop(self) -> None:
        try:
            rclpy.spin(self.bridge)
        except ExternalShutdownException:
            pass

    def _on_close(self) -> None:
        self._save_now()
        self.bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    try:
        default_config = os.path.join(
            get_package_share_directory("go2_field_config"),
            "config",
            "scan_tuning.yaml",
        )
    except PackageNotFoundError:
        default_config = os.path.expanduser("~/.config/go2_field_nav/scan_tuning.yaml")
    config_path = os.environ.get("GO2_SCAN_TUNING_FILE", default_config)
    app = ScanTuningGui(config_path=config_path)
    app.run()


if __name__ == "__main__":
    main()
