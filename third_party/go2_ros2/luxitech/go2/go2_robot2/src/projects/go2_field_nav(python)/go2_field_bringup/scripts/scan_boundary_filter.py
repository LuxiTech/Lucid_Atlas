#!/usr/bin/env python3

import math
import statistics
from typing import List

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanBoundaryFilter(Node):
    def __init__(self) -> None:
        super().__init__("scan_boundary_filter")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_obstacles")
        self.declare_parameter("min_range", 0.40)
        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("median_window", 5)
        self.declare_parameter("min_cluster_size", 3)
        self.declare_parameter("outlier_jump_threshold", 0.35)
        self.declare_parameter("boundary_only", True)
        self.declare_parameter("edge_jump_threshold", 0.20)
        self.declare_parameter("edge_keep_neighbors", 1)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._min_range = float(self.get_parameter("min_range").value)
        self._max_range = float(self.get_parameter("max_range").value)
        self._median_window = int(self.get_parameter("median_window").value)
        self._min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self._outlier_jump_threshold = float(
            self.get_parameter("outlier_jump_threshold").value
        )
        self._boundary_only = bool(self.get_parameter("boundary_only").value)
        self._edge_jump_threshold = float(
            self.get_parameter("edge_jump_threshold").value
        )
        self._edge_keep_neighbors = int(self.get_parameter("edge_keep_neighbors").value)

        if self._median_window < 1:
            self._median_window = 1
        if self._median_window % 2 == 0:
            self._median_window += 1
        if self._min_cluster_size < 1:
            self._min_cluster_size = 1
        if self._outlier_jump_threshold < 0.0:
            self._outlier_jump_threshold = 0.0
        if self._edge_jump_threshold < 0.0:
            self._edge_jump_threshold = 0.0
        if self._edge_keep_neighbors < 0:
            self._edge_keep_neighbors = 0

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._pub = self.create_publisher(
            LaserScan, self._output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, self._input_topic, self._on_scan, qos_profile_sensor_data
        )

        self.get_logger().info(
            "Scan boundary filter started: "
            f"{self._input_topic} -> {self._output_topic}, "
            f"range=[{self._min_range:.2f}, {self._max_range:.2f}], "
            f"median_window={self._median_window}, "
            f"min_cluster_size={self._min_cluster_size}, "
            f"outlier_jump_threshold={self._outlier_jump_threshold:.2f}, "
            f"boundary_only={self._boundary_only}, "
            f"edge_jump_threshold={self._edge_jump_threshold:.2f}, "
            f"edge_keep_neighbors={self._edge_keep_neighbors}"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)
        filtered = self._range_gate(ranges, self._min_range, self._max_range)
        filtered = self._median_filter(filtered, self._median_window)
        filtered = self._remove_outlier_spikes(filtered, self._outlier_jump_threshold)
        filtered = self._remove_small_clusters(filtered, self._min_cluster_size)
        if self._boundary_only:
            filtered = self._extract_boundaries(
                filtered,
                self._edge_jump_threshold,
                self._edge_keep_neighbors,
                self._min_cluster_size,
            )

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(msg.range_min, self._min_range)
        out.range_max = min(msg.range_max, self._max_range)
        out.ranges = filtered
        out.intensities = msg.intensities
        self._pub.publish(out)

    @staticmethod
    def _is_valid(v: float) -> bool:
        return math.isfinite(v)

    def _range_gate(self, values: List[float], rmin: float, rmax: float) -> List[float]:
        out = []
        for v in values:
            if not self._is_valid(v) or v < rmin or v > rmax:
                out.append(float("inf"))
            else:
                out.append(v)
        return out

    def _median_filter(self, values: List[float], window: int) -> List[float]:
        if window <= 1 or not values:
            return values

        n = len(values)
        half = window // 2
        out = [float("inf")] * n

        for i in range(n):
            if not self._is_valid(values[i]):
                continue
            buf = []
            for j in range(i - half, i + half + 1):
                if 0 <= j < n and self._is_valid(values[j]):
                    buf.append(values[j])
            if len(buf) >= 3:
                out[i] = float(statistics.median(buf))
        return out

    def _remove_small_clusters(self, values: List[float], min_cluster: int) -> List[float]:
        if min_cluster <= 1:
            return values
        n = len(values)
        keep = [False] * n
        i = 0
        while i < n:
            if not self._is_valid(values[i]):
                i += 1
                continue
            j = i
            while j + 1 < n and self._is_valid(values[j + 1]):
                j += 1
            if (j - i + 1) >= min_cluster:
                for k in range(i, j + 1):
                    keep[k] = True
            i = j + 1

        out = []
        for i, v in enumerate(values):
            out.append(v if keep[i] else float("inf"))
        return out

    def _remove_outlier_spikes(self, values: List[float], jump: float) -> List[float]:
        if jump <= 0.0 or len(values) < 3:
            return values
        out = values.copy()
        for i in range(1, len(values) - 1):
            cur = values[i]
            left = values[i - 1]
            right = values[i + 1]
            if not (self._is_valid(cur) and self._is_valid(left) and self._is_valid(right)):
                continue
            if abs(cur - left) > jump and abs(cur - right) > jump:
                out[i] = float("inf")
        return out

    def _extract_boundaries(
        self,
        values: List[float],
        edge_jump: float,
        edge_keep_neighbors: int,
        min_cluster: int,
    ) -> List[float]:
        out = [float("inf")] * len(values)
        n = len(values)
        i = 0
        while i < n:
            if not self._is_valid(values[i]):
                i += 1
                continue
            j = i
            while j + 1 < n and self._is_valid(values[j + 1]):
                j += 1

            cluster_len = j - i + 1
            if cluster_len >= min_cluster:
                keep_idx = {i, j}
                if edge_jump > 0.0 and cluster_len >= 3:
                    for k in range(i + 1, j):
                        prev_v = values[k - 1]
                        cur_v = values[k]
                        next_v = values[k + 1]
                        if (
                            abs(cur_v - prev_v) >= edge_jump
                            or abs(cur_v - next_v) >= edge_jump
                        ):
                            left = max(i, k - edge_keep_neighbors)
                            right = min(j, k + edge_keep_neighbors)
                            for m in range(left, right + 1):
                                keep_idx.add(m)
                for k in keep_idx:
                    out[k] = values[k]
            i = j + 1
        return out

    def _on_set_parameters(self, params: List[Parameter]) -> SetParametersResult:
        new_min_range = self._min_range
        new_max_range = self._max_range
        new_median_window = self._median_window
        new_min_cluster = self._min_cluster_size
        new_outlier_jump = self._outlier_jump_threshold
        new_boundary_only = self._boundary_only
        new_edge_jump = self._edge_jump_threshold
        new_edge_keep_neighbors = self._edge_keep_neighbors

        for p in params:
            if p.name == "min_range":
                new_min_range = float(p.value)
            elif p.name == "max_range":
                new_max_range = float(p.value)
            elif p.name == "median_window":
                new_median_window = int(p.value)
            elif p.name == "min_cluster_size":
                new_min_cluster = int(p.value)
            elif p.name == "outlier_jump_threshold":
                new_outlier_jump = float(p.value)
            elif p.name == "boundary_only":
                new_boundary_only = bool(p.value)
            elif p.name == "edge_jump_threshold":
                new_edge_jump = float(p.value)
            elif p.name == "edge_keep_neighbors":
                new_edge_keep_neighbors = int(p.value)

        if new_min_range < 0.0:
            return SetParametersResult(successful=False, reason="min_range must be >= 0")
        if new_max_range <= new_min_range:
            return SetParametersResult(
                successful=False, reason="max_range must be greater than min_range"
            )
        if new_median_window < 1:
            return SetParametersResult(
                successful=False, reason="median_window must be >= 1"
            )
        if new_min_cluster < 1:
            return SetParametersResult(
                successful=False, reason="min_cluster_size must be >= 1"
            )
        if new_outlier_jump < 0.0:
            return SetParametersResult(
                successful=False, reason="outlier_jump_threshold must be >= 0"
            )
        if new_edge_jump < 0.0:
            return SetParametersResult(
                successful=False, reason="edge_jump_threshold must be >= 0"
            )
        if new_edge_keep_neighbors < 0:
            return SetParametersResult(
                successful=False, reason="edge_keep_neighbors must be >= 0"
            )

        if new_median_window % 2 == 0:
            new_median_window += 1

        self._min_range = new_min_range
        self._max_range = new_max_range
        self._median_window = new_median_window
        self._min_cluster_size = new_min_cluster
        self._outlier_jump_threshold = new_outlier_jump
        self._boundary_only = new_boundary_only
        self._edge_jump_threshold = new_edge_jump
        self._edge_keep_neighbors = new_edge_keep_neighbors
        return SetParametersResult(successful=True)


def main() -> None:
    rclpy.init()
    node = ScanBoundaryFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
