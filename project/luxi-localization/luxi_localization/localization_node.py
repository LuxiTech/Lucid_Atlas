import copy
import json
import math
import os
import threading
import time
from collections import deque

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from std_msgs.msg import Float32
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker

from luxi_localization.pointcloud import pointcloud2_to_xyz_array, xyz_array_to_pointcloud2
from luxi_localization.transforms import (
    basis_from_normal_and_heading,
    floor_plane_basis,
    inverse_matrix,
    matrix_to_pose_msg,
    matrix_to_quaternion,
    pose_msg_to_matrix,
    project_points_to_floor_frame,
    project_pose_to_floor_frame,
    quaternion_to_matrix,
    rotation_from_floor_yaw,
    yaw_from_quaternion,
)


class Open3DLocalizationNode(Node):
    def __init__(self):
        super().__init__("luxi_open3d_localization")

        self.declare_parameter(
            "map_path",
            "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd",
        )
        self.declare_parameter("map_metadata_path", "")
        self.declare_parameter("input_cloud_topic", "/cloud_registered_body")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "camera_init")
        self.declare_parameter("base_frame", "body")
        self.declare_parameter("cloud_frame_mode", "body")
        self.declare_parameter("base_offset_x", 0.0)
        self.declare_parameter("base_offset_y", 0.0)
        self.declare_parameter("base_offset_z", 0.0)
        self.declare_parameter("base_yaw_offset_deg", 0.0)
        self.declare_parameter("initial_pose_is_base_frame", False)
        self.declare_parameter("max_odom_position_norm", 200.0)
        self.declare_parameter("max_odom_step", 2.0)
        self.declare_parameter("require_initial_pose", True)
        self.declare_parameter("initial_pose_mode", "body")
        self.declare_parameter("initial_pose_coordinate_frame", "floor")
        self.declare_parameter("initial_pose_z_mode", "floor_plane")
        self.declare_parameter("initial_pose_height", 0.0)
        self.declare_parameter("initial_yaw_search_enable", False)
        self.declare_parameter("initial_yaw_search_range_deg", 180.0)
        self.declare_parameter("initial_yaw_search_step_deg", 30.0)
        self.declare_parameter("initial_yaw_search_min_fitness", 0.15)
        self.declare_parameter("initial_pose_reacquire_timeout_sec", 8.0)
        self.declare_parameter("live_floor_alignment_enable", True)
        self.declare_parameter("allow_initial_pose_without_live_floor", True)
        self.declare_parameter("live_floor_fit_min_points", 250)
        self.declare_parameter("live_floor_fit_distance_threshold", 0.08)
        self.declare_parameter("live_floor_fit_ransac_n", 3)
        self.declare_parameter("live_floor_fit_iterations", 200)
        self.declare_parameter("live_floor_min_inlier_ratio", 0.15)
        self.declare_parameter("live_floor_min_abs_body_z", 0.55)
        self.declare_parameter("live_floor_use_low_height_band", True)
        self.declare_parameter("live_floor_band_min_quantile", 0.05)
        self.declare_parameter("live_floor_band_max_quantile", 0.35)
        self.declare_parameter("live_floor_min_sensor_height", 0.20)
        self.declare_parameter("live_floor_max_sensor_height", 1.50)
        self.declare_parameter(
            "floor_plane",
            [-0.357093, 0.006594, 0.934045, 1.001688],
        )
        self.declare_parameter("map_voxel_size", 0.25)
        self.declare_parameter("scan_voxel_size", 0.20)
        self.declare_parameter("scan_accumulate_frames", 5)
        self.declare_parameter("local_map_extent_x", 30.0)
        self.declare_parameter("local_map_extent_y", 30.0)
        self.declare_parameter("local_map_extent_z", 8.0)
        self.declare_parameter("max_correspondence_distance", 1.0)
        self.declare_parameter("icp_max_iteration", 30)
        self.declare_parameter("min_scan_points", 200)
        self.declare_parameter("min_local_map_points", 1000)
        self.declare_parameter("max_scan_points", 60000)
        self.declare_parameter("min_fitness_to_accept", 0.15)
        self.declare_parameter("max_rmse_to_accept", 1.5)
        self.declare_parameter("max_icp_translation_correction", 8.0)
        self.declare_parameter("max_icp_rotation_correction_deg", 25.0)
        self.declare_parameter("max_initial_icp_translation_correction", 1.20)
        self.declare_parameter("max_initial_icp_rotation_correction_deg", 45.0)
        self.declare_parameter("constrain_icp_to_floor", True)
        self.declare_parameter("icp_correction_smoothing", 1.0)
        self.declare_parameter("initial_icp_correction_smoothing", 0.25)
        self.declare_parameter("enable_multiscale_icp", True)
        self.declare_parameter("coarse_scan_voxel_size", 0.45)
        self.declare_parameter("coarse_map_voxel_size", 0.45)
        self.declare_parameter("coarse_max_correspondence_distance", 0.90)
        self.declare_parameter("initial_coarse_max_correspondence_distance", 1.50)
        self.declare_parameter("coarse_icp_max_iteration", 35)
        self.declare_parameter("enable_alignment_validation", False)
        self.declare_parameter("alignment_max_floor_normal_angle_deg", 8.0)
        self.declare_parameter("alignment_max_dominant_plane_angle_deg", 12.0)
        self.declare_parameter("alignment_validation_min_points", 600)
        self.declare_parameter("alignment_validation_min_inlier_ratio", 0.10)
        self.declare_parameter("alignment_validation_distance_threshold", 0.10)
        self.declare_parameter("alignment_validation_ransac_iterations", 120)
        self.declare_parameter("icp_initial_hold_sec", 3.0)
        self.declare_parameter("icp_required_consecutive_accepts", 3)
        self.declare_parameter("min_update_period_sec", 0.5)
        self.declare_parameter("publish_map_cloud", True)
        self.declare_parameter("publish_floor_projected_cloud", True)
        self.declare_parameter("floor_projected_cloud_min_height", 0.15)
        self.declare_parameter("floor_projected_cloud_max_height", 1.60)
        self.declare_parameter("map_publish_period_sec", 5.0)
        self.declare_parameter("publish_aligned_cloud", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_base_tf", True)
        self.declare_parameter("publish_2d_pose", True)

        self.map_path = self.get_parameter("map_path").value
        self.map_metadata_path = self.get_parameter("map_metadata_path").value
        self.input_cloud_topic = self.get_parameter("input_cloud_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.initial_pose_topic = self.get_parameter("initial_pose_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.cloud_frame_mode = self.get_parameter("cloud_frame_mode").value
        self.base_offset_x = float(self.get_parameter("base_offset_x").value)
        self.base_offset_y = float(self.get_parameter("base_offset_y").value)
        self.base_offset_z = float(self.get_parameter("base_offset_z").value)
        self.base_yaw_offset_deg = float(self.get_parameter("base_yaw_offset_deg").value)
        self.initial_pose_is_base_frame = bool(
            self.get_parameter("initial_pose_is_base_frame").value
        )
        self.body_t_base = self._make_body_t_base()
        self.base_t_body = inverse_matrix(self.body_t_base)
        self.max_odom_position_norm = float(self.get_parameter("max_odom_position_norm").value)
        self.max_odom_step = float(self.get_parameter("max_odom_step").value)
        self.require_initial_pose = bool(self.get_parameter("require_initial_pose").value)
        self.initial_pose_mode = self.get_parameter("initial_pose_mode").value
        self.initial_pose_coordinate_frame = self.get_parameter(
            "initial_pose_coordinate_frame"
        ).value
        self.initial_pose_z_mode = self.get_parameter("initial_pose_z_mode").value
        self.initial_pose_height = float(self.get_parameter("initial_pose_height").value)
        self.initial_yaw_search_enable = bool(
            self.get_parameter("initial_yaw_search_enable").value
        )
        self.initial_yaw_search_range = np.deg2rad(
            float(self.get_parameter("initial_yaw_search_range_deg").value)
        )
        self.initial_yaw_search_step = np.deg2rad(
            max(1.0, float(self.get_parameter("initial_yaw_search_step_deg").value))
        )
        self.initial_yaw_search_min_fitness = float(
            self.get_parameter("initial_yaw_search_min_fitness").value
        )
        self.initial_pose_reacquire_timeout_sec = max(
            0.0, float(self.get_parameter("initial_pose_reacquire_timeout_sec").value)
        )
        self.live_floor_alignment_enable = bool(
            self.get_parameter("live_floor_alignment_enable").value
        )
        self.allow_initial_pose_without_live_floor = bool(
            self.get_parameter("allow_initial_pose_without_live_floor").value
        )
        self.live_floor_fit_min_points = int(
            self.get_parameter("live_floor_fit_min_points").value
        )
        self.live_floor_fit_distance_threshold = float(
            self.get_parameter("live_floor_fit_distance_threshold").value
        )
        self.live_floor_fit_ransac_n = int(self.get_parameter("live_floor_fit_ransac_n").value)
        self.live_floor_fit_iterations = int(
            self.get_parameter("live_floor_fit_iterations").value
        )
        self.live_floor_min_inlier_ratio = float(
            self.get_parameter("live_floor_min_inlier_ratio").value
        )
        self.live_floor_min_abs_body_z = float(
            self.get_parameter("live_floor_min_abs_body_z").value
        )
        self.live_floor_use_low_height_band = bool(
            self.get_parameter("live_floor_use_low_height_band").value
        )
        self.live_floor_band_min_quantile = float(
            self.get_parameter("live_floor_band_min_quantile").value
        )
        self.live_floor_band_max_quantile = float(
            self.get_parameter("live_floor_band_max_quantile").value
        )
        self.live_floor_min_sensor_height = float(
            self.get_parameter("live_floor_min_sensor_height").value
        )
        self.live_floor_max_sensor_height = float(
            self.get_parameter("live_floor_max_sensor_height").value
        )
        self.floor_plane = np.asarray(self.get_parameter("floor_plane").value, dtype=np.float64)
        self._load_floor_plane_from_metadata(self.map_metadata_path)
        self.map_voxel_size = float(self.get_parameter("map_voxel_size").value)
        self.scan_voxel_size = float(self.get_parameter("scan_voxel_size").value)
        self.scan_accumulate_frames = int(self.get_parameter("scan_accumulate_frames").value)
        self.local_map_extent = np.array(
            [
                float(self.get_parameter("local_map_extent_x").value),
                float(self.get_parameter("local_map_extent_y").value),
                float(self.get_parameter("local_map_extent_z").value),
            ],
            dtype=np.float64,
        )
        self.max_correspondence_distance = float(
            self.get_parameter("max_correspondence_distance").value
        )
        self.icp_max_iteration = int(self.get_parameter("icp_max_iteration").value)
        self.min_scan_points = int(self.get_parameter("min_scan_points").value)
        self.min_local_map_points = int(self.get_parameter("min_local_map_points").value)
        self.max_scan_points = int(self.get_parameter("max_scan_points").value)
        self.min_fitness_to_accept = float(self.get_parameter("min_fitness_to_accept").value)
        self.max_rmse_to_accept = float(self.get_parameter("max_rmse_to_accept").value)
        self.max_icp_translation_correction = float(
            self.get_parameter("max_icp_translation_correction").value
        )
        self.max_icp_rotation_correction = np.deg2rad(
            float(self.get_parameter("max_icp_rotation_correction_deg").value)
        )
        self.max_initial_icp_translation_correction = float(
            self.get_parameter("max_initial_icp_translation_correction").value
        )
        self.max_initial_icp_rotation_correction = np.deg2rad(
            float(self.get_parameter("max_initial_icp_rotation_correction_deg").value)
        )
        self.constrain_icp_to_floor = bool(self.get_parameter("constrain_icp_to_floor").value)
        self.icp_correction_smoothing = float(
            self.get_parameter("icp_correction_smoothing").value
        )
        self.initial_icp_correction_smoothing = float(
            self.get_parameter("initial_icp_correction_smoothing").value
        )
        self.enable_multiscale_icp = bool(
            self.get_parameter("enable_multiscale_icp").value
        )
        self.coarse_scan_voxel_size = float(
            self.get_parameter("coarse_scan_voxel_size").value
        )
        self.coarse_map_voxel_size = float(
            self.get_parameter("coarse_map_voxel_size").value
        )
        self.coarse_max_correspondence_distance = float(
            self.get_parameter("coarse_max_correspondence_distance").value
        )
        self.initial_coarse_max_correspondence_distance = float(
            self.get_parameter("initial_coarse_max_correspondence_distance").value
        )
        self.coarse_icp_max_iteration = int(
            self.get_parameter("coarse_icp_max_iteration").value
        )
        self.enable_alignment_validation = bool(
            self.get_parameter("enable_alignment_validation").value
        )
        self.alignment_max_floor_normal_angle = np.deg2rad(
            float(self.get_parameter("alignment_max_floor_normal_angle_deg").value)
        )
        self.alignment_max_dominant_plane_angle = np.deg2rad(
            float(self.get_parameter("alignment_max_dominant_plane_angle_deg").value)
        )
        self.alignment_validation_min_points = int(
            self.get_parameter("alignment_validation_min_points").value
        )
        self.alignment_validation_min_inlier_ratio = float(
            self.get_parameter("alignment_validation_min_inlier_ratio").value
        )
        self.alignment_validation_distance_threshold = float(
            self.get_parameter("alignment_validation_distance_threshold").value
        )
        self.alignment_validation_ransac_iterations = int(
            self.get_parameter("alignment_validation_ransac_iterations").value
        )
        self.icp_initial_hold_sec = float(self.get_parameter("icp_initial_hold_sec").value)
        self.icp_required_consecutive_accepts = int(
            self.get_parameter("icp_required_consecutive_accepts").value
        )
        self.min_update_period_sec = float(self.get_parameter("min_update_period_sec").value)
        self.publish_map_cloud = bool(self.get_parameter("publish_map_cloud").value)
        self.publish_floor_projected_cloud = bool(
            self.get_parameter("publish_floor_projected_cloud").value
        )
        self.floor_projected_cloud_min_height = float(
            self.get_parameter("floor_projected_cloud_min_height").value
        )
        self.floor_projected_cloud_max_height = float(
            self.get_parameter("floor_projected_cloud_max_height").value
        )
        self.map_publish_period_sec = float(self.get_parameter("map_publish_period_sec").value)
        self.publish_aligned_cloud = bool(self.get_parameter("publish_aligned_cloud").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_base_tf = bool(self.get_parameter("publish_base_tf").value)
        self.publish_2d_pose = bool(self.get_parameter("publish_2d_pose").value)

        if self.cloud_frame_mode not in ("body", "odom"):
            raise ValueError("cloud_frame_mode must be 'body' or 'odom'")
        if self.initial_pose_mode not in ("body", "odom"):
            raise ValueError("initial_pose_mode must be 'body' or 'odom'")
        if self.initial_pose_coordinate_frame not in ("floor", "map"):
            raise ValueError("initial_pose_coordinate_frame must be 'floor' or 'map'")
        if self.initial_pose_z_mode not in ("as_received", "floor_plane"):
            raise ValueError("initial_pose_z_mode must be 'as_received' or 'floor_plane'")
        if self.floor_plane.shape[0] != 4:
            raise ValueError("floor_plane must contain four coefficients: a b c d")
        if self.scan_accumulate_frames < 1:
            self.scan_accumulate_frames = 1

        self.map_cloud = self._load_map(self.map_path)
        self.map_cloud_floor = self._project_cloud_to_floor_frame(self.map_cloud)
        self.map_t_odom = np.eye(4, dtype=np.float64)
        self.odom_t_body = None
        self.pending_map_t_body = None
        self.pending_map_t_odom = None
        self.initial_pose_map_body = None
        self.initial_yaw_search_pending = False
        self.initial_pose_reacquire_pending = False
        self.initial_pose_reacquire_until_monotonic = 0.0
        self.icp_locked = False
        self.icp_good_count = 0
        self.icp_hold_until_monotonic = 0.0
        self.live_floor_normal_odom = None
        self.live_floor_d_odom = None
        self.live_floor_inlier_ratio = 0.0
        self.has_initial_pose = not self.require_initial_pose
        self.scan_buffer_odom = deque(maxlen=self.scan_accumulate_frames)
        self.last_update_monotonic = 0.0
        self.last_odom_monotonic = None
        self.last_cloud_monotonic = None
        self.lock = threading.Lock()

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, "/localization_3d", 10)
        self.pose_2d_pub = self.create_publisher(PoseStamped, "/localization_2d", 10)
        self.marker_pub = self.create_publisher(
            Marker, "/luxi_localization/current_marker", 10
        )
        self.marker_2d_pub = self.create_publisher(
            Marker, "/luxi_localization/current_marker_2d", 10
        )
        self.confidence_pub = self.create_publisher(Float32, "/localization_3d_confidence", 10)
        self.status_pub = self.create_publisher(String, "/luxi_localization/status", latched_qos)
        self.debug_pub = self.create_publisher(
            String, "/luxi_localization/debug_json", latched_qos
        )
        self.map_cloud_pub = self.create_publisher(
            PointCloud2, "/luxi_localization/map_cloud", latched_qos
        )
        self.map_cloud_floor_pub = self.create_publisher(
            PointCloud2, "/luxi_localization/map_cloud_floor", latched_qos
        )
        self.aligned_cloud_pub = self.create_publisher(
            PointCloud2, "/luxi_localization/aligned_cloud", 2
        )
        self.aligned_cloud_floor_pub = self.create_publisher(
            PointCloud2, "/luxi_localization/aligned_cloud_floor", 2
        )

        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 50)
        self.create_subscription(PointCloud2, self.input_cloud_topic, self._on_cloud, 5)
        self.create_subscription(
            PoseWithCovarianceStamped, self.initial_pose_topic, self._on_initial_pose, 5
        )
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._publish_map_anchor_tf()
        if self.publish_map_cloud or self.publish_floor_projected_cloud:
            self.create_timer(self.map_publish_period_sec, self._publish_map_cloud)
            self._publish_map_cloud()
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            "luxi_localization ready: map=%s, cloud=%s, odom=%s"
            % (self.map_path, self.input_cloud_topic, self.odom_topic)
        )
        if self.require_initial_pose:
            self.get_logger().warn(
                "waiting for initial pose on %s before ICP starts" % self.initial_pose_topic
            )

    def _load_floor_plane_from_metadata(self, path):
        if not path:
            return
        if not os.path.exists(path):
            self.get_logger().warn("map metadata not found: %s" % path)
            return
        try:
            with open(path, "r", encoding="utf-8") as metadata_file:
                metadata = json.load(metadata_file)
            plane = metadata.get("floor_plane")
            if not metadata.get("floor_plane_used", False) or not isinstance(plane, list):
                self.get_logger().warn("map metadata has no usable floor_plane: %s" % path)
                return
            if len(plane) != 4:
                self.get_logger().warn("map metadata floor_plane must have 4 values: %s" % path)
                return
            self.floor_plane = np.asarray(plane, dtype=np.float64)
            self.get_logger().info(
                "loaded floor_plane from map metadata %s: [%.6f %.6f %.6f %.6f]"
                % (
                    path,
                    self.floor_plane[0],
                    self.floor_plane[1],
                    self.floor_plane[2],
                    self.floor_plane[3],
                )
            )
        except Exception as exc:
            self.get_logger().warn("failed to read map metadata %s: %s" % (path, exc))

    def _load_map(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(path)

        cloud = o3d.io.read_point_cloud(path)
        if cloud.is_empty():
            raise RuntimeError("map point cloud is empty: %s" % path)

        cloud = cloud.remove_non_finite_points()
        raw_points = np.asarray(cloud.points).shape[0]
        if self.map_voxel_size > 0.0:
            cloud = cloud.voxel_down_sample(self.map_voxel_size)
        down_points = np.asarray(cloud.points).shape[0]

        self.get_logger().info(
            "loaded map: %d -> %d points, voxel=%.3f"
            % (raw_points, down_points, self.map_voxel_size)
        )
        return cloud

    def _on_odom(self, msg):
        odom_t_body = pose_msg_to_matrix(msg.pose.pose)
        odom_translation = odom_t_body[:3, 3]
        if not np.all(np.isfinite(odom_t_body)):
            self._publish_status_text("rejecting odom: non-finite transform")
            self.get_logger().warn("rejecting odom: non-finite transform", throttle_duration_sec=3.0)
            return
        odom_norm = float(np.linalg.norm(odom_translation))
        if odom_norm > self.max_odom_position_norm:
            self._publish_status_text("rejecting odom: norm %.1fm > %.1fm" % (
                odom_norm,
                self.max_odom_position_norm,
            ))
            self.get_logger().warn(
                "rejecting odom: norm %.1fm > %.1fm" % (
                    odom_norm,
                    self.max_odom_position_norm,
                ),
                throttle_duration_sec=3.0,
            )
            return
        with self.lock:
            if self.odom_t_body is not None:
                odom_step = float(np.linalg.norm(odom_translation - self.odom_t_body[:3, 3]))
                if odom_step > self.max_odom_step:
                    self.icp_good_count = 0
                    self.scan_buffer_odom.clear()
                    if self.has_initial_pose:
                        previous_map_t_body = self.map_t_odom @ self.odom_t_body
                        self.map_t_odom = previous_map_t_body @ inverse_matrix(odom_t_body)
                        self._reset_icp_tracking_locked()
                        self._publish_status_text(
                            "accepted odom reset: %.2fm > %.2fm, preserving map pose"
                            % (odom_step, self.max_odom_step)
                        )
                    else:
                        self._publish_status_text(
                            "accepted odom reset before initial pose: %.2fm > %.2fm"
                            % (odom_step, self.max_odom_step)
                        )
                    self.get_logger().warn(
                        "accepted odom reset: %.2fm > %.2fm" % (odom_step, self.max_odom_step),
                        throttle_duration_sec=3.0,
                    )
            self.odom_t_body = odom_t_body
            self.last_odom_monotonic = time.monotonic()
            if self.pending_map_t_odom is not None:
                self.map_t_odom = self.pending_map_t_odom
                self.pending_map_t_odom = None
                self.has_initial_pose = True
                self.icp_locked = False
                self.initial_pose_reacquire_pending = True
                self.initial_pose_reacquire_until_monotonic = (
                    time.monotonic() + self.initial_pose_reacquire_timeout_sec
                )
                self._reset_icp_tracking_locked()
            if (
                self.pending_map_t_body is not None
                and (
                    not self.live_floor_alignment_enable
                    or self.live_floor_normal_odom is not None
                )
            ):
                self.map_t_odom = self._initial_map_t_odom_locked(
                    self.pending_map_t_body, odom_t_body
                )
                self.pending_map_t_body = None
                self.has_initial_pose = True
                self.icp_locked = False
                self.initial_pose_reacquire_pending = True
                self.initial_pose_reacquire_until_monotonic = (
                    time.monotonic() + self.initial_pose_reacquire_timeout_sec
                )
                self._reset_icp_tracking_locked()

    def _on_initial_pose(self, msg):
        initial_pose = self._initial_pose_msg_to_matrix(msg.pose.pose)
        if self.initial_pose_mode == "body" and self.initial_pose_is_base_frame:
            initial_pose = initial_pose @ self.base_t_body
        applied_map_t_odom = None
        applied_odom_t_body = None
        with self.lock:
            self.initial_pose_map_body = initial_pose
            self.initial_yaw_search_pending = True
            self.initial_pose_reacquire_pending = True
            self.initial_pose_reacquire_until_monotonic = (
                time.monotonic() + self.initial_pose_reacquire_timeout_sec
            )
            self.has_initial_pose = False if self.require_initial_pose else True
            self.icp_locked = False
            self._reset_icp_tracking_locked()
            self.scan_buffer_odom.clear()
            if self.odom_t_body is None:
                if self.initial_pose_mode == "odom":
                    self.pending_map_t_odom = initial_pose
                else:
                    self.pending_map_t_body = initial_pose
                self.get_logger().info("stored initial pose; waiting for odometry")
                return
            if self.initial_pose_mode == "odom":
                self.map_t_odom = initial_pose
            elif (
                self.live_floor_alignment_enable
                and self.live_floor_normal_odom is None
                and not self.allow_initial_pose_without_live_floor
            ):
                self.pending_map_t_body = initial_pose
                self._publish_status_text("waiting for live floor plane")
                self.get_logger().warn("stored initial pose; waiting for live floor plane")
                return
            else:
                self.map_t_odom = self._initial_map_t_odom_locked(initial_pose, self.odom_t_body)
                if self.live_floor_alignment_enable and self.live_floor_normal_odom is None:
                    self._publish_status_text("initial pose accepted before live floor plane")
            self.has_initial_pose = True
            self._reset_icp_tracking_locked()
            applied_map_t_odom = self.map_t_odom.copy()
            applied_odom_t_body = self.odom_t_body.copy()
        self.get_logger().info(
            "accepted initial pose from %s as map->%s"
            % (self.initial_pose_topic, self.initial_pose_mode)
        )
        _, _, _, initial_yaw = self._floor_pose_components(initial_pose)
        self._publish_debug_text(
            "initial_pose mode=%s xyz=(%.3f, %.3f, %.3f) yaw=%.1fdeg base_yaw_offset=%.1fdeg"
            % (
                self.initial_pose_mode,
                initial_pose[0, 3],
                initial_pose[1, 3],
                initial_pose[2, 3],
                np.rad2deg(initial_yaw),
                self.base_yaw_offset_deg,
            )
        )
        if applied_map_t_odom is not None and applied_odom_t_body is not None:
            self._publish_result(
                self.get_clock().now().to_msg(),
                applied_map_t_odom @ applied_odom_t_body,
                applied_map_t_odom,
                1.0,
            )

    def _on_cloud(self, msg):
        now = time.monotonic()
        if now - self.last_update_monotonic < self.min_update_period_sec:
            return
        self.last_update_monotonic = now
        self.last_cloud_monotonic = now

        with self.lock:
            if self.odom_t_body is None:
                self.get_logger().warn("waiting for odometry", throttle_duration_sec=5.0)
                self._publish_status_text("waiting for odometry: %s" % self.odom_topic)
                return
            odom_t_body = self.odom_t_body.copy()
            map_t_odom = self.map_t_odom.copy()
            has_initial_pose = self.has_initial_pose

        xyz = pointcloud2_to_xyz_array(msg, self.max_scan_points)
        if xyz.shape[0] < self.min_scan_points:
            self.get_logger().warn(
                "scan has too few points: %d < %d" % (xyz.shape[0], self.min_scan_points),
                throttle_duration_sec=3.0,
            )
            return

        source_current = o3d.geometry.PointCloud()
        source_current.points = o3d.utility.Vector3dVector(xyz)
        if self.scan_voxel_size > 0.0:
            source_current = source_current.voxel_down_sample(self.scan_voxel_size)

        if self.cloud_frame_mode == "body":
            source_current.transform(odom_t_body)

        with self.lock:
            self.scan_buffer_odom.append(source_current)
            source = self._merged_scan_buffer_locked()
            self._update_live_floor_plane_locked(source, odom_t_body)
            if (
                self.require_initial_pose
                and not self.has_initial_pose
                and self.pending_map_t_body is not None
                and self.live_floor_normal_odom is not None
            ):
                self.map_t_odom = self._initial_map_t_odom_locked(
                    self.pending_map_t_body, odom_t_body
                )
                self.pending_map_t_body = None
                self.has_initial_pose = True
                self.icp_locked = False
                self.initial_pose_reacquire_pending = True
                self.initial_pose_reacquire_until_monotonic = (
                    time.monotonic() + self.initial_pose_reacquire_timeout_sec
                )
                self._reset_icp_tracking_locked()
                map_t_odom = self.map_t_odom.copy()
                has_initial_pose = True
                self._publish_debug_text("initial pose applied after live floor plane fit")
            else:
                map_t_odom = self.map_t_odom.copy()
                has_initial_pose = self.has_initial_pose

        if np.asarray(source.points).shape[0] < self.min_scan_points:
            self._publish_status_text(
                "waiting for accumulated scan points: %d < %d"
                % (np.asarray(source.points).shape[0], self.min_scan_points)
            )
            return

        if self.require_initial_pose and not has_initial_pose:
            self._publish_status_text("waiting for initial pose: %s" % self.initial_pose_topic)
            self.get_logger().warn(
                "waiting for initial pose on %s" % self.initial_pose_topic,
                throttle_duration_sec=5.0,
            )
            return

        init = map_t_odom
        target = self._crop_local_map(map_t_odom, odom_t_body)
        target_points = np.asarray(target.points).shape[0]
        if target_points < self.min_local_map_points:
            fallback_map_t_body = init @ odom_t_body
            self._publish_result(msg.header.stamp, fallback_map_t_body, init, 0.0, False)
            self._publish_status_text(
                "local map too small: %d < %d, holding odom pose"
                % (target_points, self.min_local_map_points)
            )
            with self.lock:
                self.icp_good_count = 0
            self.get_logger().warn(
                "local map too small: %d < %d" % (target_points, self.min_local_map_points),
                throttle_duration_sec=3.0,
            )
            return

        with self.lock:
            phase_now = time.monotonic()
            reacquire_window_active = phase_now < self.initial_pose_reacquire_until_monotonic
            if self.initial_pose_reacquire_pending or reacquire_window_active:
                self.icp_locked = False
            initial_acquisition = (
                (not self.icp_locked)
                or self.initial_pose_reacquire_pending
                or reacquire_window_active
            )
            initial_yaw_search_pending = self.initial_yaw_search_pending
            reacquire_remaining = max(
                0.0, self.initial_pose_reacquire_until_monotonic - phase_now
            )
        if (
            initial_acquisition
            and initial_yaw_search_pending
            and self.initial_yaw_search_enable
        ):
            searched_map_t_odom, search_summary = self._search_initial_yaw(
                source, target, map_t_odom, odom_t_body
            )
            with self.lock:
                self.initial_yaw_search_pending = False
                if searched_map_t_odom is not None:
                    self.map_t_odom = searched_map_t_odom
                    map_t_odom = searched_map_t_odom.copy()
                    init = map_t_odom
            self._publish_debug_text(search_summary)
        result, raw_icp_map_t_odom = self._run_icp(source, target, init, initial_acquisition)
        accepted = (
            result.fitness >= self.min_fitness_to_accept
            and result.inlier_rmse <= self.max_rmse_to_accept
        )
        icp_map_t_odom = raw_icp_map_t_odom
        if self.constrain_icp_to_floor:
            icp_map_t_odom = self._constrain_map_t_odom_to_floor(
                init, icp_map_t_odom, odom_t_body
            )

        correction = icp_map_t_odom @ inverse_matrix(init)
        correction_translation = float(np.linalg.norm(correction[:3, 3]))
        correction_rotation = self._rotation_angle(correction[:3, :3])
        max_translation_correction = (
            self.max_initial_icp_translation_correction
            if initial_acquisition
            else self.max_icp_translation_correction
        )
        max_rotation_correction = (
            self.max_initial_icp_rotation_correction
            if initial_acquisition
            else self.max_icp_rotation_correction
        )
        correction_smoothing = (
            self.initial_icp_correction_smoothing
            if initial_acquisition
            else self.icp_correction_smoothing
        )
        candidate_map_t_odom = self._blend_transform(
            init,
            icp_map_t_odom,
            correction_smoothing,
        )
        accepted = (
            accepted
            and correction_translation <= max_translation_correction
            and correction_rotation <= max_rotation_correction
        )
        alignment_summary = "alignment=skipped"
        if accepted:
            alignment_accepted, alignment_summary = self._validate_alignment(
                source,
                target,
                candidate_map_t_odom,
            )
            accepted = accepted and alignment_accepted
        confidence = float(result.fitness / max(1.0, 1.0 + result.inlier_rmse))
        with self.lock:
            hold_remaining = max(0.0, self.icp_hold_until_monotonic - now)
            good_count = self.icp_good_count
        self._publish_debug_text(
            (
                "icp fitness=%.3f rmse=%.3f target=%d source=%d "
                "corr_t=%.3f/%.3f corr_rot_deg=%.2f/%.2f phase=%s "
                "accepted=%s good=%d/%d hold=%.1fs reacquire=%s %.1fs %s"
            )
            % (
                result.fitness,
                result.inlier_rmse,
                target_points,
                np.asarray(source.points).shape[0],
                correction_translation,
                max_translation_correction,
                np.rad2deg(correction_rotation),
                np.rad2deg(max_rotation_correction),
                "initial" if initial_acquisition else "tracking",
                str(accepted).lower(),
                good_count,
                self.icp_required_consecutive_accepts,
                hold_remaining,
                str(initial_acquisition).lower(),
                reacquire_remaining,
                alignment_summary,
            )
        )

        if accepted:
            with self.lock:
                hold_remaining = max(0.0, self.icp_hold_until_monotonic - now)
                if hold_remaining > 0.0:
                    self.icp_good_count = 0
                    good_count = 0
                    apply_icp = False
                else:
                    self.icp_good_count += 1
                    good_count = self.icp_good_count
                    apply_icp = good_count >= self.icp_required_consecutive_accepts

            if apply_icp:
                new_map_t_odom = candidate_map_t_odom
                map_t_body = new_map_t_odom @ odom_t_body

                with self.lock:
                    self.map_t_odom = new_map_t_odom
                    self.icp_locked = True
                    self.initial_pose_reacquire_pending = False
                    self.initial_pose_reacquire_until_monotonic = 0.0
                    self.icp_good_count = min(
                        self.icp_good_count, self.icp_required_consecutive_accepts
                    )
                self._publish_result(msg.header.stamp, map_t_body, new_map_t_odom, confidence)
                if self.publish_aligned_cloud:
                    self._publish_aligned_cloud(msg.header.stamp, source, new_map_t_odom)
            else:
                fallback_map_t_body = init @ odom_t_body
                self._publish_result(msg.header.stamp, fallback_map_t_body, init, 0.0, False)
                if self.publish_aligned_cloud:
                    self._publish_aligned_cloud(msg.header.stamp, source, init)
                if hold_remaining > 0.0:
                    self._publish_status_text(
                        "initial pose hold: %.1fs, ICP correction locked" % hold_remaining
                    )
                else:
                    self._publish_status_text(
                        "ICP verifying: %d/%d good frames"
                        % (good_count, self.icp_required_consecutive_accepts)
                    )
        else:
            with self.lock:
                self.icp_good_count = 0
            fallback_map_t_body = init @ odom_t_body
            self._publish_result(msg.header.stamp, fallback_map_t_body, init, 0.0, False)
            if self.publish_aligned_cloud:
                self._publish_aligned_cloud(msg.header.stamp, source, init)
            self.get_logger().warn(
                "ICP rejected: fitness=%.3f rmse=%.3f corr_t=%.3f corr_rot=%.1fdeg %s"
                % (
                    result.fitness,
                    result.inlier_rmse,
                    correction_translation,
                    np.rad2deg(correction_rotation),
                    alignment_summary,
                ),
                throttle_duration_sec=3.0,
            )
            self._publish_status_text(
                "ICP rejected: fitness=%.3f rmse=%.3f corr_t=%.2f corr_rot=%.1fdeg %s"
                % (
                    result.fitness,
                    result.inlier_rmse,
                    correction_translation,
                    np.rad2deg(correction_rotation),
                    alignment_summary,
                )
            )

    def _publish_result(self, stamp, map_t_body, map_t_odom, confidence, publish_status=True):
        map_t_base = map_t_body @ self.body_t_base
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        matrix_to_pose_msg(map_t_base, pose.pose)
        self.pose_pub.publish(pose)
        self._publish_current_marker(pose)
        if self.publish_2d_pose:
            pose_2d = self._pose_to_floor_2d(stamp, map_t_base)
            self.pose_2d_pub.publish(pose_2d)
            self._publish_current_marker_2d(pose_2d)

        self.confidence_pub.publish(Float32(data=confidence))

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.map_frame
            tf_msg.child_frame_id = self.odom_frame
            quat = matrix_to_quaternion(map_t_odom)
            tf_msg.transform.translation.x = float(map_t_odom[0, 3])
            tf_msg.transform.translation.y = float(map_t_odom[1, 3])
            tf_msg.transform.translation.z = float(map_t_odom[2, 3])
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(tf_msg)
            if self.publish_base_tf and self.base_frame != self.odom_frame:
                base_tf_msg = TransformStamped()
                base_tf_msg.header.stamp = stamp
                base_tf_msg.header.frame_id = self.map_frame
                base_tf_msg.child_frame_id = self.base_frame
                base_quat = matrix_to_quaternion(map_t_base)
                base_tf_msg.transform.translation.x = float(map_t_base[0, 3])
                base_tf_msg.transform.translation.y = float(map_t_base[1, 3])
                base_tf_msg.transform.translation.z = float(map_t_base[2, 3])
                base_tf_msg.transform.rotation.x = float(base_quat[0])
                base_tf_msg.transform.rotation.y = float(base_quat[1])
                base_tf_msg.transform.rotation.z = float(base_quat[2])
                base_tf_msg.transform.rotation.w = float(base_quat[3])
                self.tf_broadcaster.sendTransform(base_tf_msg)
        if publish_status:
            self._publish_status_text("localized: confidence=%.3f" % confidence)

    def _reset_icp_tracking_locked(self):
        self.icp_good_count = 0
        self.icp_hold_until_monotonic = time.monotonic() + max(0.0, self.icp_initial_hold_sec)

    def _make_body_t_base(self):
        yaw = math.radians(self.base_yaw_offset_deg)
        c = math.cos(yaw)
        s = math.sin(yaw)
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        mat[:3, 3] = np.array(
            [self.base_offset_x, self.base_offset_y, self.base_offset_z],
            dtype=np.float64,
        )
        return mat

    def _publish_current_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "luxi_localization"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 2.5
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.02
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _pose_to_floor_2d(self, stamp, map_t_body):
        floor_t_body = project_pose_to_floor_frame(map_t_body, self.floor_plane)
        pose_2d = PoseStamped()
        pose_2d.header.stamp = stamp
        pose_2d.header.frame_id = self.map_frame
        pose_2d.pose.position.x = float(floor_t_body[0, 3])
        pose_2d.pose.position.y = float(floor_t_body[1, 3])
        pose_2d.pose.position.z = 0.05
        matrix_to_pose_msg(floor_t_body, pose_2d.pose)
        pose_2d.pose.position.z = 0.05
        return pose_2d

    def _publish_current_marker_2d(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "luxi_localization_2d"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 1.6
        marker.scale.y = 0.28
        marker.scale.z = 0.28
        marker.color.r = 0.05
        marker.color.g = 1.0
        marker.color.b = 0.25
        marker.color.a = 1.0
        self.marker_2d_pub.publish(marker)

    def _publish_map_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        if self.publish_map_cloud:
            map_msg = xyz_array_to_pointcloud2(np.asarray(self.map_cloud.points), header)
            self.map_cloud_pub.publish(map_msg)
        if self.publish_floor_projected_cloud:
            floor_map_msg = xyz_array_to_pointcloud2(
                np.asarray(self.map_cloud_floor.points), header
            )
            self.map_cloud_floor_pub.publish(floor_map_msg)

    def _publish_map_anchor_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = "luxi_map_anchor"
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf_msg)

    def _publish_aligned_cloud(self, stamp, source, map_t_cloud):
        aligned = copy.deepcopy(source)
        aligned.transform(map_t_cloud)
        msg_header = Header()
        msg_header.stamp = stamp if stamp.sec or stamp.nanosec else self.get_clock().now().to_msg()
        msg_header.frame_id = self.map_frame
        cloud_msg = xyz_array_to_pointcloud2(np.asarray(aligned.points), msg_header)
        self.aligned_cloud_pub.publish(cloud_msg)
        floor_aligned = self._project_cloud_to_floor_frame(aligned)
        floor_cloud_msg = xyz_array_to_pointcloud2(
            np.asarray(floor_aligned.points), msg_header
        )
        self.aligned_cloud_floor_pub.publish(floor_cloud_msg)

    def _project_cloud_to_floor_frame(self, cloud):
        projected = copy.deepcopy(cloud)
        points = np.asarray(projected.points)
        if points.size:
            floor_points = project_points_to_floor_frame(points, self.floor_plane)
            mask = (
                (floor_points[:, 2] >= self.floor_projected_cloud_min_height)
                & (floor_points[:, 2] <= self.floor_projected_cloud_max_height)
            )
            floor_points = floor_points[mask]
            projected.points = o3d.utility.Vector3dVector(floor_points)
        return projected

    def _merged_scan_buffer_locked(self):
        merged = o3d.geometry.PointCloud()
        for cloud in self.scan_buffer_odom:
            merged += cloud
        if self.scan_voxel_size > 0.0:
            merged = merged.voxel_down_sample(self.scan_voxel_size)
        return merged

    def _crop_local_map(self, map_t_odom, odom_t_body):
        map_t_body = map_t_odom @ odom_t_body
        center = map_t_body[:3, 3]
        half_extent = self.local_map_extent / 2.0
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=center - half_extent,
            max_bound=center + half_extent,
        )
        return self.map_cloud.crop(bbox)

    def _initial_map_t_odom_locked(self, map_t_body_initial, odom_t_body):
        if not self.live_floor_alignment_enable or self.live_floor_normal_odom is None:
            return map_t_body_initial @ inverse_matrix(odom_t_body)

        map_normal, map_d = self._map_floor_model()
        odom_normal = np.asarray(self.live_floor_normal_odom, dtype=np.float64)
        odom_d = float(self.live_floor_d_odom) if self.live_floor_d_odom is not None else 0.0
        body_up_odom = odom_t_body[:3, 2]
        if np.dot(odom_normal, body_up_odom) < 0.0:
            odom_normal = -odom_normal
            odom_d = -odom_d

        odom_heading = odom_t_body[:3, 0]
        map_heading = map_t_body_initial[:3, 0]
        odom_basis = basis_from_normal_and_heading(odom_normal, odom_heading)
        map_basis = basis_from_normal_and_heading(map_normal, map_heading)

        map_t_odom = np.eye(4, dtype=np.float64)
        map_t_odom[:3, :3] = map_basis @ odom_basis.T
        map_t_odom[:3, 3] = map_t_body_initial[:3, 3] - map_t_odom[:3, :3] @ odom_t_body[:3, 3]
        # Also align the live scan floor height to the map floor. Without this,
        # a 2D initial pose puts the lidar/body origin on the floor and the live
        # cloud can appear below the map by roughly the sensor mounting height.
        target_floor_offset = odom_d - map_d
        current_floor_offset = float(np.dot(map_normal, map_t_odom[:3, 3]))
        map_t_odom[:3, 3] += (target_floor_offset - current_floor_offset) * map_normal
        self._publish_debug_text(
            (
                "live_floor_alignment ratio=%.3f map_normal=(%.3f %.3f %.3f) "
                "odom_normal=(%.3f %.3f %.3f) odom_d=%.3f"
            )
            % (
                self.live_floor_inlier_ratio,
                map_normal[0],
                map_normal[1],
                map_normal[2],
                odom_normal[0],
                odom_normal[1],
                odom_normal[2],
                odom_d,
            )
        )
        return map_t_odom

    def _clear_live_floor_locked(self):
        self.live_floor_normal_odom = None
        self.live_floor_d_odom = None
        self.live_floor_inlier_ratio = 0.0

    def _update_live_floor_plane_locked(self, source_odom, odom_t_body):
        if not self.live_floor_alignment_enable:
            return
        points = np.asarray(source_odom.points)
        if points.shape[0] < self.live_floor_fit_min_points:
            self._clear_live_floor_locked()
            return
        fit_points = points
        body_up = np.asarray(odom_t_body[:3, 2], dtype=np.float64)
        body_up_norm = np.linalg.norm(body_up)
        if body_up_norm > 1e-9:
            body_up = body_up / body_up_norm
        else:
            body_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        if self.live_floor_use_low_height_band:
            min_q = max(0.0, min(0.95, self.live_floor_band_min_quantile))
            max_q = max(min_q + 0.01, min(1.0, self.live_floor_band_max_quantile))
            heights = points @ body_up
            low, high = np.quantile(heights, [min_q, max_q])
            mask = (heights >= low) & (heights <= high)
            candidate_points = points[mask]
            if candidate_points.shape[0] >= self.live_floor_fit_min_points:
                fit_points = candidate_points
            else:
                self._clear_live_floor_locked()
                self._publish_status_text(
                    "live floor low band too small: %d < %d"
                    % (candidate_points.shape[0], self.live_floor_fit_min_points)
                )
                return

        fit_cloud = o3d.geometry.PointCloud()
        fit_cloud.points = o3d.utility.Vector3dVector(fit_points)
        try:
            plane_model, inliers = fit_cloud.segment_plane(
                distance_threshold=self.live_floor_fit_distance_threshold,
                ransac_n=self.live_floor_fit_ransac_n,
                num_iterations=self.live_floor_fit_iterations,
            )
        except RuntimeError as exc:
            self._clear_live_floor_locked()
            self.get_logger().warn("live floor plane fit failed: %s" % exc, throttle_duration_sec=5.0)
            return

        ratio = len(inliers) / max(1, fit_points.shape[0])
        if ratio < self.live_floor_min_inlier_ratio:
            self._clear_live_floor_locked()
            self._publish_status_text("live floor plane weak: ratio=%.3f" % ratio)
            return

        normal = np.asarray(plane_model[:3], dtype=np.float64)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            self._clear_live_floor_locked()
            return
        normal = normal / norm
        signed_plane_d = float(plane_model[3]) / norm
        if np.dot(normal, body_up) < 0.0:
            normal = -normal
            signed_plane_d = -signed_plane_d
        sensor_height = abs(signed_plane_d)

        body_up_alignment = abs(float(np.dot(normal, body_up)))
        if body_up_alignment < self.live_floor_min_abs_body_z:
            self._clear_live_floor_locked()
            self._publish_status_text(
                "live floor plane rejected: body_z_alignment=%.3f < %.3f"
                % (body_up_alignment, self.live_floor_min_abs_body_z)
            )
            return
        if (
            sensor_height < self.live_floor_min_sensor_height
            or sensor_height > self.live_floor_max_sensor_height
        ):
            self._clear_live_floor_locked()
            self._publish_status_text(
                "live floor plane rejected: sensor_height=%.3f signed_d=%.3f outside %.2f..%.2f"
                % (
                    sensor_height,
                    signed_plane_d,
                    self.live_floor_min_sensor_height,
                    self.live_floor_max_sensor_height,
                )
            )
            return

        self.live_floor_normal_odom = normal
        self.live_floor_d_odom = sensor_height
        self.live_floor_inlier_ratio = ratio
        self._publish_debug_text(
            "live_floor_plane normal=(%.3f %.3f %.3f) sensor_height=%.3f signed_d=%.3f ratio=%.3f points=%d fit=%d"
            % (
                normal[0],
                normal[1],
                normal[2],
                sensor_height,
                signed_plane_d,
                ratio,
                points.shape[0],
                fit_points.shape[0],
            )
        )

    def _initial_pose_msg_to_matrix(self, pose_msg):
        if self.initial_pose_coordinate_frame == "floor":
            return self._floor_initial_pose_msg_to_matrix(pose_msg)

        mat = pose_msg_to_matrix(pose_msg)
        if self.initial_pose_z_mode != "floor_plane":
            return mat

        x = float(pose_msg.position.x)
        y = float(pose_msg.position.y)
        a, b, c, d = self.floor_plane
        if abs(c) < 1e-9:
            self.get_logger().warn("floor_plane c is too small; keeping received z")
            return mat

        normal = self.floor_plane[:3]
        normal = normal / np.linalg.norm(normal)
        if normal[2] < 0.0:
            normal = -normal

        floor_z = float(-(a * x + b * y + d) / c)
        yaw = yaw_from_quaternion(
            [
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w,
            ]
        )
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = rotation_from_floor_yaw(normal, yaw)
        mat[:3, 3] = np.array([x, y, floor_z], dtype=np.float64)
        mat[:3, 3] += normal * self.initial_pose_height
        return mat

    def _floor_initial_pose_msg_to_matrix(self, pose_msg):
        u_axis, v_axis, normal, floor_d = floor_plane_basis(self.floor_plane)
        yaw = yaw_from_quaternion(
            [
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w,
            ]
        )
        heading = np.cos(yaw) * u_axis + np.sin(yaw) * v_axis

        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = basis_from_normal_and_heading(normal, heading)
        height = self.initial_pose_height
        if self.initial_pose_z_mode == "as_received":
            height = float(pose_msg.position.z)
        mat[:3, 3] = (
            float(pose_msg.position.x) * u_axis
            + float(pose_msg.position.y) * v_axis
            + (height - floor_d) * normal
        )
        return mat

    def _map_floor_normal(self):
        normal, _ = self._map_floor_model()
        return normal

    def _map_floor_model(self):
        normal = np.asarray(self.floor_plane[:3], dtype=np.float64)
        norm = max(np.linalg.norm(normal), 1e-12)
        normal = normal / norm
        plane_d = float(self.floor_plane[3]) / norm
        if normal[2] < 0.0:
            normal = -normal
            plane_d = -plane_d
        return normal, plane_d

    def _floor_pose_components(self, mat):
        u_axis, v_axis, normal, floor_d = floor_plane_basis(self.floor_plane)
        position = mat[:3, 3]
        heading = mat[:3, 0]
        return (
            float(position @ u_axis),
            float(position @ v_axis),
            float(position @ normal + floor_d),
            float(np.arctan2(heading @ v_axis, heading @ u_axis)),
        )

    def _matrix_from_floor_pose(self, x, y, height, yaw):
        u_axis, v_axis, normal, floor_d = floor_plane_basis(self.floor_plane)
        heading = np.cos(yaw) * u_axis + np.sin(yaw) * v_axis
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = basis_from_normal_and_heading(normal, heading)
        mat[:3, 3] = x * u_axis + y * v_axis + (height - floor_d) * normal
        return mat

    def _constrain_map_t_odom_to_floor(self, current_map_t_odom, target_map_t_odom, odom_t_body):
        current_map_t_body = current_map_t_odom @ odom_t_body
        target_map_t_body = target_map_t_odom @ odom_t_body
        current_x, current_y, current_height, current_yaw = self._floor_pose_components(
            current_map_t_body
        )
        target_x, target_y, _, target_yaw = self._floor_pose_components(target_map_t_body)
        del current_x, current_y

        yaw_delta = self._normalize_angle(target_yaw - current_yaw)
        _, _, map_normal, floor_d = floor_plane_basis(self.floor_plane)
        yaw_delta_rot = self._rotation_about_axis(map_normal, yaw_delta)
        constrained_map_t_odom = np.eye(4, dtype=np.float64)
        constrained_map_t_odom[:3, :3] = yaw_delta_rot @ current_map_t_odom[:3, :3]

        u_axis, v_axis, _, _ = floor_plane_basis(self.floor_plane)
        target_position = target_x * u_axis + target_y * v_axis + (
            current_height - floor_d
        ) * map_normal
        constrained_map_t_odom[:3, 3] = (
            target_position - constrained_map_t_odom[:3, :3] @ odom_t_body[:3, 3]
        )
        return constrained_map_t_odom

    def _run_icp(self, source, target, init, initial_acquisition):
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        transform = np.asarray(init, dtype=np.float64)
        if self.enable_multiscale_icp:
            coarse_source = source
            coarse_target = target
            if self.coarse_scan_voxel_size > 0.0:
                coarse_source = source.voxel_down_sample(self.coarse_scan_voxel_size)
            if self.coarse_map_voxel_size > 0.0:
                coarse_target = target.voxel_down_sample(self.coarse_map_voxel_size)
            coarse_distance = (
                self.initial_coarse_max_correspondence_distance
                if initial_acquisition
                else self.coarse_max_correspondence_distance
            )
            if (
                np.asarray(coarse_source.points).shape[0] >= self.min_scan_points
                and np.asarray(coarse_target.points).shape[0] >= self.min_local_map_points
                and coarse_distance > self.max_correspondence_distance
            ):
                coarse_criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.coarse_icp_max_iteration
                )
                coarse_result = o3d.pipelines.registration.registration_icp(
                    coarse_source,
                    coarse_target,
                    coarse_distance,
                    transform,
                    estimation,
                    coarse_criteria,
                )
                transform = np.asarray(coarse_result.transformation, dtype=np.float64)

        fine_criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=self.icp_max_iteration
        )
        fine_result = o3d.pipelines.registration.registration_icp(
            source,
            target,
            self.max_correspondence_distance,
            transform,
            estimation,
            fine_criteria,
        )
        return fine_result, np.asarray(fine_result.transformation, dtype=np.float64)

    def _search_initial_yaw(self, source, target, init, odom_t_body):
        if self.initial_yaw_search_step <= 0.0 or self.initial_yaw_search_range <= 0.0:
            return None, "initial_yaw_search skipped: invalid range/step"

        initial_map_t_body = init @ odom_t_body
        x, y, height, yaw = self._floor_pose_components(initial_map_t_body)
        max_steps = int(round(self.initial_yaw_search_range / self.initial_yaw_search_step))
        offsets = [0.0]
        for step in range(1, max_steps + 1):
            offset = step * self.initial_yaw_search_step
            if offset <= self.initial_yaw_search_range + 1e-6:
                offsets.append(offset)
                offsets.append(-offset)

        best = None
        best_score = -1.0
        best_offset = 0.0
        best_rmse = float("inf")
        for offset in offsets:
            candidate_body = self._matrix_from_floor_pose(
                x, y, height, self._normalize_angle(yaw + offset)
            )
            candidate_init = candidate_body @ inverse_matrix(odom_t_body)
            result, raw_map_t_odom = self._run_icp(source, target, candidate_init, True)
            candidate_map_t_odom = raw_map_t_odom
            if self.constrain_icp_to_floor:
                candidate_map_t_odom = self._constrain_map_t_odom_to_floor(
                    candidate_init, candidate_map_t_odom, odom_t_body
                )
            score = float(result.fitness / max(1.0, 1.0 + result.inlier_rmse))
            if (
                result.fitness >= self.initial_yaw_search_min_fitness
                and (
                    score > best_score
                    or (abs(score - best_score) < 1e-9 and result.inlier_rmse < best_rmse)
                )
            ):
                best = candidate_map_t_odom
                best_score = score
                best_offset = offset
                best_rmse = float(result.inlier_rmse)

        if best is None:
            return (
                None,
                "initial_yaw_search no_match candidates=%d min_fitness=%.3f"
                % (len(offsets), self.initial_yaw_search_min_fitness),
            )

        best_body = best @ odom_t_body
        _, _, _, best_yaw = self._floor_pose_components(best_body)
        return (
            best,
            (
                "initial_yaw_search selected offset=%.1fdeg yaw=%.1fdeg "
                "score=%.3f rmse=%.3f candidates=%d"
            )
            % (
                np.rad2deg(best_offset),
                np.rad2deg(best_yaw),
                best_score,
                best_rmse,
                len(offsets),
            ),
        )

    def _validate_alignment(self, source, target, map_t_odom):
        if not self.enable_alignment_validation:
            return True, "alignment=disabled"

        accepted = True
        parts = []
        map_normal, _ = self._map_floor_model()
        live_normal = self.live_floor_normal_odom
        if live_normal is None:
            return False, "alignment=rejected floor=missing"

        transformed_floor = map_t_odom[:3, :3] @ np.asarray(live_normal, dtype=np.float64)
        floor_angle = self._normal_angle(transformed_floor, map_normal)
        floor_limit = self.alignment_max_floor_normal_angle
        parts.append(
            "floor_angle_deg=%.2f/%.2f"
            % (np.rad2deg(floor_angle), np.rad2deg(floor_limit))
        )
        if floor_angle > floor_limit:
            accepted = False

        aligned_source = copy.deepcopy(source)
        aligned_source.transform(map_t_odom)
        source_plane = self._fit_dominant_plane(aligned_source)
        target_plane = self._fit_dominant_plane(target)
        if source_plane is not None and target_plane is not None:
            source_normal, _, source_ratio, source_points = source_plane
            target_normal, _, target_ratio, target_points = target_plane
            dominant_angle = self._normal_angle(source_normal, target_normal)
            parts.append(
                "dominant_plane_deg=%.2f/%.2f src_ratio=%.2f tgt_ratio=%.2f src=%d tgt=%d"
                % (
                    np.rad2deg(dominant_angle),
                    np.rad2deg(self.alignment_max_dominant_plane_angle),
                    source_ratio,
                    target_ratio,
                    source_points,
                    target_points,
                )
            )
            if dominant_angle > self.alignment_max_dominant_plane_angle:
                accepted = False
        else:
            parts.append("dominant_plane=insufficient")

        return accepted, "alignment=%s %s" % (
            "ok" if accepted else "rejected",
            " ".join(parts),
        )

    def _fit_dominant_plane(self, cloud):
        points = np.asarray(cloud.points)
        point_count = points.shape[0]
        if point_count < self.alignment_validation_min_points:
            return None
        try:
            plane_model, inliers = cloud.segment_plane(
                distance_threshold=self.alignment_validation_distance_threshold,
                ransac_n=3,
                num_iterations=self.alignment_validation_ransac_iterations,
            )
        except RuntimeError:
            return None

        ratio = len(inliers) / max(1, point_count)
        if ratio < self.alignment_validation_min_inlier_ratio:
            return None
        normal = np.asarray(plane_model[:3], dtype=np.float64)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            return None
        normal = normal / norm
        plane_d = float(plane_model[3]) / norm
        return normal, plane_d, ratio, point_count

    @staticmethod
    def _rotation_angle(rot):
        trace = max(-1.0, min(3.0, float(np.trace(rot))))
        return float(np.arccos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))

    @staticmethod
    def _normalize_angle(angle):
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    @staticmethod
    def _rotation_about_axis(axis, angle):
        axis = np.asarray(axis, dtype=np.float64)
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        x, y, z = axis
        c = math.cos(angle)
        s = math.sin(angle)
        one_minus_c = 1.0 - c
        return np.array(
            [
                [
                    c + x * x * one_minus_c,
                    x * y * one_minus_c - z * s,
                    x * z * one_minus_c + y * s,
                ],
                [
                    y * x * one_minus_c + z * s,
                    c + y * y * one_minus_c,
                    y * z * one_minus_c - x * s,
                ],
                [
                    z * x * one_minus_c - y * s,
                    z * y * one_minus_c + x * s,
                    c + z * z * one_minus_c,
                ],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _normal_angle(normal_a, normal_b):
        a = np.asarray(normal_a, dtype=np.float64)
        b = np.asarray(normal_b, dtype=np.float64)
        a = a / max(np.linalg.norm(a), 1e-12)
        b = b / max(np.linalg.norm(b), 1e-12)
        dot = abs(float(np.dot(a, b)))
        return float(np.arccos(max(-1.0, min(1.0, dot))))

    @staticmethod
    def _slerp_quaternion(q0, q1, ratio):
        q0 = np.asarray(q0, dtype=np.float64)
        q1 = np.asarray(q1, dtype=np.float64)
        dot = float(np.dot(q0, q1))
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        dot = max(-1.0, min(1.0, dot))
        if dot > 0.9995:
            q = q0 + ratio * (q1 - q0)
            return q / max(np.linalg.norm(q), 1e-12)
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * ratio
        sin_theta = np.sin(theta)
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        return s0 * q0 + s1 * q1

    def _blend_transform(self, current, target, ratio):
        ratio = max(0.0, min(1.0, float(ratio)))
        if ratio >= 0.999:
            return np.asarray(target, dtype=np.float64)
        if ratio <= 0.001:
            return np.asarray(current, dtype=np.float64)
        blended = np.eye(4, dtype=np.float64)
        current = np.asarray(current, dtype=np.float64)
        target = np.asarray(target, dtype=np.float64)
        blended[:3, 3] = current[:3, 3] + ratio * (target[:3, 3] - current[:3, 3])
        q = self._slerp_quaternion(matrix_to_quaternion(current), matrix_to_quaternion(target), ratio)
        blended[:3, :3] = quaternion_to_matrix(q)
        return blended

    def _publish_debug_text(self, text):
        self.debug_pub.publish(String(data=text))
        self.get_logger().info(text, throttle_duration_sec=1.0)

    def _on_set_parameters(self, params):
        try:
            for param in params:
                name = param.name
                value = param.value
                if name == "min_fitness_to_accept":
                    self.min_fitness_to_accept = float(value)
                elif name == "max_rmse_to_accept":
                    self.max_rmse_to_accept = float(value)
                elif name == "max_icp_translation_correction":
                    self.max_icp_translation_correction = float(value)
                elif name == "max_icp_rotation_correction_deg":
                    self.max_icp_rotation_correction = np.deg2rad(float(value))
                elif name == "max_initial_icp_translation_correction":
                    self.max_initial_icp_translation_correction = float(value)
                elif name == "max_initial_icp_rotation_correction_deg":
                    self.max_initial_icp_rotation_correction = np.deg2rad(float(value))
                elif name == "icp_correction_smoothing":
                    self.icp_correction_smoothing = float(value)
                elif name == "initial_icp_correction_smoothing":
                    self.initial_icp_correction_smoothing = float(value)
                elif name == "enable_multiscale_icp":
                    self.enable_multiscale_icp = bool(value)
                elif name == "coarse_scan_voxel_size":
                    self.coarse_scan_voxel_size = float(value)
                elif name == "coarse_map_voxel_size":
                    self.coarse_map_voxel_size = float(value)
                elif name == "coarse_max_correspondence_distance":
                    self.coarse_max_correspondence_distance = float(value)
                elif name == "initial_coarse_max_correspondence_distance":
                    self.initial_coarse_max_correspondence_distance = float(value)
                elif name == "coarse_icp_max_iteration":
                    self.coarse_icp_max_iteration = int(value)
                elif name == "enable_alignment_validation":
                    self.enable_alignment_validation = bool(value)
                elif name == "alignment_max_floor_normal_angle_deg":
                    self.alignment_max_floor_normal_angle = np.deg2rad(float(value))
                elif name == "alignment_max_dominant_plane_angle_deg":
                    self.alignment_max_dominant_plane_angle = np.deg2rad(float(value))
                elif name == "alignment_validation_min_points":
                    self.alignment_validation_min_points = int(value)
                elif name == "alignment_validation_min_inlier_ratio":
                    self.alignment_validation_min_inlier_ratio = float(value)
                elif name == "alignment_validation_distance_threshold":
                    self.alignment_validation_distance_threshold = float(value)
                elif name == "alignment_validation_ransac_iterations":
                    self.alignment_validation_ransac_iterations = int(value)
                elif name == "icp_initial_hold_sec":
                    self.icp_initial_hold_sec = float(value)
                elif name == "icp_required_consecutive_accepts":
                    self.icp_required_consecutive_accepts = max(1, int(value))
                elif name == "max_correspondence_distance":
                    self.max_correspondence_distance = float(value)
                elif name == "icp_max_iteration":
                    self.icp_max_iteration = int(value)
                elif name == "local_map_extent_x":
                    self.local_map_extent[0] = float(value)
                elif name == "local_map_extent_y":
                    self.local_map_extent[1] = float(value)
                elif name == "local_map_extent_z":
                    self.local_map_extent[2] = float(value)
                elif name == "scan_voxel_size":
                    self.scan_voxel_size = float(value)
                elif name == "max_odom_position_norm":
                    self.max_odom_position_norm = float(value)
                elif name == "max_odom_step":
                    self.max_odom_step = float(value)
                elif name == "base_offset_x":
                    self.base_offset_x = float(value)
                    self.body_t_base = self._make_body_t_base()
                    self.base_t_body = inverse_matrix(self.body_t_base)
                elif name == "base_offset_y":
                    self.base_offset_y = float(value)
                    self.body_t_base = self._make_body_t_base()
                    self.base_t_body = inverse_matrix(self.body_t_base)
                elif name == "base_offset_z":
                    self.base_offset_z = float(value)
                    self.body_t_base = self._make_body_t_base()
                    self.base_t_body = inverse_matrix(self.body_t_base)
                elif name == "base_yaw_offset_deg":
                    self.base_yaw_offset_deg = float(value)
                    self.body_t_base = self._make_body_t_base()
                    self.base_t_body = inverse_matrix(self.body_t_base)
                elif name == "initial_pose_is_base_frame":
                    self.initial_pose_is_base_frame = bool(value)
                elif name == "initial_yaw_search_enable":
                    self.initial_yaw_search_enable = bool(value)
                elif name == "initial_yaw_search_range_deg":
                    self.initial_yaw_search_range = np.deg2rad(float(value))
                elif name == "initial_yaw_search_step_deg":
                    self.initial_yaw_search_step = np.deg2rad(max(1.0, float(value)))
                elif name == "initial_yaw_search_min_fitness":
                    self.initial_yaw_search_min_fitness = float(value)
                elif name == "initial_pose_reacquire_timeout_sec":
                    self.initial_pose_reacquire_timeout_sec = max(0.0, float(value))
                elif name == "allow_initial_pose_without_live_floor":
                    self.allow_initial_pose_without_live_floor = bool(value)
                elif name == "live_floor_fit_min_points":
                    self.live_floor_fit_min_points = int(value)
                elif name == "live_floor_fit_ransac_n":
                    self.live_floor_fit_ransac_n = int(value)
                elif name == "live_floor_fit_iterations":
                    self.live_floor_fit_iterations = int(value)
                elif name == "live_floor_min_inlier_ratio":
                    self.live_floor_min_inlier_ratio = float(value)
                elif name == "live_floor_min_abs_body_z":
                    self.live_floor_min_abs_body_z = float(value)
                elif name == "live_floor_use_low_height_band":
                    self.live_floor_use_low_height_band = bool(value)
                elif name == "live_floor_band_min_quantile":
                    self.live_floor_band_min_quantile = float(value)
                elif name == "live_floor_band_max_quantile":
                    self.live_floor_band_max_quantile = float(value)
                elif name == "live_floor_min_sensor_height":
                    self.live_floor_min_sensor_height = float(value)
                elif name == "live_floor_max_sensor_height":
                    self.live_floor_max_sensor_height = float(value)
                elif name == "live_floor_fit_distance_threshold":
                    self.live_floor_fit_distance_threshold = float(value)
                elif name == "initial_pose_height":
                    self.initial_pose_height = float(value)
                elif name == "constrain_icp_to_floor":
                    self.constrain_icp_to_floor = bool(value)
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def _publish_status(self):
        now = time.monotonic()
        with self.lock:
            last_odom = self.last_odom_monotonic
            last_cloud = self.last_cloud_monotonic

        if last_odom is None:
            self._publish_status_text("waiting for odometry: %s" % self.odom_topic)
            return
        if last_cloud is None:
            self._publish_status_text("waiting for point cloud: %s" % self.input_cloud_topic)
            return
        if self.live_floor_alignment_enable and self.live_floor_normal_odom is None:
            self._publish_status_text("waiting for live floor plane")
            return
        if self.require_initial_pose and not self.has_initial_pose:
            self._publish_status_text("waiting for initial pose: %s" % self.initial_pose_topic)
            return

        self._publish_status_text(
            "running: odom_age=%.1fs cloud_age=%.1fs"
            % (now - last_odom, now - last_cloud)
        )

    def _publish_status_text(self, text):
        self.status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = Open3DLocalizationNode()
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
