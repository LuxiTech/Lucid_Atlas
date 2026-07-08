import os
from pathlib import Path
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

try:
    from PIL import Image
except ImportError as exc:
    Image = None
    PIL_IMPORT_ERROR = exc
else:
    PIL_IMPORT_ERROR = None


class GridMapPublisher(Node):
    def __init__(self):
        super().__init__("luxi_grid_map_publisher")

        self.declare_parameter(
            "map_yaml",
            "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output/test_map5_nav_floor_plane_autofit_h005_050_clean80/map.yaml",
        )
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/map")
        self.declare_parameter("publish_period_sec", 2.0)

        self.map_yaml = self.get_parameter("map_yaml").value
        self.frame_id = self.get_parameter("frame_id").value
        self.topic = self.get_parameter("topic").value
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(OccupancyGrid, self.topic, qos)
        self.map_msg = self._load_map(self.map_yaml)
        self.create_timer(self.publish_period_sec, self._publish_map)
        self._publish_map()
        self.get_logger().info(
            "published grid map: %s -> %s, size=%dx%d, resolution=%.3f"
            % (
                self.map_yaml,
                self.topic,
                self.map_msg.info.width,
                self.map_msg.info.height,
                self.map_msg.info.resolution,
            )
        )

    def _load_map(self, map_yaml):
        if Image is None:
            raise RuntimeError("Pillow is required to load PGM maps: %s" % PIL_IMPORT_ERROR)
        metadata_path, image_path = self._resolve_map_paths(map_yaml)
        if metadata_path and not os.path.exists(metadata_path):
            raise FileNotFoundError(metadata_path)
        if image_path and not os.path.exists(image_path):
            raise FileNotFoundError(image_path)

        with open(metadata_path, "r", encoding="utf-8") as yaml_file:
            metadata = yaml.safe_load(yaml_file)

        if Path(map_yaml).suffix.lower() not in (".pgm", ".png"):
            image_path = metadata.get("image")
            if not image_path:
                raise ValueError("map yaml missing image field: %s" % metadata_path)
            if not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(metadata_path), image_path)
            if not os.path.exists(image_path):
                raise FileNotFoundError(image_path)

        resolution = float(metadata["resolution"])
        origin = metadata.get("origin", [0.0, 0.0, 0.0])
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        free_thresh = float(metadata.get("free_thresh", 0.196))
        negate = int(metadata.get("negate", 0))

        image = Image.open(image_path).convert("L")
        pixels = np.asarray(image, dtype=np.float32)
        if negate:
            occ = pixels / 255.0
        else:
            occ = (255.0 - pixels) / 255.0

        data = np.full(occ.shape, -1, dtype=np.int8)
        data[occ > occupied_thresh] = 100
        data[occ < free_thresh] = 0
        data = np.flipud(data)

        msg = OccupancyGrid()
        msg.header = Header(frame_id=self.frame_id)
        msg.info = MapMetaData()
        msg.info.map_load_time = self.get_clock().now().to_msg()
        msg.info.resolution = resolution
        msg.info.width = int(image.width)
        msg.info.height = int(image.height)
        msg.info.origin = Pose()
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        yaw = float(origin[2]) if len(origin) > 2 else 0.0
        msg.info.origin.orientation.z = float(np.sin(yaw * 0.5))
        msg.info.origin.orientation.w = float(np.cos(yaw * 0.5))
        msg.data = data.reshape(-1).astype(np.int8).tolist()
        return msg

    def _resolve_map_paths(self, map_path):
        path = Path(map_path)
        suffix = path.suffix.lower()
        if suffix in (".pgm", ".png"):
            metadata_path = path.with_suffix(".yaml")
            if not metadata_path.exists():
                metadata_path = path.parent / "map.yaml"
            return str(metadata_path), str(path)
        return str(path), ""

    def _publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
