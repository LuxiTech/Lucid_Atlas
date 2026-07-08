import numpy as np
from sensor_msgs_py import point_cloud2


def pointcloud2_to_xyz_array(msg, max_points=0):
    points = point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )

    if isinstance(points, np.ndarray):
        if points.dtype.names:
            xyz = np.vstack((points["x"], points["y"], points["z"])).T
        else:
            xyz = np.asarray(points[:, :3], dtype=np.float64)
    else:
        xyz = np.asarray(list(points), dtype=np.float64)
        if xyz.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        xyz = xyz[:, :3]

    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if max_points and xyz.shape[0] > max_points:
        step = max(1, xyz.shape[0] // max_points)
        xyz = xyz[::step][:max_points]
    return np.asarray(xyz, dtype=np.float64)


def xyz_array_to_pointcloud2(xyz, header):
    xyz = np.asarray(xyz, dtype=np.float32)
    return point_cloud2.create_cloud_xyz32(header, xyz.tolist())
