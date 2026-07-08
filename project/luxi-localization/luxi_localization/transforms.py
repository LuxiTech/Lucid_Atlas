import math

import numpy as np


def normalize_quaternion(q):
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / norm


def quaternion_to_matrix(q):
    x, y, z, w = normalize_quaternion(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(matrix):
    m = np.asarray(matrix, dtype=np.float64)[:3, :3]
    trace = float(np.trace(m))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    return normalize_quaternion([x, y, z, w])


def yaw_from_quaternion(q):
    x, y, z, w = normalize_quaternion(q)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotation_from_floor_yaw(floor_normal, yaw):
    z_axis = np.asarray(floor_normal, dtype=np.float64)
    z_norm = np.linalg.norm(z_axis)
    if z_norm < 1e-12:
        z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        z_axis = z_axis / z_norm
    if z_axis[2] < 0.0:
        z_axis = -z_axis

    heading = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)
    x_axis = heading - np.dot(heading, z_axis) * z_axis
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-12:
        x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        x_axis = x_axis - np.dot(x_axis, z_axis) * z_axis
        x_axis = x_axis / np.linalg.norm(x_axis)
    else:
        x_axis = x_axis / x_norm

    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)

    return np.column_stack((x_axis, y_axis, z_axis))


def basis_from_normal_and_heading(normal, heading):
    z_axis = np.asarray(normal, dtype=np.float64)
    z_axis = z_axis / max(np.linalg.norm(z_axis), 1e-12)

    x_axis = np.asarray(heading, dtype=np.float64)
    x_axis = x_axis - np.dot(x_axis, z_axis) * z_axis
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-12:
        fallback = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(np.dot(fallback, z_axis)) > 0.95:
            fallback = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        x_axis = fallback - np.dot(fallback, z_axis) * z_axis
        x_axis = x_axis / np.linalg.norm(x_axis)
    else:
        x_axis = x_axis / x_norm

    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / max(np.linalg.norm(y_axis), 1e-12)
    x_axis = np.cross(y_axis, z_axis)
    x_axis = x_axis / max(np.linalg.norm(x_axis), 1e-12)
    return np.column_stack((x_axis, y_axis, z_axis))


def floor_plane_basis(floor_plane):
    plane = np.asarray(floor_plane, dtype=np.float64)
    if plane.shape[0] != 4:
        raise ValueError("floor_plane must contain four coefficients")

    normal = plane[:3].copy()
    norm = np.linalg.norm(normal)
    if norm < 1e-12:
        raise ValueError("floor_plane normal must be non-zero")
    normal /= norm
    floor_d = float(plane[3]) / norm
    if normal[2] < 0.0:
        normal = -normal
        floor_d = -floor_d

    seed = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(normal[0]) > 0.9:
        seed = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    u_axis = seed - np.dot(seed, normal) * normal
    u_axis = u_axis / max(np.linalg.norm(u_axis), 1e-12)
    v_axis = np.cross(normal, u_axis)
    v_axis = v_axis / max(np.linalg.norm(v_axis), 1e-12)
    return u_axis, v_axis, normal, floor_d


def project_points_to_floor_frame(points, floor_plane):
    points = np.asarray(points, dtype=np.float64)
    u_axis, v_axis, normal, floor_d = floor_plane_basis(floor_plane)
    projected = np.empty_like(points)
    projected[:, 0] = points @ u_axis
    projected[:, 1] = points @ v_axis
    projected[:, 2] = points @ normal + floor_d
    return projected


def project_pose_to_floor_frame(mat, floor_plane):
    mat = np.asarray(mat, dtype=np.float64)
    u_axis, v_axis, normal, floor_d = floor_plane_basis(floor_plane)
    position = mat[:3, 3]
    projected = np.eye(4, dtype=np.float64)
    projected[0, 3] = float(position @ u_axis)
    projected[1, 3] = float(position @ v_axis)
    projected[2, 3] = float(position @ normal + floor_d)

    heading = mat[:3, 0]
    yaw = math.atan2(float(heading @ v_axis), float(heading @ u_axis))
    projected[0, 0] = math.cos(yaw)
    projected[0, 1] = -math.sin(yaw)
    projected[1, 0] = math.sin(yaw)
    projected[1, 1] = math.cos(yaw)
    return projected


def pose_to_matrix(position, orientation):
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = quaternion_to_matrix(orientation)
    mat[:3, 3] = np.asarray(position, dtype=np.float64)
    return mat


def inverse_matrix(mat):
    mat = np.asarray(mat, dtype=np.float64)
    inv = np.eye(4, dtype=np.float64)
    rot = mat[:3, :3]
    trans = mat[:3, 3]
    inv[:3, :3] = rot.T
    inv[:3, 3] = -rot.T @ trans
    return inv


def pose_msg_to_matrix(pose_msg):
    pos = pose_msg.position
    ori = pose_msg.orientation
    return pose_to_matrix([pos.x, pos.y, pos.z], [ori.x, ori.y, ori.z, ori.w])


def matrix_to_pose_msg(mat, pose_msg):
    quat = matrix_to_quaternion(mat)
    pose_msg.position.x = float(mat[0, 3])
    pose_msg.position.y = float(mat[1, 3])
    pose_msg.position.z = float(mat[2, 3])
    pose_msg.orientation.x = float(quat[0])
    pose_msg.orientation.y = float(quat[1])
    pose_msg.orientation.z = float(quat[2])
    pose_msg.orientation.w = float(quat[3])
    return pose_msg
