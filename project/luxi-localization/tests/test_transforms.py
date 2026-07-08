import math

import numpy as np

from luxi_localization.transforms import (
    inverse_matrix,
    matrix_to_quaternion,
    pose_to_matrix,
    project_points_to_floor_frame,
    project_pose_to_floor_frame,
    quaternion_to_matrix,
)


def test_quaternion_round_trip():
    q = np.array([0.0, 0.0, math.sin(0.3), math.cos(0.3)])
    mat = quaternion_to_matrix(q)
    out = matrix_to_quaternion(mat)
    assert np.allclose(np.abs(np.dot(q, out)), 1.0, atol=1e-6)


def test_inverse_matrix():
    mat = pose_to_matrix([1.0, -2.0, 0.5], [0.0, 0.0, math.sin(0.2), math.cos(0.2)])
    assert np.allclose(mat @ inverse_matrix(mat), np.eye(4), atol=1e-6)


def test_project_points_to_horizontal_floor_frame():
    points = np.array([[1.0, 2.0, 0.5], [-1.0, 3.0, 1.2]])
    projected = project_points_to_floor_frame(points, [0.0, 0.0, 1.0, 0.0])
    assert np.allclose(projected, points, atol=1e-6)


def test_project_pose_to_tilted_floor_frame():
    plane = [-0.357093, 0.006594, 0.934045, 1.001688]
    normal = np.asarray(plane[:3], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    point_on_floor = -plane[3] * normal

    mat = np.eye(4)
    mat[:3, 3] = point_on_floor
    projected = project_pose_to_floor_frame(mat, plane)

    assert abs(projected[2, 3]) < 1e-6
    assert abs(projected[2, 0]) < 1e-6
    assert abs(projected[2, 1]) < 1e-6
