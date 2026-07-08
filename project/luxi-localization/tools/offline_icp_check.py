#!/usr/bin/env python3
import argparse
import json
import math

import numpy as np
import open3d as o3d


def make_transform(x, y, z, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    mat[:3, 3] = [x, y, z]
    return mat


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "map_path",
        nargs="?",
        default="/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd",
    )
    parser.add_argument("--voxel", type=float, default=0.25)
    parser.add_argument("--max-correspondence", type=float, default=1.0)
    parser.add_argument("--sample-points", type=int, default=30000)
    parser.add_argument("--dx", type=float, default=0.20)
    parser.add_argument("--dy", type=float, default=-0.10)
    parser.add_argument("--dz", type=float, default=0.03)
    parser.add_argument("--yaw-deg", type=float, default=3.0)
    args = parser.parse_args()

    target = o3d.io.read_point_cloud(args.map_path)
    if target.is_empty():
        raise RuntimeError("empty map: %s" % args.map_path)

    target = target.remove_non_finite_points()
    if args.voxel > 0.0:
        target = target.voxel_down_sample(args.voxel)

    target_points = np.asarray(target.points)
    if target_points.shape[0] > args.sample_points:
        rng = np.random.default_rng(7)
        indices = rng.choice(target_points.shape[0], args.sample_points, replace=False)
        source_points = target_points[indices]
    else:
        source_points = target_points.copy()

    known = make_transform(args.dx, args.dy, args.dz, math.radians(args.yaw_deg))
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(source_points)
    source.transform(np.linalg.inv(known))

    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        args.max_correspondence,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60),
    )

    error = np.linalg.inv(known) @ result.transformation
    trans_error = float(np.linalg.norm(error[:3, 3]))
    rot_trace = max(-1.0, min(3.0, float(np.trace(error[:3, :3]))))
    rot_error_rad = float(math.acos(max(-1.0, min(1.0, (rot_trace - 1.0) / 2.0))))

    print(
        json.dumps(
            {
                "map_path": args.map_path,
                "map_points": int(np.asarray(target.points).shape[0]),
                "source_points": int(np.asarray(source.points).shape[0]),
                "fitness": float(result.fitness),
                "inlier_rmse": float(result.inlier_rmse),
                "translation_error_m": trans_error,
                "rotation_error_deg": math.degrees(rot_error_rad),
                "success": bool(
                    result.fitness > 0.8 and trans_error < 0.15 and math.degrees(rot_error_rad) < 2.0
                ),
            },
            indent=2,
        )
    )

    if not (result.fitness > 0.8 and trans_error < 0.15 and math.degrees(rot_error_rad) < 2.0):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
