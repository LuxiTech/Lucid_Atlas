#!/usr/bin/env python3
"""Convert PCD point clouds into GLB meshes for Habitat-Sim.

The default path is tuned for FAST-LIO2 maps produced in this repository:

  maps/room_001.pcd -> tools/transfrom_tools/pcd_to_glb/output/room_001.glb

PCD files are point clouds, while Habitat-Sim expects triangle meshes for
stages/assets. The default "voxel" method turns occupied point-cloud cells into
small cubes, which is robust for sparse LiDAR maps. Surface reconstruction modes
are also provided for visual experiments.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np
import open3d as o3d


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_INPUT = REPO_ROOT / "maps" / "room_001.pcd"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "output" / "room_001.glb"


CubeKey = Tuple[int, int, int]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a PCD point cloud into a GLB mesh usable by Habitat-Sim."
    )
    parser.add_argument(
        "input",
        nargs="?",
        default=str(DEFAULT_INPUT),
        help=f"Input PCD path. Default: {DEFAULT_INPUT}",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=str(DEFAULT_OUTPUT),
        help=f"Output GLB path. Default: {DEFAULT_OUTPUT}",
    )
    parser.add_argument(
        "--method",
        choices=("voxel", "bpa", "poisson"),
        default="voxel",
        help="Mesh generation method. voxel is most robust for sparse LiDAR maps.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.15,
        help="Voxel size in meters for voxel meshing/downsampling.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=0,
        help="Optional cap after downsampling for surface methods. 0 disables the cap.",
    )
    parser.add_argument(
        "--color-mode",
        choices=("height", "constant", "none"),
        default="height",
        help="Vertex color mode.",
    )
    parser.add_argument(
        "--constant-color",
        nargs=3,
        type=float,
        default=(0.72, 0.74, 0.78),
        metavar=("R", "G", "B"),
        help="RGB color in 0..1 when --color-mode constant is used.",
    )
    parser.add_argument(
        "--no-ros-to-habitat",
        action="store_true",
        help="Keep ROS coordinates. By default ROS z-up is converted to glTF/Habitat y-up.",
    )
    parser.add_argument(
        "--poisson-depth",
        type=int,
        default=8,
        help="Poisson reconstruction depth.",
    )
    parser.add_argument(
        "--bpa-radius-factor",
        type=float,
        default=2.5,
        help="Ball-pivoting base radius = factor * voxel_size.",
    )
    parser.add_argument(
        "--target-triangles",
        type=int,
        default=250000,
        help="Simplify output mesh if it has more triangles than this. 0 disables simplification.",
    )
    parser.add_argument(
        "--metadata",
        default="",
        help="Optional JSON metadata output path. Default: <output>.json",
    )
    return parser.parse_args()


def ros_to_habitat_points(points: np.ndarray) -> np.ndarray:
    """Convert ROS x-forward/y-left/z-up into glTF/Habitat y-up coordinates."""
    converted = np.empty_like(points)
    converted[:, 0] = points[:, 0]
    converted[:, 1] = points[:, 2]
    converted[:, 2] = -points[:, 1]
    return converted


def height_colors(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    v_min = float(np.min(values))
    v_max = float(np.max(values))
    if math.isclose(v_min, v_max):
        t = np.full(values.shape, 0.5, dtype=np.float64)
    else:
        t = (values - v_min) / (v_max - v_min)

    # Simple blue -> green -> yellow height ramp.
    colors = np.empty((values.shape[0], 3), dtype=np.float64)
    colors[:, 0] = np.clip(1.7 * t - 0.25, 0.0, 1.0)
    colors[:, 1] = np.clip(1.4 - np.abs(t - 0.55) * 1.7, 0.0, 1.0)
    colors[:, 2] = np.clip(1.25 - 1.7 * t, 0.0, 1.0)
    return colors


def apply_vertex_colors(mesh: o3d.geometry.TriangleMesh, color_mode: str, constant_color: Iterable[float]) -> None:
    if color_mode == "none":
        return

    vertices = np.asarray(mesh.vertices)
    if vertices.size == 0:
        return

    if color_mode == "height":
        colors = height_colors(vertices[:, 1])
    else:
        color = np.asarray(tuple(constant_color), dtype=np.float64)
        colors = np.repeat(color.reshape(1, 3), vertices.shape[0], axis=0)

    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)


def load_points(path: Path, ros_to_habitat: bool) -> np.ndarray:
    pcd = o3d.io.read_point_cloud(str(path))
    points = np.asarray(pcd.points, dtype=np.float64)
    if points.size == 0:
        raise RuntimeError(f"No points loaded from {path}")

    if ros_to_habitat:
        points = ros_to_habitat_points(points)

    finite_mask = np.isfinite(points).all(axis=1)
    points = points[finite_mask]
    if points.size == 0:
        raise RuntimeError("All points are invalid after finite filtering")
    return points


def voxel_mesh(points: np.ndarray, voxel_size: float, color_mode: str, constant_color: Iterable[float]) -> o3d.geometry.TriangleMesh:
    if voxel_size <= 0:
        raise ValueError("--voxel-size must be > 0")

    keys = np.floor(points / voxel_size).astype(np.int64)
    unique_keys = np.unique(keys, axis=0)
    occupied = {tuple(key.tolist()) for key in unique_keys}

    half = voxel_size * 0.5
    corner_offsets = np.array(
        [
            [-half, -half, -half],
            [half, -half, -half],
            [half, half, -half],
            [-half, half, -half],
            [-half, -half, half],
            [half, -half, half],
            [half, half, half],
            [-half, half, half],
        ],
        dtype=np.float64,
    )

    # Each face entry: neighbor key delta, quad corner indices.
    faces = [
        ((0, 0, -1), (0, 3, 2, 1)),
        ((0, 0, 1), (4, 5, 6, 7)),
        ((0, -1, 0), (0, 1, 5, 4)),
        ((0, 1, 0), (3, 7, 6, 2)),
        ((-1, 0, 0), (0, 4, 7, 3)),
        ((1, 0, 0), (1, 2, 6, 5)),
    ]

    vertices: List[List[float]] = []
    triangles: List[List[int]] = []
    vertex_colors: List[List[float]] = []

    constant = np.asarray(tuple(constant_color), dtype=np.float64)
    min_y = float(np.min((unique_keys[:, 1].astype(np.float64) + 0.5) * voxel_size))
    max_y = float(np.max((unique_keys[:, 1].astype(np.float64) + 0.5) * voxel_size))

    def color_for(center_y: float) -> np.ndarray:
        if color_mode == "none":
            return np.zeros(3, dtype=np.float64)
        if color_mode == "constant":
            return constant
        if math.isclose(min_y, max_y):
            t = 0.5
        else:
            t = (center_y - min_y) / (max_y - min_y)
        return np.array(
            [
                np.clip(1.7 * t - 0.25, 0.0, 1.0),
                np.clip(1.4 - abs(t - 0.55) * 1.7, 0.0, 1.0),
                np.clip(1.25 - 1.7 * t, 0.0, 1.0),
            ],
            dtype=np.float64,
        )

    for key in occupied:
        center = (np.asarray(key, dtype=np.float64) + 0.5) * voxel_size
        cube_vertices = center + corner_offsets
        cube_color = color_for(center[1])

        for delta, quad in faces:
            neighbor = (key[0] + delta[0], key[1] + delta[1], key[2] + delta[2])
            if neighbor in occupied:
                continue

            base = len(vertices)
            for corner_index in quad:
                vertices.append(cube_vertices[corner_index].tolist())
                if color_mode != "none":
                    vertex_colors.append(cube_color.tolist())
            triangles.append([base, base + 1, base + 2])
            triangles.append([base, base + 2, base + 3])

    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(np.asarray(vertices, dtype=np.float64)),
        o3d.utility.Vector3iVector(np.asarray(triangles, dtype=np.int32)),
    )
    if color_mode != "none" and vertex_colors:
        mesh.vertex_colors = o3d.utility.Vector3dVector(np.asarray(vertex_colors, dtype=np.float64))

    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh


def surface_mesh(
    points: np.ndarray,
    method: str,
    voxel_size: float,
    max_points: int,
    poisson_depth: int,
    bpa_radius_factor: float,
    color_mode: str,
    constant_color: Iterable[float],
) -> o3d.geometry.TriangleMesh:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pcd = pcd.voxel_down_sample(voxel_size)

    if max_points > 0 and len(pcd.points) > max_points:
        pcd = pcd.random_down_sample(max_points / len(pcd.points))

    radius = max(voxel_size * 3.0, 0.05)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=40)
    )
    pcd.orient_normals_consistent_tangent_plane(30)

    if method == "poisson":
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=poisson_depth
        )
        density_values = np.asarray(densities)
        keep = density_values > np.quantile(density_values, 0.02)
        mesh.remove_vertices_by_mask(~keep)
        mesh = mesh.crop(pcd.get_axis_aligned_bounding_box())
    else:
        base_radius = max(voxel_size * bpa_radius_factor, 0.02)
        radii = o3d.utility.DoubleVector([base_radius, base_radius * 2.0, base_radius * 4.0])
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, radii)

    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    mesh.remove_unreferenced_vertices()
    apply_vertex_colors(mesh, color_mode, constant_color)
    mesh.compute_vertex_normals()
    return mesh


def simplify_if_needed(mesh: o3d.geometry.TriangleMesh, target_triangles: int) -> o3d.geometry.TriangleMesh:
    triangle_count = len(mesh.triangles)
    if target_triangles <= 0 or triangle_count <= target_triangles:
        return mesh

    simplified = mesh.simplify_quadric_decimation(target_triangles)
    simplified.remove_degenerate_triangles()
    simplified.remove_duplicated_triangles()
    simplified.remove_duplicated_vertices()
    simplified.remove_unreferenced_vertices()
    simplified.compute_vertex_normals()
    return simplified


def mesh_stats(mesh: o3d.geometry.TriangleMesh, input_path: Path, output_path: Path, args: argparse.Namespace) -> Dict[str, object]:
    bbox = mesh.get_axis_aligned_bounding_box()
    return {
        "input": str(input_path),
        "output": str(output_path),
        "method": args.method,
        "voxel_size": args.voxel_size,
        "ros_to_habitat": not args.no_ros_to_habitat,
        "vertices": len(mesh.vertices),
        "triangles": len(mesh.triangles),
        "bbox_min": bbox.get_min_bound().tolist(),
        "bbox_max": bbox.get_max_bound().tolist(),
    }


def main() -> None:
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    metadata_path = Path(args.metadata).expanduser().resolve() if args.metadata else output_path.with_suffix(output_path.suffix + ".json")

    if not input_path.exists():
        raise FileNotFoundError(input_path)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.parent.mkdir(parents=True, exist_ok=True)

    points = load_points(input_path, ros_to_habitat=not args.no_ros_to_habitat)

    if args.method == "voxel":
        mesh = voxel_mesh(points, args.voxel_size, args.color_mode, args.constant_color)
    else:
        mesh = surface_mesh(
            points=points,
            method=args.method,
            voxel_size=args.voxel_size,
            max_points=args.max_points,
            poisson_depth=args.poisson_depth,
            bpa_radius_factor=args.bpa_radius_factor,
            color_mode=args.color_mode,
            constant_color=args.constant_color,
        )

    mesh = simplify_if_needed(mesh, args.target_triangles)
    if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        raise RuntimeError("Generated mesh is empty")

    ok = o3d.io.write_triangle_mesh(str(output_path), mesh, write_ascii=False)
    if not ok:
        raise RuntimeError(f"Open3D failed to write {output_path}")

    stats = mesh_stats(mesh, input_path, output_path, args)
    stats["output_size_bytes"] = output_path.stat().st_size
    metadata_path.write_text(json.dumps(stats, indent=2), encoding="utf-8")

    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()
