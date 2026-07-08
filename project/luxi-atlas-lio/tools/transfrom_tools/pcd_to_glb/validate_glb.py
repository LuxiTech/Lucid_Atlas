#!/usr/bin/env python3
"""Validate a GLB mesh with trimesh."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import trimesh


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate a GLB file exported for Habitat-Sim.")
    parser.add_argument("glb", help="Path to the GLB file")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    path = Path(args.glb).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(path)

    scene = trimesh.load(path, force="scene")
    vertices = sum(len(geom.vertices) for geom in scene.geometry.values())
    faces = sum(len(geom.faces) for geom in scene.geometry.values() if hasattr(geom, "faces"))

    if scene.is_empty or vertices == 0 or faces == 0:
        raise RuntimeError(f"{path} is empty or has no triangle geometry")

    result = {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "geometry_count": len(scene.geometry),
        "vertices": vertices,
        "faces": faces,
        "bounds": scene.bounds.tolist(),
        "is_empty": bool(scene.is_empty),
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
