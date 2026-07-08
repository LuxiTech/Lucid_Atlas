#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/nvidia/project/luxi-atlas"
TOOL_DIR="$ROOT/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm"
BIN="$TOOL_DIR/build/pcd2pgm"
MAP_DIR="$ROOT/project/luxi-atlas-lio/maps"
BASELINE_MAP6="$TOOL_DIR/output/test_map6_nav_floor_plane_autofit_h005_050_clean80/map.json"
OUT_ROOT="${1:-/tmp/luxi_floor_regression}"
MAP6_OUT="$OUT_ROOT/test_map6"
MAP7_OUT="$OUT_ROOT/test_map7"

if [[ ! -x "$BIN" ]]; then
  cmake --build "$TOOL_DIR/build" -j"$(nproc)"
fi

rm -rf "$OUT_ROOT"
mkdir -p "$OUT_ROOT"

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  "$BIN" \
  --input "$MAP_DIR/test_map6.pcd" \
  --auto-floor-plane \
  --expected-floor-normal -0.351947 -0.006438 0.935998 \
  --resolution 0.05 \
  --voxel-leaf-size 0.05 \
  --min-z 0.05 \
  --max-z 0.50 \
  --min-component-cells 80 \
  --inflate-radius 0.05 \
  --output "$MAP6_OUT" >/dev/null

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  "$BIN" \
  --input "$MAP_DIR/test_map7.pcd" \
  --auto-floor-plane \
  --expected-floor-normal -0.341854 -0.016993 0.939599 \
  --resolution 0.05 \
  --voxel-leaf-size 0.05 \
  --min-z 0.05 \
  --max-z 0.50 \
  --min-component-cells 10 \
  --inflate-radius 0.05 \
  --output "$MAP7_OUT" >/dev/null

python3 - "$BASELINE_MAP6" "$MAP6_OUT/map.json" "$MAP7_OUT/map.json" <<'PY'
import json
import math
import sys

baseline6_path, map6_path, map7_path = sys.argv[1:4]
baseline6 = json.load(open(baseline6_path, "r", encoding="utf-8"))
map6 = json.load(open(map6_path, "r", encoding="utf-8"))
map7 = json.load(open(map7_path, "r", encoding="utf-8"))

def angle_deg(a, b):
    def norm(v):
        total = math.sqrt(sum(x * x for x in v))
        return [x / total for x in v]
    a = norm(a)
    b = norm(b)
    cosine = max(-1.0, min(1.0, abs(sum(x * y for x, y in zip(a, b)))))
    return math.degrees(math.acos(cosine))

checks = [
    ("map6_floor_plane_unchanged", all(abs(a - b) < 1e-6 for a, b in zip(map6["floor_plane"], baseline6["floor_plane"]))),
    ("map6_grid_unchanged", map6["width"] == baseline6["width"] and map6["height"] == baseline6["height"]),
    ("map6_points_unchanged", map6["height_filtered_points"] == baseline6["height_filtered_points"]),
    ("map7_auto_selected", map7.get("auto_floor_used") is True and map7.get("auto_floor_status") == "selected"),
    ("map7_lower_envelope_selected", map7.get("auto_floor_quantile") == -1.0),
    ("map7_normal_distinct_from_map6", 5.0 <= angle_deg(map7["floor_plane"][:3], map6["floor_plane"][:3]) <= 7.0),
    ("map7_floor_d_reasonable", 0.40 <= map7["floor_plane"][3] <= 0.55),
    ("map7_obstacle_ratio_reasonable", 0.07 <= map7.get("auto_floor_obstacle_ratio", -1.0) <= 0.10),
    ("map7_below_ratio_reasonable", 0.0 <= map7.get("auto_floor_below_ratio", -1.0) <= 0.02),
    ("map7_ground_fit_ratio_reasonable", map7.get("auto_floor_normal_inlier_ratio", 0.0) >= 0.10),
    ("map7_grid_reasonable", 730 <= map7["width"] <= 790 and 1260 <= map7["height"] <= 1320),
]

for name, ok in checks:
    print(f"{name}: {'PASS' if ok else 'FAIL'}")

if not all(ok for _, ok in checks):
    print("map6_floor_plane:", map6["floor_plane"])
    print("baseline6_floor_plane:", baseline6["floor_plane"])
    print("map7_floor_plane:", map7["floor_plane"])
    print("map7_metrics:", {
        "quantile": map7.get("auto_floor_quantile"),
        "obstacle": map7.get("auto_floor_obstacle_ratio"),
        "below": map7.get("auto_floor_below_ratio"),
        "grid": [map7.get("width"), map7.get("height")],
    })
    raise SystemExit(1)

print("map6_floor_plane:", map6["floor_plane"])
print("map7_floor_plane:", map7["floor_plane"])
print("map7_grid:", map7["width"], map7["height"])
PY
