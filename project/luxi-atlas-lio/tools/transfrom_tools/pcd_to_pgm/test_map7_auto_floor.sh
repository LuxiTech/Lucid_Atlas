#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/nvidia/project/luxi-atlas"
TOOL_DIR="$ROOT/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm"
BIN="$TOOL_DIR/build/pcd2pgm"
INPUT="$ROOT/project/luxi-atlas-lio/maps/test_map7.pcd"
OUTPUT="${1:-/tmp/test_map7_auto_floor_class}"

if [[ ! -x "$BIN" ]]; then
  cmake --build "$TOOL_DIR/build" -j"$(nproc)"
fi

rm -rf "$OUTPUT"

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  "$BIN" \
  --input "$INPUT" \
  --auto-floor-plane \
  --resolution 0.05 \
  --voxel-leaf-size 0.05 \
  --min-z 0.05 \
  --max-z 0.50 \
  --min-component-cells 10 \
  --inflate-radius 0.05 \
  --output "$OUTPUT"

python3 - "$OUTPUT/map.json" <<'PY'
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as f:
    data = json.load(f)

checks = [
    ("auto_floor_used", data.get("auto_floor_used") is True),
    ("status_selected", data.get("auto_floor_status") == "selected"),
    ("normal_fit_used", data.get("auto_floor_normal_fit_used") is True),
    ("normal_fit_inlier_ratio", data.get("auto_floor_normal_inlier_ratio", 0.0) >= 0.10),
    ("lower_envelope_selected", data.get("auto_floor_quantile") == -1.0),
    ("floor_d_reasonable", 0.40 <= data.get("floor_plane", [0, 0, 0, 0])[3] <= 0.55),
    ("obstacle_ratio_range", 0.01 <= data.get("auto_floor_obstacle_ratio", -1.0) <= 0.18),
    ("below_ratio_range", 0.0 <= data.get("auto_floor_below_ratio", 2.0) <= 0.02),
    ("height_filtered_points_reasonable", 10000 <= data.get("height_filtered_points", 0) <= 60000),
    ("occupied_cells_reasonable", 3000 <= data.get("occupied_cells_after_inflation", 0) <= 30000),
    ("floor_plane_has_4_values", isinstance(data.get("floor_plane"), list) and len(data["floor_plane"]) == 4),
]

for name, ok in checks:
    print(f"{name}: {'PASS' if ok else 'FAIL'}")

if not all(ok for _, ok in checks):
    raise SystemExit(1)

print("selected_floor_plane:", data["floor_plane"])
print("score:", data["auto_floor_score"])
print("grid:", data["width"], data["height"])
PY
