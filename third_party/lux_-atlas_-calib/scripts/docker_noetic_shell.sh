#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"

docker build -t luxi-fast-calib-noetic "$REPO_ROOT/docker/noetic"

docker run --rm -it \
  --network host \
  -e DISPLAY="${DISPLAY:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp:/host_tmp:rw \
  -v "$PROJECT_ROOT":/home/nvidia/project/luxi-atlas:rw \
  luxi-fast-calib-noetic \
  bash
