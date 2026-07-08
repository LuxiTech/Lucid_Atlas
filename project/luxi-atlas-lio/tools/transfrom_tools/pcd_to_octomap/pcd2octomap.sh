#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:${LD_PRELOAD}}"

exec "${SCRIPT_DIR}/build/pcd2octomap" "$@"
