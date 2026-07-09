#!/usr/bin/env bash
set -u

HOST_NAME="${3:-lucid-atlas}"
WIFI_INTERFACE="${1:-wlP1p1s0}"
TARGET_PORT="${2:-8082}"
SETUP_SCRIPT="/home/nvidia/project/luxi-atlas/scripts/setup_lucid_atlas_mdns.sh"

if [[ ! -x "${SETUP_SCRIPT}" ]]; then
  echo "[lucid-mdns] setup script not found: ${SETUP_SCRIPT}" >&2
  exit 0
fi

if sudo -n "${SETUP_SCRIPT}" "${HOST_NAME}" "${WIFI_INTERFACE}" "${TARGET_PORT}"; then
  echo "[lucid-mdns] http://${HOST_NAME}.local -> ${WIFI_INTERFACE}:80 -> 127.0.0.1:${TARGET_PORT}"
  exit 0
fi

echo "[lucid-mdns] unable to refresh mDNS/proxy without sudo permission." >&2
echo "[lucid-mdns] run once: sudo ${SETUP_SCRIPT} ${HOST_NAME} ${WIFI_INTERFACE} ${TARGET_PORT}" >&2
exit 0
