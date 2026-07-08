#!/usr/bin/env bash
set +u

ROOT_DIR="${LUXI_ATLAS_ROOT:-/home/nvidia/project/luxi-atlas}"
SETUP_FILE="${ROOT_DIR}/install/setup.bash"
API_HOST="${LUXI_WEB_HOST:-10.42.0.149}"
API_PORT="${LUXI_WEB_API_PORT:-8082}"
HTTP_PORT="${LUXI_WEB_HTTP_PORT:-${API_PORT}}"
API_BASE="http://${API_HOST}:${API_PORT}"

if [[ -f "${SETUP_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${SETUP_FILE}"
fi

set -u

pass_count=0
fail_count=0
warn_count=0

pass() {
  echo "[PASS] $*"
  pass_count=$((pass_count + 1))
}

fail() {
  echo "[FAIL] $*"
  fail_count=$((fail_count + 1))
}

warn() {
  echo "[WARN] $*"
  warn_count=$((warn_count + 1))
}

check_http() {
  local name="$1"
  local url="$2"
  local code
  code="$(curl -sS -o /tmp/luxi_web_check.out -w "%{http_code}" --max-time 3 "${url}" 2>/tmp/luxi_web_check.err || true)"
  if [[ "${code}" == "200" || "${code}" == "304" ]]; then
    pass "${name}: ${url}"
  else
    fail "${name}: ${url} http=${code} $(cat /tmp/luxi_web_check.err 2>/dev/null)"
  fi
}

check_topic_once() {
  local topic="$1"
  local field="${2:-header}"
  if timeout 6 ros2 topic echo --no-daemon "${topic}" --once --field "${field}" >/tmp/luxi_topic_check.out 2>/tmp/luxi_topic_check.err; then
    pass "topic has data: ${topic}"
  else
    fail "topic has no data within 6s: ${topic} $(cat /tmp/luxi_topic_check.err 2>/dev/null)"
  fi
}

check_topic_info() {
  local topic="$1"
  if ros2 topic info --no-daemon "${topic}" >/tmp/luxi_topic_info.out 2>/tmp/luxi_topic_info.err; then
    pass "topic exists: ${topic} $(tr '\n' ' ' </tmp/luxi_topic_info.out)"
  else
    fail "topic missing: ${topic} $(cat /tmp/luxi_topic_info.err 2>/dev/null)"
  fi
}

echo "Luxi web runtime check"
echo "API: ${API_BASE}"
echo "WEB: http://${API_HOST}:${HTTP_PORT}"

check_http "web page" "http://${API_HOST}:${HTTP_PORT}/"
check_http "api status" "${API_BASE}/api/status"
check_http "map status" "${API_BASE}/api/map/status"

api_status="$(curl -sS --max-time 3 "${API_BASE}/api/status" 2>/dev/null || true)"
map_status="$(curl -sS --max-time 3 "${API_BASE}/api/map/status" 2>/dev/null || true)"
waiting_initial_pose=0
if printf "%s" "${map_status}" | grep -q "waiting for initial pose"; then
  waiting_initial_pose=1
fi

check_topic_info "/cmd_vel"
check_topic_info "/web/cmd_vel"
check_topic_info "/map"
check_topic_info "/luxi_localization/map_cloud_floor"
check_topic_once "/hikrobot/cu013/rgb_img/compressed" "format"
check_topic_once "/Odometry" "header"

if [[ "${waiting_initial_pose}" -eq 0 ]]; then
  check_topic_once "/luxi_localization/aligned_cloud_floor" "header"
  check_topic_once "/localization_2d" "header"
else
  warn "localization is waiting for /initialpose; aligned cloud and /localization_2d will start after setting initial pose from the web map"
fi

if printf "%s" "${api_status}" | grep -q '"has_camera_frame":true'; then
  pass "api has cached camera frame"
else
  fail "api has no cached camera frame"
fi

if printf "%s" "${map_status}" | grep -q '"has_grid":true'; then
  pass "api has cached grid map"
else
  fail "api has no cached grid map"
fi

if printf "%s" "${map_status}" | grep -q '"has_static_cloud":true'; then
  pass "api has cached static map cloud"
else
  fail "api has no cached static map cloud"
fi

if printf "%s" "${map_status}" | grep -q '"has_odom_pose":true'; then
  pass "api has cached odom pose"
else
  fail "api has no cached odom pose"
fi

if printf "%s" "${map_status}" | grep -q '"has_live_cloud":true'; then
  pass "api has cached live cloud"
elif [[ "${waiting_initial_pose}" -eq 1 ]]; then
  warn "api has no live cloud because localization is waiting for /initialpose"
else
  fail "api has no cached live cloud"
fi

echo "Result: ${pass_count} passed, ${warn_count} warned, ${fail_count} failed"
if [[ "${fail_count}" -ne 0 ]]; then
  exit 1
fi
