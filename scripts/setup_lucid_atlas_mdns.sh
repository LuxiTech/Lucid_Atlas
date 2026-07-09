#!/usr/bin/env bash
set -euo pipefail

HOST_NAME="${1:-lucid-atlas}"
AVAHI_INTERFACES="${2:-${LUCID_ATLAS_AVAHI_INTERFACES:-wlP1p1s0}}"
TARGET_PORT="${3:-${LUCID_ATLAS_TARGET_PORT:-8082}}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROXY_SCRIPT="${ROOT_DIR}/scripts/lucid_atlas_http_proxy.py"
AVAHI_CONFIG="/etc/avahi/avahi-daemon.conf"
AVAHI_SERVICE="/etc/avahi/services/lucid-atlas-http.service"
SYSTEMD_SERVICE="/etc/systemd/system/lucid-atlas-http-proxy.service"
SUDOERS_FILE="/etc/sudoers.d/lucid-atlas-mdns"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script with sudo:"
  echo "  sudo ${ROOT_DIR}/scripts/setup_lucid_atlas_mdns.sh"
  exit 1
fi

if [[ ! -f "${PROXY_SCRIPT}" ]]; then
  echo "Missing proxy script: ${PROXY_SCRIPT}" >&2
  exit 1
fi

if ! command -v avahi-daemon >/dev/null 2>&1; then
  apt-get update
  apt-get install -y avahi-daemon avahi-utils
fi

hostnamectl set-hostname "${HOST_NAME}"

python3 - "${AVAHI_CONFIG}" "${AVAHI_INTERFACES}" <<'PY'
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
interfaces = sys.argv[2]
text = path.read_text(encoding="utf-8").splitlines()

def set_section(section, settings):
    global text
    header = f"[{section}]"
    if header not in text:
        text.append("")
        text.append(header)
    section_index = text.index(header)
    next_section = len(text)
    for index in range(section_index + 1, len(text)):
        if text[index].startswith("[") and text[index].endswith("]"):
            next_section = index
            break

    lines = text[section_index + 1:next_section]
    seen = set()
    updated = []
    for line in lines:
        stripped = line.strip()
        key = stripped.split("=", 1)[0].lstrip("#").strip() if "=" in stripped else ""
        if key in settings:
            if key not in seen:
                updated.append(f"{key}={settings[key]}")
                seen.add(key)
            continue
        updated.append(line)

    for key, value in settings.items():
        if key not in seen:
            updated.append(f"{key}={value}")

    text = text[:section_index + 1] + updated + text[next_section:]

set_section(
    "server",
    {
        "use-ipv4": "yes",
        "use-ipv6": "no",
        "allow-interfaces": interfaces,
    },
)
set_section(
    "publish",
    {
        "publish-addresses": "yes",
        "publish-aaaa-on-ipv4": "no",
        "publish-a-on-ipv6": "no",
    },
)
path.write_text("\n".join(text) + "\n", encoding="utf-8")
PY

mkdir -p /etc/avahi/services
cat > "${AVAHI_SERVICE}" <<EOF
<?xml version="1.0" standalone='no'?>
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">
<service-group>
  <name replace-wildcards="yes">Lucid Atlas Web on %h</name>
  <service>
    <type>_http._tcp</type>
    <port>80</port>
    <txt-record>path=/</txt-record>
  </service>
</service-group>
EOF

cat > "${SYSTEMD_SERVICE}" <<EOF
[Unit]
Description=Lucid Atlas HTTP port 80 proxy
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 ${PROXY_SCRIPT} --listen 0.0.0.0 --listen-port 80 --target-host 127.0.0.1 --target-port ${TARGET_PORT}
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
EOF

cat > "${SUDOERS_FILE}" <<EOF
nvidia ALL=(root) NOPASSWD: ${ROOT_DIR}/scripts/setup_lucid_atlas_mdns.sh
nvidia ALL=(root) NOPASSWD: ${ROOT_DIR}/scripts/setup_lucid_atlas_mdns.sh *
EOF
chmod 0440 "${SUDOERS_FILE}"
visudo -cf "${SUDOERS_FILE}" >/dev/null

systemctl daemon-reload
systemctl enable --now avahi-daemon
systemctl restart avahi-daemon
systemctl enable --now lucid-atlas-http-proxy.service
systemctl restart lucid-atlas-http-proxy.service

echo "Configured ${HOST_NAME}.local -> this device via Avahi."
echo "Avahi interfaces: ${AVAHI_INTERFACES}"
echo "Forwarding http://${HOST_NAME}.local:80 to local port ${TARGET_PORT}."
echo "Open: http://${HOST_NAME}.local"
