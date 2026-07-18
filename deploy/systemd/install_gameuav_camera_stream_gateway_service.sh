#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

SERVICE_SRC="${SCRIPT_DIR}/gameuav-camera-stream-gateway.service"
ENV_SRC="${SCRIPT_DIR}/gameuav-camera-stream-gateway.env"
SERVICE_DST="/etc/systemd/system/gameuav-camera-stream-gateway.service"
ENV_DIR="/etc/gameuav"
ENV_DST="${ENV_DIR}/camera-stream-gateway.env"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run with sudo:"
  echo "  sudo ${BASH_SOURCE[0]}"
  exit 1
fi

install -d "${ENV_DIR}"
install -m 0644 "${ENV_SRC}" "${ENV_DST}"

sed "s#/home/uav/Desktop/uav_project/gameuav#${WORKSPACE_ROOT}#g" \
  "${ENV_DST}" > "${ENV_DST}.tmp"
mv "${ENV_DST}.tmp" "${ENV_DST}"

sed "s#/home/uav/Desktop/uav_project/gameuav#${WORKSPACE_ROOT}#g" \
  "${SERVICE_SRC}" > "${SERVICE_DST}"

systemctl daemon-reload
systemctl enable gameuav-camera-stream-gateway.service

echo "Installed gameuav-camera-stream-gateway.service"
echo "Edit ${ENV_DST} if needed, then run:"
echo "  sudo systemctl restart gameuav-camera-stream-gateway.service"
echo "  systemctl status gameuav-camera-stream-gateway.service"
