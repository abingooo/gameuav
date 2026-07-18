#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ENV_SRC="${SCRIPT_DIR}/gameuav-gateway.env"
ENV_DIR="/etc/gameuav"
ENV_DST="${ENV_DIR}/gateway.env"

SERVICES=(
  "gameuav-ros-to-net-gateway.service"
  "gameuav-net-to-ros-gateway.service"
)

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

for service in "${SERVICES[@]}"; do
  sed "s#/home/uav/Desktop/uav_project/gameuav#${WORKSPACE_ROOT}#g" \
    "${SCRIPT_DIR}/${service}" > "/etc/systemd/system/${service}"
done

systemctl daemon-reload
systemctl enable "${SERVICES[@]}"

echo "Installed GameUAV gateway services:"
printf '  %s\n' "${SERVICES[@]}"
echo "Edit ${ENV_DST} if needed, then run:"
echo "  sudo systemctl restart gameuav-ros-to-net-gateway.service"
echo "  sudo systemctl restart gameuav-net-to-ros-gateway.service"
