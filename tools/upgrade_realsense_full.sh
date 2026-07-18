#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIRMWARE="${ROOT_DIR}/runtime/realsense_firmware/D4XX_FW_Image-5.17.3.10.bin"
SERIAL="${1:-832112071797}"
TOKEN="${GAMEUAV_AUTH_TOKEN:-uavuavuavuav}"

if [[ ! -f "${FIRMWARE}" ]]; then
  echo "Firmware file not found: ${FIRMWARE}" >&2
  exit 1
fi

cd "${ROOT_DIR}"

echo "[1/7] Stopping ROS flight/camera modules"
python3 tools/agentctl.py stop egoctrl --timeout 20 --auth-token "${TOKEN}" || true
python3 tools/agentctl.py stop realsense --timeout 20 --auth-token "${TOKEN}" || true
python3 tools/agentctl.py stop roscore --timeout 20 --auth-token "${TOKEN}" || true

echo "[2/7] Installing latest librealsense packages from configured apt repo"
sudo apt-get update
sudo apt-get install -y \
  librealsense2 \
  librealsense2-dbg \
  librealsense2-dev \
  librealsense2-dkms \
  librealsense2-gl \
  librealsense2-udev-rules \
  librealsense2-utils

echo "[3/7] Reloading udev rules"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[4/7] Current device info before firmware update"
rs-enumerate-devices | sed -n '1,35p'

echo "[5/7] Flashing RealSense firmware ${FIRMWARE} to serial ${SERIAL}"
rs-fw-update -s "${SERIAL}" -f "${FIRMWARE}"

echo "[6/7] Waiting for device to re-enumerate"
sleep 10
rs-enumerate-devices | sed -n '1,35p'

echo "[7/7] Rebuilding RealSense ROS wrapper and local helper against upgraded librealsense"
catkin_make --force-cmake --pkg realsense2_camera gameuav_usb_camera

echo
echo "Upgrade command sequence finished."
echo "Power-cycle the D435 or reboot the UAV before final flight validation."
