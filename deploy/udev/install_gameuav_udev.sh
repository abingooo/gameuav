#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULE_SRC="${SCRIPT_DIR}/99-gameuav-serial.rules"
RULE_DST="/etc/udev/rules.d/99-gameuav-serial.rules"
RUN_USER="${SUDO_USER:-${USER}}"

if [[ ! -f "${RULE_SRC}" ]]; then
  echo "missing udev rule file: ${RULE_SRC}" >&2
  exit 1
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo "please run with sudo: sudo ${BASH_SOURCE[0]}" >&2
  exit 1
fi

install -m 0644 "${RULE_SRC}" "${RULE_DST}"

if id "${RUN_USER}" >/dev/null 2>&1; then
  usermod -aG dialout "${RUN_USER}"
fi

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

echo "installed ${RULE_DST}"
echo "current serial links:"
ls -l /dev/gameuav_px4 /dev/gameuav_tiplight /dev/serial/by-id 2>/dev/null || true
echo
echo "If '${RUN_USER}' was newly added to dialout, log out and log back in."
