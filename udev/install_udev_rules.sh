#!/usr/bin/env bash
################################################################################
#
#  MIVIA Rover Sensing - udev Rules Installer
#  File: install_udev_rules.sh
#
################################################################################
#
#  DESCRIPTION
#  -----------
#  Installs the udev rule(s) shipped in this directory into
#  /etc/udev/rules.d/, then reloads udev so they take effect immediately
#  (no reboot required for devices plugged in afterwards; already-connected
#  devices are re-triggered too).
#
#  USAGE
#  -----
#  sudo ./install_udev_rules.sh
#
################################################################################

set -euo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd -P)"
readonly SYSTEMD_UDEV_DIR="/etc/udev/rules.d"

log() {
  printf '[%s] %s\n' "$(date -u +'%Y-%m-%dT%H:%M:%SZ')" "$*" >&2
}

die() {
  log "ERROR: $*"
  exit 1
}

[ "${EUID}" -eq 0 ] || die "Run this script with sudo (it writes to ${SYSTEMD_UDEV_DIR})."

shopt -s nullglob
rule_files=("${SCRIPT_DIR}"/*.rules)
shopt -u nullglob

[ "${#rule_files[@]}" -gt 0 ] || die "No .rules files found in ${SCRIPT_DIR}."

for rule_file in "${rule_files[@]}"; do
  dst="${SYSTEMD_UDEV_DIR}/$(basename "${rule_file}")"
  cp -f "${rule_file}" "${dst}"
  chmod 0644 "${dst}"
  log "Installed: ${dst}"
done

udevadm control --reload-rules
udevadm trigger

log "udev rules reloaded. Re-plug the sensor if its symlink doesn't appear immediately."
log "Verify with: ls -l /dev/xsens_imu"
