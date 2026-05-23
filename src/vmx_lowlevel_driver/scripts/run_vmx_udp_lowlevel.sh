#!/usr/bin/env bash
# Runtime entrypoint for the VMX direct UDP low-level daemon.
#
# VMX side is intentionally ROS-free:
#   - receive motor duty commands by UDP on VMX_UDP_CMD_PORT (default 15000)
#   - publish IMU + Titan encoder/RPM telemetry by UDP on VMX_UDP_TELEM_PORT (default 15001)
#   - call Studica VMX HAL directly, same path as titan_port_test
#
# Run as root so the VMX HAL can claim pigpio/SPI/CAN.

set -euo pipefail

WS_ROOT="${VMX_LOWLEVEL_WS:-/home/vmx/lowlevel_cpp_ws}"
PKG_PREFIX="${WS_ROOT}/install/vmx_lowlevel_driver"
DAEMON="${PKG_PREFIX}/lib/vmx_lowlevel_driver/vmx_udp_lowlevel_daemon"

if [[ ! -x "${DAEMON}" ]]; then
  echo >&2 "ERROR: vmx_udp_lowlevel_daemon not found or not executable at ${DAEMON}."
  echo >&2 "       Build first: bash src/vmx_lowlevel_driver/scripts/build_vmx_release.sh"
  exit 1
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo >&2 "WARNING: not running as root (EUID=${EUID}); VMX HAL open may fail."
fi

export LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib/vmxpi:/usr/local/lib/studica_drivers:${LD_LIBRARY_PATH:-}"

exec "${DAEMON}" "$@"
