#!/usr/bin/env bash
# Launch the ROS-free VMX UDP low-level daemon detached and line-buffered.
#
# This keeps the VMX runtime clean: one Studica HAL process owns pigpio/SPI/CAN.
# Logs:
#   sudo tail -f /var/log/vmx-udp-lowlevel.log

set -euo pipefail

LOG="${VMX_UDP_LOWLEVEL_LOG:-/var/log/vmx-udp-lowlevel.log}"
PIDFILE="${VMX_UDP_LOWLEVEL_PIDFILE:-/run/vmx-udp-lowlevel.pid}"
WS_ROOT="${VMX_LOWLEVEL_WS:-/home/vmx/lowlevel_cpp_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALLED_RUNNER="${WS_ROOT}/install/vmx_lowlevel_driver/share/vmx_lowlevel_driver/scripts/run_vmx_udp_lowlevel.sh"

if [[ -f "${INSTALLED_RUNNER}" ]]; then
  RUNNER="${INSTALLED_RUNNER}"
else
  RUNNER="${SCRIPT_DIR}/run_vmx_udp_lowlevel.sh"
fi

if [[ ! -f "${RUNNER}" ]]; then
  echo >&2 "ERROR: run_vmx_udp_lowlevel.sh not found."
  exit 1
fi

if [[ ${EUID} -ne 0 ]]; then
  echo "INFO: re-invoking under sudo to claim pigpio/SPI/CAN..."
  exec sudo --preserve-env=VMX_LOWLEVEL_WS,VMX_UDP_LOWLEVEL_LOG,VMX_UDP_CMD_PORT,VMX_UDP_TELEM_PORT,VMX_UDP_PC_IP,VMX_TITAN_CAN_ID,VMX_TITAN_MOTOR_FREQ,VMX_TICKS_PER_ROTATION,VMX_WHEEL_RADIUS_M,VMX_UDP_CONTROL_HZ,VMX_UDP_TELEMETRY_HZ,VMX_UDP_CMD_TIMEOUT_SEC,VMX_TITAN_STOP_MODE,VMX_TITAN_ZERO_HOLD_SEC,VMX_TITAN_CURRENT_LIMIT_A,VMX_TITAN_CURRENT_LIMIT_MODE \
    -- bash "${BASH_SOURCE[0]}" "$@"
fi

if pgrep -f "/vmx_lowlevel_driver/.*/vmx_udp_lowlevel_daemon" >/dev/null 2>&1; then
  echo >&2 "ERROR: vmx_udp_lowlevel_daemon is already running. Stop it first:"
  echo >&2 "       sudo pkill -TERM -f vmx_udp_lowlevel_daemon"
  exit 1
fi

install -d -m 0755 "$(dirname "${LOG}")"
rm -f "${LOG}"

(
  exec </dev/null >"${LOG}" 2>&1
  export LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib/vmxpi:/usr/local/lib/studica_drivers:${LD_LIBRARY_PATH:-}"
  exec stdbuf -oL -eL bash "${RUNNER}" "$@"
) &
disown "$!"

DAEMON_PID=""
for _ in $(seq 1 40); do
  DAEMON_PID="$(pgrep -f /vmx_lowlevel_driver/.*/vmx_udp_lowlevel_daemon | head -1 || true)"
  if [[ -n "${DAEMON_PID}" ]]; then
    break
  fi
  sleep 0.5
done

if [[ -z "${DAEMON_PID}" ]]; then
  echo >&2 "ERROR: vmx_udp_lowlevel_daemon did not appear within 20 s. Last 40 log lines:"
  tail -40 "${LOG}" 2>&1 >&2 || true
  exit 1
fi

if ! ( umask 022 && echo "${DAEMON_PID}" > "${PIDFILE}" ) 2>/dev/null; then
  echo >&2 "WARNING: failed to write PID file ${PIDFILE}; systemd supervision may not work."
fi

echo "vmx_udp_lowlevel_daemon started, pid=${DAEMON_PID}, log -> ${LOG}"
