#!/usr/bin/env bash
# Spin each Titan motor M0..M3 one at a time so you can note physical wheel direction.
# Uses /titan/motor_power in VMX Titan port order.
#
# Default: raw duty +POWER on that port only (others 0). On many 4WD layouts the
# right pair (M0+M1) turns one way and the left pair (M2+M3) the opposite for the
# same "+" command — mirrored mounting, not a wiring error.
#
# Usage:
#   ./scripts/test_motor_spin_direction.sh [power] [seconds_per_motor]
# Env (match your LAN / DDS):
#   ROS_DOMAIN_ID (default 0), ROS_LOCALHOST_ONLY (default 0)
#   USE_CALIB_SIGN=1 — duty on active port = power × motor_command_sign_m*
#     (defaults: SIGN_M0=1 SIGN_M1=1 SIGN_M2=-1 SIGN_M3=-1, same as vmx_highlevel.yaml).

# Default 0.25 — values like 0.12 are often too weak to see motion (Titan / gearbox / friction).
POWER="${1:-0.25}"
SECS="${2:-3}"
RATE_HZ="50"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

set -eo pipefail

# Khớp subscriber vmx_lowlevel (VOLATILE). Mặc định ros2 CLI = TRANSIENT_LOCAL → dễ lệch QoS / publisher “ma”.
MP_QOS=(--qos-reliability reliable --qos-durability volatile)

stop_all() {
  ros2 topic pub "${MP_QOS[@]}" --once /titan/motor_power std_msgs/msg/Float32MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0]}" >/dev/null
}

trap 'stop_all || true' EXIT

# shellcheck disable=SC2034
SIGN_M0="${SIGN_M0:-1}"
SIGN_M1="${SIGN_M1:-1}"
SIGN_M2="${SIGN_M2:--1}"
SIGN_M3="${SIGN_M3:--1}"

port_signed_duty() {
  local port="$1"
  local s
  case "$port" in
    0) s="$SIGN_M0" ;;
    1) s="$SIGN_M1" ;;
    2) s="$SIGN_M2" ;;
    3) s="$SIGN_M3" ;;
    *) s="1" ;;
  esac
  awk -v p="$POWER" -v s="$s" 'BEGIN { printf "%.4f", p * s }'
}

spin_one() {
  local idx="$1"
  local m0 m1 m2 m3
  local duty_msg
  m0=0.0; m1=0.0; m2=0.0; m3=0.0
  if [[ "${USE_CALIB_SIGN:-0}" == "1" ]]; then
    case "$idx" in
      0) m0="$(port_signed_duty 0)" ;;
      1) m1="$(port_signed_duty 1)" ;;
      2) m2="$(port_signed_duty 2)" ;;
      3) m3="$(port_signed_duty 3)" ;;
      *) echo "bad index"; exit 1 ;;
    esac
  else
    case "$idx" in
      0) m0="$POWER" ;;
      1) m1="$POWER" ;;
      2) m2="$POWER" ;;
      3) m3="$POWER" ;;
      *) echo "bad index"; exit 1 ;;
    esac
  fi
  echo ""
  if [[ "${USE_CALIB_SIGN:-0}" == "1" ]]; then
    duty_msg="M${idx}: Titan duty [${m0}, ${m1}, ${m2}, ${m3}] (${SECS}s, USE_CALIB_SIGN — same polarity scale as vmx_highlevel per port)"
  else
    duty_msg="M${idx}: raw Titan +${POWER} on port ${idx} only (${SECS}s). Others 0."
  fi
  echo ">>> ${duty_msg}"
  echo "    Observe wheel direction."
  # Use timeout + foreground pub: killing background `ros2 topic pub` can invalidate
  # the next ros2 CLI invocation in the same shell (rcl context).
  timeout "${SECS}" ros2 topic pub "${MP_QOS[@]}" --rate "$RATE_HZ" /titan/motor_power std_msgs/msg/Float32MultiArray \
    "{data: [${m0}, ${m1}, ${m2}, ${m3}]}" >/dev/null 2>&1 || true
  stop_all
  echo ">>> M${idx}: stop. Pause..."
  sleep 1
}

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "Power per motor=${POWER}, hold=${SECS}s. Ctrl+C aborts (motors stopped by trap)."
if [[ "${USE_CALIB_SIGN:-0}" == "1" ]]; then
  echo "USE_CALIB_SIGN=1 (signs M0..M3: ${SIGN_M0} ${SIGN_M1} ${SIGN_M2} ${SIGN_M3})"
else
  echo "Raw +duty on each port: M0+M1 (phải) thường ngược chiều M2+M3 (trái) — đối xứng khung. Bình thường."
fi
stop_all
sleep 0.5

for i in 0 1 2 3; do
  spin_one "$i"
done

echo "Done. All motors stopped."
