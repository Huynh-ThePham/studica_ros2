#!/usr/bin/env bash
# 4WD thẳng (khớp vmx_highlevel): M0,M1 = +P; M2,M3 = −P (sign_m2 = sign_m3 = −1).
# Bánh nên nhấc khỏi đất.
#
# Usage: test_4wd_straight_lowlevel.sh [power] [seconds]
# Env: ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY (match VMX)

set -eo pipefail

POWER="${1:-0.2}"
SECS="${2:-3}"
RATE_HZ="50"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

MP_QOS=(--qos-reliability reliable --qos-durability volatile)

stop_all() {
  ros2 topic pub "${MP_QOS[@]}" --once /titan/motor_power std_msgs/msg/Float32MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0]}" >/dev/null
}

trap 'stop_all || true' EXIT

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  Straight 4WD: M0,M1=+P; M2,M3=−P (${SECS}s)"
stop_all
sleep 0.3
mn="$(awk -v p="$POWER" 'BEGIN{printf "%.4f", -p}')"
timeout "${SECS}" ros2 topic pub "${MP_QOS[@]}" --rate "$RATE_HZ" /titan/motor_power std_msgs/msg/Float32MultiArray \
  "{data: [${POWER}, ${POWER}, ${mn}, ${mn}]}" >/dev/null 2>&1 || true
stop_all
echo "Done (stopped)."
