#!/usr/bin/env bash
# Chỉ bật một cổng Titan M0..M3 để kiểm tra (dây, Titan, hộp số).
#
# Usage:
#   bash scripts/test_single_motor.sh <0-3> [power] [seconds]
#   bash scripts/test_single_motor.sh <0-3> [power] [seconds] neg   # gửi −power
#
# Env: ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY (khớp VMX)

PORT="${1:?usage: $0 <0-3> [power] [seconds] [neg]}"
POWER="${2:-0.22}"
SECS="${3:-5}"
NEG_FLAG="${4:-}"

if [[ ! "$PORT" =~ ^[0-3]$ ]]; then
  echo "port phải 0..3"
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

set -eo pipefail

MP_QOS=(--qos-reliability reliable --qos-durability volatile)

stop_all() {
  ros2 topic pub "${MP_QOS[@]}" --once /titan/motor_power std_msgs/msg/Float32MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0]}" >/dev/null
}

trap 'stop_all || true' EXIT

VAL="$POWER"
if [[ "$NEG_FLAG" == "neg" ]]; then
  VAL="$(awk -v p="$POWER" 'BEGIN { printf "%.4f", -p }')"
fi

m0=0.0 m1=0.0 m2=0.0 m3=0.0
case "$PORT" in
  0) m0="$VAL" ;;
  1) m1="$VAL" ;;
  2) m2="$VAL" ;;
  3) m3="$VAL" ;;
esac

RATE_HZ="50"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  M${PORT}  Titan duty=${VAL}  ${SECS}s"
echo "Theo dõi: ros2 topic echo /titan/motor${PORT}/speed  và bánh. Ctrl+C → dừng."
stop_all
sleep 0.3

timeout "${SECS}" ros2 topic pub "${MP_QOS[@]}" --rate "$RATE_HZ" /titan/motor_power std_msgs/msg/Float32MultiArray \
  "{data: [${m0}, ${m1}, ${m2}, ${m3}]}" >/dev/null 2>&1 || true

stop_all
echo "Xong M${PORT}."
