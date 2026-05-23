#!/usr/bin/env bash
# Test 4WD vi sai: tới, lùi, quẹo phải, quẹo trái — publish /titan/motor_power trực tiếp
# (không cần vmx_highlevel đang chạy). Khớp vmx_highlevel.yaml:
#   right_power → M0+M1, left_power → M2+M3; motor_command_sign_m2 = m3 = -1
#
# Usage: test_motion_4wd_maneuvers.sh [linear_scale] [angular_scale] [seconds_each] [pause]
#   linear_scale: |v| tối đa kiểu cmd_vel linear.x (~0.12–0.25)
#   angular_scale: |ω| rad/s cho quẹo (~0.6–1.2)
#
set -eo pipefail

LIN="${1:-0.18}"
ANG="${2:-0.9}"
SECS="${3:-3}"
PAUSE="${4:-2}"
RATE_HZ="40"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

MP_QOS=(--qos-reliability reliable --qos-durability volatile)

WHEELBASE="0.28"
MAX_WHEEL_V="0.70"
MAX_P="0.55"

# Trả về "m0 m1 m2 m3" (float) từ linear (m/s) và angular (rad/s) như diff_drive_highlevel_node.
powers_from_twist() {
  local linear="$1"
  local angular="$2"
  python3 -c "
import math
lin, ang = float('$linear'), float('$angular')
wb, mw, lim = float('$WHEELBASE'), float('$MAX_WHEEL_V'), float('$MAX_P')
lv = lin - ang * wb * 0.5
rv = lin + ang * wb * 0.5
def c(x):
    return max(-lim, min(lim, x / mw))
lp, rp = c(lv), c(rv)
# Bố cổng: right M0,M1 | left M2,M3 → Titan [rp, rp, -lp, -lp]
m0, m1, m2, m3 = rp, rp, -lp, -lp
print(f'{m0:.5f} {m1:.5f} {m2:.5f} {m3:.5f}')
"
}

stop_all() {
  ros2 topic pub "${MP_QOS[@]}" --once /titan/motor_power std_msgs/msg/Float32MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0]}" >/dev/null
}

run_phase() {
  local name="$1"
  local lin="$2"
  local ang="$3"
  read -r m0 m1 m2 m3 <<< "$(powers_from_twist "$lin" "$ang")"
  echo ""
  echo ">>> ${name}  (tương đương cmd_vel lin.x=${lin} ang.z=${ang})"
  echo "    motor_power [M0,M1,M2,M3] = [${m0}, ${m1}, ${m2}, ${m3}]"
  timeout "${SECS}" ros2 topic pub "${MP_QOS[@]}" --rate "$RATE_HZ" /titan/motor_power std_msgs/msg/Float32MultiArray \
    "{data: [${m0}, ${m1}, ${m2}, ${m3}]}" >/dev/null 2>&1 || true
  stop_all
  echo "    dừng. Nghỉ ${PAUSE}s..."
  sleep "${PAUSE}"
}

trap 'stop_all || true' EXIT

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Tham số: |lin|=${LIN} m/s, |ang|=${ANG} rad/s, mỗi pha ${SECS}s (max motor |power|=${MAX_P})"
echo "Nhấc bánh hoặc chỗ trống an toàn. Chiều ROS: ang.z>0 ≈ quẹo trái, ang.z<0 ≈ quẹo phải."
stop_all
sleep 0.5

run_phase "TIẾN" "${LIN}" "0"
run_phase "LÙI" "-${LIN}" "0"
run_phase "QUẸO PHẢI (ang.z âm)" "0" "-${ANG}"
run_phase "QUẸO TRÁI (ang.z dương)" "0" "${ANG}"

echo ""
echo "Xong bốn pha. Nếu quẹo ngược mong đời: đảo dấu tham số ANG hoặc motor_command_sign trên M0..M3."
stop_all
