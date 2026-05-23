#!/usr/bin/env bash
# Test từng cặp: right_power → M0+M1, left_power → M2+M3; rồi thẳng 4 bánh.
#
# Usage: test_motor_pairs_sync.sh [power] [seconds_per_phase] [pause_seconds]

POWER="${1:-0.22}"
SECS="${2:-6}"
PAUSE="${3:-2}"
# 20 Hz matches vmx_highlevel.yaml. The PC UDP bridge downsamples to the
# VMX daemon's 10 Hz direct-HAL cadence, which bench-tested closest to
# titan_port_test while keeping CAN traffic conservative.
RATE_HZ="${RATE_HZ:-20}"

# Match src/vmx_highlevel/config/vmx_highlevel.yaml.
GAIN_M0="${GAIN_M0:-1.00}"
GAIN_M1="${GAIN_M1:-1.00}"
GAIN_M2="${GAIN_M2:-1.04}"
GAIN_M3="${GAIN_M3:-1.03}"

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

read_rpm() {
  local p="$1"
  timeout 2.5 ros2 topic echo "/titan/motor${p}/speed" --once 2>/dev/null | awk '/data:/{print $2; exit}'
}

sample_ports() {
  local tag="$1"
  shift
  echo "    ${tag}"
  for p in "$@"; do
    v="$(read_rpm "$p")"
    echo "      M${p}/speed = ${v:-?}"
  done
}

phase_power() {
  local label="$1"
  local m0="$2" m1="$3" m2="$4" m3="$5"
  shift 5
  local -a show_ports=("$@")

  echo ""
  echo ">>> ${label}"
  ros2 topic pub "${MP_QOS[@]}" --rate "$RATE_HZ" /titan/motor_power std_msgs/msg/Float32MultiArray \
    "{data: [${m0}, ${m1}, ${m2}, ${m3}]}" >/dev/null 2>&1 &
  local pid=$!
  local samp=2
  if (( SECS < 4 )); then samp=1; fi
  sleep "$samp"
  sample_ports "RPM ~${samp}s (cặp bật: cùng dấu nếu đồng bộ chiều)" "${show_ports[@]}"
  local rest=$((SECS - samp))
  if (( rest > 0 )); then
    sleep "$rest"
  fi
  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  stop_all
}

trap 'stop_all || true' EXIT

trim_power() {
  local sign="$1" gain="$2"
  awk -v p="$POWER" -v s="$sign" -v g="$gain" 'BEGIN{
    v = p * s * g;
    if (v > 1.0) v = 1.0;
    if (v < -1.0) v = -1.0;
    printf "%.4f", v;
  }'
}

# Khớp vmx_highlevel: M0,M1 cùng chiều phải; M2,M3 đảo dấu cho phía trái.
m0pos() { trim_power 1 "$GAIN_M0"; }
m1pos() { trim_power 1 "$GAIN_M1"; }
m2neg() { trim_power -1 "$GAIN_M2"; }
m3neg() { trim_power -1 "$GAIN_M3"; }

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  base_power=${POWER}  mỗi pha ${SECS}s  nghỉ ${PAUSE}s"
echo "trim gains: M0=${GAIN_M0} M1=${GAIN_M1} M2=${GAIN_M2} M3=${GAIN_M3}"
stop_all
sleep 0.5

phase_power "Cặp M0+M1 (right_power, calib chiều)" \
  "$(m0pos)" "$(m1pos)" "0.0" "0.0" 0 1
echo ">>> Dừng. Nghỉ ${PAUSE}s..."; sleep "${PAUSE}"

phase_power "Cặp M2+M3 (left_power, calib chiều)" \
  "0.0" "0.0" "$(m2neg)" "$(m3neg)" 2 3
echo ">>> Dừng. Nghỉ ${PAUSE}s..."; sleep "${PAUSE}"

phase_power "Cả 4 thẳng — M2,M3 = −P (đồng bộ hai cặp)" \
  "$(m0pos)" "$(m1pos)" "$(m2neg)" "$(m3neg)" 0 1 2 3

echo ""
echo "Đọc nhanh: mỗi pha hai bánh bật lên nên RPM cùng dấu khi calib xong."
echo "Quan sát bánh lúc quay là chuẩn nhất; RPM lấy mẫu một lần có thể nhiễu."
stop_all
