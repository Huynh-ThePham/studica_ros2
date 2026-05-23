#!/usr/bin/env bash
# Dừng mọi ros2 topic pub nền còn sót trên /titan/motor_power (hay gặp khi Ctrl+C / kill không sạch).
# Chạy trước khi test nếu bánh “không chạy” hoặc ros2 topic info báo nhiều publisher lạ.

set -euo pipefail
if pkill -f "ros2 topic pub.*titan/motor_power" 2>/dev/null; then
  echo "Đã gửi SIGTERM tới các ros2 topic pub /titan/motor_power."
else
  echo "Không thấy process khớp (OK)."
fi
sleep 0.5
pgrep -af "ros2 topic pub.*motor_power" || echo "Sạch."
