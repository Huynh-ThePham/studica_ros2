# VMX Low-Level Direct UDP Architecture

Tài liệu này chốt kiến trúc đang dùng cho robot 4WD differential drive:
VMX chỉ là lớp low-level kiểu MCU mạnh, không chạy ROS 2 runtime. ROS 2, odom,
TF, NAV2, SLAM, RViz và các thuật toán nặng chạy trên PC.

## Kiến Trúc Chuẩn

```text
PC ROS 2 high-level
  /cmd_vel_joy or /cmd_vel_nav
    -> vmx_cmd_vel_mux_node      (optional source arbitration)
  /cmd_vel
    -> vmx_highlevel_node        (mix diff-drive, odom, tf)
    -> /titan/motor_power
    -> vmx_udp_bridge_node       (ROS <-> UDP)
    -> Ethernet UDP

VMX low-level, no ROS
  vmx_udp_lowlevel_daemon
    -> Studica VMX HAL C++
    -> navX IMU
    -> Titan Quad CAN
    -> encoders / motors
```

VMX không chạy:

- `rclcpp`, DDS, ROS graph.
- NAV2, SLAM, planner, controller, behavior tree, RViz.
- `/cmd_vel` source arbitration, mixing, odom integration, TF robot tree.

VMX chỉ làm:

- Nhận duty từng motor qua UDP.
- Gọi Studica C++ HAL trực tiếp để điều khiển Titan Quad.
- Đọc navX IMU, encoder count và RPM từng kênh.
- Gửi telemetry UDP về PC.
- Watchdog: mất command thì gửi zero duty.

## Network

```text
VMX IP:        172.22.11.2
SSH user:      vmx
SSH password:  password
UDP command:   PC -> VMX 172.22.11.2:15000
UDP telemetry: VMX -> PC port 15001
```

PC Ethernet nối VMX không đặt gateway. Wi-Fi giữ default route để PC vẫn có
Internet.

Kiểm tra:

```bash
ping -c 3 172.22.11.2
ssh vmx@172.22.11.2
```

## Package Và File Chính

VMX package: [`src/vmx_lowlevel_driver`](../src/vmx_lowlevel_driver)

- `src/vmx_udp_lowlevel_daemon.cpp`: daemon C++ chạy trên VMX, không phụ thuộc ROS.
- `src/titan_port_test.cpp`: test HAL direct chính hãng để so sánh motor/encoder.
- `scripts/run_vmx_udp_lowlevel.sh`: chạy daemon trong workspace VMX.
- `scripts/launch_vmx_udp_lowlevel.sh`: launch daemon detached, ghi log.
- `scripts/send_vmx_udp_stop.py`: gửi gói zero duty trước khi systemd stop.
- `systemd/vmx-udp-lowlevel.service`: service auto-start duy nhất trên VMX.

PC package: [`src/vmx_highlevel`](../src/vmx_highlevel)

- `vmx_highlevel/diff_drive_highlevel_node.py`: nhận `/cmd_vel`, tính motor power,
  publish `/odom` và `/tf`. Mặc định đang khóa layout 4WD thật:
  `M0,M1` là bên phải, `M2,M3` là bên trái, chạy thẳng là `[+,+,-,-]`.
- `vmx_highlevel/udp_bridge_node.py`: bridge ROS topic sang UDP direct và publish
  telemetry thành ROS topic.
- `vmx_highlevel/cmd_vel_mux_node.py`: chọn nguồn `/cmd_vel_joy` hoặc
  `/cmd_vel_nav`, đọc `/joy` để ưu tiên deadman, publish `/cmd_vel`.
- `config/vmx_highlevel.yaml`: geometry, sign/gain, odom config.
- `config/vmx_udp_bridge.yaml`: IP/port UDP.
- `config/vmx_teleop_joy.yaml`: mapping tay cầm.
- `config/vmx_cmd_vel_mux.yaml`: timeout/priority cho command mux.
- `launch/vmx_highlevel.launch.py`: chạy cả high-level node và UDP bridge.
- `launch/vmx_highlevel_with_gamepad.launch.py`: chạy high-level kèm tay cầm.

## Topic ROS Trên PC

VMX không publish ROS trực tiếp. Các topic sau do `vmx_udp_bridge_node` và
`vmx_highlevel_node` trên PC tạo ra:

```text
/imu                         sensor_msgs/msg/Imu
/titan/motor0/encoder        std_msgs/msg/Int32
/titan/motor1/encoder        std_msgs/msg/Int32
/titan/motor2/encoder        std_msgs/msg/Int32
/titan/motor3/encoder        std_msgs/msg/Int32
/titan/motor0/speed          std_msgs/msg/Float32
/titan/motor1/speed          std_msgs/msg/Float32
/titan/motor2/speed          std_msgs/msg/Float32
/titan/motor3/speed          std_msgs/msg/Float32
/lowlevel_status             std_msgs/msg/String
/titan/motor_power           std_msgs/msg/Float32MultiArray
/odom                        nav_msgs/msg/Odometry
/tf                          tf2_msgs/msg/TFMessage
/highlevel_status            std_msgs/msg/String
/cmd_vel_mux_status          std_msgs/msg/String
/joy                         sensor_msgs/msg/Joy        (khi dùng gamepad)
```

NAV2/teleop chỉ cần publish:

```text
/cmd_vel_nav                 geometry_msgs/msg/Twist  (NAV2/autonomy khi dùng mux)
/cmd_vel_joy                 geometry_msgs/msg/Twist  (gamepad teleop)
```

Nếu không dùng mux, một nguồn điều khiển duy nhất có thể publish trực tiếp
`/cmd_vel`. Khi dùng gamepad hoặc NAV2 song song, chỉ mux được publish `/cmd_vel`.
Giữ `LB` trên tay cầm sẽ chặn `/cmd_vel_nav`; nếu stick ở giữa, mux publish
zero để dừng robot ở lớp PC high-level.

## Build Và Deploy VMX

Từ PC:

```bash
cd /home/theph/ws_studica
rsync -az --delete src/vmx_lowlevel_driver/ \
  vmx@172.22.11.2:/home/vmx/lowlevel_cpp_ws/src/vmx_lowlevel_driver/

ssh vmx@172.22.11.2 '
  cd /home/vmx/lowlevel_cpp_ws &&
  bash src/vmx_lowlevel_driver/scripts/build_vmx_release.sh &&
  echo password | sudo -S install -Dm644 \
    install/vmx_lowlevel_driver/share/vmx_lowlevel_driver/systemd/vmx-udp-lowlevel.service \
    /etc/systemd/system/vmx-udp-lowlevel.service &&
  echo password | sudo -S systemctl daemon-reload &&
  echo password | sudo -S systemctl enable --now vmx-udp-lowlevel.service
'
```

Kiểm tra service:

```bash
ssh vmx@172.22.11.2 '
  systemctl is-enabled vmx-udp-lowlevel.service
  systemctl is-active vmx-udp-lowlevel.service
  pidof vmx_udp_lowlevel_daemon
  echo password | sudo -S tail -80 /var/log/vmx-udp-lowlevel.log
'
```

Log tốt phải có:

```text
VMX HAL: pigpio library version 69 opened.
VMX HAL: SPI Aux Channel 2 opened with baudrate of 4000000.
VMX HAL: Established communication ... firmware version 3.0.419
IMU init ok connected=true
Titan init ok id=42 ... Hardware: Titan Quad
```

Trên Raspberry Pi 4 không dùng cpuinfo shim giả Pi 3. Shim đó làm pigpio chọn sai
peripheral base và có thể treo ở SPI Aux Channel 2.

## Build Và Chạy PC High-Level

```bash
cd /home/theph/ws_studica
source /opt/ros/humble/setup.bash
colcon build --packages-select vmx_highlevel --symlink-install
source install/setup.bash

ros2 launch vmx_highlevel vmx_highlevel.launch.py
```

Kiểm tra topic:

```bash
ros2 topic list | sort
ros2 topic echo --field data /lowlevel_status --once
ros2 topic echo /imu --once
ros2 topic echo /odom --once
```

## Test Direct UDP

Test một motor không cần ROS:

```bash
cd /home/theph/ws_studica
python3 scripts/test_vmx_udp_direct.py \
  --host 172.22.11.2 --ports 0 --speed 0.35 --duration 2.0 --rate 10.0 --verbose
```

Kết quả sau fix ngày 2026-05-19:

```text
M0 @0.35, 2s: delta ~= -1858 ticks, rpm ~= 35-40
```

So sánh HAL direct trên VMX:

```bash
ssh vmx@172.22.11.2 '
  echo password | sudo -S systemctl stop vmx-udp-lowlevel.service
  cd /home/vmx/lowlevel_cpp_ws
  echo password | sudo -S env LD_LIBRARY_PATH=/usr/local/lib:/usr/local/lib/vmxpi:/usr/local/lib/studica_drivers \
    install/vmx_lowlevel_driver/lib/vmx_lowlevel_driver/titan_port_test \
    --ports 0 --speed 0.35 --duration 2.0 --minimal
  echo password | sudo -S systemctl start vmx-udp-lowlevel.service
'
```

Kết quả chuẩn tham chiếu:

```text
M0 @0.35, 2s: delta ~= -1985 ticks
```

Test từng kênh sau fix:

```text
M0 @0.22, 1s: delta ~= -544 ticks
M1 @0.22, 1s: delta ~= -446 ticks
M2 @0.22, 1s: delta ~= -371 ticks
M3 @0.22, 1s: delta ~= -468 ticks
```

## Test Qua ROS PC

Khi `ros2 launch vmx_highlevel vmx_highlevel.launch.py` đang chạy:

```bash
ros2 topic echo --field data /lowlevel_status --once

timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"

ros2 topic echo --field data /lowlevel_status --once
ros2 topic echo --field data /highlevel_status --once
```

Kết quả test cập nhật ngày 2026-05-20:

```text
forward /cmd_vel 0.12 m/s, 1.0s:
  encoder delta=[-376,-350,255,257]
  highlevel odom_x=0.0812
backward /cmd_vel -0.12 m/s, 1.0s:
  encoder delta=[400,360,-174,-348]
  final lowlevel rpm=[0,0,0,0]
```

Điều này xác nhận đường PC `/cmd_vel` -> `/titan/motor_power` -> UDP -> VMX
-> Titan hoạt động.

## Lưu Ý Quan Trọng Về Titan Enable/Disable

Điểm đã fix:

- Không gọi `Titan::Enable(true)` ở mọi control tick. Chỉ enable một lần khi
  chuyển từ idle sang running, sau đó lặp `SetSpeed`.
- Không gọi `Titan::Enable(false)` khi daemon vẫn tiếp tục chạy. Driver Studica
  gửi `DISABLED_FLAG` dạng periodic 10 ms; nếu để process sống, frame disabled
  có thể cạnh tranh với enabled và làm motor chỉ nhích rất yếu.
- Titan được cấu hình stop mode `1` (brake). Theo tài liệu Studica, mode `0` là
  coast và mode `1` là brake.
- Khi timeout hoặc nhận lệnh zero: daemon gửi `SetSpeed(0)` cho các kênh active,
  sau đó giữ zero thêm `VMX_TITAN_ZERO_HOLD_SEC` để bánh dừng dứt khoát. Khi
  process thật sự thoát, daemon mới disable Titan.

Quy tắc này làm UDP direct giống bài test HAL mạnh của hãng: enable một lần,
resend `SetSpeed` theo chu kỳ 10 Hz.

## Tham Số Cần Calibrate Trên PC

Trong [`src/vmx_highlevel/config/vmx_highlevel.yaml`](../src/vmx_highlevel/config/vmx_highlevel.yaml):

- `fixed_4wd_layout`: để `true` cho robot hiện tại. Khi bật, port map,
  `motor_command_sign` và `encoder_sign` bị khóa theo robot 4WD đã lắp cứng:
  phải `M0,M1`, trái `M2,M3`.
- `wheel_radius`
- `wheelbase`
- `ticks_per_rotation`
- `front_left_port`, `front_right_port`, `rear_left_port`, `rear_right_port`
  chỉ dùng khi `fixed_4wd_layout=false`
- `motor_command_sign_m0..m3` chỉ dùng khi `fixed_4wd_layout=false`
- `motor_command_gain_m0..m3`
- `motor_min_power_m0..m3`
- `encoder_sign_m0..m3` chỉ dùng khi `fixed_4wd_layout=false`
- `max_linear_velocity`
- `max_angular_velocity`
- `max_motor_power`

VMX không bù sign/gain và không xử lý động học. Mọi calibration robot nằm trên PC.

## Vấn Đề Thời Gian VMX

VMX không có RTC ổn định khi chỉ nối LAN trực tiếp. Sau reboot, thời gian có thể
quay về ngày cũ, gây warning kiểu:

```text
Clock skew detected. Your build may be incomplete.
```

Nếu build trên VMX sau khi reboot, nên set thời gian hoặc cấu hình NTP qua PC:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S date -s "2026-05-19 11:00:00"'
```

Runtime UDP dùng monotonic clock nên motor watchdog không phụ thuộc wall-clock.

## Tài Liệu Chính Hãng

- Studica ROS2: https://github.com/Studica-Robotics/ROS2
- Studica ROS2 docs: https://docs.dev.studica.com/en/latest/docs/ROS2/
- VMX docs: https://docs.dev.studica.com/en/latest/docs/VMX/
- VMX OS images: https://learn.studica.com/docs/ws/vmx/os-images/
