# VMX Operation And Connection Guide

Date validated: 2026-05-20  
Target platform: Studica VMX on Raspberry Pi 4, Ubuntu 22.04, VMX firmware `3.0.419`  
Runtime architecture: VMX direct UDP low-level daemon, ROS 2 high-level on PC

## 1. Architecture Summary

The VMX is configured as a lightweight low-level controller:

```text
PC Ubuntu / ROS 2 Humble
  /cmd_vel
    -> vmx_highlevel_node
    -> /titan/motor_power
    -> vmx_udp_bridge_node
    -> UDP command packets

Ethernet LAN
  PC -> VMX UDP/15000
  VMX -> PC UDP/15001

VMX
  vmx_udp_lowlevel_daemon
    -> Studica VMX HAL C++
    -> navX IMU
    -> Titan Quad CAN
    -> motor power + encoder/RPM telemetry
```

The VMX does not run ROS 2 nodes. It only runs one HAL owner process:

```text
vmx_udp_lowlevel_daemon
```

All robot logic stays on the PC:

- `/cmd_vel` handling
- differential-drive mixing
- motor sign/gain calibration
- `/odom`
- `/tf`
- NAV2, SLAM, RViz, planners and controllers

## 2. Network Contract

Default addresses:

```text
VMX Ethernet IP:       172.22.11.2
PC Ethernet IP:        172.22.11.10/24
VMX SSH user:          vmx
VMX SSH password:      password
UDP command port:      15000  (PC -> VMX)
UDP telemetry port:    15001  (VMX -> PC)
```

The Ethernet profile must not define a gateway. Keep Wi-Fi as the PC default
route for Internet access.

Check PC interface:

```bash
ip -br addr show enp0s31f6
ip route get 172.22.11.2
```

Expected route:

```text
172.22.11.2 dev enp0s31f6 src 172.22.11.10
```

## 3. Connect To VMX

Ping:

```bash
ping -c 3 172.22.11.2
```

SSH:

```bash
ssh vmx@172.22.11.2
```

If SSH asks for the host fingerprint, type:

```text
yes
```

## 4. VMX Service Operation

The production service is:

```text
vmx-udp-lowlevel.service
```

Check status:

```bash
ssh vmx@172.22.11.2 '
  systemctl is-enabled vmx-udp-lowlevel.service
  systemctl is-active vmx-udp-lowlevel.service
  pidof vmx_udp_lowlevel_daemon
'
```

Start:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S systemctl start vmx-udp-lowlevel.service'
```

Restart:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S systemctl restart vmx-udp-lowlevel.service'
```

Stop safely:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S systemctl stop vmx-udp-lowlevel.service'
```

The systemd stop path sends zero duty before stopping the daemon.

Follow logs:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S tail -f /var/log/vmx-udp-lowlevel.log'
```

Healthy log example:

```text
VMX HAL:  pigpio library version 69 opened.
VMX HAL:  SPI Aux Channel 2 opened with baudrate of 4000000.
VMX HAL:  Established communication with VMX board ... firmware version 3.0.419
IMU init ok connected=true firmware="3.0"
Titan Driver Started!
Titan init ok id=42 firmware="Firmware Version: [1.0.0]" hardware="Hardware: Titan Quad, Version: 1"
```

## 5. Build And Deploy VMX Low-Level

From PC:

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

If VMX wall-clock is wrong after reboot, builds may print clock-skew warnings.
Runtime is not affected because watchdog timing uses monotonic time.

Optional time fix:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S date -s "2026-05-20 15:30:00"'
```

## 6. Build And Run PC High-Level

On PC:

```bash
cd /home/theph/ws_studica
source /opt/ros/humble/setup.bash
colcon build --packages-select vmx_highlevel --symlink-install
source install/setup.bash
ros2 launch vmx_highlevel vmx_highlevel.launch.py
```

This starts:

```text
vmx_udp_bridge_node       UDP <-> ROS bridge
vmx_highlevel_node        /cmd_vel -> /titan/motor_power, encoder+IMU -> /odom+/tf
static_transform_publisher base_link -> imu_link
```

## 7. ROS Topics On PC

Expected topics:

```text
/cmd_vel
/imu
/lowlevel_status
/highlevel_status
/odom
/tf
/tf_static
/titan/motor_power
/titan/motor0/encoder
/titan/motor1/encoder
/titan/motor2/encoder
/titan/motor3/encoder
/titan/motor0/speed
/titan/motor1/speed
/titan/motor2/speed
/titan/motor3/speed
```

Quick checks:

```bash
ros2 topic list | sort
ros2 topic echo --field data /lowlevel_status --once
ros2 topic echo --field data /highlevel_status --once
ros2 topic echo /imu --once
ros2 topic echo /odom --once
```

Healthy low-level status example:

```text
mode=udp_direct connected=true vmx_ok=true imu_ok=true titan_ok=true motor_enabled=false
```

## 8. Direct UDP Motor Test

Use this before testing ROS `/cmd_vel`. Lift wheels before running.

Single motor:

```bash
python3 scripts/test_vmx_udp_direct.py \
  --host 172.22.11.2 --ports 0 --speed 0.16 --duration 0.7 --rate 10.0
```

All motors one by one:

```bash
for p in 0 1 2 3; do
  echo "=== M${p} ==="
  python3 scripts/test_vmx_udp_direct.py \
    --host 172.22.11.2 --ports "$p" --speed 0.16 --duration 0.7 --rate 10.0
  sleep 0.25
done
```

Known result from 2026-05-20 before brake/zero-hold tuning:

```text
M0 @0.16, 0.7s: delta=-204
M1 @0.16, 0.7s: delta=-187
M2 @0.16, 0.7s: delta=-23
M3 @0.16, 0.7s: delta=-442
```

M2 needs a higher command to overcome low-speed friction/deadband:

```bash
python3 scripts/test_vmx_udp_direct.py \
  --host 172.22.11.2 --ports 2 --speed 0.22 --duration 0.8 --rate 10.0 --verbose
```

Known result from 2026-05-20:

```text
M2 @0.22, 0.8s: delta=-199
```

Post-fix stop validation from 2026-05-20 with Titan brake mode and zero-hold:

```text
M0 @0.16, 0.7s: delta=-163, rpm_after=0.0
M1 @0.16, 0.7s: delta=-183, rpm_after=0.0
M2 @0.16, 0.7s: delta=-47,  rpm_after=0.0
M3 @0.16, 0.7s: delta=-208, rpm_after=0.0
All four @0.16, 0.8s: delta=[-230,-217,-61,-209], rpm_after=[0,0,0,0]
Idle drift after high-level test for 2s: delta=[0,0,0,0]
```

### Reproducible Low-Level Characterization

For lab records, publications, and repeatable maintenance checks, use the
ROS-free characterization script. It drives each Titan port with the same raw
duty command, records encoder/RPM response, and reports weak channels relative
to the median of the other motors.

```bash
python3 scripts/characterize_vmx_lowlevel.py \
  --host 172.22.11.2 \
  --speeds 0.12,0.16,0.20,0.24 \
  --directions both \
  --duration 0.75 \
  --rate 10.0 \
  --csv /tmp/vmx-lowlevel-characterization.csv \
  --json /tmp/vmx-lowlevel-characterization.json
```

Current M2 finding from 2026-05-20:

```text
UDP direct +0.16, 0.5s:
  M0=-146 ticks, M1=-137 ticks, M2=-65 ticks, M3=-136 ticks
  M2 ratio_to_other_median=0.47 -> WEAK

HAL direct +0.16, 0.75s:
  M0 ~= -226 ticks
  M2 ~= -102 ticks, or ~= -132 ticks with --minimal
  M3 ~= -244 ticks
```

Because M2 is weak in both UDP direct and HAL direct, the root cause is not ROS,
UDP, packet timing, or `SetPIDType(0)`. Continue hardware isolation by swapping
the physical M2 motor/encoder cable with a known-good channel. If the weakness
follows the motor/wheel, inspect gearbox friction, encoder mounting, motor, and
wiring. If the weakness remains on Titan M2, inspect Titan M2 output, fuse
contact, current-limit/config app settings, and the M2 connector.

## Titan Update and Configuration App Baseline

Use the Studica Titan Update and Configuration App over the Titan DFU USB port
to make the persistent controller settings explicit before publishing data.

Recommended baseline for this VMX low-level stack:

| Setting | Value |
| --- | --- |
| CAN ID | `42` |
| Firmware | latest version offered by the app; record the exact version |
| Encoder resolution | actual motor encoder CPR for the installed motor; keep M0..M3 identical when motors are identical |
| Current limit | same value on M0..M3; `20 A` when using 20 A motor fuses and motors rated for it |
| Motor idle mode | brake |
| S-curve sensitivity | same value on all channels; minimum/disabled for direct duty characterization |
| Limit switches | disabled unless physical limit switches are installed and verified |
| Automatic bounce back | disabled unless the mechanism needs it |

After changing the CAN ID, power-cycle Titan. The VMX service must use the same
CAN ID through `VMX_TITAN_CAN_ID`.

The daemon also applies runtime safety settings on startup:

```text
VMX_TITAN_STOP_MODE=1
VMX_TITAN_ZERO_HOLD_SEC=1.00
VMX_TITAN_CURRENT_LIMIT_A=20.0
VMX_TITAN_CURRENT_LIMIT_MODE=-1
```

`VMX_TITAN_CURRENT_LIMIT_MODE=-1` intentionally leaves the mode unchanged
because Studica documents the current-limit value but does not define the mode
semantics in the public Titan app page. If you deliberately set a current-limit
mode in the app, document it in the experiment log and set the same value for
all four channels.

## 9. PC ROS Command Test

Run the launch first:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vmx_highlevel vmx_highlevel.launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo --field data /lowlevel_status --once

timeout 2.0 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.08}, angular: {z: 0.0}}"

sleep 0.8
ros2 topic echo --field data /lowlevel_status --once
ros2 topic echo --field data /highlevel_status --once
```

Known result from 2026-05-20 after encoder sign calibration:

```text
forward /cmd_vel 0.12 m/s, 1.0s:
  encoder delta=[-376,-350,255,257]
  highlevel odom_x=0.0812
backward /cmd_vel -0.12 m/s, 1.0s:
  encoder delta=[400,360,-174,-348]
  final lowlevel rpm=[0,0,0,0]
```

This confirms:

```text
/cmd_vel -> vmx_highlevel_node -> /titan/motor_power -> vmx_udp_bridge_node
-> UDP -> VMX daemon -> Titan Quad
```

## 10. Safe Stop After Tests

Stop ROS command output:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

ros2 topic pub --once /titan/motor_power std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0]}"
```

Stop PC launch:

```bash
pkill -INT -f 'ros2 launch vmx_highlevel' || true
pkill -TERM -f 'vmx_udp_bridge_node|vmx_highlevel_node|static_transform_publisher' || true
```

## 11. Gamepad Teleop On PC

Gamepad control belongs on the PC high-level side. The VMX daemon remains
ROS-free and receives only final motor duty commands over UDP.

Start the full high-level stack with joystick teleop:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vmx_highlevel vmx_highlevel_with_gamepad.launch.py joy_dev:=/dev/input/js0
```

If `vmx_highlevel.launch.py` is already running, start only the joystick/mux
layer:

```bash
ros2 launch vmx_highlevel vmx_gamepad_teleop.launch.py joy_dev:=/dev/input/js0
```

Default mapping:

| Control | Function |
| --- | --- |
| `LB` | deadman / enable |
| left stick vertical | linear `x` |
| left stick horizontal | angular `z` |
| `RB` | turbo |

Topic contract:

```text
/joy                 sensor_msgs/msg/Joy
/cmd_vel_joy         geometry_msgs/msg/Twist from teleop_twist_joy
/cmd_vel_nav         geometry_msgs/msg/Twist from NAV2/autonomy
/cmd_vel             mux output consumed by vmx_highlevel_node
/cmd_vel_mux_status  text status for source selection
```

When the mux is running, NAV2 must publish to `/cmd_vel_nav`, not directly to
`/cmd_vel`. Joystick has priority while `LB` is held. Holding `LB` with centered
sticks publishes zero and blocks autonomy. After active joystick motion, the mux
holds priority briefly so the stop command reaches the robot before autonomy can
resume.

Checks:

```bash
ros2 topic echo /joy --once
ros2 topic echo /cmd_vel_joy --once
ros2 topic echo --field data /cmd_vel_mux_status --once
ros2 topic echo --field data /lowlevel_status --once
```

Safety behavior:

- no deadman button: teleop does not command motion,
- deadman held with centered sticks: mux publishes zero and overrides NAV2,
- no joystick or NAV command: mux publishes zero `/cmd_vel`,
- VMX command timeout: daemon sends zero duty and holds zero briefly.

## 12. Deadband Compensation

Deadband compensation belongs on the PC high-level side, not on the VMX daemon.
The default high-level config currently uses:

```yaml
motor_min_power_m0: 0.20
motor_min_power_m1: 0.20
motor_min_power_m2: 0.20
motor_min_power_m3: 0.20
```

This avoids commanding the robot around `|duty| ~= 0.16`, where the latest
Titan characterization showed the wheels are not equally repeatable. Set these
values back to `0.0` only when the upper controller deliberately ramps through
the deadband and you want the smoothest possible low-speed experiment.

Optionally stop VMX service:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S systemctl stop vmx-udp-lowlevel.service'
```

If the service remains active, it should be idle:

```text
motor_enabled=false
rpm=[0.0,0.0,0.0,0.0]
```

## 11. Troubleshooting

### Ping OK But No Telemetry

Check:

```bash
ssh vmx@172.22.11.2 '
  systemctl is-active vmx-udp-lowlevel.service
  pidof vmx_udp_lowlevel_daemon
  echo password | sudo -S tail -80 /var/log/vmx-udp-lowlevel.log
'
```

Also check PC firewall and whether UDP/15001 is already bound by another process.

### VMX HAL Cannot Open

Symptoms:

```text
initMboxBlock: init mbox zaps failed
VMX HAL: Error initializing pigpio library.
```

Fix:

```bash
ssh vmx@172.22.11.2 '
  echo password | sudo -S systemctl stop vmx-udp-lowlevel.service
  echo password | sudo -S rm -f /run/pigpio.pid /var/run/pigpio.pid /dev/pigpio /dev/pigout
'
```

If still failing, reboot VMX:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S reboot'
```

### Motor Weak Or One Channel Different

Run direct UDP motor tests first. If direct UDP is OK but `/cmd_vel` is not, check
PC high-level parameters:

```text
fixed_4wd_layout
motor_command_sign_m0..m3
motor_command_gain_m0..m3
encoder_sign_m0..m3
front_left_port / front_right_port / rear_left_port / rear_right_port
```

On the current hard-mounted 4WD differential robot, keep
`fixed_4wd_layout=true` in `vmx_highlevel.yaml`. That locks the control layout:

```text
right side = M0 + M1
left side  = M2 + M3
forward    = [M0, M1, M2, M3] = [+,+,-,-]
```

With this fixed layout enabled, port map, command sign and encoder sign are not
tuned from YAML. Tune only geometry, per-port gain and deadband unless the robot
is rewired. M2 on the current robot needs more than `0.16` duty for reliable
movement.

### Titan Direct Reference Test

Stop the daemon before direct HAL reference test:

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

## 12. Official References

- Studica ROS2: https://github.com/Studica-Robotics/ROS2
- Studica ROS2 docs: https://docs.dev.studica.com/en/latest/docs/ROS2/
- Studica VMX docs: https://docs.dev.studica.com/en/latest/docs/VMX/
- VMX OS images: https://learn.studica.com/docs/ws/vmx/os-images/
