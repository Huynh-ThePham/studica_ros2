# VMX Direct Low-Level Runbook

Operational runbook for the ROS-free VMX direct UDP daemon.

## Start And Stop

Start:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S systemctl start vmx-udp-lowlevel.service'
```

Stop:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S systemctl stop vmx-udp-lowlevel.service'
```

Restart:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S systemctl restart vmx-udp-lowlevel.service'
```

Status and logs:

```bash
ssh vmx@172.22.11.2 '
  systemctl is-enabled vmx-udp-lowlevel.service
  systemctl is-active vmx-udp-lowlevel.service
  pidof vmx_udp_lowlevel_daemon
  echo password | sudo -S tail -100 /var/log/vmx-udp-lowlevel.log
'
```

## Expected Healthy Log

```text
vmx_udp_lowlevel_daemon starting ...
VMX HAL: pigpio library version 69 opened.
VMX HAL: SPI Aux Channel 2 opened with baudrate of 4000000.
VMX HAL: Established communication ... firmware version 3.0.419
IMU init ok connected=true
Titan Driver Started!
Titan init ok id=42 ... Hardware: Titan Quad
```

## Deploy Updated VMX Code

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
  echo password | sudo -S systemctl restart vmx-udp-lowlevel.service
'
```

## Direct HAL Reference Test

Use this when you need to prove Titan, motor power and encoders work without the
UDP daemon.

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

Current known-good reference:

```text
M0 @0.35, 2s: delta ~= -1985 ticks
```

## UDP Direct Test

```bash
python3 scripts/test_vmx_udp_direct.py \
  --host 172.22.11.2 --ports 0 --speed 0.35 --duration 2.0 --rate 10.0 --verbose
```

Current known-good UDP result:

```text
M0 @0.35, 2s: delta ~= -1858 ticks
```

## PC ROS Test

```bash
cd /home/theph/ws_studica
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vmx_highlevel vmx_highlevel.launch.py
```

In another terminal:

```bash
ros2 topic echo --field data /lowlevel_status --once
ros2 topic echo /imu --once
timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"
ros2 topic echo --field data /highlevel_status --once
```

## Troubleshooting

### HAL Fails With `initMboxBlock`

Symptom:

```text
initMboxBlock: init mbox zaps failed
VMX HAL: Error initializing pigpio library.
```

Actions:

```bash
ssh vmx@172.22.11.2 '
  echo password | sudo -S systemctl stop vmx-udp-lowlevel.service
  echo password | sudo -S rm -f /run/pigpio.pid /var/run/pigpio.pid /dev/pigpio /dev/pigout
'
```

If it persists, reboot VMX. This clears stuck GPU mailbox/pigpio state.

### Motor Moves Weakly

Check that daemon behavior still matches the bench-fixed rule:

- enable Titan once when entering active output,
- resend `SetSpeed`,
- use Titan brake idle mode,
- send zero duty on idle/timeout and keep zero active briefly during stop hold,
- disable Titan only when daemon exits.

Do not reintroduce periodic `Titan::Enable(false)` while the daemon remains alive.

### Time/Clock Skew During Build

VMX can lose wall-clock time after reboot. Runtime uses monotonic time and is not
affected, but builds may warn about future file timestamps. Set time manually or
configure NTP over the PC route.

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S date -s "2026-05-19 11:00:00"'
```
