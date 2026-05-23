# Studica VMX Control Stack

Minimal control repository for a Studica VMX robot.

This repo intentionally contains only two runtime layers:

- `src/vmx_lowlevel_driver`: VMX-side C++ UDP daemon. It talks directly to
  the Studica VMX HAL, Titan Quad CAN, navX IMU, encoders and watchdog logic.
- `src/vmx_highlevel`: PC-side ROS 2 high-level bridge. It converts `/cmd_vel`
  into Titan motor duty commands, publishes IMU/encoder/odom telemetry, and
  optionally runs gamepad teleop/mux.

It does not include VIO, SLAM, datasets, calibration, Nav2, sensor drivers, or
research tooling. Those belong in the larger research workspace.

## Runtime Contract

```text
PC ROS 2
  /cmd_vel
    -> vmx_highlevel_node
    -> /titan/motor_power
    -> vmx_udp_bridge_node
    -> UDP command packets

VMX
  vmx_udp_lowlevel_daemon
    -> Studica VMX HAL C++
    -> Titan Quad CAN + navX IMU + encoders
    -> UDP telemetry packets

PC ROS 2
  /imu
  /titan/motor{0..3}/encoder
  /titan/motor{0..3}/speed
  /odom
  /tf
```

Default network contract:

```text
VMX IP:        172.22.11.2
UDP command:   PC -> VMX :15000
UDP telemetry: VMX -> PC :15001
SSH:           vmx@172.22.11.2
```

Default 4WD mapping:

```text
M0,M1: right side
M2,M3: left side
Forward motor duty: [+,+,-,-]
```

## PC Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vmx_highlevel --symlink-install
source install/setup.bash
```

Run high-level control:

```bash
ros2 launch vmx_highlevel vmx_highlevel.launch.py
```

Run with gamepad teleop:

```bash
ros2 launch vmx_highlevel vmx_highlevel_with_gamepad.launch.py joy_dev:=/dev/input/js0
```

When the gamepad mux is running, autonomous software should publish to
`/cmd_vel_nav`; the mux is the only writer to `/cmd_vel`.

## VMX Deploy

Build/deploy the low-level daemon on the VMX image where the Studica HAL is
installed under `/usr/local`:

```bash
rsync -az --delete src/vmx_lowlevel_driver/ \
  vmx@172.22.11.2:/home/vmx/lowlevel_cpp_ws/src/vmx_lowlevel_driver/

ssh vmx@172.22.11.2 '
  cd /home/vmx/lowlevel_cpp_ws &&
  bash src/vmx_lowlevel_driver/scripts/build_vmx_release.sh
'
```

Install systemd service on VMX:

```bash
ssh vmx@172.22.11.2 '
  cd /home/vmx/lowlevel_cpp_ws &&
  sudo install -Dm644 \
    install/vmx_lowlevel_driver/share/vmx_lowlevel_driver/systemd/vmx-udp-lowlevel.service \
    /etc/systemd/system/vmx-udp-lowlevel.service &&
  sudo systemctl daemon-reload &&
  sudo systemctl enable --now vmx-udp-lowlevel.service
'
```

## Verification

```bash
python3 scripts/vmx_doctor.py --host 172.22.11.2
ros2 topic echo --field data /lowlevel_status --once
ros2 topic echo /imu --once
python3 scripts/test_vmx_udp_direct.py --host 172.22.11.2 --ports 0 --speed 0.20 --duration 1.0
```

For reproducible motor evidence:

```bash
mkdir -p artifacts
python3 scripts/characterize_vmx_lowlevel.py \
  --host 172.22.11.2 \
  --speeds 0.16,0.20,0.24,0.30 \
  --directions both \
  --csv artifacts/vmx-lowlevel-characterization.csv \
  --json artifacts/vmx-lowlevel-characterization.json
```

## Docs

- [UDP protocol](docs/vmx-udp-protocol.md)
- [Architecture](docs/vmx-lowlevel-ros2-architecture.md)
- [Operation guide](docs/vmx-operation-guide.md)
- [LAN setup](docs/vmx-lowlevel-lan.md)
- [Runbook](docs/vmx-lowlevel-runbook.md)
- [Reproducibility guide](docs/reproducibility-guide.md)
- [Hardware config template](docs/hardware-config-template.md)
- [Release checklist](docs/release-checklist.md)

## Safety

Always lift the robot wheels during motor tests. The VMX daemon uses a command
watchdog and sends zero duty on command loss, but physical safety still depends
on safe bench setup and an operator who can remove power.
