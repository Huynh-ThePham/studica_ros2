# Reproducibility Guide

This guide defines the minimum steps required for another lab to clone the
repository, deploy the VMX low-level daemon, run the PC ROS 2 bridge, and record
repeatable validation evidence.

## Supported Baseline

| Item | Baseline |
| --- | --- |
| PC OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| VMX OS | Ubuntu 22.04 on Raspberry Pi 4 |
| VMX firmware | record from daemon log, validated with `3.0.419` |
| Titan CAN ID | `42` by default |
| VMX IP | `172.22.11.2` |
| UDP command | PC -> VMX `15000` |
| UDP telemetry | VMX -> PC `15001` |

Official vendor documentation:

- Studica ROS 2: https://docs.dev.studica.com/en/latest/docs/ROS2/
- Studica VMX: https://docs.dev.studica.com/en/latest/docs/VMX/
- Studica Titan: https://docs.dev.studica.com/en/latest/docs/Titan/
- Titan Update and Configuration App:
  https://docs.dev.studica.com/en/latest/docs/Titan/update-and-config.html

## Clone And Build On PC

```bash
mkdir -p ~/ws_studica/src
cd ~/ws_studica
git clone <REPOSITORY_URL> .

source /opt/ros/humble/setup.bash
colcon build --packages-select vmx_highlevel --symlink-install
source install/setup.bash
```

The repository intentionally does not commit `build/`, `install/`, `log/` or
runtime artifacts.

## Record Hardware Configuration

Before running motor tests, copy the template and fill it in:

```bash
cp docs/hardware-config-template.md docs/hardware-config.local.md
```

Record at least:

- VMX firmware and Raspberry Pi model,
- Titan firmware and CAN ID,
- Titan current limit, idle mode, S-curve and limit-switch settings,
- motor and encoder model/CPR,
- wheel radius and wheelbase,
- motor port mapping M0..M3,
- calibration parameters in `src/vmx_highlevel/config/vmx_highlevel.yaml`.

For the current assembled robot, record `fixed_4wd_layout=true`: right side
`M0,M1`, left side `M2,M3`, forward duty `[+,+,-,-]`.

Do not compare motor characterization data between robots unless these fields
are known.

## Deploy VMX Low-Level

```bash
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

Healthy VMX log markers:

```text
VMX HAL:  SPI Aux Channel 2 opened with baudrate of 4000000.
VMX HAL:  Established communication with VMX board ...
IMU init ok connected=true
Titan Driver Started!
Titan init ok id=42 ...
```

## Tier 0: Non-Moving Doctor

Run this first. It sends zero-duty packets only.

```bash
python3 scripts/vmx_doctor.py --host 172.22.11.2
```

Pass criteria:

- `vmx_ok=true`
- `imu_ok=true`
- `motor_enabled=false`
- `checksum_error=false`
- idle RPM is `[0.0, 0.0, 0.0, 0.0]`

`titan_ok=false` is acceptable only for IMU-only bring-up without a Titan Quad.

## Tier 1: ROS Topic Contract

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vmx_highlevel vmx_highlevel.launch.py
```

In another terminal:

```bash
ros2 topic list
ros2 topic echo --field data /lowlevel_status --once
ros2 topic echo /imu --once
ros2 topic echo /odom --once
```

Required topics:

```text
/cmd_vel
/imu
/odom
/tf
/lowlevel_status
/highlevel_status
/titan/motor0/encoder ... /titan/motor3/encoder
/titan/motor0/speed   ... /titan/motor3/speed
```

Optional gamepad teleop topics:

```text
/joy
/cmd_vel_joy
/cmd_vel_nav
/cmd_vel_mux_status
```

## Tier 2: Motor Characterization

Lift the robot wheels before running this tier.

```bash
mkdir -p artifacts
python3 scripts/characterize_vmx_lowlevel.py \
  --host 172.22.11.2 \
  --speeds 0.16,0.20,0.24,0.30 \
  --directions both \
  --duration 0.75 \
  --rate 10.0 \
  --csv artifacts/vmx-lowlevel-characterization.csv \
  --json artifacts/vmx-lowlevel-characterization.json
```

Pass criteria for the current robot after Titan app configuration:

- all ports pass at `|duty| >= 0.20`,
- `|duty| ~= 0.16` may expose motor deadband and should not be used as the
  primary operating point for equal-wheel experiments,
- all `after_rpm` values are `0.0`,
- `after_motor_enabled=false`.

## Tier 3: PC High-Level `/cmd_vel`

With `ros2 launch vmx_highlevel vmx_highlevel.launch.py` running:

```bash
python3 scripts/test_highlevel_cmd_vel_delta.py \
  --linear 0.12 \
  --duration 1.0 \
  --warmup 0.4 \
  --rate 30.0 \
  --mode straight
```

Pass criteria:

- `/cmd_vel` has a subscriber,
- encoder deltas have the expected signs for forward/backward motion,
- `/highlevel_status` reports `encoders_ready=true` and `imu_ready=true`,
- `/lowlevel_status` returns to `motor_enabled=false` and RPM zero after stop.

## Tier 4: Gamepad Teleop

The gamepad stack is PC-side only and uses standard ROS 2 packages:

```bash
ros2 launch vmx_highlevel vmx_highlevel_with_gamepad.launch.py joy_dev:=/dev/input/js0
```

Default controls:

- `LB`: deadman / enable,
- left stick vertical: linear velocity,
- left stick horizontal: yaw,
- `RB`: turbo.

Pass criteria:

- `/joy` publishes when the controller is moved,
- `/cmd_vel_joy` publishes only while the deadman is held,
- `/cmd_vel_mux_status` reports `source=joy` while `LB` is held,
- holding `LB` with centered sticks publishes zero and overrides `/cmd_vel_nav`,
- `/cmd_vel_mux_status` returns to `source=idle` or `source=nav` after release,
- `/lowlevel_status` returns to `motor_enabled=false` and RPM zero after stop.

When using NAV2 with the mux, remap NAV2 command output to `/cmd_vel_nav`.
Only the mux should write `/cmd_vel`.

## Artifact Set For Publications

For a reproducible experiment, archive:

- commit hash or release tag,
- `docs/hardware-config.local.md`,
- VMX service environment from
  `systemctl show vmx-udp-lowlevel.service -p Environment`,
- `/var/log/vmx-udp-lowlevel.log`,
- characterization CSV/JSON,
- ROS bag or topic captures used by the paper,
- final calibration YAML.

## Safety Boundary

This repository provides low-level communication and repeatability tools. It
does not replace a physical emergency stop, risk assessment, enclosure, or lab
safety procedure.
