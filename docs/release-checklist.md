# Release Checklist

Use this checklist before publishing the repository for a lab, community or
commercial release.

## Required Hardware Validation

- VMX boots on the intended Raspberry Pi model.
- Pi 4/Pi 5 does not use any Pi 3 cpuinfo shim.
- `/dev/spidev1.2` exists and VMX HAL opens SPI Aux Channel 2 at 4 MHz.
- Titan Quad receives 12 V motor power.
- Motor fuse rating and status LEDs are checked before software tests.
- Titan CAN ID is `42` or config is updated consistently.
- Titan app settings are recorded: firmware, encoder CPR, current limit, idle
  mode, S-curve sensitivity and limit-switch state.
- `titan_port_test` can run at least one motor using direct HAL.
- `scripts/test_vmx_udp_direct.py` produces encoder deltas close to direct HAL.
- `scripts/characterize_vmx_lowlevel.py` has been run and saved as CSV/JSON for
  motor-to-motor repeatability evidence.
- `scripts/vmx_doctor.py` passes before any moving motor test.
- `docs/hardware-config.local.md` or equivalent artifact is filled in for the
  robot under test.

## Software Validation

On PC:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vmx_highlevel --symlink-install
python3 -m py_compile \
  src/vmx_highlevel/vmx_highlevel/udp_protocol.py \
  src/vmx_highlevel/vmx_highlevel/udp_bridge_node.py \
  scripts/test_vmx_udp_direct.py \
  scripts/characterize_vmx_lowlevel.py \
  scripts/vmx_doctor.py
```

On VMX:

```bash
cd /home/vmx/lowlevel_cpp_ws
bash src/vmx_lowlevel_driver/scripts/build_vmx_release.sh
systemctl is-active vmx-udp-lowlevel.service
tail -80 /var/log/vmx-udp-lowlevel.log
```

Runtime ROS checks:

```bash
ros2 launch vmx_highlevel vmx_highlevel.launch.py
ros2 topic echo --field data /lowlevel_status --once
ros2 topic echo /imu --once
ros2 topic echo /odom --once
```

Optional gamepad checks:

```bash
ros2 launch vmx_highlevel vmx_gamepad_teleop.launch.py joy_dev:=/dev/input/js0
ros2 topic echo /joy --once
ros2 topic echo --field data /cmd_vel_mux_status --once
```

Non-moving readiness check:

```bash
python3 scripts/vmx_doctor.py --host 172.22.11.2
```

## Public Repository Hygiene

- Root `README.md` explains architecture, quick start, safety and docs.
- `docs/reproducibility-guide.md` explains clone-to-validation workflow.
- `docs/hardware-config-template.md` exists for experiment metadata.
- `LICENSE` exists and matches package metadata.
- `.gitignore` excludes `build/`, `install/`, `log/` and Python caches.
- No generated `__pycache__`, build artifacts or logs are committed.
- Package versions are updated consistently.
- Maintainer names/emails are replaced with the official project owner before
  external publication.
- Protocol docs and code constants are updated together.

## Safety Release Notes

Document the following in release notes:

- Wheels must be lifted during motor tests.
- VMX daemon uses Titan brake idle mode and sends repeated zero duty on command
  timeout.
- Gamepad control runs on the PC side through a deadman button and `/cmd_vel`
  mux; holding the deadman with centered sticks publishes zero and overrides
  autonomy; VMX never reads joystick hardware directly.
- Titan is disabled on daemon exit, not on every idle transition, to avoid the
  Studica periodic disable-frame conflict observed during bench testing.
- This package provides low-level control infrastructure only; system-level
  safety, emergency stop hardware and risk assessment remain the integrator's
  responsibility.
