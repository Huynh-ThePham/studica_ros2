# Contributing

This project separates hard real-time-adjacent motor I/O from high-level ROS 2
behavior. Contributions should preserve that boundary.

## Design Rules

- VMX-side code must stay ROS-free.
- VMX-side code may call Studica HAL directly and should remain lightweight.
- PC-side code owns ROS 2 topics, odometry, TF, kinematics, trims and calibration.
- UDP protocol changes require a version bump and documentation update.
- Motor safety behavior must be tested on raised wheels before field use.

## Quality Gates

Run before submitting changes:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vmx_highlevel --symlink-install
python3 -m py_compile \
  src/vmx_highlevel/vmx_highlevel/udp_protocol.py \
  src/vmx_highlevel/vmx_highlevel/udp_bridge_node.py \
  scripts/test_vmx_udp_direct.py \
  scripts/characterize_vmx_lowlevel.py \
  scripts/vmx_doctor.py
colcon test --packages-select vmx_highlevel --event-handlers console_direct+
```

On VMX:

```bash
cd /home/vmx/lowlevel_cpp_ws
bash src/vmx_lowlevel_driver/scripts/build_vmx_release.sh
systemctl is-active vmx-udp-lowlevel.service
```

Before moving motors, run the non-moving doctor:

```bash
python3 scripts/vmx_doctor.py --host 172.22.11.2
```

## Documentation

Any behavior change must update the relevant document:

- `docs/vmx-udp-protocol.md` for packet changes.
- `docs/vmx-lowlevel-ros2-architecture.md` for architecture or runtime changes.
- `docs/reproducibility-guide.md` for clone/build/validation workflow changes.
- `docs/release-checklist.md` for validation changes.
