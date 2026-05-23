# Control Repo Scope

This repository is the deployable control stack for Studica VMX robots.

Included:

- VMX low-level daemon and HAL test utility.
- PC high-level ROS 2 UDP bridge.
- Differential-drive high-level controller.
- Gamepad teleop and command mux.
- Motor, UDP, and integration verification scripts.
- Low-level protocol and deployment documentation.

Excluded:

- VIO/SLAM.
- Camera/LiDAR/GPS drivers.
- Dataset recording and benchmarking.
- Calibration research workflows.
- Nav2 and autonomous navigation stacks.

The excluded pieces should depend on this control stack, not live inside it.
