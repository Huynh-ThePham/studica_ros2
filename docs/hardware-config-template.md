# Hardware Configuration Template

Copy this file to `docs/hardware-config.local.md` and fill it in for each robot
or experiment. Keep the completed copy with the experiment artifacts.

## Experiment

| Field | Value |
| --- | --- |
| Date | |
| Operator | |
| Robot name / revision | |
| Repository commit / tag | |
| Notes | |

## PC

| Field | Value |
| --- | --- |
| OS | Ubuntu 22.04 |
| ROS 2 distro | Humble |
| Network interface | |
| PC Ethernet IP | |
| ROS_DOMAIN_ID | |

## VMX

| Field | Value |
| --- | --- |
| Raspberry Pi model | |
| VMX OS image | |
| VMX IP | `172.22.11.2` |
| VMX firmware | |
| VMX service active | yes / no |
| `VMX_TITAN_CAN_ID` | `42` |
| `VMX_TITAN_MOTOR_FREQ` | `15600` |
| `VMX_TITAN_STOP_MODE` | `1` brake |
| `VMX_TITAN_CURRENT_LIMIT_A` | `20.0` |

## Titan Quad

Record these from the Titan Update and Configuration App.

| Field | M0 | M1 | M2 | M3 |
| --- | --- | --- | --- | --- |
| Motor connected | | | | |
| Encoder connected | | | | |
| Encoder CPR | | | | |
| Current limit (A) | | | | |
| Current-limit mode | | | | |
| Idle mode | brake / coast | brake / coast | brake / coast | brake / coast |
| S-curve sensitivity | | | | |
| Limit switches enabled | | | | |
| Auto bounce back | | | | |
| Fuse rating / condition | | | | |

| Titan field | Value |
| --- | --- |
| CAN ID | |
| Firmware version | |
| Hardware version | |
| Status LED state when enabled | |
| 12 V input measured | |

## Robot Geometry

| Field | Value |
| --- | --- |
| Drive type | 4WD differential |
| Wheel radius (m) | |
| Wheelbase (m) | |
| Track width / effective wheelbase note | |
| Ticks per rotation used in YAML | |

## Motor Mapping

| Robot position | Titan port | Command sign | Encoder sign | Gain | Min power |
| --- | --- | --- | --- | --- | --- |
| Front left | | | | | |
| Rear left | | | | | |
| Front right | | | | | |
| Rear right | | | | | |

## Validation Artifacts

| Artifact | Path |
| --- | --- |
| VMX daemon log | |
| Doctor output | |
| Characterization CSV | |
| Characterization JSON | |
| ROS topic / bag capture | |
| Final calibration YAML | |

## Observations

Record observed asymmetry, mechanical changes, motor swaps and any excluded
trials here.
