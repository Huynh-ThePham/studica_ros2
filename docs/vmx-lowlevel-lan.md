# VMX Direct LAN Setup

This document covers the PC <-> VMX Ethernet link used by the direct UDP
low-level architecture.

## Topology

```text
PC Ubuntu 22.04                         Studica VMX Ubuntu 22.04
Ethernet interface                      Ethernet
172.22.11.10/24  <---- LAN cable ---->  172.22.11.2/24

PC ROS 2                                VMX direct daemon
UDP telemetry :15001                    UDP command :15000
```

The VMX Ethernet address used by Studica images is:

```text
172.22.11.2
```

## PC NetworkManager Profile

Example profile used on this workstation:

```text
connection: vmx-lowlevel-lan
interface:  enp0s31f6
address:    172.22.11.10/24
gateway:    none
never-default: yes
```

Create it once:

```bash
nmcli con add type ethernet ifname enp0s31f6 con-name vmx-lowlevel-lan \
  ipv4.method manual ipv4.addresses 172.22.11.10/24 ipv6.method disabled
nmcli con mod vmx-lowlevel-lan ipv4.never-default yes
nmcli con up vmx-lowlevel-lan
```

Rules:

- Do not configure a gateway on the VMX Ethernet profile.
- Keep Wi-Fi or another interface as the PC default route.
- Keep PC and VMX on `172.22.11.0/24`.

## Connectivity Checks

```bash
ip -br addr show enp0s31f6
ip route get 172.22.11.2
ping -c 3 172.22.11.2
ssh vmx@172.22.11.2
```

VMX Ubuntu credentials:

```text
username: vmx
password: password
```

## UDP Runtime Checks

VMX service:

```bash
ssh vmx@172.22.11.2 '
  systemctl is-active vmx-udp-lowlevel.service
  pidof vmx_udp_lowlevel_daemon
  echo password | sudo -S tail -60 /var/log/vmx-udp-lowlevel.log
'
```

Direct UDP motor/telemetry test from PC:

```bash
python3 scripts/test_vmx_udp_direct.py \
  --host 172.22.11.2 --ports 0 --speed 0.20 --duration 1.0
```

ROS 2 runs only on PC:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vmx_highlevel vmx_highlevel.launch.py
ros2 topic echo --field data /lowlevel_status --once
ros2 topic echo /imu --once
```

## Troubleshooting

If ping works but UDP telemetry does not:

- Check `vmx-udp-lowlevel.service` is active.
- Check no firewall blocks UDP/15000 or UDP/15001 on the PC.
- Ensure no second process owns the VMX HAL.
- Restart the daemon:

```bash
ssh vmx@172.22.11.2 'echo password | sudo -S systemctl restart vmx-udp-lowlevel.service'
```

If VMX HAL cannot open pigpio/SPI:

- Stop all VMX HAL processes.
- Remove stale pigpio FIFO/pid files.
- Reboot VMX if mailbox state remains stuck.

```bash
ssh vmx@172.22.11.2 '
  echo password | sudo -S systemctl stop vmx-udp-lowlevel.service
  echo password | sudo -S rm -f /run/pigpio.pid /var/run/pigpio.pid /dev/pigpio /dev/pigout
'
```

## References

- Architecture: [vmx-lowlevel-ros2-architecture.md](vmx-lowlevel-ros2-architecture.md)
- Protocol: [vmx-udp-protocol.md](vmx-udp-protocol.md)
- Studica VMX docs: https://docs.dev.studica.com/en/latest/docs/VMX/
