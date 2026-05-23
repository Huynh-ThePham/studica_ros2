#!/usr/bin/env python3
"""Non-moving readiness checks for a VMX direct UDP deployment.

The doctor is intended for first-time users cloning the repository. It checks
the PC workspace, VMX reachability, systemd service state and UDP telemetry
without commanding non-zero motor output.
"""

from __future__ import annotations

import argparse
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_PYTHON = REPO_ROOT / "src" / "vmx_highlevel"
if str(SRC_PYTHON) not in sys.path:
    sys.path.insert(0, str(SRC_PYTHON))

from vmx_highlevel.udp_protocol import (  # noqa: E402
    STATUS_CHECKSUM_ERROR,
    STATUS_COMMAND_TIMEOUT,
    STATUS_IMU_OK,
    STATUS_MOTOR_ENABLED,
    STATUS_STOPPING,
    STATUS_TITAN_OK,
    STATUS_VMX_OK,
    build_command_packet,
    parse_telemetry_packet,
)


Result = Tuple[str, str, str]


def run_command(command: Sequence[str], timeout_s: float = 5.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        list(command),
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
    )


def add(results: List[Result], level: str, name: str, detail: str) -> None:
    results.append((level, name, detail))
    print(f"[{level}] {name}: {detail}")


def check_local_tools(results: List[Result]) -> None:
    for tool in ("python3", "ssh", "ping"):
        path = shutil.which(tool)
        if path:
            add(results, "PASS", f"tool {tool}", path)
        else:
            add(results, "FAIL", f"tool {tool}", "not found in PATH")

    for tool in ("ros2", "colcon"):
        path = shutil.which(tool)
        if path:
            add(results, "PASS", f"tool {tool}", path)
        else:
            add(results, "WARN", f"tool {tool}", "not found; source ROS 2 setup before PC-side build/run")


def check_repo_layout(results: List[Result]) -> None:
    required = [
        "README.md",
        "LICENSE",
        "docs/vmx-udp-protocol.md",
        "docs/reproducibility-guide.md",
        "src/vmx_lowlevel_driver/src/vmx_udp_lowlevel_daemon.cpp",
        "src/vmx_highlevel/vmx_highlevel/udp_protocol.py",
        "scripts/characterize_vmx_lowlevel.py",
    ]
    for relative in required:
        path = REPO_ROOT / relative
        if path.exists():
            add(results, "PASS", relative, "exists")
        else:
            add(results, "FAIL", relative, "missing")


def check_ping(results: List[Result], host: str) -> None:
    try:
        proc = run_command(["ping", "-c", "1", "-W", "1", host], timeout_s=3.0)
    except subprocess.TimeoutExpired:
        add(results, "FAIL", "ping VMX", f"{host} timed out")
        return
    if proc.returncode == 0:
        add(results, "PASS", "ping VMX", host)
    else:
        add(results, "FAIL", "ping VMX", proc.stdout.strip().splitlines()[-1] if proc.stdout else host)


def check_ssh(results: List[Result], host: str, user: str) -> None:
    target = f"{user}@{host}"
    command = [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=3",
        target,
        "systemctl is-active vmx-udp-lowlevel.service; "
        "pidof vmx_udp_lowlevel_daemon || true; "
        "tail -n 1 /var/log/vmx-udp-lowlevel.log 2>/dev/null || true",
    ]
    try:
        proc = run_command(command, timeout_s=8.0)
    except subprocess.TimeoutExpired:
        add(results, "WARN", "ssh VMX", f"{target} timed out")
        return
    if proc.returncode != 0:
        add(results, "WARN", "ssh VMX", "BatchMode SSH failed; install SSH key or check credentials")
        return
    lines = [line.strip() for line in proc.stdout.splitlines() if line.strip()]
    active = lines[0] if lines else "unknown"
    if active == "active":
        add(results, "PASS", "vmx-udp-lowlevel.service", "active")
    else:
        add(results, "FAIL", "vmx-udp-lowlevel.service", f"state={active}")
    if len(lines) > 1 and lines[1].isdigit():
        add(results, "PASS", "vmx daemon PID", lines[1])
    else:
        add(results, "FAIL", "vmx daemon PID", "not found")


def check_udp_telemetry(results: List[Result], host: str, command_port: int, telemetry_port: int) -> None:
    command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        telemetry_socket.bind(("0.0.0.0", telemetry_port))
        telemetry_socket.settimeout(0.2)
    except OSError as exc:
        add(results, "FAIL", "UDP telemetry bind", str(exc))
        command_socket.close()
        telemetry_socket.close()
        return

    try:
        for sequence in range(1, 8):
            packet = build_command_packet(sequence, [0.0, 0.0, 0.0, 0.0], enable=False)
            command_socket.sendto(packet, (host, command_port))
            time.sleep(0.05)

        latest = None
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            try:
                packet, _ = telemetry_socket.recvfrom(2048)
                latest = parse_telemetry_packet(packet)
            except socket.timeout:
                continue
            except ValueError:
                continue
        if latest is None:
            add(results, "FAIL", "UDP telemetry", "no valid packet received")
            return

        status = latest.status_bits
        add(results, "PASS", "UDP telemetry", f"seq={latest.sequence} last_cmd={latest.last_command_sequence}")
        for name, bit, required in (
            ("vmx_ok", STATUS_VMX_OK, True),
            ("imu_ok", STATUS_IMU_OK, True),
            ("titan_ok", STATUS_TITAN_OK, False),
            ("motor_enabled", STATUS_MOTOR_ENABLED, False),
            ("stopping", STATUS_STOPPING, False),
            ("cmd_timeout", STATUS_COMMAND_TIMEOUT, False),
            ("checksum_error", STATUS_CHECKSUM_ERROR, False),
        ):
            value = bool(status & bit)
            if required and not value:
                add(results, "FAIL", name, "false")
            elif name in ("motor_enabled", "checksum_error") and value:
                add(results, "FAIL", name, "true")
            else:
                add(results, "PASS", name, str(value).lower())
        rpm = [round(value, 1) for value in latest.rpm]
        add(results, "PASS", "idle rpm", str(rpm))
    finally:
        command_socket.close()
        telemetry_socket.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Check VMX low-level readiness without moving motors.")
    parser.add_argument("--host", default="172.22.11.2")
    parser.add_argument("--ssh-user", default="vmx")
    parser.add_argument("--command-port", type=int, default=15000)
    parser.add_argument("--telemetry-port", type=int, default=15001)
    parser.add_argument("--skip-ssh", action="store_true")
    parser.add_argument("--skip-udp", action="store_true")
    args = parser.parse_args()

    results: List[Result] = []
    check_local_tools(results)
    check_repo_layout(results)
    check_ping(results, args.host)
    if not args.skip_ssh:
        check_ssh(results, args.host, args.ssh_user)
    if not args.skip_udp:
        check_udp_telemetry(results, args.host, args.command_port, args.telemetry_port)

    failures = sum(1 for level, _, _ in results if level == "FAIL")
    warnings = sum(1 for level, _, _ in results if level == "WARN")
    print(f"\nsummary: failures={failures} warnings={warnings} checks={len(results)}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
