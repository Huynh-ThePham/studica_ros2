from vmx_highlevel.udp_protocol import (
    COMMAND_SIZE,
    MOTOR_COUNT,
    TELEMETRY_SIZE,
    build_command_packet,
    parse_telemetry_packet,
)


def test_command_packet_size_is_stable():
    packet = build_command_packet(1, [0.0] * MOTOR_COUNT, enable=False)
    assert len(packet) == COMMAND_SIZE == 44


def test_rejects_wrong_motor_count():
    try:
        build_command_packet(1, [0.0, 0.0, 0.0], enable=False)
    except ValueError:
        return
    raise AssertionError("short motor command was accepted")


def test_rejects_short_telemetry_packet():
    try:
        parse_telemetry_packet(bytes(TELEMETRY_SIZE - 1))
    except ValueError:
        return
    raise AssertionError("short telemetry packet was accepted")
