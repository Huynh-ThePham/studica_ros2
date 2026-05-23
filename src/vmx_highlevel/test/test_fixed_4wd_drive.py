import pytest

from vmx_highlevel.diff_drive_highlevel_node import (
    FIXED_4WD_MOTOR_COMMAND_SIGN,
    FIXED_4WD_WHEEL_PORTS,
    compute_4wd_motor_power,
)


def motor_power(linear: float, angular: float):
    return compute_4wd_motor_power(
        linear_velocity=linear,
        angular_velocity=angular,
        wheelbase=0.28,
        max_wheel_linear_velocity=0.70,
        max_motor_power=0.70,
        wheel_ports=FIXED_4WD_WHEEL_PORTS,
        motor_command_sign=FIXED_4WD_MOTOR_COMMAND_SIGN,
        motor_command_gain=[1.0, 1.0, 1.0, 1.0],
        motor_min_power=[0.0, 0.0, 0.0, 0.0],
    )


def test_fixed_4wd_forward_mapping_is_stable():
    assert motor_power(0.14, 0.0) == pytest.approx([0.20, 0.20, -0.20, -0.20])


def test_fixed_4wd_backward_mapping_is_stable():
    assert motor_power(-0.14, 0.0) == pytest.approx([-0.20, -0.20, 0.20, 0.20])


def test_fixed_4wd_positive_yaw_mapping_is_stable():
    assert motor_power(0.0, 1.0) == pytest.approx([0.20, 0.20, 0.20, 0.20])


def test_fixed_4wd_negative_yaw_mapping_is_stable():
    assert motor_power(0.0, -1.0) == pytest.approx([-0.20, -0.20, -0.20, -0.20])
