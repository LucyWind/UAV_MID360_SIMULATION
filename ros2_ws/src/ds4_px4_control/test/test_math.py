import math

import pytest

from ds4_px4_control.ds4_position_control import (
    apply_deadzone,
    body_velocity_to_enu,
    wrap_pi,
    yaw_from_quaternion,
)


def test_deadzone_zeroes_small_values():
    assert apply_deadzone(0.05, 0.08) == 0.0
    assert apply_deadzone(-0.08, 0.08) == 0.0


def test_deadzone_preserves_sign_and_full_scale():
    assert apply_deadzone(1.0, 0.08) == 1.0
    assert apply_deadzone(-1.0, 0.08) == -1.0
    assert apply_deadzone(0.54, 0.08) == pytest.approx(0.5)


def test_wrap_pi():
    assert wrap_pi(0.0) == pytest.approx(0.0)
    assert wrap_pi(math.pi) == pytest.approx(-math.pi)
    assert wrap_pi(3.0 * math.pi) == pytest.approx(-math.pi)


def test_yaw_from_quaternion():
    half_yaw = math.pi / 4.0
    assert yaw_from_quaternion(
        0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)
    ) == pytest.approx(math.pi / 2.0)


def test_simultaneous_forward_and_right_channels():
    east, north = body_velocity_to_enu(2.0, 1.0, math.pi / 2.0)
    assert east == pytest.approx(1.0)
    assert north == pytest.approx(2.0)
