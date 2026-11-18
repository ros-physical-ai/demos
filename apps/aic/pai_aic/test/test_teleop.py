"""Keyboard teleop: key presses become a twist that decays after the hold window."""

import numpy as np

from pai_aic.teleop import FAST, SLOW, TwistCommander

HOLD_S = 0.25


def make():
    """A commander with the default hold window."""
    return TwistCommander(hold_s=HOLD_S)


def test_idle_commander_outputs_a_zero_twist():
    """No key pressed, no motion."""
    np.testing.assert_array_equal(make().twist(now=0.0), np.zeros(6))


def test_a_pressed_key_drives_its_axis_at_the_current_speed():
    """`d` is +X linear at the fast speed the commander starts in."""
    c = make()
    c.press("d", now=1.0)
    np.testing.assert_allclose(c.twist(now=1.0), [FAST.linear, 0, 0, 0, 0, 0])


def test_the_axis_returns_to_zero_after_the_hold_window():
    """Terminal auto-repeat sustains a key; silence releases it."""
    c = make()
    c.press("d", now=1.0)
    np.testing.assert_allclose(c.twist(now=1.0 + HOLD_S - 0.01), [FAST.linear, 0, 0, 0, 0, 0])
    np.testing.assert_array_equal(c.twist(now=1.0 + HOLD_S + 0.01), np.zeros(6))


def test_slow_and_fast_keys_switch_the_speed():
    """`k` selects slow, `l` selects fast."""
    c = make()
    c.press("k", now=0.0)
    c.press("f", now=0.0)
    np.testing.assert_allclose(c.twist(now=0.0), [0, 0, SLOW.linear, 0, 0, 0])
    c.press("l", now=0.1)
    c.press("f", now=0.1)
    np.testing.assert_allclose(c.twist(now=0.1), [0, 0, FAST.linear, 0, 0, 0])


def test_shift_keys_drive_the_angular_axes():
    """Uppercase letters are the shifted bindings: W is +angular X, A is -angular Y; q is -angular Z."""
    c = make()
    c.press("W", now=0.0)
    c.press("A", now=0.0)
    c.press("q", now=0.0)
    np.testing.assert_allclose(c.twist(now=0.0), [0, 0, 0, FAST.angular, -FAST.angular, -FAST.angular])


def test_two_axes_combine():
    """Diagonal motion from two held keys."""
    c = make()
    c.press("d", now=0.0)
    c.press("s", now=0.0)
    np.testing.assert_allclose(c.twist(now=0.0), [FAST.linear, FAST.linear, 0, 0, 0, 0])


def test_unknown_keys_are_ignored():
    """Keys outside the layout do nothing."""
    c = make()
    c.press("x", now=0.0)
    np.testing.assert_array_equal(c.twist(now=0.0), np.zeros(6))
    assert not c.quit_requested


def test_escape_requests_quit():
    """Esc stops the teleop."""
    c = make()
    c.press("\x1b", now=0.0)
    assert c.quit_requested
