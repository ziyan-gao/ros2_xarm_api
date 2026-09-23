"""Test guarded force-sign contact qualification."""

from safe_servo_package.moveit_servo_bridge import MoveItServoBridge
from types import SimpleNamespace


def test_torque_switch_keeps_default_guard_and_disables_both_thresholds():
    node = SimpleNamespace(torque_norm=.6, torque_limit=.5)
    assert MoveItServoBridge._torque_exceeded(node)
    node.torque_protection_enabled = False
    assert not MoveItServoBridge._torque_exceeded(node)
    assert not MoveItServoBridge._torque_exceeded(node, .8)
    node.torque_protection_enabled = True
    node.torque_norm = .45
    assert not MoveItServoBridge._torque_exceeded(node)
    assert MoveItServoBridge._torque_exceeded(node, .8)


def test_sign_reversal_alone_does_not_trigger_touch_contact():
    """Reject a noisy sign crossing below the force magnitude threshold."""
    assert not MoveItServoBridge._touch_contact_reached(
        1.4, 4.0, True, True, True)


def test_observable_sign_mode_requires_magnitude_and_reversal():
    """Require both signals when the baseline sign is reliable."""
    assert not MoveItServoBridge._touch_contact_reached(
        4.1, 4.0, True, True, False)
    assert MoveItServoBridge._touch_contact_reached(
        4.1, 4.0, True, True, True)


def test_magnitude_is_sufficient_when_sign_mode_is_disabled():
    """Retain magnitude-only behavior for ordinary pickup."""
    assert MoveItServoBridge._touch_contact_reached(
        4.1, 4.0, False, True, False)


def test_unobservable_baseline_sign_falls_back_to_magnitude():
    """Avoid deadlock when the initial force lies inside the sign deadband."""
    assert MoveItServoBridge._touch_contact_reached(
        4.1, 4.0, True, False, False)
