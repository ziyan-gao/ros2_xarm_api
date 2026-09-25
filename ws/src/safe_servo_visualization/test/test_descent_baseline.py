from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from safe_servo_visualization.clearance_transfer import ClearanceTransfer


class Harness(ClearanceTransfer, SimpleNamespace):
    TRANSPORT_VALIDATING = 'VALIDATING'


def make_harness():
    h = Harness(direct_moveit_active=True, clearance_phase='descend',
                state='VALIDATING', transport_is_return=False, operation_id=1,
                transport_route_generation=2, transport_trajectory=object(),
                latest_joint_positions=(0.,0.), force_timeout=.5,
                latest_force_z=0., last_force_time=10.,
                transport_contact_baseline=None)
    h.get_logger = lambda: Mock()
    h._transport_execute = Mock()
    h._fault = Mock()
    return h


@pytest.mark.parametrize('force', [0., .01, -3.])
def test_descent_captures_latest_force_without_waiting(monkeypatch, force):
    h = make_harness()
    monkeypatch.setattr('safe_servo_visualization.clearance_transfer.time.monotonic', lambda: 10.)
    h.latest_force_z = force
    assert not h._wait_descent_baseline()
    h._transport_execute.assert_not_called()
    assert h.transport_contact_baseline == pytest.approx(force)
    assert h.transport_descent_time == 0.
    assert not h._wait_descent_baseline()
    assert h.descent_baseline_gate is None


@pytest.mark.parametrize('mode', ['missing', 'nonfinite', 'stale'])
def test_missing_or_stale_force_never_authorizes_descent(monkeypatch, mode):
    h = make_harness()
    monkeypatch.setattr('safe_servo_visualization.clearance_transfer.time.monotonic', lambda: 10.)
    if mode == 'missing':
        h.latest_force_z = None
    elif mode == 'nonfinite':
        h.latest_force_z = float('nan')
    else:
        h.last_force_time = 9.
    assert h._wait_descent_baseline()
    h._transport_execute.assert_not_called()
    h._fault.assert_called_once()
    assert h.transport_contact_baseline is None


@pytest.mark.parametrize('phase', ['direct', 'continuous'])
def test_direct_route_latches_force_baseline_before_first_motion(monkeypatch, phase):
    h = make_harness()
    h.clearance_phase = phase
    h.clearance_has_descent = False
    h.latest_force_z = 12.
    monkeypatch.setattr('safe_servo_visualization.clearance_transfer.time.monotonic', lambda: 10.)
    assert not h._wait_descent_baseline()
    assert h.transport_contact_baseline == 12.
    assert h.transport_descent_time == 0.
    h.latest_force_z = 18.
    assert not h._wait_descent_baseline()
    assert h.transport_contact_baseline == 12.
