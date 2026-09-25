import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from std_srvs.srv import Trigger
from safe_servo_visualization.node_restart import ApplicationRestart


class Monitor(SimpleNamespace):
    blocked = ApplicationRestart.blocked
    restart = ApplicationRestart.restart
    tick = ApplicationRestart.tick
    spawn = ApplicationRestart.spawn
    close = ApplicationRestart.close


def monitor():
    now = time.monotonic()
    return Monitor(identity='pickup_supervisor', stopping=False,
        session_time=now, session={'state': 'FAULT', 'downstream': {'scene': None, 'supervisor': 'FAULT'}},
        joints_time=now, stationary_since=now-2, process=None, message='', publisher=Mock(),
        get_logger=lambda: Mock(), command=[sys.executable, '-c', 'import time; time.sleep(60)'])


def test_graceful_restart_replaces_only_own_exited_child():
    h = monitor()
    h.spawn()
    original = h.process
    try:
        response = h.restart(None, Trigger.Response())
        assert response.success
        original.wait(timeout=3)
        h.tick()
        assert h.process.pid != original.pid
        assert not h.stopping and h.process.poll() is None
    finally:
        h.close()


def test_timeout_keeps_old_process_and_does_not_spawn_another():
    h = monitor()
    h.process = Mock(pid=12345)
    h.process.poll.return_value = None
    h.spawn = Mock()
    h.stopping, h.deadline = True, time.monotonic()-1
    h.tick()
    h.spawn.assert_not_called()
    h.process.kill.assert_not_called()
    assert 'timed out' in h.message


@pytest.mark.parametrize('change', [
    {'session_time': 0.}, {'joints_time': 0.}, {'stationary_since': None},
    {'session': {'state': 'RUNNING'}}, {'session': {'state': 'FAULT', 'random_active': True}},
    {'session': {'state': 'FAULT', 'automatic_motion_active': True}},
    {'session': {'state': 'FAULT', 'downstream': {'place': 'DESCENDING'}}},
])
def test_restart_rejects_active_or_unverified_motion(change):
    h = monitor()
    h.__dict__.update(change)
    assert not h.restart(None, Trigger.Response()).success


def test_crashed_target_can_restart_with_stale_active_state():
    h = monitor()
    h.session['downstream']['supervisor'] = 'TRANSPORT_EXECUTING'
    assert not h.blocked()


def test_hardware_not_in_application_restart_scope():
    with pytest.raises(ValueError, match='not allowlisted'):
        ApplicationRestart('ros2_control_node', [])
