"""Retiming never blocks ROS callbacks or revives obsolete operations."""
from concurrent.futures import Future as Task
from threading import Event

import pytest

from safe_servo_visualization.continuous_transport import ContinuousTransport, retime_snapshot
from test_continuous_transport import planned_harness, Future


class DeferredPool:
    def submit(self, fn, *args):
        self.fn, self.args = fn, args
        self.task = Task()
        self.task.set_running_or_notify_cancel()
        return self.task


def pending_plan():
    h, response = planned_harness()
    pool = h._retime_pool = DeferredPool()
    ContinuousTransport._transport_planned(h, Future(response))
    return h, pool, response


def test_callback_returns_before_work_and_uses_snapshot():
    h, pool, response = pending_plan()
    assert not pool.task.done()
    assert h.state == h.TRANSPORT_PLANNING
    assert pool.args[0] is not response.solution.joint_trajectory
    assert pool.args[1] is not h.transport_joint_limits
    h._poll_transport_timing()
    assert h.state == h.TRANSPORT_PLANNING
    pool.task.set_result(pool.fn(*pool.args))
    assert h.state == h.TRANSPORT_PLANNING  # Worker cannot advance ROS state.
    h._poll_transport_timing()
    assert h.state == h.TRANSPORT_VALIDATING
    assert not h.fault


@pytest.mark.parametrize('change', ['abort', 'operation', 'route'])
def test_obsolete_result_never_validated(change):
    h, pool, _ = pending_plan()
    if change == 'abort':
        h.state = h.FAULT
    elif change == 'operation':
        h.operation_id = 'next'
    else:
        h.transport_route_generation = 1
    h._poll_transport_timing()
    # Even if state later matches, an observed cancellation stays canceled.
    h.state = h.TRANSPORT_PLANNING
    h.operation_id = 'test-operation'
    h.transport_route_generation = 0
    pool.task.set_result(pool.fn(*pool.args))
    h._poll_transport_timing()
    assert h.state == h.TRANSPORT_PLANNING
    assert h._retime_pending is None
    assert not hasattr(h, 'transport_trajectory')


def test_worker_failure_reported_on_main_thread():
    h, pool, _ = pending_plan()
    pool.task.set_exception(ValueError('bad timing'))
    assert not h.fault
    h._poll_transport_timing()
    assert 'bad timing' in h.fault


def test_speed_change_requires_replan():
    h, pool, _ = pending_plan()
    pool.task.set_result(pool.fn(*pool.args))
    h.motion_speed_percent = 80.
    h._poll_transport_timing()
    assert 'speed changed' in h.fault


def test_real_worker_leaves_callback_available(monkeypatch):
    entered, release = Event(), Event()

    def blocked(*args):
        entered.set()
        assert release.wait(5)
        return retime_snapshot(*args)

    monkeypatch.setattr('safe_servo_visualization.continuous_transport.retime_snapshot', blocked)
    h, response = planned_harness()
    try:
        ContinuousTransport._transport_planned(h, Future(response))
        assert entered.wait(2)
        assert not h._retime_pending[0].done()
        # The ROS thread can poll, publish, and process abort while work runs.
        h._poll_transport_timing()
        h.publish_status()
        h._fault('abort')
        h._poll_transport_timing()
        release.set()
        h._retime_pending[0].result(timeout=5)
        h._poll_transport_timing()
        assert h.state == h.FAULT
        assert not hasattr(h, 'transport_trajectory')
    finally:
        release.set()
        h._retime_pool.shutdown(wait=True)
