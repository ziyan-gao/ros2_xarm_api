"""Task policy handshake and ownership, without robot commands."""
import json
import time
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import pytest
import rclpy
from rclpy.parameter import Parameter
from safe_servo_visualization.task_motion_policy import TaskMotionPolicy, FIELDS, validated_policy
from safe_servo_visualization.tasks import task_module
from safe_servo_visualization.task_test_node import TaskWorker


def executor():
    n = NS(state='IDLE', ACTIVE={'EXECUTING'}, task_policy_owner='token',
           task_policy_seen=time.monotonic(), get_logger=lambda: Mock())
    for attr, _, _ in FIELDS.values():
        setattr(n, attr, 1.)
    return n


def request(motion=None):
    return [Parameter('task_motion_policy', value=json.dumps(dict(
        task='pack_new', token='token', motion=motion or task_module('pack_new').MOTION)))]


def test_policy_reload_changes_executor_without_restarting_executor():
    n = executor()
    assert TaskMotionPolicy._task_policy_parameters(n, request()).successful
    assert n.transport_force_threshold == 8 and n.place_force_threshold == 4
    changed = dict(task_module('pack_new').MOTION, transport_force_n=9., retreat_speed_mm_s=12.)
    assert TaskMotionPolicy._task_policy_parameters(n, request(changed)).successful
    assert n.transport_force_threshold == 9 and n.return_clearance_speed == 12
    assert task_module('repack').MOTION['transport_force_n'] == 8


@pytest.mark.parametrize('case', ['active', 'owner', 'stale', 'invalid'])
def test_rejected_policy_never_partially_changes_settings(case):
    n = executor()
    motion = dict(task_module('pack_new').MOTION)
    if case == 'active': n.state = 'EXECUTING'
    if case == 'owner': n.task_policy_owner = 'someone-else'
    if case == 'stale': n.task_policy_seen -= 4
    if case == 'invalid': motion['place_force_n'] = float('nan')
    assert not TaskMotionPolicy._task_policy_parameters(n, request(motion)).successful
    assert all(getattr(n, attr) == 1 for attr, _, _ in FIELDS.values())


def test_worker_does_not_start_until_service_and_matching_fresh_status_ack():
    rclpy.init()
    n = TaskWorker('repack', 'epoch')
    try:
        n.state = 'CONFIGURING_TASK'
        n.token = 'token'
        n.phase_started = time.monotonic()
        future = Future()
        n.policy_client = Mock()
        n.policy_client.service_is_ready.return_value = True
        n.policy_client.call_async.return_value = future
        with patch('safe_servo_visualization.task_test_node.PickPlaceTest.start') as start:
            start.return_value = NS(success=True)
            n.configure_task()
            n.configure_task()
            assert not start.called
            future.set_result(NS(result=NS(successful=True)))
            n.status['supervisor'] = {'task_policy_token':'old'}
            n.received['supervisor'] = time.monotonic()
            n.configure_task()
            assert not start.called
            n.status['supervisor']['task_policy_token'] = 'token'
            n.configure_task()
            start.assert_called_once()
            assert n.policy_client.call_async.call_count == 1
    finally:
        n.worker.shutdown(wait=False, cancel_futures=True)
        n.destroy_node()
        rclpy.shutdown()
