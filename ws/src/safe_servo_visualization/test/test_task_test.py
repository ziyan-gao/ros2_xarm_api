"""Offline process isolation checks; no robot endpoints or commands are present."""
import json
import time
from types import SimpleNamespace as NS
from unittest.mock import patch

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from safe_servo_visualization.task_test_node import TaskSession, TaskWorker, checkpoint, restore


def test_checkpoint_is_copied_and_json_item_keys_restored():
    source = NS(location='pallet', record={'obstacle_id': 'placed_item_1'}, target=None,
                test_items={1: {'location': 'pallet'}}, active_item=1, test_slot=0,
                two_item_mode=True, new_item_sam_enabled=True, sequence=123)
    data = json.loads(json.dumps(checkpoint(source)))
    target = NS()
    restore(target, data)
    assert target.test_items == source.test_items
    target.record['obstacle_id'] = 'changed'
    assert source.record['obstacle_id'] == 'placed_item_1'


def test_real_workers_restart_independently_and_crash_keeps_checkpoint():
    rclpy.init()
    node = TaskSession()
    def spin_until(condition, timeout=30):
        end = time.monotonic()+timeout
        while not condition() and time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.05)
        assert condition()
    try:
        spin_until(lambda: len(node.task_info)==4)
        assert all(p['process'].poll() is None for p in node.processes.values())
        original = {k: p['process'].pid for k, p in node.processes.items()}
        # Exercise real DDS dispatch/reply. Worker independently rejects missing dependencies.
        with patch.object(node, 'check_preconditions', return_value=''):
            assert node.start('pack_new', Trigger.Response()).success
        spin_until(lambda: node.state=='FAULT')
        assert 'status missing or stale' in node.fault
        node.state, node.fault, node.last_operation = 'IDLE', '', None
        node.record = {'obstacle_id': 'placed_item_1'}
        node.location = 'pallet'
        # Missing actual stationary feedback must block restart.
        result = node.restart('unpack', Trigger.Response())
        assert not result.success and 'feedback' in result.message
        # The test has no hardware. Bypass only the physical stop check for this process test.
        with patch.object(node, 'restart_reason', return_value=''):
            assert node.restart('unpack', Trigger.Response()).success
        spin_until(lambda: node.processes['unpack']['process'].pid!=original['unpack']
                   and 'unpack' in node.task_info)
        assert node.record == {'obstacle_id': 'placed_item_1'}
        assert all(node.processes[k]['process'].pid==v for k, v in original.items() if k!='unpack')
        # Simulate loss of the active task (without sending it a motion command).
        node.state = 'ESTIMATING'
        node.active = dict(task='pack_new', token='never-dispatched', context=checkpoint(node))
        node.processes['pack_new']['process'].kill()
        node.processes['pack_new']['process'].wait(timeout=5)
        node.tick()
        assert node.state=='FAULT' and node.active is None
        assert node.record == {'obstacle_id': 'placed_item_1'}
        assert not node.random_active
        # Restart neither clears a fault nor replays an operation.
        with patch.object(node, 'restart_reason', return_value=''):
            assert node.restart('pack_new', Trigger.Response()).success
        spin_until(lambda: 'pack_new' in node.task_info)
        assert node.state=='FAULT' and node.active is None
    finally:
        node.close()
        node.worker.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()


def test_worker_rejects_old_epoch_and_deduplicates_commands():
    rclpy.init()
    worker = TaskWorker('pack_new', 'current')
    try:
        context = checkpoint(worker)
        with patch.object(worker, 'check_preconditions', return_value=''), patch.object(worker, 'configure_task') as start:
            start.return_value = NS(success=True)
            worker.command(String(data=json.dumps(dict(epoch='old', token='a', context=context))))
            assert not start.called
            msg = String(data=json.dumps(dict(epoch='current', token='a', context=context)))
            worker.command(msg)
            worker.command(msg)
            assert start.call_count == 1
        worker.state = 'ESTIMATING'
        worker.command_received = time.monotonic()-4
        worker.tick()
        assert worker.state=='FAULT'
        assert 'heartbeat lost' in worker.fault
    finally:
        worker.worker.shutdown(wait=False, cancel_futures=True)
        worker.destroy_node()
        rclpy.shutdown()


def test_restart_requires_a_stationary_window_not_slow_accumulating_motion():
    node = NS(previous_joints=None, stationary_since=None, joints_received=0.)
    names = ['joint'+str(i) for i in range(1, 7)]
    with patch('safe_servo_visualization.task_test_node.time.monotonic') as clock:
        for i in range(150):
            clock.return_value = 10+i*.01
            # No velocity field: accumulated position still prevents a false stop.
            TaskSession.joints(node, JointState(name=names, position=[i*.00005]*6))
        assert node.stationary_since is None or clock.return_value-node.stationary_since < 1.
        for i in range(150):
            clock.return_value = 12+i*.01
            TaskSession.joints(node, JointState(name=names, position=[.01]*6))
        assert clock.return_value-node.stationary_since >= 1.


def test_routine_reset_keeps_inventory_and_checks_downstream_and_scene():
    rclpy.init()
    with patch.object(TaskSession, 'spawn'):
        node = TaskSession()
    try:
        node.state, node.fault, node.location = 'FAULT', 'operator abort', 'pallet'
        node.record = {'obstacle_id': 'placed_item_1'}
        node.test_items = {0: {'location': 'pallet', 'record': dict(node.record)}}
        before = checkpoint(node)
        node.status['scene'] = {'placed_item_ids': ['placed_item_1']}
        with patch.object(node, 'reset_interlock', return_value=''), patch.object(node, 'restart_reason', return_value=''), patch(
                'safe_servo_visualization.task_test_node.PickPlaceTest.check_preconditions',
                return_value='pickup is not idle/ready: FAULT'):
            response = node.reset(None, Trigger.Response())
            assert not response.success and 'pickup' in response.message
            assert checkpoint(node) == before and node.state == 'FAULT'
        with patch.object(node, 'reset_interlock', return_value=''), patch.object(node, 'restart_reason', return_value=''), patch(
                'safe_servo_visualization.task_test_node.PickPlaceTest.check_preconditions', return_value=''):
            node.status['scene'] = {'placed_item_ids': []}
            assert not node.reset(None, Trigger.Response()).success
            assert checkpoint(node) == before
            node.status['scene'] = {'placed_item_ids': ['placed_item_1']}
            assert node.reset(None, Trigger.Response()).success
            assert checkpoint(node) == before
            assert node.state == 'READY' and node.fault == ''
    finally:
        node.close()
        node.worker.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()


def test_reset_waits_for_downstream_ack_and_state_then_unlocks_without_forgetting_item():
    from concurrent.futures import Future
    from unittest.mock import Mock
    rclpy.init()
    with patch.object(TaskSession, 'spawn'):
        node = TaskSession()
    try:
        node.state, node.fault, node.location = 'FAULT', 'aborted', 'pallet'
        node.record = {'obstacle_id': 'placed_item_1'}
        node.status = {key: {'state': 'IDLE'} for key in node.TOPICS}
        node.status['scene'] = {'placed_item_ids': ['placed_item_1']}
        node.status['pickup'] = {'state': 'FAULT'}
        saved = checkpoint(node)
        future = Future()
        client = Mock()
        client.service_is_ready.return_value = True
        client.call_async.return_value = future
        node.reset_clients['pickup'] = client
        with patch.object(node, 'reset_interlock', return_value=''), patch.object(
                node, 'restart_reason', return_value=''), patch(
                'safe_servo_visualization.task_test_node.PickPlaceTest.check_preconditions', return_value=''):
            assert node.reset(None, Trigger.Response()).success
            node.reset_tick()
            assert node.state == 'FAULT' and node.recovery
            future.set_result(Trigger.Response(success=True))
            node.reset_tick()
            assert node.state == 'FAULT' and node.recovery
            node.status['pickup']['state'] = 'IDLE'
            node.received['pickup'] = time.monotonic()
            node.reset_tick()
            node.reset_tick()
            assert node.state == 'READY' and not node.recovery
            assert checkpoint(node) == saved
            client.call_async.assert_called_once()
    finally:
        node.close()
        node.worker.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()


def test_empty_reset_clears_prepared_then_pack_new_is_allowed():
    from concurrent.futures import Future
    from unittest.mock import Mock
    rclpy.init()
    with patch.object(TaskSession, 'spawn'):
        n = TaskSession()
    try:
        now = time.monotonic()
        n.state, n.fault = 'FAULT', 'aborted'
        n.status = {key: {'state':'IDLE'} for key in n.TOPICS}
        n.status['motion']['state'] = 'PREPARED'
        n.status['supervisor'].update(robot_error=0, ft_recovery_required=False)
        n.status['staging']['slots'] = [{'slot':0, 'occupied':False}]
        n.received = {key:now for key in n.TOPICS}
        n.pallet_locked, n.pallet_received = True, now
        n.joints_received, n.stationary_since = now, now-2
        n.task_received['pack_new'] = now
        n.task_info['pack_new'] = {'ready':True}
        n.processes['pack_new'] = dict(process=Mock(pid=123, poll=lambda:None), stopping=False, epoch='e')
        f = Future()
        n.reset_clients['motion'] = Mock()
        n.reset_clients['motion'].service_is_ready.return_value = True
        n.reset_clients['motion'].call_async.return_value = f
        assert n.restart_reason() == ''
        assert n.reset(None, Trigger.Response()).success
        n.reset_tick()
        f.set_result(Trigger.Response(success=True))
        n.reset_tick()
        assert n.state == 'FAULT'  # PREPARED status still outstanding.
        n.status['motion']['state'] = 'IDLE'
        n.received['motion'] = time.monotonic()
        n.reset_tick()
        n.reset_tick()
        assert n.state == 'IDLE' and not n.recovery
        assert n.check_preconditions('pack_new') == ''
    finally:
        n.close()
        n.worker.shutdown(wait=False, cancel_futures=True)
        n.destroy_node()
        rclpy.shutdown()
