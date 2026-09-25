"""One process per test task; the session alone owns scheduling and inventory.

Workers reuse the production test state machine, never their own robot drivers.
Restart replaces Python code, not a motion request. Interrupted work stays faulted.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from .tasks import task_module
from sensor_msgs.msg import JointState

from .pick_place_test_node import PickPlaceTest, STEPS, placed_ids

# Only confirmed task context crosses processes; futures, clients and ROS state do not.
CONTEXT = ('location', 'record', 'target', 'test_items', 'active_item', 'test_slot',
           'two_item_mode', 'new_item_sam_enabled', 'sequence')
PROGRESS = ('step', 'fault', 'owner', 'expected')


def checkpoint(node):
    return deepcopy({key: getattr(node, key) for key in CONTEXT})


def restore(node, data):
    for key in CONTEXT:
        setattr(node, key, deepcopy(data[key]))
    node.test_items = {int(k): v for k, v in node.test_items.items()}


class TaskWorker(PickPlaceTest):
    def __init__(self, task, epoch):
        self.task, self.epoch, self.token = task, epoch, None
        self.command_received = time.monotonic()
        self.seen = set()
        self.configuration_future = None
        self.task_definition = task_module(task)
        super().__init__('test_task_'+task, controls=False)
        self.create_subscription(String, '/test_tasks/'+task+'/command', self.command, 10)
        self.policy_client = self.create_client(SetParametersAtomically, '/pickup_supervisor/set_parameters_atomically')

    def command(self, message):
        try:
            data = json.loads(message.data)
            if data.get('epoch') != self.epoch:
                return
            self.command_received = time.monotonic()
            token = data.get('token')
            if token != self.token and self.busy:
                self.fail('session cancelled task; no automatic resume')
                return
            if not token or token in self.seen:
                return
            self.seen.add(token)
            self.token = token
            restore(self, data['context'])
            self.state, self.fault = 'READY', ''
            if self.planning is not None and not self.planning.done():
                self.fail('previous planning worker is still finishing; restart this task node')
                return
            if self.loader.pending is not None:
                self.loader.discard_pending(self.loader.pending.sequence_id)
            reason = self.check_preconditions(self.task)
            if reason:
                self.fail(reason)
                return
            self.step = self.task
            self.phase('CONFIGURING_TASK')
            self.configuration_future = None
            self.configure_task()
        except (ValueError, KeyError, TypeError) as exc:
            self.fail('invalid session command: '+str(exc))

    def configure_task(self):
        if self.state != 'CONFIGURING_TASK':
            return
        if time.monotonic()-self.phase_started > 10.:
            self.fail('task policy acknowledgement timeout; no task motion started')
            return
        if self.configuration_future is None:
            if not self.policy_client.service_is_ready():
                return
            value = json.dumps(dict(task=self.task, token=self.token, motion=self.task_definition.MOTION))
            request = SetParametersAtomically.Request(parameters=[
                Parameter('task_motion_policy', value=value).to_parameter_msg()])
            self.configuration_future = self.policy_client.call_async(request)
            return
        if not self.configuration_future.done():
            return
        try:
            result = self.configuration_future.result().result
            if not result.successful:
                if result.reason.startswith('waiting for task session'):
                    self.configuration_future = None
                    return
                raise ValueError(result.reason)
            if (not self.fresh('supervisor') or
                    self.status['supervisor'].get('task_policy_token') != self.token):
                return
            self.state = 'READY'
            response = super().start(self.task, Trigger.Response(), automatic=True)
            self.random_step_pending = False
            if not response.success and self.state != 'FAULT':
                self.fail(response.message)
        except Exception as exc:
            self.fail('task policy configuration failed: '+str(exc))

    def tick(self):
        if self.busy and time.monotonic()-self.command_received > 3.:
            self.fail('test session heartbeat lost; task cancelled')
        if self.state == 'CONFIGURING_TASK':
            self.configure_task()
        else:
            super().tick()

    def publish_payload(self, payload):
        payload.update(epoch=self.epoch, token=self.token, context=checkpoint(self),
                       expected=self.expected, sequence=self.sequence,
                       task_module=self.task_definition.__name__, motion_policy=self.task_definition.MOTION,
                       ready=all(self.fresh(k) for k in self.TOPICS))
        super().publish_payload(payload)


class TaskSession(PickPlaceTest):
    def __init__(self):
        self.processes = {}
        self.active = None
        self.last_operation = None
        self.recovery = None
        self.recovery_message = ''
        self.task_info = {}
        self.task_received = {}
        self.stationary_since = None
        self.joints_received = 0.
        self.previous_joints = None
        super().__init__()
        self.directory = tempfile.TemporaryDirectory(prefix='xarm-test-tasks-')
        self.params_path = Path(self.directory.name)/'params.yaml'
        params = {name: self.get_parameter(name).value for name in self.list_parameters([], 0).names
                  if name not in ('use_sim_time',) and not name.startswith('qos_overrides.')}
        # JSON is YAML: pass the same resolved ROS parameters to every worker.
        self.params_path.write_text(json.dumps({'/**': {'ros__parameters': params}}))
        self.commands = {task: self.create_publisher(String, '/test_tasks/'+task+'/command', 10)
                         for task in STEPS}
        for task in STEPS:
            self.create_subscription(String, '/test_task_'+task+'/status',
                                     lambda msg, task=task: self.task_status(task, msg), 10)
            self.create_service(Trigger, '/test_tasks/'+task+'/restart',
                                lambda req, res, task=task: self.restart(task, res))
            self.spawn(task)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_service(Trigger, '/test_tasks/reconcile', self.reconcile)
        self.create_service(Trigger, '/test_tasks/clear_bookkeeping', self.clear_bookkeeping)
        self.reset_clients = {key: self.create_client(Trigger, self.TOPICS[key].removesuffix('/status')+'/reset')
                              for key in ('cycle', 'pickup', 'place', 'staging', 'supervisor', 'motion')}

    def spawn(self, task):
        epoch = uuid.uuid4().hex
        # Allowlisted module + task, no shell or robot-control process restart.
        process = subprocess.Popen([sys.executable, '-m', __name__, task, epoch,
                                    '--ros-args', '--params-file', str(self.params_path)],
                                   start_new_session=True)
        self.processes[task] = dict(process=process, epoch=epoch, stopping=False)
        self.task_info.pop(task, None)
        self.task_received.pop(task, None)
        self.get_logger().info(f'task node test_task_{task} started, pid={process.pid}')

    def joints(self, msg):
        now = time.monotonic()
        values = dict(zip(msg.name, msg.position))
        # Require actual feedback continuity; empty or incomplete feedback is not stopped.
        valid = all('joint'+str(i) in values and math.isfinite(values['joint'+str(i)])
                    for i in range(1, 7))
        still = (valid and self.previous_joints is not None and now-self.joints_received < .5
                 and all(abs(values[k]-self.previous_joints.get(k, float('inf'))) < .0005
                         for k in values)
                 and all(math.isfinite(v) and abs(v) < .005 for v in msg.velocity))
        self.stationary_since = (self.stationary_since or now) if still else None
        if not still:
            self.previous_joints = values if valid else None
        self.joints_received = now

    def restart_reason(self):
        if self.busy or self.active or self.random_active:
            return 'stop the test and wait for the current task to finish/abort first'
        if (time.monotonic()-self.joints_received > .5 or self.stationary_since is None or
                time.monotonic()-self.stationary_since < 1.):
            return 'waiting for fresh, stationary joint feedback (1 second)'
        for key in ('motion', 'place', 'cycle', 'staging', 'pickup', 'supervisor', 'random', 'policy'):
            value = self.status.get(key, {})
            if not self.fresh(key) or value.get('state') not in (
                    'IDLE', 'READY', 'SUCCEEDED', 'FAULT', 'PREPARED', 'OBJECT_INFO_READY', 'AWAITING_GRASP'):
                return key+' must be stopped with fresh status before restarting a task'
            if any(value.get(k) for k in ('continuous_loading_enabled', 'continuous_run_active',
                                         'auto_start_pick_place')):
                return key+' automatic loading must be disabled'
        return ''

    def restart(self, task, response):
        reason = 'task reset in progress' if self.recovery else self.restart_reason()
        entry = self.processes[task]
        if reason or entry['stopping']:
            response.message = reason or 'restart already in progress'
            return response
        process = entry['process']
        if process.poll() is None:
            process.terminate()
            entry.update(stopping=True, stop_started=time.monotonic())
        else:
            self.spawn(task)
        response.success = True
        response.message = f'restarting test_task_{task}; inventory/fault retained, no motion replayed'
        return response

    def check_preconditions(self, step=None, slot_id=None):
        if step and self.recovery:
            return 'task reset in progress: '+self.recovery_message
        reason = super().check_preconditions(step, slot_id)
        if reason or not step:
            return reason
        entry = self.processes.get(step)
        if (not entry or entry['stopping'] or entry['process'].poll() is not None or
                time.monotonic()-self.task_received.get(step, 0.) > 3. or
                not self.task_info.get(step, {}).get('ready')):
            return 'task node test_task_'+step+' is not ready; see node status/restart'
        return ''

    def start(self, step, response, automatic=False):
        if step not in STEPS:
            response.message = 'unknown task'
            return response
        if self.random_active and not automatic:
            response.message = 'random scheduler owns the robot'
            return response
        if self._two_enabled() and not automatic:
            response.message = 'two-item mode uses the random scheduler'
            return response
        reason = self.check_preconditions(step)
        if reason:
            response.message = reason
            return response
        self.generation += 1
        self.step, self.fault = step, ''
        self.random_step_pending = automatic
        self.active = dict(task=step, token=uuid.uuid4().hex, context=checkpoint(self))
        self.last_operation = (step, self.active['token'], self.processes[step]['epoch'])
        self.phase('DISPATCHING')
        response.success, response.message = True, 'started test_task_'+step
        return response

    def task_status(self, task, message):
        try:
            data = json.loads(message.data)
            if data.get('epoch') != self.processes[task]['epoch']:
                return
            self.task_info[task] = data
            self.task_received[task] = time.monotonic()
            if (self.state == 'FAULT' and not self.active and
                    self.last_operation == (task, data.get('token'), data.get('epoch'))):
                # A detach acknowledgement can arrive just after cancellation.
                # Preserve that inventory update without advancing or replaying the task.
                restore(self, data['context'])
                return
            if (not self.active or self.active['task'] != task or
                    data.get('token') != self.active['token']):
                return
            restore(self, data['context'])
            for field in PROGRESS:
                setattr(self, field, data.get(field))
            self.events.clear()
            self.events.extend(data.get('events', []))
            self.worker_sam_token = data.get('sam_request_id')
            if data['state'] == 'FAULT':
                self.active = None
                self.fail(data.get('fault') or task+' failed')
            elif data['state'] == 'READY':
                self.active = None
                self.finish(self.location)
            elif self.state != data['state']:
                self.state, self.phase_started = data['state'], time.monotonic()
            self.publish_status()  # Forward SAM tokens before its inspection request can arrive.
        except (ValueError, TypeError, KeyError) as exc:
            if self.active and self.active['task'] == task:
                self.fail('invalid task status: '+str(exc))

    def ack(self, msg):
        pass  # Target handshakes belong to the worker, not the session mirror.

    def target_seen(self, msg):
        pass

    def receive(self, key, msg):
        # Inventory updates belong to the active worker. The session observes dependencies.
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.status[key], self.received[key] = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def fail(self, reason):
        self.active = None
        super().fail(reason)

    def tick(self):
        if self.recovery:
            self.reset_tick()
        for task, entry in self.processes.items():
            process = entry['process']
            if entry['stopping']:
                if process.poll() is not None:
                    self.spawn(task)
                elif time.monotonic()-entry['stop_started'] > 3.:
                    process.kill()  # Only our stopped, allowlisted task process.
                continue
            if self.active and self.active['task'] == task:
                if process.poll() is not None or time.monotonic()-self.task_received.get(task, 0.) > 3.:
                    self.fail('test_task_'+task+' exited or heartbeat lost; checkpoint retained')
            command = dict(epoch=entry['epoch'], token=None)
            if self.active and self.active['task'] == task:
                command.update(self.active)
            self.commands[task].publish(String(data=json.dumps(command)))
        if self.state == 'DISPATCHING' and time.monotonic()-self.phase_started > 10.:
            self.fail('task acceptance timeout; command cancelled, no retry')
        if not self.busy and not self.recovery:
            self.random_tick()

    def reset_interlock(self):
        reason = self.restart_reason()
        scene, supervisor = self.status.get('scene', {}), self.status.get('supervisor', {})
        if reason:
            return reason
        if not self.fresh('scene'):
            return 'waiting for fresh scene status'
        if scene.get('attached_item_id') or scene.get('attachment_pending') or self.location == 'carried':
            return 'item is held/attachment pending; recover the physical item first'
        if supervisor.get('robot_error') != 0 or supervisor.get('ft_recovery_required'):
            return 'robot/FT fault requires recovery first'
        return ''

    def reset(self, request, response):
        reason = 'reset already in progress' if self.recovery else self.reset_interlock()
        if reason:
            response.message = reason
            return response
        queue = [key for key in self.reset_clients
                 if self.status.get(key, {}).get('state') == 'FAULT' or
                 (key == 'motion' and self.status.get(key, {}).get('state') == 'PREPARED')]
        if not queue:
            return self.reconcile(request, response)
        self.recovery = dict(queue=queue, future=None, started=time.monotonic())
        self.recovery_message = 'resetting downstream faults; item records retained'
        response.success, response.message = True, self.recovery_message
        self.publish_status()
        return response

    def reset_tick(self):
        recovery = self.recovery
        reason = self.reset_interlock()
        if reason or time.monotonic()-recovery['started'] > 15.:
            self.recovery_message = reason or 'reset confirmation timed out; '+self.recovery_message
            self.recovery = None
            self.get_logger().warning(self.recovery_message)
            return
        queue, future = recovery['queue'], recovery['future']
        if not queue:
            result = self.reconcile(None, Trigger.Response())
            self.recovery_message = result.message
            if result.success:
                self.recovery = None
            return
        key = queue[0]
        if future is not None:
            if not future.done():
                return
            try:
                result = future.result()
                if result is None or not result.success:
                    raise ValueError(result.message if result else 'no reset response')
            except Exception as exc:
                self.recovery_message = key+' reset failed: '+str(exc)
                self.recovery = None
                return
            # Service success alone is not evidence that the node has changed state.
            if (self.received.get(key, 0.) <= recovery['sent'] or
                    self.status[key].get('state') in ('FAULT', 'PREPARED')):
                self.recovery_message = 'waiting for '+key+' reset state confirmation'
                return
            queue.pop(0)
            recovery['future'] = None
        elif self.status[key].get('state') not in ('FAULT', 'PREPARED'):
            queue.pop(0)  # An upstream reset may already have reset this node.
        elif self.reset_clients[key].service_is_ready():
            recovery['sent'] = time.monotonic()
            recovery['future'] = self.reset_clients[key].call_async(Trigger.Request())
            self.recovery_message = 'resetting '+key
        else:
            self.recovery_message = 'waiting for '+key+' reset service'

    def clear_bookkeeping(self, request, response):
        if self.recovery:
            response.message = 'task reset in progress'
            return response
        response = super().reset(request, response)
        if response.success:
            self.last_operation = None
        return response

    def reconcile(self, request, response):
        reason = self.restart_reason() or super().check_preconditions()
        if reason:
            response.message = reason
            return response
        if (self.location not in ('empty', 'pallet', 'slot') or
                (self.location != 'empty' and not self.record)):
            response.message = 'item location uncertain/carried; use physical recovery first'
            return response
        if self.record:
            obstacle = self.record.get('obstacle_id')
            if ((self.location == 'pallet' and obstacle not in placed_ids(self.status['scene'])) or
                    (self.location == 'slot' and (not self.slot().get('occupied') or
                     obstacle != self.slot().get('obstacle_id')))):
                response.message = 'recorded item does not match scene/slot; reconcile physically first'
                return response
        self.last_operation = None
        self.generation += 1
        self.fault = ''
        self.phase('READY' if self.record or self.test_items else 'IDLE')
        response.success, response.message = True, 'checkpoint confirmed; choose a task explicitly (no motion resumed)'
        return response

    def publish_payload(self, payload):
        payload['task_policy_token'] = self.active['token'] if self.active else None
        payload['sam_request_id'] = getattr(self, 'worker_sam_token', None) if self.active else None
        payload['task_nodes'] = {task: dict(
            node='test_task_'+task, pid=entry['process'].pid,
            alive=entry['process'].poll() is None,
            fresh=time.monotonic()-self.task_received.get(task, 0.) < 3.,
            restarting=entry['stopping'],
            state=self.task_info.get(task, {}).get('state', 'STARTING'),
            task_module=self.task_info.get(task, {}).get('task_module'),
            motion_policy=self.task_info.get(task, {}).get('motion_policy'),
            fault=self.task_info.get(task, {}).get('fault', ''))
            for task, entry in self.processes.items()}
        payload['automatic_motion_active'] = any(
            value.get(key) for value in self.status.values() for key in
            ('continuous_loading_enabled', 'continuous_run_active', 'auto_start_pick_place'))
        payload['restart_blocked'] = self.restart_reason()
        payload['recovery_blocked'] = self.restart_reason() or super().check_preconditions()
        payload['reset_preserves_inventory'] = True
        payload['reset_in_progress'] = bool(self.recovery)
        payload['reset_message'] = self.recovery_message
        payload['downstream_faults'] = {k: v.get('fault', '') for k, v in self.status.items()
                                        if v.get('fault')}
        super().publish_payload(payload)

    def close(self):
        if self.busy and rclpy.ok():
            self.fail('session shutting down')
        for entry in self.processes.values():
            if entry['process'].poll() is None:
                entry['process'].terminate()
        for entry in self.processes.values():
            try:
                entry['process'].wait(timeout=3)
            except subprocess.TimeoutExpired:
                entry['process'].kill()
                entry['process'].wait()
        self.directory.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = TaskSession()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.worker.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    task, epoch = sys.argv[1:3]
    if task not in STEPS:
        raise SystemExit('unknown task')
    rclpy.init(args=sys.argv[3:])
    node = TaskWorker(task, epoch)
    try:
        rclpy.spin(node)
    finally:
        node.worker.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
