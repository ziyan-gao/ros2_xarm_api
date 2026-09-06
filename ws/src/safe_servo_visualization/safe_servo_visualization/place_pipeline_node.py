import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PlacePipeline(Node):
    """Move to pre-place, guarded-place, retreat, and return to observation."""

    IDLE = 'IDLE'
    MOVE_TRANSFER = 'MOVE_TRANSFER'
    LOAD_PRE_PLACE = 'LOAD_PRE_PLACE'
    CONTACT_PLACE = 'CONTACT_PLACE'
    MOVE_OBSERVATION = 'MOVE_OBSERVATION'
    SUCCEEDED = 'SUCCEEDED'
    FAULT = 'FAULT'
    ABORTING = 'ABORTING'
    ACTIVE = {MOVE_TRANSFER, LOAD_PRE_PLACE, CONTACT_PLACE, MOVE_OBSERVATION, ABORTING}

    def __init__(self):
        super().__init__('place_pipeline')
        self.declare_parameter('motion_timeout_sec', 120.0)
        self.declare_parameter('post_loading_settle_sec', 0.75)
        self.motion_timeout = float(
            self.get_parameter('motion_timeout_sec').value)
        self.post_loading_settle = float(
            self.get_parameter('post_loading_settle_sec').value)
        self.state, self.fault, self.pending_motion = self.IDLE, '', None
        self.operation_id = 0
        self.phase_started = None
        self.motion_status = {}
        self.supervisor_status = {}
        self.scene_status = {}
        self.pallet_status = 'UNLOCALIZED'
        self.expected_motion_operation_id = None
        self.expected_supervisor_operation_id = None
        self.loading_succeeded_at = None
        self.transfer_fallback_used = False
        self.transfer_fallback_reason = ''
        self.status_pub = self.create_publisher(String, '/place_pipeline/status', 10)
        self.create_subscription(
            String, '/motion_coordinator/status', self._motion_status, 10)
        self.create_subscription(
            String, '/pickup_supervisor/status', self._supervisor_status, 10)
        self.create_subscription(
            String, '/planning_scene_obstacles/status', self._scene_status, 10)
        self.create_subscription(
            String, '/pallet_localization/status', self._pallet_status, 10)
        self.plan_transfer = self.create_client(
            Trigger, '/motion_coordinator/plan_transfer')
        self.plan_observation = self.create_client(
            Trigger, '/motion_coordinator/plan_observation')
        self.execute_motion = self.create_client(
            Trigger, '/motion_coordinator/execute')
        self.cancel_motion = self.create_client(
            Trigger, '/motion_coordinator/cancel')
        self.start_place = self.create_client(
            Trigger, '/pickup_supervisor/start_place')
        self.start_loading = self.create_client(
            Trigger, '/pickup_supervisor/start_loading')
        self.start_transfer_fallback = self.create_client(
            Trigger, '/pickup_supervisor/start_transfer_fallback')
        self.accept_direct_transfer = self.create_client(
            Trigger, '/motion_coordinator/accept_direct_transfer')
        self.abort_supervisor = self.create_client(
            Trigger, '/pickup_supervisor/abort')
        self.create_service(Trigger, '/place_pipeline/start', self.start_callback)
        self.create_service(Trigger, '/place_pipeline/abort', self.abort_callback)
        self.create_service(Trigger, '/place_pipeline/reset', self.reset_callback)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_status)

    @staticmethod
    def _decode(message):
        try:
            return json.loads(message.data)
        except (TypeError, ValueError):
            return {}

    def _motion_status(self, message):
        self.motion_status = self._decode(message)

    def _supervisor_status(self, message):
        self.supervisor_status = self._decode(message)

    def _scene_status(self, message):
        self.scene_status = self._decode(message)

    def _pallet_status(self, message):
        self.pallet_status = message.data

    def start_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'place pipeline already active in {self.state}'
            return response
        if not self.scene_status.get('attached_item_id'):
            return self._reject_start(
                response, 'no carried item is attached; complete pickup first')
        if self.pallet_status != 'LOCKED':
            return self._reject_start(response, 'pallet pose is not LOCKED')
        if not self.plan_transfer.service_is_ready():
            return self._reject_start(
                response, 'transfer planning service is unavailable')
        self.operation_id += 1
        self.fault = ''
        self.transfer_fallback_used = False
        self.transfer_fallback_reason = ''
        self.state = self.MOVE_TRANSFER
        self.pending_motion = 'transfer'
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        self.expected_supervisor_operation_id = None
        self.phase_started = time.monotonic()
        future = self.plan_transfer.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done: self._plan_completed(done, 'transfer'))
        response.success = True
        response.message = 'place pipeline started: planning nominal transfer'
        self.publish_status()
        return response

    def _reject_start(self, response, reason):
        self.state = self.FAULT
        self.fault = reason
        response.success = False
        response.message = reason
        self.get_logger().error(reason)
        self.publish_status()
        return response

    def _plan_completed(self, future, label):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'plan {label} failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'plan {label} rejected: {message}')

    def _execute(self):
        if not self.execute_motion.service_is_ready():
            self._fault('motion execution service is unavailable')
            return
        future = self.execute_motion.call_async(Trigger.Request())
        future.add_done_callback(self._execute_completed)

    def _execute_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'motion execution failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'motion execution rejected: {message}')

    def tick(self):
        if self.state not in self.ACTIVE or self.state == self.ABORTING:
            return
        if time.monotonic() - self.phase_started > self.motion_timeout:
            self._fault(f'place pipeline timed out in {self.state}')
            return
        motion_fault_matches = (
            self.motion_status.get('state') == 'FAULT' and
            self.expected_motion_operation_id is not None and
            int(self.motion_status.get('operation_id', -1)) >=
            self.expected_motion_operation_id)
        if motion_fault_matches and self.state != self.CONTACT_PLACE:
            reason = self.motion_status.get('fault', 'MoveIt motion failed')
            if (self.state == self.MOVE_TRANSFER and
                    self.pending_motion == 'transfer'):
                self._begin_transfer_fallback(reason)
                return
            if not (self.state == self.MOVE_TRANSFER and
                    self.pending_motion in (
                        'transfer_fallback_starting',
                        'transfer_fallback_executing',
                        'transfer_fallback_accepting')) and not (
                            self.transfer_fallback_used and
                            self.state == self.LOAD_PRE_PLACE):
                self._fault(reason)
                return
        if self.state == self.MOVE_TRANSFER:
            self._tick_transfer()
        elif self.state == self.LOAD_PRE_PLACE:
            self._tick_loading()
        elif self.state == self.CONTACT_PLACE:
            self._tick_contact_place()
        elif self.state == self.MOVE_OBSERVATION:
            self._tick_observation()

    def _tick_transfer(self):
        if self.pending_motion in (
                'transfer_fallback_starting',
                'transfer_fallback_executing'):
            if self.supervisor_status.get('operation_kind') != 'transfer':
                return
            if (self.expected_supervisor_operation_id is None or
                    int(self.supervisor_status.get('operation_id', -1)) <
                    self.expected_supervisor_operation_id):
                return
            state = self.supervisor_status.get('state')
            if state == 'FAULT':
                self._fault(self.supervisor_status.get(
                    'fault', 'direct transfer fallback failed'))
            elif (state == 'SUCCEEDED' and
                  self.supervisor_status.get('direct_transfer_succeeded')):
                self.transfer_fallback_used = True
                self.get_logger().warning(
                    'direct xArm transfer fallback reached the transfer pose; '
                    'normalizing the MoveIt coordinator state')
                self._accept_direct_transfer()
            return
        operation_matches = (
            self.expected_motion_operation_id is not None and
            int(self.motion_status.get('operation_id', -1)) >=
            self.expected_motion_operation_id)
        if not operation_matches or self.motion_status.get('target') != 'transfer':
            return
        state = self.motion_status.get('state')
        if state == 'PLANNED' and self.pending_motion == 'transfer':
            self.pending_motion = 'transfer_executing'
            self._execute()
        elif state == 'SUCCEEDED' and self.pending_motion == 'transfer_executing':
            self.pending_motion = None
            self._begin_loading_motion()

    def _begin_transfer_fallback(self, reason):
        if not self.start_transfer_fallback.service_is_ready():
            self._fault(
                f'{reason}; direct transfer fallback service is unavailable')
            return
        self.transfer_fallback_reason = str(reason)
        self.pending_motion = 'transfer_fallback_starting'
        self.expected_supervisor_operation_id = int(
            self.supervisor_status.get('operation_id', 0)) + 1
        self.phase_started = time.monotonic()
        self.get_logger().warning(
            f'MoveIt transfer planning failed: {reason}; requesting direct '
            'xArm service fallback without MoveIt collision checking')
        future = self.start_transfer_fallback.call_async(Trigger.Request())
        future.add_done_callback(self._start_transfer_fallback_completed)
        self.publish_status()

    def _start_transfer_fallback_completed(self, future):
        if self.pending_motion != 'transfer_fallback_starting':
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'direct transfer fallback start failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'direct transfer fallback rejected: {message}')
            return
        self.pending_motion = 'transfer_fallback_executing'

    def _accept_direct_transfer(self):
        if not self.accept_direct_transfer.service_is_ready():
            self._fault(
                'direct transfer reached its target, but the motion '
                'coordinator acknowledgement service is unavailable')
            return
        self.pending_motion = 'transfer_fallback_accepting'
        future = self.accept_direct_transfer.call_async(Trigger.Request())
        future.add_done_callback(self._accept_direct_transfer_completed)

    def _accept_direct_transfer_completed(self, future):
        if self.pending_motion != 'transfer_fallback_accepting':
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'direct transfer acknowledgement failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'direct transfer acknowledgement rejected: {message}')
            return
        self.pending_motion = None
        self.get_logger().warning(
            'direct transfer accepted by the motion coordinator; continuing '
            'with linear loading')
        self._begin_loading_motion()

    def _begin_loading_motion(self):
        if not self.start_loading.service_is_ready():
            self._fault('linear loading service is unavailable')
            return
        self.state = self.LOAD_PRE_PLACE
        self.phase_started = time.monotonic()
        self.expected_supervisor_operation_id = int(
            self.supervisor_status.get('operation_id', 0)) + 1
        future = self.start_loading.call_async(Trigger.Request())
        future.add_done_callback(self._start_loading_completed)

    def _start_loading_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'linear loading start failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'linear loading rejected: {message}')

    def _tick_loading(self):
        if self.supervisor_status.get('operation_kind') != 'loading':
            return
        if int(self.supervisor_status.get('operation_id', -1)) < int(
                self.expected_supervisor_operation_id or 0):
            return
        state = self.supervisor_status.get('state')
        if state == 'FAULT':
            self._fault(self.supervisor_status.get('fault', 'linear loading failed'))
        elif state == 'SUCCEEDED':
            if self.supervisor_status.get('place_fallback_used'):
                self.loading_succeeded_at = None
                self.get_logger().warning(
                    'linear loading completed through contact fallback: '
                    f'{self.supervisor_status.get("place_fallback_reason", "")}; '
                    'returning to observation without starting safe-servo place')
                self._begin_observation_motion()
                return
            if self.loading_succeeded_at is None:
                self.loading_succeeded_at = time.monotonic()
                self.get_logger().info(
                    'linear loading succeeded; waiting '
                    f'{self.post_loading_settle:.2f} s for TCP telemetry')
                return
            if (time.monotonic() - self.loading_succeeded_at <
                    self.post_loading_settle):
                return
            if not self.start_place.service_is_ready():
                self._fault('guarded place supervisor is unavailable')
                return
            self.state = self.CONTACT_PLACE
            self.loading_succeeded_at = None
            self.phase_started = time.monotonic()
            self.expected_supervisor_operation_id = int(
                self.supervisor_status.get('operation_id', 0)) + 1
            future = self.start_place.call_async(Trigger.Request())
            future.add_done_callback(self._start_place_completed)

    def _start_place_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'guarded place start failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'guarded place rejected: {message}')

    def _tick_contact_place(self):
        if self.supervisor_status.get('operation_kind') != 'place':
            return
        if (self.expected_supervisor_operation_id is None or
                int(self.supervisor_status.get('operation_id', -1)) <
                self.expected_supervisor_operation_id):
            return
        state = self.supervisor_status.get('state')
        if state == 'FAULT':
            self._fault(self.supervisor_status.get('fault', 'guarded place failed'))
        elif state == 'SUCCEEDED':
            if self.supervisor_status.get('place_fallback_used'):
                self.get_logger().warning(
                    'guarded place completed through singularity fallback: '
                    f'{self.supervisor_status.get("place_fallback_reason", "")}; '
                    'returning to observation')
            self._begin_observation_motion()

    def _begin_observation_motion(self):
        if not self.plan_observation.service_is_ready():
            self._fault('observation planning service is unavailable')
            return
        self.state = self.MOVE_OBSERVATION
        self.pending_motion = 'observation'
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        self.phase_started = time.monotonic()
        future = self.plan_observation.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done: self._plan_completed(done, 'observation'))

    def _tick_observation(self):
        state = self.motion_status.get('state')
        operation_matches = (
            self.expected_motion_operation_id is not None and
            int(self.motion_status.get('operation_id', -1)) >=
            self.expected_motion_operation_id)
        if (operation_matches and state == 'PLANNED' and
                self.pending_motion == 'observation'):
            self.pending_motion = 'observation_executing'
            self._execute()
        elif (operation_matches and state == 'SUCCEEDED' and
              self.motion_status.get('target') == 'observation'):
            self.pending_motion = None
            self.state = self.SUCCEEDED
            self.publish_status()

    def _fault(self, reason):
        self.fault, self.state, self.pending_motion = reason, self.FAULT, None
        self.get_logger().error(reason)
        self.publish_status()

    def abort_callback(self, _request, response):
        if self.state not in self.ACTIVE:
            response.message = f'no active place pipeline in {self.state}'
            return response
        self.state = self.ABORTING
        if self.cancel_motion.service_is_ready():
            self.cancel_motion.call_async(Trigger.Request())
        if self.abort_supervisor.service_is_ready():
            self.abort_supervisor.call_async(Trigger.Request())
        self._fault('place pipeline aborted by operator')
        response.success = True
        response.message = 'place pipeline abort requested'
        return response

    def reset_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'cannot reset active place pipeline in {self.state}'
            return response
        self.state, self.fault, self.pending_motion = self.IDLE, '', None
        self.expected_motion_operation_id = None
        self.expected_supervisor_operation_id = None
        self.loading_succeeded_at = None
        self.transfer_fallback_used = False
        self.transfer_fallback_reason = ''
        response.success = True
        response.message = 'place pipeline reset to IDLE'
        self.publish_status()
        return response

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'state': self.state, 'fault': self.fault,
            'operation_id': self.operation_id,
            'motion_state': self.motion_status.get('state'),
            'motion_target': self.motion_status.get('target'),
            'supervisor_state': self.supervisor_status.get('state'),
            'place_fallback_used': bool(
                self.supervisor_status.get('place_fallback_used', False)),
            'place_fallback_reason': self.supervisor_status.get(
                'place_fallback_reason', ''),
            'transfer_fallback_used': self.transfer_fallback_used,
            'transfer_fallback_reason': self.transfer_fallback_reason,
            'attached_item_id': self.scene_status.get('attached_item_id', ''),
            'pallet_status': self.pallet_status,
        }, separators=(',', ':'))
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PlacePipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
