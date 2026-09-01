import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PlacePipeline(Node):
    """Move to pre-place, guarded-place, retreat, and return to observation."""

    IDLE = 'IDLE'
    MOVE_PRE_PLACE = 'MOVE_PRE_PLACE'
    CONTACT_PLACE = 'CONTACT_PLACE'
    MOVE_OBSERVATION = 'MOVE_OBSERVATION'
    SUCCEEDED = 'SUCCEEDED'
    FAULT = 'FAULT'
    ABORTING = 'ABORTING'
    ACTIVE = {MOVE_PRE_PLACE, CONTACT_PLACE, MOVE_OBSERVATION, ABORTING}

    def __init__(self):
        super().__init__('place_pipeline')
        self.declare_parameter('motion_timeout_sec', 120.0)
        self.motion_timeout = float(
            self.get_parameter('motion_timeout_sec').value)
        self.state, self.fault, self.pending_motion = self.IDLE, '', None
        self.operation_id = 0
        self.phase_started = None
        self.motion_status = {}
        self.supervisor_status = {}
        self.scene_status = {}
        self.pallet_status = 'UNLOCALIZED'
        self.expected_motion_operation_id = None
        self.expected_supervisor_operation_id = None
        self.status_pub = self.create_publisher(String, '/place_pipeline/status', 10)
        self.create_subscription(
            String, '/motion_coordinator/status', self._motion_status, 10)
        self.create_subscription(
            String, '/pickup_supervisor/status', self._supervisor_status, 10)
        self.create_subscription(
            String, '/planning_scene_obstacles/status', self._scene_status, 10)
        self.create_subscription(
            String, '/pallet_localization/status', self._pallet_status, 10)
        self.plan_pre_place = self.create_client(
            Trigger, '/motion_coordinator/plan_pre_place')
        self.plan_observation = self.create_client(
            Trigger, '/motion_coordinator/plan_observation')
        self.execute_motion = self.create_client(
            Trigger, '/motion_coordinator/execute')
        self.cancel_motion = self.create_client(
            Trigger, '/motion_coordinator/cancel')
        self.start_place = self.create_client(
            Trigger, '/pickup_supervisor/start_place')
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
        if not self.plan_pre_place.service_is_ready():
            return self._reject_start(
                response, 'pre-place planning service is unavailable')
        self.operation_id += 1
        self.fault = ''
        self.state = self.MOVE_PRE_PLACE
        self.pending_motion = 'pre_place'
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        self.expected_supervisor_operation_id = None
        self.phase_started = time.monotonic()
        future = self.plan_pre_place.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done: self._plan_completed(done, 'pre-place'))
        response.success = True
        response.message = 'place pipeline started: planning pre-place'
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
        if self.motion_status.get('state') == 'FAULT' and self.state != self.CONTACT_PLACE:
            self._fault(self.motion_status.get('fault', 'MoveIt motion failed'))
            return
        if self.state == self.MOVE_PRE_PLACE:
            self._tick_pre_place()
        elif self.state == self.CONTACT_PLACE:
            self._tick_contact_place()
        elif self.state == self.MOVE_OBSERVATION:
            self._tick_observation()

    def _tick_pre_place(self):
        state = self.motion_status.get('state')
        operation_matches = (
            self.expected_motion_operation_id is not None and
            int(self.motion_status.get('operation_id', -1)) >=
            self.expected_motion_operation_id)
        if (operation_matches and state == 'PLANNED' and
                self.pending_motion == 'pre_place'):
            self.pending_motion = 'pre_place_executing'
            self._execute()
        elif (operation_matches and state == 'SUCCEEDED' and
              self.motion_status.get('target') == 'pre_place'):
            if not self.start_place.service_is_ready():
                self._fault('guarded place supervisor is unavailable')
                return
            self.state = self.CONTACT_PLACE
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
