import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class PickupPipeline(Node):
    """Run observation, detection, pre-grasp, and supervised pickup."""

    IDLE = 'IDLE'
    MOVE_OBSERVATION = 'MOVE_OBSERVATION'
    WAIT_DETECTION = 'WAIT_DETECTION'
    MOVE_PREGRASP = 'MOVE_PREGRASP'
    SERVO_PICKUP = 'SERVO_PICKUP'
    SUCCEEDED = 'SUCCEEDED'
    FAULT = 'FAULT'
    ABORTING = 'ABORTING'

    ACTIVE = {
        MOVE_OBSERVATION, WAIT_DETECTION, MOVE_PREGRASP, SERVO_PICKUP, ABORTING,
    }

    def __init__(self):
        super().__init__('pickup_pipeline')
        self.declare_parameter('refined_boxes_topic', '/pointcloud_detection/boxes')
        self.declare_parameter('target_box_id', -1)
        self.declare_parameter('max_detection_age_sec', 0.5)
        self.declare_parameter('detection_timeout_sec', 20.0)
        self.declare_parameter('detection_stable_frames', 3)
        self.declare_parameter('motion_timeout_sec', 120.0)

        def p(name):
            return self.get_parameter(name).value
        self.target_box_id = int(p('target_box_id'))
        self.max_detection_age = float(p('max_detection_age_sec'))
        self.detection_timeout = float(p('detection_timeout_sec'))
        self.detection_stable_frames = max(1, int(p('detection_stable_frames')))
        self.motion_timeout = float(p('motion_timeout_sec'))

        self.state = self.IDLE
        self.fault = ''
        self.operation_id = 0
        self.phase_started = None
        self.motion_status = {}
        self.pickup_status = {}
        self.refined_boxes = {}
        self.stable_detection_count = 0
        self.pending_motion = None
        self.expected_motion_operation_id = None
        self.expected_pickup_operation_id = None

        self.status_pub = self.create_publisher(String, '/pickup_pipeline/status', 10)
        self.create_subscription(
            String, '/motion_coordinator/status', self.motion_status_callback, 10)
        self.create_subscription(
            String, '/pickup_supervisor/status', self.pickup_status_callback, 10)
        self.create_subscription(
            MarkerArray, str(p('refined_boxes_topic')),
            self.refined_boxes_callback, 10)

        self.plan_observation_client = self.create_client(
            Trigger, '/motion_coordinator/plan_observation')
        self.plan_pregrasp_client = self.create_client(
            Trigger, '/motion_coordinator/plan_pregrasp')
        self.execute_motion_client = self.create_client(
            Trigger, '/motion_coordinator/execute')
        self.cancel_motion_client = self.create_client(
            Trigger, '/motion_coordinator/cancel')
        self.reset_motion_client = self.create_client(
            Trigger, '/motion_coordinator/reset')
        self.start_pickup_client = self.create_client(
            Trigger, '/pickup_supervisor/start')
        self.abort_pickup_client = self.create_client(
            Trigger, '/pickup_supervisor/abort')
        self.reset_pickup_client = self.create_client(
            Trigger, '/pickup_supervisor/reset')

        self.create_service(Trigger, '/pickup_pipeline/start', self.start_callback)
        self.create_service(Trigger, '/pickup_pipeline/abort', self.abort_callback)
        self.create_service(Trigger, '/pickup_pipeline/reset', self.reset_callback)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info('pickup pipeline ready')

    def motion_status_callback(self, message):
        try:
            self.motion_status = json.loads(message.data)
        except (TypeError, ValueError):
            self.motion_status = {}

    def pickup_status_callback(self, message):
        try:
            self.pickup_status = json.loads(message.data)
        except (TypeError, ValueError):
            self.pickup_status = {}

    def refined_boxes_callback(self, message):
        boxes = {}
        for marker in message.markers:
            if (marker.ns == 'depth_refined_boxes' and
                    marker.action == Marker.ADD and
                    marker.header.frame_id == 'link_base'):
                boxes[int(marker.id)] = marker
        self.refined_boxes = boxes

    def _set_fault(self, reason):
        if self.state == self.FAULT:
            return
        self.fault = reason
        self.state = self.FAULT
        self.pending_motion = None
        self.get_logger().error(reason)
        self.publish_status()

    def _phase_elapsed(self):
        if self.phase_started is None:
            return 0.0
        return time.monotonic() - self.phase_started

    def _fresh_box(self):
        if self.target_box_id >= 0:
            marker = self.refined_boxes.get(self.target_box_id)
            if marker is None:
                return None
            candidates = [marker]
        elif len(self.refined_boxes) == 1:
            candidates = [next(iter(self.refined_boxes.values()))]
        else:
            return None
        marker = candidates[0]
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(
            marker.header.stamp)).nanoseconds * 1e-9
        if age < -0.05 or age > self.max_detection_age:
            return None
        return marker

    def start_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'pickup pipeline already active in {self.state}'
            return response
        if self.pickup_status.get('state') not in (None, 'IDLE', 'SUCCEEDED', 'FAULT'):
            response.message = (
                f"pickup supervisor is busy in {self.pickup_status.get('state')}")
            return response
        motion_state = self.motion_status.get('state', 'UNKNOWN')
        if motion_state in ('PLANNING', 'EXECUTING', 'CANCELING'):
            response.message = f'motion coordinator is busy in {motion_state}'
            return response
        if not self.plan_observation_client.service_is_ready():
            response.message = 'motion coordinator is unavailable'
            return response

        self.operation_id += 1
        self.fault = ''
        self.stable_detection_count = 0
        self.pending_motion = 'observation'
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        self.expected_pickup_operation_id = None
        self.phase_started = time.monotonic()
        self.state = self.MOVE_OBSERVATION
        future = self.plan_observation_client.call_async(Trigger.Request())
        future.add_done_callback(self._plan_observation_completed)
        response.success = True
        response.message = 'pickup pipeline started: moving to observation'
        return response

    def _plan_observation_completed(self, future):
        if self.state != self.MOVE_OBSERVATION:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'plan observation failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._set_fault(f'plan observation rejected: {message}')
            return

    def _begin_execute(self, label):
        if not self.execute_motion_client.service_is_ready():
            self._set_fault('motion execution service is unavailable')
            return
        self.pending_motion = label
        future = self.execute_motion_client.call_async(Trigger.Request())
        future.add_done_callback(self._execute_completed)

    def _execute_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            if self.state != self.ABORTING:
                self._set_fault(f'motion execution failed: {exc}')
            return
        if result is None or not result.success:
            if self.state != self.ABORTING:
                message = 'no response' if result is None else result.message
                self._set_fault(f'motion execution rejected: {message}')

    def abort_callback(self, _request, response):
        if self.state not in self.ACTIVE:
            response.message = f'no active pickup pipeline in state {self.state}'
            return response
        self.state = self.ABORTING
        self.fault = 'pickup pipeline aborted by operator'
        if self.cancel_motion_client.service_is_ready():
            self.cancel_motion_client.call_async(Trigger.Request())
        if self.abort_pickup_client.service_is_ready():
            self.abort_pickup_client.call_async(Trigger.Request())
        self.state = self.FAULT
        response.success = True
        response.message = 'pickup pipeline abort requested'
        self.publish_status()
        return response

    def reset_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'cannot reset active pipeline in {self.state}'
            return response
        if self.reset_pickup_client.service_is_ready():
            self.reset_pickup_client.call_async(Trigger.Request())
        if self.reset_motion_client.service_is_ready():
            self.reset_motion_client.call_async(Trigger.Request())
        self.state = self.IDLE
        self.fault = ''
        self.pending_motion = None
        self.stable_detection_count = 0
        self.expected_motion_operation_id = None
        self.expected_pickup_operation_id = None
        response.success = True
        response.message = 'pickup pipeline reset to IDLE'
        self.publish_status()
        return response

    def tick(self):
        if self.state == self.MOVE_OBSERVATION:
            self._tick_move_observation()
        elif self.state == self.WAIT_DETECTION:
            self._tick_wait_detection()
        elif self.state == self.MOVE_PREGRASP:
            self._tick_move_pregrasp()
        elif self.state == self.SERVO_PICKUP:
            self._tick_servo_pickup()

    def _tick_move_observation(self):
        motion_state = self.motion_status.get('state')
        if self._phase_elapsed() > self.motion_timeout:
            self._set_fault('timed out moving to observation')
            return
        if motion_state == 'FAULT':
            self._set_fault(
                self.motion_status.get('fault', 'observation motion failed'))
            return
        current_operation = int(self.motion_status.get('operation_id', -1))
        operation_matches = (
            self.expected_motion_operation_id is not None and
            current_operation >= self.expected_motion_operation_id)
        if (operation_matches and motion_state == 'PLANNED' and
                self.pending_motion == 'observation'):
            self._begin_execute('observation')
            return
        if (operation_matches and motion_state == 'SUCCEEDED' and
                self.motion_status.get('target') == 'observation'):
            self.pending_motion = None
            self.phase_started = time.monotonic()
            self.stable_detection_count = 0
            self.state = self.WAIT_DETECTION
            self.publish_status()

    def _tick_wait_detection(self):
        if self._phase_elapsed() > self.detection_timeout:
            self._set_fault('timed out waiting for refined box detection')
            return
        marker = self._fresh_box()
        if marker is None:
            self.stable_detection_count = 0
            return
        self.stable_detection_count += 1
        if self.stable_detection_count < self.detection_stable_frames:
            return
        if not self.plan_pregrasp_client.service_is_ready():
            self._set_fault('pre-grasp planning service is unavailable')
            return
        self.pending_motion = 'pregrasp'
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        self.phase_started = time.monotonic()
        self.state = self.MOVE_PREGRASP
        future = self.plan_pregrasp_client.call_async(Trigger.Request())
        future.add_done_callback(self._plan_pregrasp_completed)
        self.publish_status()

    def _plan_pregrasp_completed(self, future):
        if self.state != self.MOVE_PREGRASP:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'plan pre-grasp failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._set_fault(f'plan pre-grasp rejected: {message}')

    def _tick_move_pregrasp(self):
        motion_state = self.motion_status.get('state')
        if self._phase_elapsed() > self.motion_timeout:
            self._set_fault('timed out moving to pre-grasp')
            return
        if motion_state == 'FAULT':
            self._set_fault(
                self.motion_status.get('fault', 'pre-grasp motion failed'))
            return
        current_operation = int(self.motion_status.get('operation_id', -1))
        operation_matches = (
            self.expected_motion_operation_id is not None and
            current_operation >= self.expected_motion_operation_id)
        if (operation_matches and motion_state == 'PLANNED' and
                self.pending_motion == 'pregrasp'):
            self._begin_execute('pregrasp')
            return
        target = str(self.motion_status.get('target', ''))
        if (operation_matches and motion_state == 'SUCCEEDED' and
                target.startswith('pregrasp_box_')):
            self.pending_motion = None
            if not self.start_pickup_client.service_is_ready():
                self._set_fault('pickup supervisor is unavailable')
                return
            self.phase_started = time.monotonic()
            self.state = self.SERVO_PICKUP
            self.expected_pickup_operation_id = int(
                self.pickup_status.get('operation_id', 0)) + 1
            future = self.start_pickup_client.call_async(Trigger.Request())
            future.add_done_callback(self._start_pickup_completed)
            self.publish_status()

    def _start_pickup_completed(self, future):
        if self.state != self.SERVO_PICKUP:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'pickup supervisor start failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._set_fault(f'pickup supervisor rejected start: {message}')

    def _tick_servo_pickup(self):
        if self._phase_elapsed() > self.motion_timeout:
            self._set_fault('timed out during supervised pickup')
            return
        pickup_state = self.pickup_status.get('state', 'UNKNOWN')
        current_operation = int(
            self.pickup_status.get('operation_id', -1))
        if (self.expected_pickup_operation_id is None or
                current_operation < self.expected_pickup_operation_id):
            return
        if pickup_state == 'FAULT':
            self._set_fault(self.pickup_status.get('fault', 'pickup failed'))
            return
        if pickup_state == 'SUCCEEDED':
            self.state = self.SUCCEEDED
            self.pending_motion = None
            self.publish_status()

    def publish_status(self):
        marker = self._fresh_box()
        box_summary = None
        if marker is not None:
            box_summary = {
                'id': int(marker.id),
                'x_m': float(marker.pose.position.x),
                'y_m': float(marker.pose.position.y),
                'z_m': float(marker.pose.position.z),
                'size_m': [
                    float(marker.scale.x),
                    float(marker.scale.y),
                    float(marker.scale.z),
                ],
            }
        message = String()
        message.data = json.dumps({
            'state': self.state,
            'fault': self.fault,
            'operation_id': self.operation_id,
            'motion_state': self.motion_status.get('state'),
            'motion_target': self.motion_status.get('target'),
            'pickup_state': self.pickup_status.get('state'),
            'stable_detection_count': self.stable_detection_count,
            'detected_box': box_summary,
        }, separators=(',', ':'))
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PickupPipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
