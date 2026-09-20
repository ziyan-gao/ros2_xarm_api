import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def round_dimension_down_m(value_m, increment_mm=5.0):
    """Floor a positive metric dimension to a millimetre increment."""
    value_mm = float(value_m) * 1000.0
    increment_mm = float(increment_mm)
    if (not math.isfinite(value_mm) or value_mm <= 0.0 or
            not math.isfinite(increment_mm) or increment_mm <= 0.0):
        raise ValueError('dimension and rounding increment must be positive')
    rounded_mm = math.floor((value_mm + 1e-9) / increment_mm) * increment_mm
    if rounded_mm <= 0.0:
        raise ValueError('rounded dimension must remain positive')
    return rounded_mm / 1000.0


def summarize_box_samples(samples):
    """Average box samples and report their maximum measurement spreads."""
    if not samples:
        raise ValueError('at least one box sample is required')
    positions = np.asarray([sample[0] for sample in samples], dtype=float)
    quaternions = np.asarray([sample[1] for sample in samples], dtype=float)
    dimensions = np.asarray([sample[2] for sample in samples], dtype=float)
    if (not np.isfinite(positions).all() or
            not np.isfinite(quaternions).all() or
            not np.isfinite(dimensions).all() or
            np.any(dimensions <= 0.0)):
        raise ValueError('box samples contain invalid geometry')
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(norms <= 1e-9):
        raise ValueError('box samples contain an invalid quaternion')
    quaternions = quaternions / norms[:, None]

    mean_position = positions.mean(axis=0)
    position_spread = float(np.max(np.linalg.norm(
        positions - mean_position, axis=1)))
    reference = quaternions[0]
    quaternions[np.sum(quaternions * reference, axis=1) < 0.0] *= -1.0
    values, vectors = np.linalg.eigh(quaternions.T @ quaternions)
    mean_quaternion = vectors[:, int(np.argmax(values))]
    if np.dot(mean_quaternion, reference) < 0.0:
        mean_quaternion *= -1.0
    angular_spread = float(np.max(2.0 * np.arccos(
        np.clip(np.abs(quaternions @ mean_quaternion), 0.0, 1.0))))
    mean_dimensions = dimensions.mean(axis=0)
    dimension_spread = float(np.max(np.abs(
        dimensions - mean_dimensions)))
    return {
        'position': mean_position,
        'quaternion': mean_quaternion,
        'dimensions': mean_dimensions,
        'position_spread_m': position_spread,
        'angular_spread_rad': angular_spread,
        'dimension_spread_m': dimension_spread,
    }


class PickupPipeline(Node):
    """Run observation, detection, pre-grasp, and supervised pickup."""

    IDLE = 'IDLE'
    MOVE_OBSERVATION = 'MOVE_OBSERVATION'
    WAIT_DETECTION = 'WAIT_DETECTION'
    MOVE_PREGRASP = 'MOVE_PREGRASP'
    SERVO_PICKUP = 'SERVO_PICKUP'
    OBJECT_INFO_READY = 'OBJECT_INFO_READY'
    RETURN_OBSERVATION = 'RETURN_OBSERVATION'
    SUCCEEDED = 'SUCCEEDED'
    FAULT = 'FAULT'
    ABORTING = 'ABORTING'

    ACTIVE = {
        MOVE_OBSERVATION, WAIT_DETECTION, MOVE_PREGRASP, SERVO_PICKUP,
        RETURN_OBSERVATION, ABORTING,
    }

    def __init__(self):
        super().__init__('pickup_pipeline')
        self.declare_parameter('refined_boxes_topic', '/pointcloud_detection/boxes')
        self.declare_parameter('target_box_id', -1)
        self.declare_parameter('max_detection_age_sec', 0.5)
        self.declare_parameter('detection_timeout_sec', 20.0)
        self.declare_parameter('detection_stable_frames', 20)
        self.declare_parameter('detection_position_tolerance_m', 0.005)
        self.declare_parameter('detection_angle_tolerance_deg', 2.0)
        self.declare_parameter('detection_dimension_tolerance_m', 0.01)
        self.declare_parameter('stable_box_publish_settle_sec', 0.15)
        self.declare_parameter('xy_dimension_rounding_mm', 5.0)
        self.declare_parameter('motion_timeout_sec', 120.0)
        self.declare_parameter('pregrasp_settle_sec', 0.75)

        def p(name):
            return self.get_parameter(name).value
        self.target_box_id = int(p('target_box_id'))
        self.max_detection_age = float(p('max_detection_age_sec'))
        self.detection_timeout = float(p('detection_timeout_sec'))
        self.detection_stable_frames = max(1, int(p('detection_stable_frames')))
        self.detection_position_tolerance = max(
            0.0005, float(p('detection_position_tolerance_m')))
        self.detection_angle_tolerance = math.radians(max(
            0.1, float(p('detection_angle_tolerance_deg'))))
        self.detection_dimension_tolerance = max(
            0.001, float(p('detection_dimension_tolerance_m')))
        self.stable_box_publish_settle = max(
            0.05, float(p('stable_box_publish_settle_sec')))
        self.xy_dimension_rounding_mm = max(
            0.1, float(p('xy_dimension_rounding_mm')))
        self.motion_timeout = float(p('motion_timeout_sec'))
        self.pregrasp_settle = max(0.0, float(p('pregrasp_settle_sec')))

        self.state = self.IDLE
        self.fault = ''
        self.operation_id = 0
        self.phase_started = None
        self.motion_status = {}
        self.pickup_status = {}
        self.refined_boxes = {}
        self.stable_detection_count = 0
        self.detection_samples = []
        self.detection_sample_id = None
        self.last_detection_stamp = None
        self.detection_spread = None
        self.stable_box_published_at = None
        self.pending_motion = None
        self.expected_motion_operation_id = None
        self.expected_pickup_operation_id = None
        self.pregrasp_succeeded_at = None
        self.estimation_only = False
        self.grasp_requested = False
        self.finalized_result_published = False

        self.status_pub = self.create_publisher(String, '/pickup_pipeline/status', 10)
        self.object_info_pub = self.create_publisher(
            Float64MultiArray, '/object_info_estimation/result', 10)
        self.object_info_marker_pub = self.create_publisher(
            MarkerArray, '/object_info_estimation/markers', 10)
        self.stable_box_pub = self.create_publisher(
            MarkerArray, '/object_info_estimation/stable_boxes', 10)
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
            Trigger, '/pickup_supervisor/start_probe')
        self.grasp_at_contact_client = self.create_client(
            Trigger, '/pickup_supervisor/grasp_at_contact')
        self.grasp_hold_client = self.create_client(
            Trigger, '/pickup_supervisor/grasp_and_hold')
        self.defer_lift = False
        self.clear_object_info_client = self.create_client(
            Trigger, '/pickup_supervisor/clear_object_info')
        self.abort_pickup_client = self.create_client(
            Trigger, '/pickup_supervisor/abort')
        self.retreat_pickup_client = self.create_client(
            Trigger, '/pickup_supervisor/retreat')
        self.reset_pickup_client = self.create_client(
            Trigger, '/pickup_supervisor/reset')

        self.create_service(Trigger, '/pickup_pipeline/start', self.start_callback)
        self.create_service(Trigger, '/pickup_pipeline/start_for_transport', self.start_for_transport)
        self.create_service(
            Trigger, '/pickup_pipeline/estimate_object_info',
            self.estimate_object_info_callback)
        self.create_service(
            Trigger, '/pickup_pipeline/discard_object_info',
            self.discard_object_info_callback)
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
        if (self.state == self.OBJECT_INFO_READY and
                self.pickup_status.get('operation_kind') == 'pick_approach' and
                not self.pickup_status.get('object_info_obtained')):
            # The known-item approach owns departure from the measured box.
            # Only clear the obsolete latch/visual; do not send retreat/reset.
            self.state = self.IDLE
            self.finalized_result_published = False
            self._clear_object_markers()
            self.publish_status()

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
        self.pregrasp_succeeded_at = None
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
        if self.state not in self.ACTIVE:
            self.defer_lift = False
        return self._start(response, estimation_only=False)

    def start_for_transport(self, _request, response):
        if self.state not in self.ACTIVE:
            self.defer_lift = True
        return self._start(response, estimation_only=False)

    def _grasp_client(self):
        return self.grasp_hold_client if getattr(self, 'defer_lift', False) else self.grasp_at_contact_client

    def estimate_object_info_callback(self, _request, response):
        return self._start(response, estimation_only=True)

    def discard_object_info_callback(self, _request, response):
        if self.state != self.OBJECT_INFO_READY:
            response.message = (
                f'object information is not waiting at contact; state={self.state}')
            return response
        if not self.retreat_pickup_client.service_is_ready():
            response.message = 'pickup retreat service is unavailable'
            return response
        self.state = self.RETURN_OBSERVATION
        self.pending_motion = 'contact_retreat'
        self.expected_pickup_operation_id = int(
            self.pickup_status.get('operation_id', 0)) + 1
        self.phase_started = time.monotonic()
        future = self.retreat_pickup_client.call_async(Trigger.Request())
        future.add_done_callback(self._discard_retreat_started)
        response.success = True
        response.message = (
            'object rejected without grasp; retreating and returning to observation')
        self.publish_status()
        return response

    def _start(self, response, estimation_only):
        if self.state in self.ACTIVE:
            response.message = f'pickup pipeline already active in {self.state}'
            return response
        supervisor_state = self.pickup_status.get('state')
        ready_at_contact = (
            supervisor_state == 'AWAITING_GRASP' and
            bool(self.pickup_status.get('object_info_obtained')))
        if supervisor_state not in (None, 'IDLE', 'SUCCEEDED', 'FAULT') and not ready_at_contact:
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
        self.estimation_only = bool(estimation_only)
        self.grasp_requested = False
        self.finalized_result_published = False
        self.stable_detection_count = 0
        self._reset_detection_samples()
        if ready_at_contact:
            if self.estimation_only:
                self.state = self.OBJECT_INFO_READY
                self._publish_finalized_object_info()
                response.success = True
                response.message = 'object information is already ready at contact'
                self.publish_status()
                return response
            if not self._grasp_client().service_is_ready():
                response.message = 'grasp-at-contact service is unavailable'
                return response
            self.pending_motion = None
            self.expected_pickup_operation_id = int(
                self.pickup_status.get('operation_id', 0))
            self.phase_started = time.monotonic()
            self.state = self.SERVO_PICKUP
            self.grasp_requested = True
            future = self._grasp_client().call_async(Trigger.Request())
            future.add_done_callback(self._grasp_at_contact_completed)
            response.success = True
            response.message = 'object information ready; pickup started at contact'
            self.publish_status()
            return response

        self.pending_motion = 'observation'
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        self.expected_pickup_operation_id = None
        self.pregrasp_succeeded_at = None
        self.phase_started = time.monotonic()
        self.state = self.MOVE_OBSERVATION
        future = self.plan_observation_client.call_async(Trigger.Request())
        future.add_done_callback(self._plan_observation_completed)
        response.success = True
        response.message = (
            'object information estimation started: moving to observation'
            if self.estimation_only else
            'pickup pipeline started: estimating object information first')
        return response

    def _grasp_at_contact_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'grasp-at-contact failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._set_fault(f'grasp-at-contact rejected: {message}')

    def _discard_retreat_started(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'contact retreat start failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._set_fault(f'contact retreat was rejected: {message}')

    def _plan_observation_completed(self, future):
        if self.state not in (self.MOVE_OBSERVATION, self.RETURN_OBSERVATION):
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
        self._reset_detection_samples()
        self.expected_motion_operation_id = None
        self.expected_pickup_operation_id = None
        self.pregrasp_succeeded_at = None
        self.estimation_only = False
        self.grasp_requested = False
        self.finalized_result_published = False
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
        elif self.state == self.RETURN_OBSERVATION:
            self._tick_return_observation()

    def _tick_return_observation(self):
        if self._phase_elapsed() > self.motion_timeout:
            self._set_fault('timed out returning from object-information contact')
            return
        if self.pending_motion == 'contact_retreat':
            pickup_state = self.pickup_status.get('state', 'UNKNOWN')
            current_operation = int(
                self.pickup_status.get('operation_id', -1))
            if (self.expected_pickup_operation_id is None or
                    current_operation < self.expected_pickup_operation_id):
                return
            if pickup_state == 'FAULT':
                self._set_fault(
                    self.pickup_status.get('fault', 'contact retreat failed'))
                return
            if pickup_state != 'SUCCEEDED':
                return
            if not self.plan_observation_client.service_is_ready():
                self._set_fault('motion coordinator is unavailable')
                return
            self.pending_motion = 'discard_observation'
            self.expected_motion_operation_id = int(
                self.motion_status.get('operation_id', 0)) + 1
            future = self.plan_observation_client.call_async(Trigger.Request())
            future.add_done_callback(self._plan_observation_completed)
            return

        motion_state = self.motion_status.get('state', 'UNKNOWN')
        current_operation = int(self.motion_status.get('operation_id', -1))
        if (self.expected_motion_operation_id is None or
                current_operation < self.expected_motion_operation_id):
            return
        if motion_state == 'FAULT':
            self._set_fault(
                self.motion_status.get('fault', 'observation motion failed'))
            return
        if (motion_state == 'PLANNED' and
                self.pending_motion == 'discard_observation'):
            self._begin_execute('discard_observation')
            return
        if (motion_state == 'SUCCEEDED' and
                self.motion_status.get('target') == 'observation'):
            self.pending_motion = None
            if self.clear_object_info_client.service_is_ready():
                self.clear_object_info_client.call_async(Trigger.Request())
            self._clear_object_markers()
            self.state = self.SUCCEEDED
            self.publish_status()

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
            self._reset_detection_samples()
            self.state = self.WAIT_DETECTION
            self.publish_status()

    def _tick_wait_detection(self):
        if self._phase_elapsed() > self.detection_timeout:
            self._set_fault('timed out waiting for refined box detection')
            return
        if self.stable_box_published_at is not None:
            if (time.monotonic() - self.stable_box_published_at <
                    self.stable_box_publish_settle):
                return
            self._start_pregrasp_plan()
            return
        marker = self._fresh_box()
        if marker is None:
            return
        stamp = (int(marker.header.stamp.sec), int(marker.header.stamp.nanosec))
        if stamp == self.last_detection_stamp:
            return
        self.last_detection_stamp = stamp
        marker_id = int(marker.id)
        if self.detection_sample_id not in (None, marker_id):
            self._reset_detection_samples()
        self.detection_sample_id = marker_id
        try:
            sample = self._marker_sample(marker)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        self.detection_samples.append(sample)
        if len(self.detection_samples) > self.detection_stable_frames:
            self.detection_samples.pop(0)
        self.stable_detection_count = len(self.detection_samples)
        if self.stable_detection_count < self.detection_stable_frames:
            return
        summary = summarize_box_samples(self.detection_samples)
        self.detection_spread = {
            'position_mm': summary['position_spread_m'] * 1000.0,
            'angle_deg': math.degrees(summary['angular_spread_rad']),
            'dimension_mm': summary['dimension_spread_m'] * 1000.0,
        }
        if (summary['position_spread_m'] > self.detection_position_tolerance or
                summary['angular_spread_rad'] > self.detection_angle_tolerance or
                summary['dimension_spread_m'] >
                self.detection_dimension_tolerance):
            return
        stable_marker = self._averaged_marker(marker, summary)
        message = MarkerArray()
        message.markers = [stable_marker]
        self.stable_box_pub.publish(message)
        self.stable_box_published_at = time.monotonic()
        self.get_logger().info(
            'accepted %d distinct point-cloud box estimates: '
            'position spread=%.1f mm, angle spread=%.1f deg, '
            'dimension spread=%.1f mm' % (
                self.detection_stable_frames,
                self.detection_spread['position_mm'],
                self.detection_spread['angle_deg'],
                self.detection_spread['dimension_mm']))
        self.publish_status()

    def _start_pregrasp_plan(self):
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

    def _reset_detection_samples(self):
        self.stable_detection_count = 0
        self.detection_samples = []
        self.detection_sample_id = None
        self.last_detection_stamp = None
        self.detection_spread = None
        self.stable_box_published_at = None

    @staticmethod
    def _marker_sample(marker):
        position = np.array([
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
        ], dtype=float)
        quaternion = np.array([
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ], dtype=float)
        dimensions = np.array([
            marker.scale.x,
            marker.scale.y,
            marker.scale.z,
        ], dtype=float)
        values = np.concatenate((position, quaternion, dimensions))
        if not np.isfinite(values).all() or np.any(dimensions <= 0.0):
            raise ValueError(
                f'refined box {int(marker.id)} has invalid geometry')
        return position, quaternion, dimensions

    def _averaged_marker(self, source, summary):
        marker = Marker()
        marker.header.frame_id = 'link_base'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'stable_depth_refined_boxes'
        marker.id = int(source.id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        (marker.pose.position.x, marker.pose.position.y,
         marker.pose.position.z) = map(float, summary['position'])
        (marker.pose.orientation.x, marker.pose.orientation.y,
         marker.pose.orientation.z, marker.pose.orientation.w) = map(
             float, summary['quaternion'])
        marker.scale.x, marker.scale.y, marker.scale.z = map(
            float, summary['dimensions'])
        marker.color.r = 0.15
        marker.color.g = 1.0
        marker.color.b = 0.25
        marker.color.a = 0.65
        return marker

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
            if self.pregrasp_succeeded_at is None:
                self.pregrasp_succeeded_at = time.monotonic()
                self.get_logger().info(
                    f'pre-grasp motion succeeded; waiting '
                    f'{self.pregrasp_settle:.2f} s for TCP telemetry to settle')
                self.publish_status()
                return
            if (time.monotonic() - self.pregrasp_succeeded_at <
                    self.pregrasp_settle):
                return
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
        if (pickup_state == 'AWAITING_GRASP' and
                self.pickup_status.get('object_info_obtained')):
            self._publish_finalized_object_info()
            if self.estimation_only:
                self.state = self.OBJECT_INFO_READY
                self.pending_motion = None
                self.publish_status()
                return
            if not self.grasp_requested:
                if not self._grasp_client().service_is_ready():
                    self._set_fault('grasp-at-contact service is unavailable')
                    return
                self.grasp_requested = True
                future = self._grasp_client().call_async(
                    Trigger.Request())
                future.add_done_callback(self._grasp_at_contact_completed)
            return
        if pickup_state == 'SUCCEEDED':
            self.state = self.SUCCEEDED
            self.pending_motion = None
            if self.clear_object_info_client.service_is_ready():
                self.clear_object_info_client.call_async(Trigger.Request())
            self._clear_object_markers()
            self.publish_status()

    def _publish_finalized_object_info(self):
        if self.finalized_result_published:
            return
        info = self._rounded_corrected_object()
        if not isinstance(info, dict):
            return
        try:
            # Dedicated result layout: id, position XYZ, quaternion XYZW,
            # dimensions XYZ.  Yaw from the depth fit is converted here so
            # consumers receive the same 11-value geometry layout used by
            # item localization.
            yaw = float(info['yaw_rad'])
            result = Float64MultiArray()
            result.data = [
                float(info['box_id']),
                float(info['x_m']), float(info['y_m']),
                float(info['center_z_m']),
                0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0),
                float(info['size_x_m']), float(info['size_y_m']),
                float(info['size_z_m']),
            ]
            # Optional reporting-only extension. Preserve the original eleven
            # motion/planning fields and the displayed box shape unchanged.
            measured = self.pickup_status['corrected_object']
            result.data.extend(float(measured[key]) for key in
                               ('size_x_m', 'size_y_m', 'size_z_m'))
        except (KeyError, TypeError, ValueError):
            self._set_fault('contact-corrected object information is incomplete')
            return
        self.object_info_pub.publish(result)
        markers = MarkerArray()
        cube = Marker()
        cube.header.frame_id = 'link_base'
        cube.header.stamp = self.get_clock().now().to_msg()
        cube.ns, cube.id = 'contact_corrected_object', int(info['box_id'])
        cube.type, cube.action = Marker.CUBE, Marker.ADD
        cube.pose.position.x = float(info['x_m'])
        cube.pose.position.y = float(info['y_m'])
        cube.pose.position.z = float(info['center_z_m'])
        cube.pose.orientation.z = math.sin(yaw / 2.0)
        cube.pose.orientation.w = math.cos(yaw / 2.0)
        cube.scale.x = float(info['size_x_m'])
        cube.scale.y = float(info['size_y_m'])
        cube.scale.z = float(info['size_z_m'])
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = (
            0.15, 1.0, 0.25, 0.65)
        label = Marker()
        label.header = cube.header
        label.ns, label.id = 'contact_corrected_object_label', int(info['box_id'])
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x = cube.pose.position.x
        label.pose.position.y = cube.pose.position.y
        label.pose.position.z = float(info['top_z_m']) + 0.05
        label.pose.orientation.w = 1.0
        label.scale.z = 0.035
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = 'ID %d: %.0f x %.0f x %.0f mm' % (
            int(info['box_id']), float(info['size_x_m']) * 1000.0,
            float(info['size_y_m']) * 1000.0,
            float(info['size_z_m']) * 1000.0)
        markers.markers = [cube, label]
        self.object_info_marker_pub.publish(markers)
        self.finalized_result_published = True

    def _rounded_corrected_object(self):
        info = self.pickup_status.get('corrected_object')
        if not isinstance(info, dict):
            return info
        result = dict(info)
        try:
            result['size_x_m'] = round_dimension_down_m(
                result['size_x_m'], self.xy_dimension_rounding_mm)
            result['size_y_m'] = round_dimension_down_m(
                result['size_y_m'], self.xy_dimension_rounding_mm)
        except (KeyError, TypeError, ValueError):
            return info
        return result

    def _clear_object_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers = [clear]
        self.object_info_marker_pub.publish(markers)

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
            'required_detection_samples': self.detection_stable_frames,
            'detection_spread': self.detection_spread,
            'pregrasp_settling': (
                self.state == self.MOVE_PREGRASP and
                self.pregrasp_succeeded_at is not None),
            'estimation_only': self.estimation_only,
            'object_info_obtained': bool(
                self.pickup_status.get('object_info_obtained')),
            'corrected_object': self._rounded_corrected_object(),
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
