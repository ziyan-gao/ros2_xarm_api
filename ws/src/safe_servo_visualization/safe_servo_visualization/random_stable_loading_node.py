import json
import math
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from packing.real_platform_loading import (
    NoStableLoadingPosition,
    RealPlatformRandomLoader,
)
from packing.threejs_visualization import ThreeLiveServer, ThreeVisualizationBuilder
from packing_env.visualization.config import DEFAULT_VISUAL_CONFIG


class RandomStableLoadingNode(Node):
    """Bridge real item dimensions to stable pallet-frame loading targets."""

    ACTIVE_PIPELINE_STATES = {'PICKING', 'PLACING', 'ABORTING'}

    def __init__(self):
        super().__init__('random_stable_loading')
        self.declare_parameter('container_size_mm', [450, 550, 450])
        self.declare_parameter('clearance_mm', 10)
        self.declare_parameter('seed', 101)
        self.declare_parameter('scan_downscale', 2)
        self.declare_parameter('candidate_sample_fraction', 0.3)
        self.declare_parameter('com_bound_ratio', 0.2)
        self.declare_parameter('auto_start_pick_place', False)
        self.declare_parameter('continuous_loading_enabled', False)
        self.declare_parameter('localization_timeout_sec', 30.0)
        self.declare_parameter('target_republish_sec', 0.5)
        self.declare_parameter('visualization_enabled', True)
        self.declare_parameter('visualization_port', 8765)
        self.declare_parameter('visualization_bind_host', '0.0.0.0')
        self.declare_parameter('visualization_public_host', '127.0.0.1')
        self.declare_parameter('visualization_poll_ms', 500)

        container_size = tuple(
            int(value) for value in
            self.get_parameter('container_size_mm').value)
        self.loader = RealPlatformRandomLoader(
            container_size=container_size,
            clearance_mm=int(self.get_parameter('clearance_mm').value),
            seed=int(self.get_parameter('seed').value),
            scan_downscale=int(self.get_parameter('scan_downscale').value),
            candidate_sample_fraction=float(
                self.get_parameter('candidate_sample_fraction').value),
            com_bound_ratio=float(
                self.get_parameter('com_bound_ratio').value),
        )
        self.auto_start = bool(
            self.get_parameter('auto_start_pick_place').value)
        self.continuous_loading_enabled = bool(
            self.get_parameter('continuous_loading_enabled').value)
        self.continuous_run_active = False
        self.target_republish_sec = max(
            0.1, float(self.get_parameter('target_republish_sec').value))
        self.localization_timeout_sec = max(
            1.0, float(self.get_parameter('localization_timeout_sec').value))

        self.state = 'IDLE'
        self.fault = ''
        self.last_result = ''
        self.target_acknowledged = False
        self.pallet_status = 'UNLOCALIZED'
        self.pipeline_status = {}
        self.expected_pipeline_operation_id = None
        self.last_target_publish = 0.0
        self.localization_started = None
        self.cycle_auto_start = False
        self.abort_requested = False
        self.abort_started = None
        self.start_request_pending = False
        self.planning_future = None
        self.planning_worker = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix='stable_loading')
        self.visualization_server = None
        self.visualization_builder = None
        self.visualization_url = ''
        self.visualization_fault = ''

        self.target_pub = self.create_publisher(
            Float64MultiArray, '/random_stable_loading/target', 10)
        self.status_pub = self.create_publisher(
            String, '/random_stable_loading/status', 10)
        self.create_subscription(
            Float64MultiArray, '/item_localization/result',
            self.item_result_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/random_stable_loading/target_applied',
            self.target_applied_callback, 10)
        self.create_subscription(
            String, '/pick_place_pipeline/status',
            self.pipeline_status_callback, 10)
        self.create_subscription(
            String, '/pallet_localization/status',
            self.pallet_status_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/random_stable_loading/config',
            self.config_callback, 10)

        self.pick_place_client = self.create_client(
            Trigger, '/pick_place_pipeline/start')
        self.pick_place_abort_client = self.create_client(
            Trigger, '/pick_place_pipeline/abort')
        self.pick_place_reset_client = self.create_client(
            Trigger, '/pick_place_pipeline/reset')
        self.item_localization_start_client = self.create_client(
            Trigger, '/item_localization/start')
        self.item_localization_clear_client = self.create_client(
            Trigger, '/item_localization/clear')
        self.create_service(
            Trigger, '/random_stable_loading/start',
            self.start_loading_callback)
        self.create_service(
            Trigger, '/random_stable_loading/abort',
            self.abort_loading_callback)
        self.create_service(
            Trigger, '/random_stable_loading/start_pick_place',
            self.start_pick_place_callback)
        self.create_service(
            Trigger, '/random_stable_loading/discard_pending',
            self.discard_pending_callback)
        self.create_service(
            Trigger, '/random_stable_loading/reset_pallet',
            self.reset_pallet_callback)
        self.create_service(
            SetBool, '/random_stable_loading/set_continuous',
            self.set_continuous_callback)

        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_status)
        self._start_visualization()
        self._push_visualization('Real-platform loading: empty pallet')
        self.get_logger().info(
            'random stable loading ready: container=%s mm, clearance=%d mm, '
            'vertical_filter=true, sample_fraction=%.2f, '
            'com_bound_ratio=%.2f, auto_start=%s' % (
                container_size,
                self.loader.clearance_mm,
                self.loader.candidate_sample_fraction,
                self.loader.com_bound_ratio,
                self.auto_start))

    def _start_visualization(self):
        if not bool(self.get_parameter('visualization_enabled').value):
            return
        try:
            self.visualization_builder = ThreeVisualizationBuilder(
                DEFAULT_VISUAL_CONFIG)
            self.visualization_server = ThreeLiveServer(
                plot_dir='/tmp/random_stable_loading_visualization',
                port=int(self.get_parameter('visualization_port').value),
                bind_host=str(
                    self.get_parameter('visualization_bind_host').value),
                public_host=str(
                    self.get_parameter('visualization_public_host').value),
                poll_ms=int(self.get_parameter('visualization_poll_ms').value),
            )
            self.visualization_url = self.visualization_server.start()
            self.get_logger().info(
                f'live stable-loading visualization: {self.visualization_url}')
        except Exception as exc:
            self.visualization_server = None
            self.visualization_builder = None
            self.visualization_fault = str(exc)
            self.get_logger().error(
                f'Three.js visualization is unavailable: {exc}')

    def _push_visualization(self, title):
        if self.visualization_server is None or self.visualization_builder is None:
            return
        pending_item = self.loader.pending_item_for_visualization()
        try:
            frame = self.visualization_builder.build(
                self.loader.env,
                title,
                highlighted_items=(
                    [] if pending_item is None else [pending_item]),
                show_anchor=True,
                show_ems=False,
            )
            self.visualization_server.push(frame)
        except Exception as exc:
            self.visualization_fault = str(exc)
            self.get_logger().error(
                f'failed to update Three.js visualization: {exc}')

    @staticmethod
    def _decode_status(message):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return {}
        return value if isinstance(value, dict) else {}

    def item_result_callback(self, message):
        if len(message.data) < 11:
            self._set_fault(
                'item localization result must contain id, pose, and XYZ dimensions')
            return
        if (self.state not in ('IDLE', 'LOCALIZING', 'WAITING_NEXT_ITEM') or
                self.loader.pending is not None):
            self.get_logger().warning(
                f'ignored new item result while loading state is {self.state}')
            return
        item_id = int(round(message.data[0]))
        dimensions_mm = tuple(float(value) * 1000.0 for value in message.data[8:11])
        if not all(math.isfinite(value) and value > 0.0 for value in dimensions_mm):
            self._set_fault(f'invalid item dimensions: {dimensions_mm}')
            return

        self.state = 'PLANNING'
        self.localization_started = None
        self.fault = ''
        self.last_result = ''
        self.target_acknowledged = False
        self.planning_future = self.planning_worker.submit(
            self.loader.plan,
            item_id=item_id,
            dimensions_mm=dimensions_mm,
        )
        self.get_logger().info(
            'planning stable target for item %d, dimensions=(%d, %d, %d) mm' % (
                item_id, *(int(round(value)) for value in dimensions_mm)))
        self.publish_status()

    def target_applied_callback(self, message):
        pending = self.loader.pending
        if pending is None or not message.data:
            return
        if int(round(message.data[0])) != pending.sequence_id:
            return
        if len(message.data) >= 2 and message.data[1] < 0.5:
            reason_code = int(round(message.data[2])) if len(message.data) >= 3 else 0
            reasons = {
                1: 'target contains invalid values',
                2: 'target exceeds configured pallet X extent',
                3: 'target exceeds configured pallet Y extent',
            }
            self._set_fault(
                'pallet localization rejected loading target: '
                f'{reasons.get(reason_code, "unspecified reason")}')
            return
        self.target_acknowledged = True
        if self.state == 'WAITING_TARGET_ACK':
            self.state = 'TARGET_READY'
            self.get_logger().info(
                f'loading target {pending.sequence_id} applied by pallet localization')
            self.publish_status()

    def pipeline_status_callback(self, message):
        self.pipeline_status = self._decode_status(message)

    def pallet_status_callback(self, message):
        self.pallet_status = message.data

    def config_callback(self, message):
        if not message.data:
            self.get_logger().warning(
                'ignored empty random stable-loading configuration')
            return
        try:
            ratio = float(message.data[0])
            if not math.isfinite(ratio):
                raise ValueError('ratio must be finite')
            if self.state == 'PLANNING':
                self.get_logger().warning(
                    'ignored COM-bound ratio change while candidate planning '
                    'is in progress')
                self.publish_status()
                return
            self.loader.set_com_bound_ratio(ratio)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(
                f'ignored invalid COM-bound ratio configuration: {exc}')
            self.publish_status()
            return
        self.get_logger().info(
            f'Neuromeka COM-bound ratio set to {ratio:.0%}')
        self.publish_status()

    def tick(self):
        if self.planning_future is not None and self.planning_future.done():
            self._finish_planning()
            return

        if self.state == 'LOCALIZING':
            if (self.localization_started is not None and
                    time.monotonic() - self.localization_started >=
                    self.localization_timeout_sec):
                self._clear_item_localization()
                self._set_fault('incoming-item localization timed out')
            return

        if self.state == 'WAITING_NEXT_ITEM':
            # Continuous mode intentionally waits without a timeout. The item
            # localization node only publishes after its refined-box pose and
            # dimensions have passed the configured stability test.
            return

        if self.state == 'PLANNING':
            return

        if self.state in ('WAITING_TARGET_ACK', 'TARGET_READY'):
            self._republish_target_if_due()
            if (self.state == 'TARGET_READY' and
                    (self.auto_start or self.cycle_auto_start) and
                    self.pallet_status == 'LOCKED'):
                self._start_pick_place()
            return

        if self.state == 'ABORTING':
            if self.planning_future is not None or self.start_request_pending:
                return
            pipeline_state = self.pipeline_status.get('state')
            current_operation = int(
                self.pipeline_status.get('operation_id', -1))
            operation_is_current = (
                self.expected_pipeline_operation_id is None or
                current_operation >= self.expected_pipeline_operation_id)
            if operation_is_current and pipeline_state == 'SUCCEEDED':
                self._commit_succeeded_pick_place()
            elif (operation_is_current and
                    pipeline_state not in self.ACTIVE_PIPELINE_STATES and
                    self.abort_started is not None and
                    time.monotonic() - self.abort_started >= 1.0):
                self._finish_abort()
            return

        if self.state != 'EXECUTING':
            return
        current_operation = int(self.pipeline_status.get('operation_id', -1))
        if (self.expected_pipeline_operation_id is None or
                current_operation < self.expected_pipeline_operation_id):
            return
        pipeline_state = self.pipeline_status.get('state')
        if pipeline_state == 'FAULT':
            self._set_fault(
                self.pipeline_status.get('fault', 'PickAndPlace failed'))
        elif pipeline_state == 'SUCCEEDED':
            self._commit_succeeded_pick_place()

    def _finish_planning(self):
        if self.planning_future is None or not self.planning_future.done():
            return
        future = self.planning_future
        self.planning_future = None
        try:
            pending = future.result()
        except (NoStableLoadingPosition, TypeError, ValueError, RuntimeError) as exc:
            if self.abort_requested:
                self._finish_abort()
                return
            self._set_fault(str(exc))
            return
        if self.abort_requested:
            self._finish_abort()
            return
        self.state = 'WAITING_TARGET_ACK'
        self._publish_target(pending)
        self.get_logger().info(
            'selected target %d: stable=%d, vertical=%d, sampled=%d, '
            'corner=(%d, %d, %d) mm, rotate_90=%s, '
            'virtual_dim=(%d, %d, %d) mm' % (
                pending.sequence_id,
                pending.stable_candidate_count,
                pending.vertical_candidate_count,
                pending.sampled_candidate_count,
                pending.placement.flb.x,
                pending.placement.flb.y,
                pending.placement.flb.z,
                pending.placement.rotated,
                pending.virtual_dim.dx,
                pending.virtual_dim.dy,
                pending.virtual_dim.dz,
            ))
        self._push_visualization(
            f'Pending item {pending.item_id}: target {pending.sequence_id}')
        self.publish_status()

    def _publish_target(self, pending):
        message = Float64MultiArray()
        message.data = list(pending.target_values)
        self.target_pub.publish(message)
        self.last_target_publish = time.monotonic()

    def _republish_target_if_due(self):
        pending = self.loader.pending
        if pending is None:
            return
        if time.monotonic() - self.last_target_publish >= self.target_republish_sec:
            self._publish_target(pending)

    def start_loading_callback(self, _request, response):
        if self.state == 'TARGET_READY':
            self.continuous_run_active = self.continuous_loading_enabled
            result = self.start_pick_place_callback(_request, response)
            if result.success:
                self.cycle_auto_start = True
            return result
        if self.state != 'IDLE' or self.loader.pending is not None:
            response.message = (
                f'random loading is not idle; current state is {self.state}')
            return response
        if self.pallet_status != 'LOCKED':
            response.message = 'pallet pose is not LOCKED'
            return response
        pipeline_state = self.pipeline_status.get('state')
        if pipeline_state in self.ACTIVE_PIPELINE_STATES:
            response.message = f'PickAndPlace is already active in {pipeline_state}'
            return response
        if not self.item_localization_start_client.service_is_ready():
            response.message = 'incoming-item localization service is unavailable'
            return response

        self.state = 'LOCALIZING'
        self.fault = ''
        self.last_result = ''
        self.localization_started = time.monotonic()
        self.cycle_auto_start = True
        self.continuous_run_active = self.continuous_loading_enabled
        self.abort_requested = False
        future = self.item_localization_start_client.call_async(Trigger.Request())
        future.add_done_callback(self._localization_start_completed)
        response.success = True
        response.message = 'stable random loading started; measuring incoming item'
        self.publish_status()
        return response

    def _localization_start_completed(self, future):
        if self.state not in ('LOCALIZING', 'WAITING_NEXT_ITEM'):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'incoming-item localization start failed: {exc}')
            return
        if result is None or not result.success:
            reason = 'no response' if result is None else result.message
            self._set_fault(
                f'incoming-item localization start rejected: {reason}')

    def abort_loading_callback(self, _request, response):
        if self.state == 'IDLE' and self.loader.pending is None:
            response.message = 'there is no active stable random loading cycle'
            return response

        self.cycle_auto_start = False
        self.continuous_run_active = False
        self.abort_requested = True
        self.abort_started = time.monotonic()
        self._clear_item_localization()

        pipeline_state = self.pipeline_status.get('state')
        robot_active = (
            self.state in ('STARTING', 'EXECUTING') or
            pipeline_state in self.ACTIVE_PIPELINE_STATES)
        if robot_active:
            if not self.pick_place_abort_client.service_is_ready():
                self.abort_requested = False
                response.message = 'PickAndPlace abort service is unavailable'
                return response
            self.state = 'ABORTING'
            self.pick_place_abort_client.call_async(Trigger.Request())
            response.success = True
            response.message = 'robot abort requested; waiting for motion to stop'
            self.publish_status()
            return response

        if self.state == 'PLANNING' and self.planning_future is not None:
            self.state = 'ABORTING'
            response.success = True
            response.message = 'planning abort requested'
            self.publish_status()
            return response

        self._finish_abort()
        response.success = True
        response.message = self.last_result
        return response

    def _clear_item_localization(self):
        if self.item_localization_clear_client.service_is_ready():
            self.item_localization_clear_client.call_async(Trigger.Request())

    def _finish_abort(self):
        pending = self.loader.pending
        if pending is not None:
            self.loader.discard_pending(pending.sequence_id)
        self.state = 'IDLE'
        self.fault = ''
        self.last_result = 'stable random loading aborted; pending target discarded'
        self.localization_started = None
        self.cycle_auto_start = False
        self.abort_requested = False
        self.abort_started = None
        self.start_request_pending = False
        self.target_acknowledged = False
        self.expected_pipeline_operation_id = None
        self._push_visualization(self.last_result)
        self.publish_status()

    def _commit_succeeded_pick_place(self):
        pending = self.loader.pending
        if pending is None:
            self._set_fault('PickAndPlace succeeded without a pending placement')
            return
        try:
            placed = self.loader.commit(pending.sequence_id)
        except Exception as exc:
            self._set_fault(
                f'robot succeeded but packing-state commit failed: {exc}')
            return
        self.last_result = (
            f'committed item {pending.item_id} at '
            f'({placed.FLB.x}, {placed.FLB.y}, {placed.FLB.z}) mm')
        continue_loading = (
            self.continuous_loading_enabled and self.continuous_run_active)
        self.state = 'IDLE'
        self.fault = ''
        self.localization_started = None
        self.cycle_auto_start = False
        self.abort_requested = False
        self.abort_started = None
        self.start_request_pending = False
        self.expected_pipeline_operation_id = None
        self.target_acknowledged = False
        self.get_logger().info(self.last_result)
        self._push_visualization(self.last_result)
        if continue_loading:
            self._start_continuous_localization()
        else:
            self.continuous_run_active = False
            self.publish_status()

    def _start_continuous_localization(self):
        if self.pallet_status != 'LOCKED':
            self._set_fault(
                'continuous loading stopped: pallet pose is not LOCKED')
            return
        if not self.item_localization_start_client.service_is_ready():
            self._set_fault(
                'continuous loading stopped: incoming-item localization '
                'service is unavailable')
            return
        self.state = 'WAITING_NEXT_ITEM'
        self.fault = ''
        self.localization_started = None
        self.cycle_auto_start = True
        self.abort_requested = False
        future = self.item_localization_start_client.call_async(
            Trigger.Request())
        future.add_done_callback(self._localization_start_completed)
        self.get_logger().info(
            'observation pose reached; waiting for the next stable refined item')
        self.publish_status()

    def set_continuous_callback(self, request, response):
        self.continuous_loading_enabled = bool(request.data)
        if self.continuous_loading_enabled:
            if (self.state not in ('IDLE', 'FAULT') or
                    self.loader.pending is not None):
                self.continuous_run_active = True
            response.message = (
                'continuous stable random loading enabled; press Random '
                'loading to start')
        else:
            self.continuous_run_active = False
            if self.state == 'WAITING_NEXT_ITEM':
                self._clear_item_localization()
                self.state = 'IDLE'
                self.localization_started = None
                self.cycle_auto_start = False
                self.last_result = (
                    'continuous loading disabled while waiting for next item')
            response.message = 'continuous stable random loading disabled'
        response.success = True
        self.publish_status()
        return response

    def start_pick_place_callback(self, _request, response):
        if self.state != 'TARGET_READY':
            response.message = f'loading target is not ready; current state is {self.state}'
            return response
        if self.pallet_status != 'LOCKED':
            response.message = 'pallet pose is not LOCKED'
            return response
        return self._start_pick_place(response)

    def _start_pick_place(self, response=None):
        if not self.pick_place_client.service_is_ready():
            if response is not None:
                response.message = 'PickAndPlace service is unavailable'
            return response
        pipeline_state = self.pipeline_status.get('state')
        if pipeline_state in self.ACTIVE_PIPELINE_STATES:
            if response is not None:
                response.message = f'PickAndPlace is already active in {pipeline_state}'
            return response

        self.expected_pipeline_operation_id = int(
            self.pipeline_status.get('operation_id', 0)) + 1
        self.state = 'STARTING'
        self.start_request_pending = True
        future = self.pick_place_client.call_async(Trigger.Request())
        future.add_done_callback(self._start_completed)
        if response is not None:
            response.success = True
            response.message = 'PickAndPlace start requested'
        self.publish_status()
        return response

    def _start_completed(self, future):
        self.start_request_pending = False
        try:
            result = future.result()
        except Exception as exc:
            if self.state == 'ABORTING':
                self.expected_pipeline_operation_id = None
                self._finish_abort()
                return
            self._set_fault(f'PickAndPlace start failed: {exc}')
            return
        if self.state == 'ABORTING':
            if (result is not None and result.success and
                    self.pick_place_abort_client.service_is_ready()):
                self.pick_place_abort_client.call_async(Trigger.Request())
            elif result is None or not result.success:
                self.expected_pipeline_operation_id = None
                self._finish_abort()
            return
        if self.state != 'STARTING':
            return
        if result is None or not result.success:
            reason = 'no response' if result is None else result.message
            self._set_fault(f'PickAndPlace start rejected: {reason}')
            return
        self.state = 'EXECUTING'
        self.publish_status()

    def discard_pending_callback(self, _request, response):
        if self.state in (
                'LOCALIZING', 'PLANNING', 'STARTING', 'EXECUTING', 'ABORTING'):
            response.message = f'cannot discard a target in state {self.state}'
            return response
        pending = self.loader.pending
        if pending is None:
            response.message = 'there is no pending loading target'
            return response
        self.loader.discard_pending(pending.sequence_id)
        self.state = 'IDLE'
        self.fault = ''
        self.cycle_auto_start = False
        self.continuous_run_active = False
        self.abort_requested = False
        self.abort_started = None
        self.target_acknowledged = False
        response.success = True
        response.message = f'discarded loading target {pending.sequence_id}'
        self._push_visualization(response.message)
        self.publish_status()
        return response

    def reset_pallet_callback(self, _request, response):
        if self.state in ('LOCALIZING', 'PLANNING', 'STARTING', 'EXECUTING', 'ABORTING'):
            response.message = f'cannot reset pallet state in {self.state}'
            return response
        if self.pipeline_status.get('state') in self.ACTIVE_PIPELINE_STATES:
            response.message = 'cannot reset while PickAndPlace is active'
            return response
        self._clear_item_localization()
        if self.pick_place_reset_client.service_is_ready():
            self.pick_place_reset_client.call_async(Trigger.Request())
        self.loader.reset()
        self.state = 'IDLE'
        self.fault = ''
        self.last_result = 'virtual pallet state cleared by operator'
        self.localization_started = None
        self.cycle_auto_start = False
        self.continuous_run_active = False
        self.abort_requested = False
        self.abort_started = None
        self.start_request_pending = False
        self.target_acknowledged = False
        self.expected_pipeline_operation_id = None
        response.success = True
        response.message = self.last_result
        self._push_visualization(self.last_result)
        self.publish_status()
        return response

    def _set_fault(self, reason):
        self.state = 'FAULT'
        self.fault = str(reason)
        self.localization_started = None
        self.cycle_auto_start = False
        self.continuous_run_active = False
        self.abort_requested = False
        self.abort_started = None
        self.start_request_pending = False
        self.get_logger().error(self.fault)
        self.publish_status()

    def publish_status(self):
        pending = self.loader.pending
        payload = {
            'state': self.state,
            'fault': self.fault,
            'last_result': self.last_result,
            'pallet_status': self.pallet_status,
            'auto_start_pick_place': self.auto_start,
            'continuous_loading_enabled': self.continuous_loading_enabled,
            'continuous_run_active': self.continuous_run_active,
            'selection_pipeline': (
                'stable_then_vertical_then_random_subset_then_min_xyz_sum'),
            'candidate_sample_fraction': (
                self.loader.candidate_sample_fraction),
            'com_bound_ratio': self.loader.com_bound_ratio,
            'placed_items': len(self.loader.env.container.placed_items),
            'utilization': self.loader.env.container.utilization,
            'visualization_url': self.visualization_url or None,
            'visualization_fault': self.visualization_fault,
            'pending_sequence_id': (
                None if pending is None else pending.sequence_id),
            'pending_item_id': None if pending is None else pending.item_id,
            'target_acknowledged': self.target_acknowledged,
        }
        if pending is not None:
            payload['target_corner_mm'] = [
                pending.placement.flb.x,
                pending.placement.flb.y,
                pending.placement.flb.z,
            ]
            payload['rotate_item_90_deg'] = pending.placement.rotated
            payload['raw_dimension_mm'] = list(map(int, pending.raw_dim.raw()))
            payload['virtual_dimension_mm'] = list(
                map(int, pending.virtual_dim.raw()))
            payload['stable_candidate_count'] = (
                pending.stable_candidate_count)
            payload['vertical_candidate_count'] = (
                pending.vertical_candidate_count)
            payload['sampled_candidate_count'] = (
                pending.sampled_candidate_count)
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self.status_pub.publish(message)

    def destroy_node(self):
        self.planning_worker.shutdown(wait=False, cancel_futures=True)
        if self.visualization_server is not None:
            self.visualization_server.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RandomStableLoadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
