import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .pick_place_workflow import pick_and_place


class PickPlacePipeline(Node):
    ACTIVE = {'PICKING', 'PLACING', 'ABORTING'}

    def __init__(self):
        super().__init__('pick_place_pipeline')
        self.declare_parameter('cycle_timeout_sec', 300.0)
        self.declare_parameter('continuous_transport_enabled', True)
        self.continuous_transport = bool(self.get_parameter('continuous_transport_enabled').value)
        self.timeout = float(self.get_parameter('cycle_timeout_sec').value)
        self.state, self.fault, self.operation_id = 'IDLE', '', 0
        self.pick_only = False
        self.started = None
        self.pickup_status, self.place_status = {}, {}
        self.expected_pickup_id = self.expected_place_id = None
        self.status_pub = self.create_publisher(String, '/pick_place_pipeline/status', 10)
        self.create_subscription(String, '/pickup_pipeline/status',
                                 lambda msg: self._status(msg, True), 10)
        self.create_subscription(String, '/place_pipeline/status',
                                 lambda msg: self._status(msg, False), 10)
        self.start_pickup = self.create_client(Trigger, '/pickup_pipeline/start')
        self.start_pickup_hold = self.create_client(Trigger, '/pickup_pipeline/start_for_transport')
        self.abort_pickup = self.create_client(Trigger, '/pickup_pipeline/abort')
        self.reset_pickup = self.create_client(Trigger, '/pickup_pipeline/reset')
        self.start_place = self.create_client(Trigger, '/place_pipeline/start')
        self.start_continuous_place = self.create_client(Trigger, '/place_pipeline/start_continuous')
        self.start_chained_place = self.create_client(Trigger, '/place_pipeline/start_continuous_chained')
        self.abort_place = self.create_client(Trigger, '/place_pipeline/abort')
        self.reset_place = self.create_client(Trigger, '/place_pipeline/reset')
        self.create_service(Trigger, '/pick_place_pipeline/start', self.start)
        self.create_service(Trigger, '/pick_place_pipeline/start_chained', self.start_chained)
        self.create_service(
            Trigger, '/pick_place_pipeline/retry_place', self.retry_place)
        self.create_service(
            Trigger, '/pick_place_pipeline/start_pick_only',
            self.start_pick_only)
        self.create_service(Trigger, '/pick_place_pipeline/abort', self.abort)
        self.create_service(Trigger, '/pick_place_pipeline/reset', self.reset)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_status)

    def _status(self, message, pickup):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            value = {}
        if pickup:
            self.pickup_status = value
        else:
            self.place_status = value

    def start(self, _request, response):
        return self._start(response, pick_only=False)

    def start_chained(self, _request, response):
        return self._start(response, pick_only=False, return_to_observation=False)

    def start_pick_only(self, _request, response):
        return self._start(response, pick_only=True)

    def _start(self, response, pick_only, return_to_observation=True):
        if self.state in self.ACTIVE:
            response.message = f'PickAndPlace already active in {self.state}'
            return response
        self.workflow = pick_and_place('incoming', 'pallet', return_to_observation=return_to_observation)
        self.use_continuous = (getattr(self, 'continuous_transport', False) or
                               not return_to_observation) and not pick_only
        pickup_client = self.start_pickup_hold if self.use_continuous else self.start_pickup
        if not pickup_client.service_is_ready():
            response.message = 'pickup pipeline is unavailable'
            return response
        self.operation_id += 1
        self.pick_only = bool(pick_only)
        self.state, self.fault = 'PICKING', ''
        self.started = time.monotonic()
        self.expected_pickup_id = int(self.pickup_status.get('operation_id', 0)) + 1
        self.expected_place_id = None
        future = pickup_client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._start_done(done, 'pickup'))
        response.success = True
        response.message = (
            'Pick-only cycle started' if self.pick_only else
            'PickAndPlace started')
        return response

    def retry_place(self, _request, response):
        if (self.state != 'FAULT' or
                not self.fault.startswith('KINEMATIC_REJECTED:')):
            response.message = 'retry requires a pre-motion kinematic rejection'
            return response
        client = self._placement_client()
        if not client.service_is_ready():
            response.message = 'place pipeline is unavailable'
            return response
        self.operation_id += 1
        self.state, self.fault = 'PLACING', ''
        self.started = time.monotonic()
        self.expected_place_id = int(self.place_status.get('operation_id', 0)) + 1
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._start_done(done, 'place'))
        response.success = True
        response.message = 'retrying placement of the carried item'
        self.publish_status()
        return response

    def _start_done(self, future, label):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'{label} start failed: {exc}')
            return
        if result is None or not result.success:
            self._fault(f'{label} start rejected: '
                        f'{"no response" if result is None else result.message}')

    def _placement_client(self):
        if not getattr(self, 'use_continuous', False):
            return self.start_place
        workflow = getattr(self, 'workflow', None)
        if workflow is not None and not workflow.return_to_observation:
            return self.start_chained_place
        return self.start_continuous_place

    def tick(self):
        if self.state not in self.ACTIVE or self.state == 'ABORTING':
            return
        if time.monotonic() - self.started > self.timeout:
            self._fault(f'PickAndPlace timed out in {self.state}')
            return
        status = self.pickup_status if self.state == 'PICKING' else self.place_status
        expected = self.expected_pickup_id if self.state == 'PICKING' else self.expected_place_id
        if int(status.get('operation_id', -1)) < int(expected or 0):
            return
        if status.get('state') == 'FAULT':
            self._fault(status.get('fault', f'{self.state.lower()} failed'))
        elif status.get('state') == 'SUCCEEDED' and self.state == 'PICKING':
            if self.pick_only:
                self.state = 'SUCCEEDED'
                return
            client = self._placement_client()
            if not client.service_is_ready():
                self._fault('place pipeline is unavailable')
                return
            self.state = 'PLACING'
            self.expected_place_id = int(self.place_status.get('operation_id', 0)) + 1
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda done: self._start_done(done, 'place'))
        elif status.get('state') == 'SUCCEEDED' and self.state == 'PLACING':
            self.state = 'SUCCEEDED'

    def abort(self, _request, response):
        if self.state not in self.ACTIVE:
            response.message = f'no active PickAndPlace in {self.state}'
            return response
        client = self.abort_pickup if self.state == 'PICKING' else self.abort_place
        self.state = 'ABORTING'
        if client.service_is_ready():
            client.call_async(Trigger.Request())
        self._fault('PickAndPlace aborted by operator')
        response.success, response.message = True, 'PickAndPlace abort requested'
        return response

    def reset(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'cannot reset active PickAndPlace in {self.state}'
            return response
        for client in (self.reset_pickup, self.reset_place):
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        self.state, self.fault = 'IDLE', ''
        self.pick_only = False
        self.expected_pickup_id = self.expected_place_id = None
        response.success, response.message = True, 'PickAndPlace reset to IDLE'
        return response

    def _fault(self, reason):
        self.state, self.fault = 'FAULT', reason
        self.get_logger().error(reason)

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'state': self.state, 'fault': self.fault,
            'operation_id': self.operation_id,
            'pick_only': self.pick_only,
            'pickup_state': self.pickup_status.get('state'),
            'place_state': self.place_status.get('state'),
        }, separators=(',', ':'))
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlacePipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
