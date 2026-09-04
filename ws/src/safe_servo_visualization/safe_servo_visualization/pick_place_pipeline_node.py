import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PickPlacePipeline(Node):
    ACTIVE = {'PICKING', 'PLACING', 'ABORTING'}

    def __init__(self):
        super().__init__('pick_place_pipeline')
        self.declare_parameter('cycle_timeout_sec', 300.0)
        self.timeout = float(self.get_parameter('cycle_timeout_sec').value)
        self.state, self.fault, self.operation_id = 'IDLE', '', 0
        self.started = None
        self.pickup_status, self.place_status = {}, {}
        self.expected_pickup_id = self.expected_place_id = None
        self.status_pub = self.create_publisher(String, '/pick_place_pipeline/status', 10)
        self.create_subscription(String, '/pickup_pipeline/status',
                                 lambda msg: self._status(msg, True), 10)
        self.create_subscription(String, '/place_pipeline/status',
                                 lambda msg: self._status(msg, False), 10)
        self.start_pickup = self.create_client(Trigger, '/pickup_pipeline/start')
        self.abort_pickup = self.create_client(Trigger, '/pickup_pipeline/abort')
        self.reset_pickup = self.create_client(Trigger, '/pickup_pipeline/reset')
        self.start_place = self.create_client(Trigger, '/place_pipeline/start')
        self.abort_place = self.create_client(Trigger, '/place_pipeline/abort')
        self.reset_place = self.create_client(Trigger, '/place_pipeline/reset')
        self.create_service(Trigger, '/pick_place_pipeline/start', self.start)
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
        if self.state in self.ACTIVE:
            response.message = f'PickAndPlace already active in {self.state}'
            return response
        if not self.start_pickup.service_is_ready():
            response.message = 'pickup pipeline is unavailable'
            return response
        self.operation_id += 1
        self.state, self.fault = 'PICKING', ''
        self.started = time.monotonic()
        self.expected_pickup_id = int(self.pickup_status.get('operation_id', 0)) + 1
        self.expected_place_id = None
        future = self.start_pickup.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._start_done(done, 'pickup'))
        response.success, response.message = True, 'PickAndPlace started'
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
            if not self.start_place.service_is_ready():
                self._fault('place pipeline is unavailable')
                return
            self.state = 'PLACING'
            self.expected_place_id = int(self.place_status.get('operation_id', 0)) + 1
            future = self.start_place.call_async(Trigger.Request())
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
