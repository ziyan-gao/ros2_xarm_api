"""Shared prepare/execute/acknowledge handshake for known-item approaches."""
import time
from std_srvs.srv import Trigger


class PickPathClient:
    def __init__(self, node, fault):
        self.node = node
        self.fault = fault
        self.prepare = node.create_client(Trigger, '/motion_coordinator/prepare_pick_waypoints')
        self.start = node.create_client(Trigger, '/pickup_supervisor/start_pick_waypoints')
        self.accept = node.create_client(Trigger, '/motion_coordinator/accept_pick_waypoints')
        self.abort = node.create_client(Trigger, '/pickup_supervisor/abort')
        self.generation = 0
        self.phase = 'idle'

    def reset(self, target_id):
        self.generation += 1
        self.target_id = target_id
        self.started = time.monotonic()
        self.phase = 'prepared'
        self.pending = False
        self.next_ack = 0.

    def cancel(self, *, stop=False):
        running = self.phase in ('executing', 'acknowledging', 'confirmed')
        self.generation += 1
        self.phase = 'idle'
        if stop and running and self.abort.service_is_ready():
            try:
                self.abort.call_async(Trigger.Request())
            except Exception:
                # Preserve the fault transition even if the stop request
                # cannot be delivered. No subsequent motion is authorized.
                pass

    def _fail(self, reason):
        self.cancel(stop=True)
        self.fault(reason)
        return False

    def _call(self, client, *, acknowledgement=False):
        if not client.service_is_ready():
            self.node.get_logger().warning('waiting for pickup-path service', throttle_duration_sec=5.)
            self.started = time.monotonic()
            if client is self.start:
                self.phase = 'prepared'
            return False
        generation = self.generation
        self.pending = True
        def done(future):
            if generation != self.generation or self.phase == 'idle':
                return
            self.pending = False
            try:
                result = future.result()
                if result is None or not result.success:
                    if acknowledgement:
                        # The supervisor topic may reach its two subscribers
                        # at different times. Retry this read-only completion
                        # acknowledgement within a bounded five-second window.
                        self.next_ack = time.monotonic() + .2
                        return
                    raise ValueError('no response' if result is None else result.message)
                if acknowledgement:
                    self.phase = 'confirmed'
            except Exception as exc:
                self._fail(f'pickup-path request failed: {exc}')
        try:
            client.call_async(Trigger.Request()).add_done_callback(done)
        except Exception as exc:
            return self._fail(f'pickup-path request failed: {exc}')
        return False

    def tick(self, motion, supervisor):
        if self.phase in ('idle', 'done'):
            return False
        now = time.monotonic()
        if self.phase in ('prepared', 'acknowledging', 'confirmed'):
            self.started = now  # No trajectory has started; await matching input data.
        if now-self.started > 180:
            return self._fail('pickup-path preparation/execution timed out')
        current = int(motion.get('operation_id', -1))
        if current < self.target_id:
            return False
        if current != self.target_id or motion.get('transfer_context') != 'known_pick':
            return self._fail('pickup-path target changed during execution')
        if motion.get('state') == 'FAULT':
            return self._fail(motion.get('fault', 'pickup-path target failed'))
        if self.phase == 'prepared':
            if motion.get('state') != 'PREPARED':
                return False
            if (supervisor.get('prepared_pick_target_id') != self.target_id or
                    not self.start.service_is_ready()):
                self.node.get_logger().warning('waiting for supervisor pickup-target readiness', throttle_duration_sec=5.)
                return False
            self.supervisor_id = int(supervisor.get('operation_id', 0)) + 1
            self.phase = 'executing'
            return self._call(self.start)
        current = int(supervisor.get('operation_id', -1))
        if current < self.supervisor_id:
            return False
        if current != self.supervisor_id:
            return self._fail('pickup supervisor operation changed during approach')
        if supervisor.get('state') == 'FAULT':
            return self._fail(supervisor.get('fault', 'pickup approach failed'))
        if self.phase in ('acknowledging', 'confirmed') and now-self.ack_started > 5:
            self.node.get_logger().warning('waiting for pickup-path completion acknowledgement', throttle_duration_sec=5.)
            self.ack_started = now
        if supervisor.get('state') != 'SUCCEEDED' or self.pending:
            return False
        if (not supervisor.get('pick_path_completed') or
                supervisor.get('pick_path_target_id') != self.target_id):
            return self._fail('pickup approach completion was not verified')
        if self.phase == 'executing':
            self.phase = 'acknowledging'
            self.ack_started = now
        if self.phase == 'confirmed':
            if motion.get('state') == 'SUCCEEDED':
                self.phase = 'done'
                return True
            return False
        if now-self.ack_started > 5:
            self.node.get_logger().warning('waiting for pickup-path completion acknowledgement', throttle_duration_sec=5.)
            self.ack_started = now
        if now >= self.next_ack:
            return self._call(self.accept, acknowledgement=True)
        return False
