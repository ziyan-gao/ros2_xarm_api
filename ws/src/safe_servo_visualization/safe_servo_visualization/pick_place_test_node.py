"""Deliberate, one-item hardware tests using the production motion executors.

Optional bounded random sequencing, but no MCTS, private robot commands or fault recovery. A
completed step is a checkpoint; an interrupted step requires reconciliation.
"""
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import json
import math
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, String
from std_srvs.srv import Trigger, SetBool
from tf2_ros import Buffer, TransformListener
from xarm_msgs.srv import SetInt16

from packing.real_platform_loading import RealPlatformRandomLoader
from .pick_path_client import PickPathClient
from .random_stable_loading_node import round_down_to_increment, round_up_to_increment
from .pallet_item_record import pallet_record, retrieval_values
from .two_item_test import TwoItemTest


STEPS = ('pack_new', 'unpack', 'pack_slot', 'repack')
IDLE_STATES = ('IDLE', 'SUCCEEDED')


def placed_ids(scene):
    return set(scene.get('placed_item_ids') or []).union(
        scene.get('placed_item_visual_ids') or [])

class PickPlaceTest(TwoItemTest, Node):
    TOPICS = dict(scene='/planning_scene_obstacles/status',
                  pickup='/pickup_pipeline/status', supervisor='/pickup_supervisor/status',
                  motion='/motion_coordinator/status', place='/place_pipeline/status',
                  cycle='/pick_place_pipeline/status', staging='/staging_slots/status',
                  random='/random_stable_loading/status', policy='/policy_loading/status')
    SERVICES = dict(estimate='/pickup_pipeline/estimate_object_info',
                    incoming='/pick_place_pipeline/start_chained',
                    place='/place_pipeline/start_continuous_chained',
                    grasp='/pickup_supervisor/start_for_transport',
                    store='/staging_slots/store_chained', retrieve='/staging_slots/retrieve_chained',
                    abort_cycle='/pick_place_pipeline/abort', abort_pickup='/pickup_pipeline/abort',
                    abort_place='/place_pipeline/abort', abort_supervisor='/pickup_supervisor/abort',
                    abort_motion='/motion_coordinator/cancel', abort_staging='/staging_slots/abort')

    def __init__(self):
        super().__init__('pick_place_test')
        defaults = dict(container_size_mm=[450, 550, 450], clearance_mm=10,
                        clearance_mode='one_sided', seed=0, scan_downscale=1,
                        com_bound_ratio=.1, height_tolerance=0., use_fm=True,
                        vertical_loading_filter_enabled=True,
                        packing_height_resolution_mm=5, transfer_corner_height_m=.47,
                        random_test_max_steps=100, random_test_seed=-1)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda name: self.get_parameter(name).value
        self.random_max_steps = int(p('random_test_max_steps'))
        self.random_config_seed = int(p('random_test_seed'))
        if not 1 <= self.random_max_steps <= 10000:
            raise ValueError('random_test_max_steps must be in [1, 10000]')
        self.random_active = False
        self.two_item_mode = False
        self.test_items = {}
        self.active_item = 0
        self.random_pack_unpack_only = False
        self.random_step_pending = False
        self.random_completed = 0
        self.random_counts = {s: 0 for s in STEPS}
        self.random_message = 'not started'
        self.random_seed = None
        self.random_next_at = 0.
        self.test_slot = 0
        self.container = tuple(p('container_size_mm'))
        self.clearance = max(float(p('transfer_corner_height_m')), self.container[2]/1000+.02)
        self.height_grid = int(p('packing_height_resolution_mm'))
        self.loader = RealPlatformRandomLoader(
            container_size=self.container, **{k: p(k) for k in (
                'clearance_mm', 'clearance_mode', 'seed', 'scan_downscale',
                'com_bound_ratio', 'height_tolerance', 'use_fm',
                'vertical_loading_filter_enabled')})
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.planning = None
        self.status = {}
        self.received = {}
        self.state, self.location, self.step, self.fault = 'IDLE', 'empty', '', ''
        self.record = None
        self.target = None
        self.events = deque(maxlen=16)
        self.generation = 0
        # Retrieval IDs also become int32 RViz marker IDs downstream.
        self.sequence = 1000000 + int(time.time()) % 100000000
        self.phase_started = time.monotonic()
        self.accepted = False
        self.pallet_locked = False
        self.pallet_received = 0.
        self.target_ack = False
        self.last_publish = 0.
        self.expected = None
        self.owner = None
        self.future = None
        self.slot_seen_active = False
        self.scene_before = set()
        self.removed_id = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.test_clients = {k: self.create_client(Trigger, v) for k, v in self.SERVICES.items()}
        self.remove = self.create_client(SetInt16, '/planning_scene_obstacles/remove_placed_item')
        self.pick_path = PickPathClient(self, self.fail)
        self.pub = self.create_publisher(String, '/pick_place_test/status', 10)
        self.target_pub = self.create_publisher(Float64MultiArray, '/random_stable_loading/target', 10)
        self.source_pub = self.create_publisher(Float64MultiArray, '/staging_slots/retrieve_target', 10)
        self.store_selection = self.create_publisher(Int32, '/staging_slots/select_store', 10)
        self.retrieve_selection = self.create_publisher(Int32, '/staging_slots/select_retrieve', 10)
        for key, topic in self.TOPICS.items():
            self.create_subscription(String, topic, lambda m, k=key: self.receive(k, m), 10)
        self.create_subscription(String, '/pallet_localization/status', self.pallet_status, 10)
        self.create_subscription(Float64MultiArray, '/random_stable_loading/target_applied', self.ack, 10)
        self.create_subscription(Float64MultiArray, '/random_stable_loading/target', self.target_seen, 10)
        for step in STEPS:
            self.create_service(Trigger, '/pick_place_test/'+step,
                                lambda req, res, s=step: self.start(s, res))
        self.create_service(Trigger, '/pick_place_test/abort', self.abort)
        self.create_service(Trigger, '/pick_place_test/reset', self.reset)
        self.create_service(Trigger, '/pick_place_test/start_random', self.start_random)
        self.create_service(Trigger, '/pick_place_test/stop_random', self.stop_random)
        self.create_service(SetBool, '/pick_place_test/set_random_pack_unpack_only',
                            self.set_random_pack_unpack_only)
        self.create_service(SetBool, '/pick_place_test/set_two_item_mode', self.set_two_item_mode)
        self.create_timer(.1, self.tick)
        self.create_timer(.5, self.publish_status)

    @property
    def busy(self):
        return self.state not in ('IDLE', 'READY', 'FAULT')

    def receive(self, key, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            self.status[key], self.received[key] = value, time.monotonic()
            if key in ('staging', 'scene'):
                self._record_slot_release()
            if key in ('supervisor', 'scene', 'place', 'cycle'):
                self._record_pallet_release()
        except (TypeError, ValueError):
            pass

    def _record_slot_release(self):
        # Preserve physical inventory even when the subsequent retreat faults.
        # This does not mark the operation successful or authorize new motion.
        if (self.step != 'unpack' or self.location != 'carried' or
                self.state not in ('STORE_SLOT_AND_HANDOFF', 'FAULT') or
                not self.record or not self.slot().get('occupied') or
                self.status.get('scene', {}).get('attached_item_id')):
            return
        self.record['obstacle_id'] = self.slot().get('obstacle_id')
        self.record['size_mm'] = [v*1000 for v in self.slot()['size_m']]
        self.record['slot_id'] = self.test_slot
        self.location = 'slot'
        self._sync_item()

    def pallet_status(self, msg):
        self.pallet_locked = msg.data == 'LOCKED'
        self.pallet_received = time.monotonic()

    def _record_pallet_release(self):
        """Inventory commit on detach + one new scene item, not on retreat success."""
        if (self.step not in ('pack_new', 'pack_slot', 'repack') or
                getattr(self, 'placement_started_sequence', None) != self.sequence or
                self.owner not in ('place', 'cycle') or
                self.state not in ('PICK_AND_PLACE_NEW', 'PLACING', 'CONFIRM_PALLET_ITEM', 'FAULT') or
                not self.target or int(self.target[0]) != self.sequence or
                not self.fresh('scene') or not self.fresh('supervisor')):
            return
        scene = self.status['scene']
        if self.owner in ('place', 'cycle') and (
                not self.fresh(self.owner) or
                self.status[self.owner].get('operation_id') != self.expected):
            return
        if scene.get('attached_item_id'):
            self.placement_attachment_seen_sequence = self.sequence
            return
        if (scene.get('attachment_pending') or
                getattr(self, 'placement_attachment_seen_sequence', None) != self.sequence):
            return
        new = placed_ids(scene)-self.scene_before
        if len(new) != 1:
            return  # Ambiguous inventory must never be guessed.
        first = not self.record or self.record.get('release_sequence') != self.sequence
        if first:
            self.record = pallet_record(self.target)
            self.record['virtual_size_mm'] = list(self.target[9:12])
            self.record.update(obstacle_id=next(iter(new)), release_sequence=self.sequence)
        elif self.record.get('obstacle_id') != next(iter(new)):
            return
        self.location = 'pallet'
        rpy = self.status['supervisor'].get('pallet_release_rpy_rad')
        try:
            rpy = [float(v) for v in rpy] if isinstance(rpy, (list, tuple)) else None
        except (TypeError, ValueError):
            rpy = None
        if (isinstance(rpy, (list, tuple)) and len(rpy) == 3 and
                all(math.isfinite(float(v)) for v in rpy)):
            self.record['release_tcp_rpy_rad'] = list(rpy)
        if first:
            self.get_logger().info('TEST PLACEMENT RECORDED before retreat completion '+json.dumps(self.record))
        self._sync_item()

    def fresh(self, key):
        return time.monotonic()-self.received.get(key, 0.) < 3.

    def slot(self, slot_id=None):
        slot_id = self.test_slot if slot_id is None else slot_id
        return next((s for s in self.status.get('staging', {}).get('slots', [])
                     if s.get('slot') == slot_id), {})

    def check_preconditions(self, step=None, slot_id=None):
        if self.busy:
            return 'test is running: '+self.state
        if step and self.state == 'FAULT':
            return 'fault latched; reconcile the physical item and reset first'
        for key in self.TOPICS:
            if not self.fresh(key):
                return key+' status missing or stale'
        if not self.pallet_locked or time.monotonic()-self.pallet_received > 3:
            return 'locked pallet status missing or stale'
        for key in ('random', 'policy'):
            s = self.status[key]
            if s.get('state') != 'IDLE' or s.get('placed_items', 0) or any(s.get(k) for k in (
                    'continuous_loading_enabled', 'continuous_run_active', 'auto_start_pick_place')):
                return 'reset '+key+' loader and disable its automatic loading first'
        for key in ('motion', 'place', 'cycle', 'staging', 'pickup', 'supervisor'):
            allowed = (*IDLE_STATES, 'OBJECT_INFO_READY', 'AWAITING_GRASP')
            if self.status[key].get('state') not in allowed:
                return key+' is not idle/ready: '+str(self.status[key].get('state'))
        supervisor = self.status['supervisor']
        if supervisor.get('robot_error') != 0 or supervisor.get('ft_recovery_required'):
            return 'robot/force-sensor fault must be resolved first'
        scene = self.status['scene']
        if scene.get('attached_item_id') or scene.get('attachment_pending'):
            return 'test requires an empty gripper; do not clear a physically held item'
        slot = self.slot(slot_id)
        if not slot:
            return 'selected slot status unavailable'
        if step == 'pack_new' and self.location != 'empty':
            return 'test already owns an item'
        if step in ('unpack', 'repack') and self.location != 'pallet':
            return 'test item is not on the pallet'
        if step == 'pack_slot' and self.location != 'slot':
            return 'test item is not in a slot'
        if step == 'pack_slot' and not slot.get('occupied'):
            return 'recorded slot is now empty; reconcile inventory'
        if step == 'pack_slot' and self.record.get('obstacle_id') != slot.get('obstacle_id'):
            return 'slot item record changed; reconcile inventory'
        if step in ('pack_new', 'unpack') and slot.get('occupied'):
            return 'selected slot must be empty'
        if step == 'unpack' and max(self.record['size_mm'][:2]) > 250:
            return 'item cannot fit in 250 x 250 mm staging slot'
        ids = placed_ids(scene)
        allowed_ids = set()
        if self._two_enabled():
            allowed_ids.update(v['record'].get('obstacle_id') for v in self.test_items.values())
        if self.record and self.location in ('slot', 'pallet'):
            allowed_ids.add(self.record.get('obstacle_id'))
        # Other occupied staging slots are allowed, but untracked pallet boxes are not.
        allowed_ids.update(s.get('obstacle_id') for s in self.status['staging'].get('slots', [])
                           if s.get('occupied'))
        if ids-allowed_ids:
            return 'untracked placed items exist; start with an empty pallet'
        if step and self.location == 'pallet' and self.record.get('obstacle_id') not in ids:
            return 'test pallet marker/obstacle disappeared; reconcile inventory'
        return ''

    def phase(self, state):
        self._sync_item()
        if state in ('PICK_AND_PLACE_NEW', 'PLACING'):
            self.placement_started_sequence = self.sequence
        self.state, self.phase_started = state, time.monotonic()
        self.accepted = False
        self.future = None
        self.events.append(dict(step=self.step, phase=state, location=self.location))
        self.get_logger().info(json.dumps(self.events[-1]))
        self.publish_status()

    def start(self, step, response, automatic=False):
        if self.random_active and not automatic:
            response.message = 'random test owns the robot; stop it before manual steps'
            return response
        if step not in STEPS:
            response.message = 'unknown test step'
            return response
        if self._two_enabled() and not automatic:
            response.message = 'two-item mode uses Start random test; disable it on an empty test for manual steps'
            return response
        reason = self.check_preconditions(step)
        if reason:
            response.message = reason
            return response
        self.generation += 1
        self.random_step_pending = automatic
        self.step, self.fault = step, ''
        self.target = None
        self.placement_started_sequence = None
        self.placement_attachment_seen_sequence = None
        self.sequence += 1
        if step == 'pack_new':
            self.phase('ESTIMATING')
            self.call('estimate', 'pickup')
        elif step == 'unpack':
            if max(self.record['size_mm'][:2]) > 250:
                response.message = 'item cannot fit in 250 x 250 mm staging slot'
                return response
            self.begin_pallet_pick()
        else:
            self.plan(self.record['item_id'], self.record['size_mm'])
        response.success = self.state != 'FAULT'
        response.message = self.fault or ('started '+step+'; real robot motion')
        return response

    def call(self, name, owner=None, client=None, request=None):
        client = client or self.test_clients[name]
        self.owner = owner
        self.expected = (int(self.status[owner].get('operation_id', 0))+1
                         if owner and owner != 'staging' else None)
        if not client.service_is_ready():
            self.fail(name+' service unavailable')
            return
        generation, phase = self.generation, self.state
        def done(future):
            if generation != self.generation or self.state != phase:
                return
            try:
                result = future.result()
                if result is None or (hasattr(result, 'ret') and result.ret != 0) or (
                        hasattr(result, 'success') and not result.success):
                    raise ValueError(str(result))
                self.accepted = True
            except Exception as exc:
                self.fail(name+' rejected: '+str(exc))
        try:
            self.future = client.call_async(request or Trigger.Request())
            self.future.add_done_callback(done)
        except Exception as exc:
            self.fail(name+' failed: '+str(exc))

    def completed(self, owner, state='SUCCEEDED'):
        status = self.status[owner]
        current = int(status.get('operation_id', -1))
        if current > self.expected:
            self.fail(owner+' operation replaced by another request')
            return False
        return self.accepted and current == self.expected and status.get('state') == state

    def plan(self, item_id, dimensions):
        self.phase('PLANNING_RANDOM_TARGET')
        excluded = set()
        if self.step == 'repack':
            # Exclude both orientations at the old virtual corner.
            excluded = {(*self.record['virtual_corner_mm'], r) for r in (False, True)}
        options = {}
        if self._two_enabled():
            item_id = self.active_item
            options['placement_filter'] = self._floor_filter()
        self.planning = self.worker.submit(self.loader.plan, item_id=item_id,
                                           dimensions_mm=tuple(dimensions),
                                           excluded_placements=excluded, **options)

    def publish_target(self):
        self.target_pub.publish(Float64MultiArray(data=self.target))
        self.last_publish = time.monotonic()

    def ack(self, msg):
        if self.state != 'WAIT_TARGET_ACK' or not self.target or not msg.data:
            return
        if int(msg.data[0]) != self.sequence:
            return
        if len(msg.data) < 2 or msg.data[1] < .5:
            self.fail('pallet rejected test target: '+str(list(msg.data)))
        else:
            self.target_ack = True

    def target_seen(self, msg):
        if self.busy and self.target is not None and msg.data:
            if list(msg.data) != self.target:
                self.fail('loading target was replaced by another publisher')

    def begin_pallet_pick(self):
        # Validate before removing the source obstacle or issuing motion.
        rpy = self.record.get('release_tcp_rpy_rad')
        if (not isinstance(rpy, (list, tuple)) or len(rpy) != 3 or
                not all(math.isfinite(float(v)) for v in rpy)):
            self.fail('missing recorded release TCP orientation; source item left unchanged')
            return
        self.removed_id = self.record['obstacle_id']
        self.phase('REMOVE_SOURCE_OBSTACLE')
        self.call('remove source obstacle', client=self.remove,
                  request=SetInt16.Request(data=int(self.removed_id.removeprefix('placed_item_'))))

    def begin_pick_path(self):
        tf = self.tf_buffer.lookup_transform('link_base', 'pallet_frame', rclpy.time.Time()).transform
        t, q = tf.translation, tf.rotation
        values = retrieval_values(self.record, self.sequence, [t.x, t.y, t.z],
                                  [q.x, q.y, q.z, q.w], self.clearance)
        self.source_pub.publish(Float64MultiArray(data=values))
        self.phase('PREPARE_PICK_PATH')
        self.pick_path.reset(int(self.status['motion'].get('operation_id', 0))+1)

    def begin_stage(self, store):
        self.phase('SELECT_STORE_SLOT' if store else 'SELECT_RETRIEVE_SLOT')
        (self.store_selection if store else self.retrieve_selection).publish(Int32(data=self.test_slot))
        self.slot_seen_active = False

    def begin_place(self):
        if not self.status['scene'].get('attached_item_id'):
            return  # scene acknowledgement follows successful pickup
        self.placement_attachment_seen_sequence = self.sequence
        self.scene_before = placed_ids(self.status['scene'])
        self.location = 'carried'
        self.phase('PLACING')
        self.call('place', 'place')

    def finish(self, location):
        self.location = location
        self._sync_item()
        self.get_logger().info('TEST CHECKPOINT '+json.dumps(dict(
            step=self.step, location=location, record=self.record)))
        if self.loader.pending is not None:
            self.loader.discard_pending(self.loader.pending.sequence_id)
        if self.random_step_pending:
            self.random_step_pending = False
            self.random_completed += 1
            self.random_counts[self.step] += 1
            self.random_next_at = time.monotonic() + 2.
            if self.random_active:
                self.random_message = 'waiting for next confirmed checkpoint'
            if self.random_completed >= self.random_max_steps:
                self.random_active = False
                self.random_message = 'step limit reached'
            self.get_logger().info('RANDOM TEST RESULT '+json.dumps(dict(
                seed=self.random_seed, completed=self.random_completed,
                step=self.step, slot=self.test_slot, location=location,
                counts=self.random_counts)))
        self.phase('READY')
        if self._two_enabled() and self.step == 'pack_new' and len(self.test_items) == 1:
            self.random_active = False
            self.random_message = 'Item 1 loaded. Present item 2 at the new-item area, then press Start random test.'

    def random_choices(self):
        """Choose operation uniformly, then a free slot uniformly, not one item per slot."""
        if self._two_enabled():
            return self._two_choices()
        choices = {}
        free = sorted({s['slot'] for s in self.status.get('staging', {}).get('slots', [])
                       if type(s.get('slot')) is int and 0 <= s['slot'] < 6
                       and s.get('occupied') is False})
        for step in STEPS:
            if step == 'repack' and self.random_pack_unpack_only:
                continue
            slots = free if step in ('pack_new', 'unpack') else [self.test_slot]
            valid = [slot for slot in slots if not self.check_preconditions(step, slot)]
            if valid:
                choices[step] = valid
        return choices

    def set_random_pack_unpack_only(self, request, response):
        if self.random_active or self.busy:
            response.message = 'stop random scheduling and finish the current step before changing test mode'
            return response
        self.random_pack_unpack_only = bool(request.data)
        response.success = True
        response.message = ('random test: pack/unpack only (no repack)' if request.data
                            else 'random test: pack/unpack/repack')
        self.publish_status()
        return response

    def start_random(self, _request, response):
        if self.random_active or self.busy or self.state == 'FAULT':
            response.message = 'test busy or fault latched; no random run started'
            return response
        if not self.random_choices():
            response.message = 'no valid test operation; check item, inventory and dependency status'
            return response
        self.random_seed = (time.time_ns() if self.random_config_seed < 0 else self.random_config_seed)
        self.random_rng = random.Random(self.random_seed)
        self.random_completed = 0
        self.random_counts = {s: 0 for s in STEPS}
        self.random_active = True
        self.random_next_at = time.monotonic() + 2.
        self.random_message = 'armed; next operation starts in 2 seconds'
        self.get_logger().info(
            f'RANDOM TEST START seed={self.random_seed} limit={self.random_max_steps} '
            f'pack_unpack_only={self.random_pack_unpack_only}')
        self.publish_status()
        response.success, response.message = True, self.random_message
        return response

    def stop_random(self, _request, response):
        self.random_active = False
        self.random_message = 'stopped scheduling; current step may finish'
        self.publish_status()
        response.success, response.message = True, self.random_message
        return response

    def random_tick(self):
        if not self.random_active:
            return
        # Check all common interlocks even during the quiet interval.
        reason = self.check_preconditions()
        if reason or self.state == 'FAULT':
            self.fail('random test interlock: '+(reason or self.fault))
            return
        if time.monotonic() < self.random_next_at:
            return
        choices = self.random_choices()
        if not choices:
            self.fail('random test: no valid next operation; reconcile inventory')
            return
        step = self.random_rng.choice(list(choices))
        selection = self.random_rng.choice(choices[step])
        if self._two_enabled():
            key, slot = selection
            self._select_item(key)
            self.test_slot = slot
        else:
            self.test_slot = selection
        self.random_message = f'operation {self.random_completed+1}: {step}, slot {self.test_slot}'
        self.get_logger().info('RANDOM TEST CHOICE '+json.dumps(dict(
            seed=self.random_seed, number=self.random_completed+1, step=step,
            slot=self.test_slot, available=choices, location=self.location)))
        response = self.start(step, Trigger.Response(), automatic=True)
        if not response.success:
            self.fail('random step rejected: '+response.message)

    def tick(self):
        try:
            if self.busy:
                self._tick()
            else:
                self.random_tick()
        except Exception as exc:
            self.fail(str(exc))

    def _tick(self):
        self._record_slot_release()
        self._record_pallet_release()
        if (self.state == 'RETRIEVE_SLOT' and self.fresh('staging') and
                self.status['staging'].get('state') == 'INSPECTION_DEPTH'):
            # Keep hardware/status fault checks active, but allow the operator
            # to improve lighting for as long as measurement needs.
            self.phase_started = time.monotonic()
        elapsed = time.monotonic()-self.phase_started
        limit = 180 if self.state == 'PLANNING_RANDOM_TARGET' else 240
        if elapsed > limit:
            return self.fail('phase timeout: '+self.state)
        if self.future is not None and not self.accepted and elapsed > 10:
            return self.fail('service acceptance timeout; request will not be retried')
        for key in self.TOPICS:
            if not self.fresh(key):
                return self.fail(key+' status became stale')
        if not self.pallet_locked or time.monotonic()-self.pallet_received > 3:
            return self.fail('pallet lock lost')
        for key in ('random', 'policy'):
            if self.status[key].get('state') != 'IDLE' or any(self.status[key].get(k) for k in (
                    'continuous_loading_enabled', 'continuous_run_active', 'auto_start_pick_place')):
                return self.fail('another loader started: '+key)
        for key in ('pickup', 'supervisor', 'motion', 'place', 'cycle', 'staging'):
            if self.status[key].get('state') == 'FAULT':
                return self.fail(key+': '+str(self.status[key].get('fault')))
        if self.state == 'ESTIMATING' and self.completed('pickup', 'OBJECT_INFO_READY'):
            info = self.status['pickup'].get('corrected_object') or {}
            sizes = [round_down_to_increment(float(info[k])*1000, 5)
                     for k in ('size_x_m', 'size_y_m')]
            sizes.append(round_up_to_increment(float(info['size_z_m'])*1000, self.height_grid))
            if max(sizes[:2]) > 250:
                return self.fail('estimated item exceeds slot footprint; gripper remains open')
            self.plan(int(info['box_id']), sizes)
        elif self.state == 'PLANNING_RANDOM_TARGET' and self.planning.done():
            pending = self.planning.result()
            self.target = list(pending.target_values)
            self.target[0] = float(self.sequence)
            self.target_ack = False
            self.phase('WAIT_TARGET_ACK')
            self.publish_target()
            self.get_logger().info('TEST TARGET '+json.dumps(self.target))
        elif self.state == 'WAIT_TARGET_ACK':
            if not self.target_ack:
                if elapsed > 10:
                    return self.fail('target acknowledgement timeout')
                if time.monotonic()-self.last_publish > .5:
                    self.publish_target()
            elif elapsed >= .5:
                if self.step == 'pack_new':
                    self.scene_before = placed_ids(self.status['scene'])
                    self.phase('PICK_AND_PLACE_NEW')
                    self.call('incoming', 'cycle')
                elif self.step == 'pack_slot':
                    self.begin_stage(False)
                else:
                    self.begin_pallet_pick()
        elif self.state == 'REMOVE_SOURCE_OBSTACLE':
            if self.accepted and self.removed_id not in placed_ids(self.status['scene']):
                self.begin_pick_path()
            elif elapsed > 5:
                self.fail('source obstacle removal timeout')
        elif self.state == 'PREPARE_PICK_PATH' and elapsed > .3:
            self.phase('PICK_PATH')
            self.call('prepare pickup path', client=self.pick_path.prepare)
        elif self.state == 'PICK_PATH' and self.accepted:
            if self.pick_path.tick(self.status['motion'], self.status['supervisor']):
                self.phase('SETTLE_PRE_PICK')
        elif self.state == 'SETTLE_PRE_PICK' and elapsed >= .75:
            self.phase('GRASP_PALLET')
            self.call('grasp', 'supervisor')
        elif self.state == 'GRASP_PALLET' and self.completed('supervisor'):
            if not self.status['scene'].get('attached_item_id'):
                return
            self.location = 'carried'
            if self.step == 'unpack':
                self.begin_stage(True)
            else:
                self.begin_place()
        elif self.state in ('SELECT_STORE_SLOT', 'SELECT_RETRIEVE_SLOT'):
            store = self.state == 'SELECT_STORE_SLOT'
            field = 'selected_store_slot' if store else 'selected_retrieve_slot'
            if elapsed > .3 and self.status['staging'].get(field) == self.test_slot:
                slot = self.slot()
                if (not slot or (store and slot.get('occupied') is not False) or
                        (not store and (not slot.get('occupied') or
                         slot.get('obstacle_id') != self.record.get('obstacle_id')))):
                    return self.fail('selected slot inventory changed before staging command')
                self.phase('STORE_SLOT_AND_HANDOFF' if store else 'RETRIEVE_SLOT')
                self.call('store' if store else 'retrieve', 'staging')
            elif elapsed > 5:
                self.fail('slot selection acknowledgement timeout')
        elif self.state in ('STORE_SLOT_AND_HANDOFF', 'RETRIEVE_SLOT'):
            staging = self.status['staging']
            if staging.get('state') not in (*IDLE_STATES, 'FAULT'):
                self.slot_seen_active = True
            if not self.accepted or not self.slot_seen_active or staging.get('state') != 'SUCCEEDED':
                return
            if self.state == 'STORE_SLOT_AND_HANDOFF':
                if not self.slot().get('occupied') or self.status['scene'].get('attached_item_id'):
                    return
                self.record['obstacle_id'] = self.slot().get('obstacle_id')
                self.record['size_mm'] = [v*1000 for v in self.slot()['size_m']]
                self.record['slot_id'] = self.test_slot
                if staging.get('return_to_observation') is not False:
                    return self.fail('store did not confirm overhead-handoff mode')
                self.finish('slot')
            elif not self.slot().get('occupied'):
                self.begin_place()
        elif ((self.state == 'PICK_AND_PLACE_NEW' and self.completed('cycle')) or
              (self.state == 'PLACING' and self.completed('place'))):
            self.phase('CONFIRM_PALLET_ITEM')
        elif self.state == 'CONFIRM_PALLET_ITEM':
            if (self.location == 'pallet' and self.record and
                    self.record.get('release_sequence') == self.sequence):
                if 'release_tcp_rpy_rad' not in self.record:
                    self.fail('item placed but release orientation missing; pickup/repack blocked')
                    return
                self.finish('pallet')
            elif elapsed > 5:
                self.fail('placement finished but item registration is missing/ambiguous')

    def fail(self, reason):
        self._record_pallet_release()
        self._sync_item()
        self.random_active = False
        self.random_step_pending = False
        self.random_message = 'fault; automatic scheduling disabled'
        if self.state == 'FAULT':
            return
        old = self.state
        self.generation += 1  # late service replies cannot advance the test
        self.pick_path.cancel(stop=True)
        self.fault = f'{self.step}/{old}: {reason}'
        self.phase('FAULT')
        self.get_logger().error(self.fault+'; no automatic retry or release')
        # Cancel coordinators as well as their executors, so no next phase is queued.
        for name in ('abort_cycle', 'abort_pickup', 'abort_place', 'abort_staging',
                     'abort_supervisor', 'abort_motion'):
            client = self.test_clients[name]
            if client.service_is_ready():
                try:
                    client.call_async(Trigger.Request())
                except Exception as exc:
                    self.get_logger().error(name+' could not be delivered: '+str(exc))

    def abort(self, _request, response):
        if self.random_active and not self.busy:
            return self.stop_random(_request, response)
        if not self.busy:
            response.message = 'no active test motion; use the normal recovery controls'
            return response
        self.fail('operator abort; physical location may differ from last checkpoint')
        response.success, response.message = True, 'stop requested, not confirmed; gripper is not released'
        return response

    def reset(self, _request, response):
        if self.random_active:
            response.message = 'stop random testing before reset'
            return response
        reason = self.check_preconditions()
        if reason or (self.planning is not None and not self.planning.done()):
            response.message = reason or 'planning worker is still finishing'
            return response
        other_slots = {s.get('obstacle_id') for s in self.status['staging'].get('slots', [])
                       if s.get('slot') != self.test_slot and s.get('occupied')}
        if self._two_enabled():
            tracked = {v['record'].get('obstacle_id') for v in self.test_items.values()}
            if tracked & (placed_ids(self.status['scene']) | other_slots):
                response.message = 'physically clear both test items and scene/slot records before reset'
                return response
        if placed_ids(self.status['scene'])-other_slots or self.slot().get('occupied'):
            response.message = 'physically clear test item and its scene/slot records before reset'
            return response
        self.loader.reset()
        self.generation += 1
        self.record = self.target = None
        self.test_items = {}
        self.active_item = 0
        self.location, self.step, self.fault = 'empty', '', ''
        self.phase('IDLE')
        response.success, response.message = True, 'test reset; no robot or inventory clear command sent'
        return response

    def publish_status(self):
        reasons = {s: self.check_preconditions(s) for s in STEPS}
        self.pub.publish(String(data=json.dumps(dict(
            state=self.state, step=self.step, location=self.location, fault=self.fault,
            phase_elapsed_sec=round(time.monotonic()-self.phase_started, 1),
            allowed={s: not r for s, r in reasons.items()}, blocked=reasons,
            record=self.record, target=self.target, events=list(self.events),
            downstream={k: v.get('state') for k, v in self.status.items()},
            expected_operation_id=self.expected, owner=self.owner,
            selected_test_slot=self.test_slot, random_active=self.random_active,
            random_pack_unpack_only=self.random_pack_unpack_only,
            two_item_mode=self._two_enabled(),
            test_items=getattr(self, 'test_items', {}),
            active_item=getattr(self, 'active_item', 0),
            random_completed=self.random_completed, random_max_steps=self.random_max_steps,
            random_seed=(None if self.random_seed is None else str(self.random_seed)),
            random_counts=self.random_counts,
            random_message=self.random_message,
            random_start_allowed=(not self.random_active and bool(self.random_choices())),
            container_size_mm=self.container, transfer_corner_height_m=self.clearance,
        ), separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceTest()
    try:
        rclpy.spin(node)
    finally:
        node.worker.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()
