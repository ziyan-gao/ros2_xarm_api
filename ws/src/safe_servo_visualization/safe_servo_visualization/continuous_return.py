"""Post-release return orchestration; execution uses ContinuousTransport checks."""
import math
import time

import yaml
from std_srvs.srv import SetBool


class ContinuousReturn:
    RETURN_DISABLING = 'RETURN_DISABLING'

    def _init_continuous_return(self):
        self.declare_parameter('continuous_return_enabled', True)
        self.declare_parameter('return_clearance_speed_mm_s', 10.0)
        self.return_clearance_speed = float(self.get_parameter('return_clearance_speed_mm_s').value)
        if not math.isfinite(self.return_clearance_speed) or not 0 < self.return_clearance_speed <= 30:
            raise ValueError('return_clearance_speed_mm_s must be in (0, 30]')
        self.return_clearance_pending = False
        self.return_escape_active = False
        self.return_escape_count = 0
        self.declare_parameter('continuous_return_waypoint_file', '/workspace/config/taught_waypoints.yaml')
        self.continuous_return_enabled = bool(self.get_parameter('continuous_return_enabled').value)
        self.return_waypoint_file = str(self.get_parameter('continuous_return_waypoint_file').value)
        self.continuous_return_target_id = None
        self.continuous_return_restoring = False
        self.continuous_return_completed = False
        self.return_staged_fallback_used = False
        self.transport_is_return = False
        self.return_to_observation = True
        self.return_goal_joints = None

    def _should_return_continuously(self):
        return (getattr(self, 'continuous_return_target_id', None) is not None and
                self.continuous_return_target_id == self.motion_status.get('operation_id') and
                not self.post_retreat_fault and
                (self.operation_kind == 'place' or
                 (self.operation_kind == 'loading' and self.loading_contact_fallback)))

    def _begin_continuous_return(self):
        if self.planning_scene_status.get('attached_item_id') or self.state != self.DETACHING:
            self._fault('continuous return requires confirmed release and scene detachment')
            return
        if not self.enable_client.service_is_ready():
            self._fault('cannot confirm Servo disabled before continuous return')
            return
        try:
            self.return_pre_place_z = float(self.motion_status['pre_place_tcp_xyz_m'][2])
            if not math.isfinite(self.return_pre_place_z):
                raise ValueError('non-finite pre-place height')
            raised = getattr(self, 'raised_retreat_pose', None)
            if raised and raised.get('target_id') == self.motion_status.get('operation_id'):
                original = tuple(map(float, self.motion_status['pre_place_tcp_xyz_m']))
                saved = tuple(map(float, raised['original_xyz']))
                xyz = tuple(map(float, raised['xyz']))
                if (len(original) != 3 or len(saved) != 3 or len(xyz) != 3 or
                        not all(math.isfinite(v) for v in (*original, *saved, *xyz)) or
                        max(abs(a-b) for a,b in zip(original, saved)) > 1e-6 or
                        math.hypot(xyz[0]-original[0], xyz[1]-original[1]) > .002 or
                        not original[2] <= xyz[2] <= original[2]+.050+1e-6 or
                        xyz[2] > self.servo_bounds_mm[5]/1000.):
                    raise ValueError('verified raised retreat target changed or exceeds bounds')
                self.return_pre_place_z = xyz[2]
                self.get_logger().info(
                    f'retaining verified raised pre-place for slow retreat: '
                    f'link_tcp Z={xyz[2]:.4f} m (nominal {original[2]:.4f} m)')
        except (KeyError, TypeError, IndexError, ValueError) as exc:
            self._fault(f'continuous return pre-place height unavailable: {exc}')
            return
        self.return_escape_active = False
        self.return_escape_count = 0
        self.return_clearance_pending = True
        self.return_staged_fallback_used = False
        self.continuous_return_target_id = None  # Consume the one-cycle request.
        self.continuous_contact_retreat = False
        self.transport_is_return = True
        self.transport_is_pick = False
        self.continuous_return_restoring = True
        self.return_handoff_started = time.monotonic()
        self.state = self.RETURN_DISABLING
        operation = self.operation_id
        request = SetBool.Request()
        request.data = False
        self.enable_client.call_async(request).add_done_callback(
            lambda future: self._return_servo_disabled(future, operation))

    def _return_servo_disabled(self, future, operation):
        if operation != self.operation_id or self.state != self.RETURN_DISABLING:
            return
        try:
            response = future.result()
            if response is None or not response.success:
                raise ValueError('Servo did not confirm pause')
            # Reuse lifecycle + repeated mode/state confirmation, controller
            # activation, new-joint-sample gate, and post_restore_settle.
            # No trajectory is sent from this service-completion callback.
            # First clear the placed item slowly in direct Cartesian mode.
            # The existing retreat handoff deactivates ROS writers and waits
            # for confirmed exclusive mode-0 ownership before sending motion.
            self.direct_target_z = self.return_pre_place_z
            self.direct_place_stepping = False
            self._begin_direct_retreat()
        except Exception as exc:
            self._fault(f'continuous return handoff failed: {exc}')

    def _return_transport_target(self):
        target = dict(self.motion_status)
        if target.get('transfer_context') != 'staging_store':
            return target
        resolved = getattr(self, 'verified_slot_transfer_target', None)
        if resolved is not None:
            if (resolved.get('operation_id') != target.get('operation_id') or
                    resolved.get('transfer_context') != 'staging_store'):
                raise ValueError('verified slot transfer belongs to a different target')
            target['pre_place_tcp_xyz_m'] = list(resolved['pre_place_tcp_xyz_m'])
            target['transfer_tcp_quaternion_xyzw'] = list(resolved['transfer_tcp_quaternion_xyzw'])
        elif getattr(self, 'transport_slot_yaw_flipped', False):
            raise ValueError('flipped slot transfer has no verified retreat target')
        return target

    def _plan_continuous_return(self):
        self.continuous_return_restoring = False
        try:
            if getattr(self, 'return_escape_active', False):
                self._verify_return_escape()
            if getattr(self, 'return_clearance_pending', False):
                if (self.robot_tcp_xyz is None or self.robot_state_time is None or
                        time.monotonic() - self.robot_state_time > self.status_timeout):
                    raise ValueError('fresh SDK TCP feedback unavailable after slow retreat')
                actual_z = self.robot_tcp_xyz[2] + self.direct_tcp_z_offset
                if not math.isfinite(actual_z):
                    raise ValueError('invalid TCP height after slow retreat')
                if actual_z < self.return_pre_place_z - self.tolerance:
                    raise ValueError('slow retreat did not reach pre-place height')
                self.return_clearance_pending = False
            if self.planning_scene_status.get('attached_item_id'):
                raise ValueError('item is still attached')
            if not self.pallet_locked:
                raise ValueError('pallet pose is not locked')
            self.return_goal_joints = None
            if getattr(self, 'return_to_observation', True):
                with open(self.return_waypoint_file, encoding='utf-8') as stream:
                    observation = yaml.safe_load(stream)['waypoints']['observation']
                positions = dict(zip(observation['joint_names'], observation['positions_rad']))
                self.return_goal_joints = tuple(float(positions[name]) for name in self.arm_joint_names)
                if not all(math.isfinite(q) for q in self.return_goal_joints):
                    raise ValueError('invalid observation joints')
                for name,q in zip(self.arm_joint_names,self.return_goal_joints):
                    lo,hi,_ = self.transport_joint_limits[name]
                    if not lo <= q <= hi:
                        raise ValueError(f'saved observation exceeds {name} limits')
            if not all(c.service_is_ready() for c in (
                    self.transport_fk,self.transport_cartesian,self.state_validity_client)):
                raise ValueError('MoveIt validation services unavailable')
            self.transport_seed = tuple(self.latest_joint_positions)
            # Empty tool: initial vertical retreat is exempt; subsequent
            # lateral/rotating motion retains timed-state collision checks.
            self.transport_scene = None
            self.transport_target = self._return_transport_target()
            # Retain the elevated TCP level recorded for the carried item,
            # rather than lowering the return merely because the tool is empty.
            self.return_clearance_z = float(self.motion_status['transfer_tcp_z_m'])
            if not getattr(self, 'return_to_observation', True):
                self.return_clearance_z = max(self.return_clearance_z,
                                              float(getattr(self, 'transport_high_z', 0.0)))
            self.transport_started = time.monotonic()
            self.transport_descent_time = None
            self.transport_feedback_time = 0.0
            self.transfer_goal_handle = None
            self.state = self.TRANSPORT_PLANNING
            if self.return_goal_joints is None:
                self.transport_target['transport_corner_clearance_z_m'] = self.return_clearance_z
                self._transport_fk_request(self.transport_seed, self._transport_start_fk)
            else:
                self._transport_fk_request(self.return_goal_joints,self._return_observation_fk)
        except Exception as exc:
            self._fault(f'continuous return preparation failed: {exc}')

    def _try_return_ik_escape(self):
        """Only a near-start, forward vertical empty-tool IK failure may recover.

        Called only for diagnostic NO_IK_SOLUTION, never collision, timeout,
        remote path failures, reverse observation planning or execution failures.
        Each 5 mm step reuses the slow retreat's exclusive-control handoff.
        """
        if (not getattr(self, 'transport_is_return', False) or
                self.state != self.TRANSPORT_DIAGNOSING or
                getattr(self, 'return_to_observation', True) or
                getattr(self, 'return_goal_joints', None) is not None or
                not self._return_ik_failure_near_start() or
                self.transfer_goal_handle is not None):
            return False
        try:
            self._staged_return_target()  # Fresh state, release, unchanged joints/target, ceiling.
            if self.planning_scene_status.get('attachment_pending'):
                raise ValueError('attachment pending; release is not unambiguous')
            now = time.monotonic()
            if (self.last_force_time is None or not 0 <= now-self.last_force_time <= self.force_timeout or
                    self.latest_force_z is None or not math.isfinite(self.latest_force_z)):
                raise ValueError('fresh finite force feedback required')
            if not self.enable_client.service_is_ready():
                raise ValueError('Servo pause service unavailable')
            xyz = self._link_tcp_xyz()
            count = getattr(self, 'return_escape_count', 0)
            if count == 0:
                self.return_escape_origin = tuple(xyz)
                self.return_escape_deadline = now + 45.
            if count >= 6 or now >= self.return_escape_deadline:
                raise ValueError('bounded recovery exhausted (30 mm / 6 steps / 45 s)')
            if math.hypot(xyz[0]-self.return_escape_origin[0], xyz[1]-self.return_escape_origin[1]) > .003:
                raise ValueError('empty tool left the verified retreat column')
            target = xyz[2] + .005
            if (target > self.return_escape_origin[2] + .030 + 1e-6 or
                    target > self.return_clearance_z or target > self.servo_bounds_mm[5]/1000.):
                raise ValueError('upward recovery exceeds permitted height')
            self.return_escape_count = count + 1
            self.return_escape_target = target
            self.return_escape_force = self.latest_force_z
            self.return_escape_active = True
            self.return_clearance_pending = True  # Slow speed and upward-only clamp.
            self.continuous_return_restoring = True
            self.return_handoff_started = now
            self.transport_route_generation = getattr(self, 'transport_route_generation', 0) + 1
            self.state = self.RETURN_DISABLING
            operation = self.operation_id
            self.get_logger().warning(
                f'post-release IK recovery step {self.return_escape_count}/6: '
                f'raise 5 mm to link_tcp Z={target:.4f} m, then restore control and replan')
            self.enable_client.call_async(SetBool.Request(data=False)).add_done_callback(
                lambda future: self._return_escape_paused(future, operation))
        except Exception as exc:
            self._fault(f'post-release upward recovery blocked: {exc}')
        return True

    def _return_ik_failure_near_start(self):
        """A fraction alone is insufficient: require a nearby upward probe too."""
        fraction = getattr(self, 'transport_partial_fraction', 1.)
        if not math.isfinite(fraction) or not 0. <= fraction <= .02:
            return False
        if fraction <= 1e-9:
            return True  # Preserve the existing zero-progress recovery.
        probe = getattr(self, 'transport_partial_probe_xyz', None)
        start = getattr(self, 'transport_start_xyz', None)
        if (probe is None or start is None or len(probe) != 3 or len(start) != 3 or
                not all(math.isfinite(v) for v in (*probe, *start)) or
                getattr(self, 'transport_partial_probe_frame', None) != 'link_base'):
            return False
        return (math.hypot(probe[0]-start[0], probe[1]-start[1]) <= .003 and
                0. <= probe[2]-start[2] <= .015+1e-9)

    def _return_escape_paused(self, future, operation):
        if (operation != self.operation_id or self.state != self.RETURN_DISABLING or
                not getattr(self, 'return_escape_active', False)):
            return
        try:
            result = future.result()
            if result is None or not result.success:
                raise ValueError('Servo pause not confirmed')
            self._staged_return_target()  # Recheck after the asynchronous pause.
            if self._return_escape_tick():
                return
            xyz = self._link_tcp_xyz()
            if (abs(xyz[2]-(self.return_escape_target-.005)) > .001 or
                    math.hypot(xyz[0]-self.return_escape_origin[0], xyz[1]-self.return_escape_origin[1]) > .003):
                raise ValueError('TCP moved before upward recovery')
            self.direct_target_z = self.return_escape_target
            self.direct_place_stepping = False
            self._begin_direct_retreat()
        except Exception as exc:
            self._fault(f'post-release recovery handoff blocked: {exc}')

    def _return_escape_tick(self):
        if not getattr(self, 'return_escape_active', False):
            return False
        now = time.monotonic()
        if (now >= self.return_escape_deadline or self.robot_error != 0 or
                self.robot_state_time is None or not 0 <= now-self.robot_state_time <= self.status_timeout or
                not self.pallet_locked or self.planning_scene_status.get('attached_item_id') or
                self.planning_scene_status.get('attachment_pending') or
                self.motion_status.get('operation_id') != self.transport_target.get('operation_id') or
                self.last_force_time is None or not 0 <= now-self.last_force_time <= self.force_timeout or
                self.latest_force_z is None or not math.isfinite(self.latest_force_z) or
                abs(self.latest_force_z-self.return_escape_force) >= self.place_force_threshold):
            self._fault('post-release recovery stopped: timeout, hardware fault or invalid/excess force')
            return True
        return False

    def _verify_return_escape(self):
        if self._return_escape_tick():
            raise ValueError('upward recovery safety guard failed')
        xyz = self._link_tcp_xyz()  # Fresh TF after normal ROS restoration/settle gate.
        if (abs(xyz[2]-self.return_escape_target) > .002 or
                math.hypot(xyz[0]-self.return_escape_origin[0], xyz[1]-self.return_escape_origin[1]) > .003):
            raise ValueError('upward recovery endpoint mismatch')
        self.return_escape_active = False
        self.get_logger().info('post-release upward step verified; replanning from fresh measured joints')

    def _return_observation_fk(self, future):
        try:
            xyz,q = self._transport_pose(future.result())
            self.transport_target.update(pre_place_tcp_xyz_m=list(xyz),
                                         transfer_tcp_quaternion_xyzw=list(q),
                                         transport_corner_clearance_z_m=self.return_clearance_z)
            self._transport_fk_request(self.transport_seed,self._transport_start_fk)
        except Exception as exc:
            self._fault(f'observation FK failed before continuous return: {exc}')

    def _staged_return_target(self):
        """Validate a released, stationary robot before the legacy vertical retreat."""
        now = time.monotonic()
        if (self.robot_error != 0 or self.robot_mode != 1 or
                self.robot_state not in (0, 2) or
                getattr(self, 'ft_recovery_required', False)):
            raise ValueError('robot is not healthy in ROS control mode')
        if any(stamp is None or not 0 <= now - stamp <= self.status_timeout
               for stamp in (self.robot_state_time, self.last_joint_state_time)):
            raise ValueError('fresh robot and joint feedback required')
        if (not self.pallet_locked or self.post_retreat_fault or
                self.planning_scene_status.get('attached_item_id') or
                self.motion_status.get('operation_id') != self.transport_target.get('operation_id')):
            raise ValueError('release, pallet or operation prerequisites changed')
        joints = tuple(self.latest_joint_positions or ())
        if (not joints or len(joints) != len(self.transport_seed) or
                any(not math.isfinite(q) or abs(q - seed) > .01
                    for q, seed in zip(joints, self.transport_seed))):
            raise ValueError('robot moved during return planning')
        actual_z = self.robot_tcp_xyz[2] + self.direct_tcp_z_offset
        target_z = float(self.return_clearance_z)
        if (not all(math.isfinite(z) for z in (actual_z, target_z)) or
                actual_z < self.return_pre_place_z - self.tolerance or
                target_z < actual_z or target_z > self.servo_bounds_mm[5] / 1000.):
            raise ValueError('staged retreat must rise from pre-place to a valid overhead height')
        return target_z

    def _fallback_staged_return(self, reason):
        # Used for missing MoveIt timing or exceeded smooth-retiming budget. Collision,
        # malformed-state, hardware and execution failures still fault.
        if (not getattr(self, 'transport_is_return', False) or
                self.state != self.TRANSPORT_PLANNING or
                getattr(self, 'return_staged_fallback_used', False)):
            return False
        try:
            if self.transfer_goal_handle is not None:
                raise ValueError('a return trajectory goal already exists')
            self.direct_target_z = self._staged_return_target()
            if not self.enable_client.service_is_ready():
                raise ValueError('Servo pause service unavailable')
            self.return_staged_fallback_used = True
            self.continuous_return_target_id = None
            self.continuous_return_restoring = False
            self.continuous_return_completed = False
            self.return_clearance_pending = False
            self.direct_place_stepping = False
            self.return_handoff_started = time.monotonic()
            self.state = self.RETURN_DISABLING  # Invalidate pending transport callbacks.
            self.get_logger().warning(
                f'continuous return rejected ({reason}); falling back to staged '
                'vertical retreat to overhead waypoint, then observation planning')
            operation = self.operation_id
            request = SetBool.Request()
            request.data = False
            self.enable_client.call_async(request).add_done_callback(
                lambda future: self._staged_return_servo_disabled(future, operation))
        except Exception as exc:
            self._fault(f'staged return fallback blocked: {exc}')
        return True

    def _staged_return_servo_disabled(self, future, operation):
        if operation != self.operation_id or self.state != self.RETURN_DISABLING:
            return
        try:
            response = future.result()
            if response is None or not response.success:
                raise ValueError('Servo did not confirm pause')
            self.direct_target_z = self._staged_return_target()
            # Existing controller deactivation -> confirmed mode 0 -> vertical
            # retreat -> ROS restoration and fresh-feedback settling gate.
            # _finish_retreat then reports SUCCEEDED, so PlacePipeline uses its
            # original separate observation plan (completed remains False).
            self._begin_direct_retreat()
        except Exception as exc:
            self._fault(f'staged return handoff failed: {exc}')
