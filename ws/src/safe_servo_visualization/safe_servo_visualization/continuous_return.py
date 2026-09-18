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
        self.declare_parameter('continuous_return_waypoint_file', '/workspace/config/taught_waypoints.yaml')
        self.continuous_return_enabled = bool(self.get_parameter('continuous_return_enabled').value)
        self.return_waypoint_file = str(self.get_parameter('continuous_return_waypoint_file').value)
        self.continuous_return_target_id = None
        self.continuous_return_restoring = False
        self.continuous_return_completed = False
        self.return_staged_fallback_used = False
        self.transport_is_return = False

    def _should_return_continuously(self):
        return (getattr(self, 'continuous_return_target_id', None) is not None and
                self.continuous_return_target_id == self.motion_status.get('operation_id') and
                not self.staging_place_active and not self.post_retreat_fault and
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
        except (KeyError, TypeError, IndexError, ValueError) as exc:
            self._fault(f'continuous return pre-place height unavailable: {exc}')
            return
        self.return_clearance_pending = True
        self.return_staged_fallback_used = False
        self.continuous_return_target_id = None  # Consume the one-cycle request.
        self.continuous_contact_retreat = False
        self.transport_is_return = True
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

    def _plan_continuous_return(self):
        self.continuous_return_restoring = False
        try:
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
            self.transport_scene = None  # Empty tool; robot geometry stays collision checked.
            self.transport_target = dict(self.motion_status)
            # Retain the elevated TCP level recorded for the carried item,
            # rather than lowering the return merely because the tool is empty.
            self.return_clearance_z = float(self.motion_status['transfer_tcp_z_m'])
            self.transport_started = time.monotonic()
            self.transport_descent_time = None
            self.transport_feedback_time = 0.0
            self.transfer_goal_handle = None
            self.state = self.TRANSPORT_PLANNING
            self._transport_fk_request(self.return_goal_joints,self._return_observation_fk)
        except Exception as exc:
            self._fault(f'continuous return preparation failed: {exc}')

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
