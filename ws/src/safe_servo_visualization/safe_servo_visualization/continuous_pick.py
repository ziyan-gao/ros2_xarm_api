"""Known-item empty-tool approach using the shared transport executor."""
import time
import math
import yaml
from .transport_alternatives import waypoint_candidates
from .waypoint_search import GRID_SECONDS, next_grid

from std_srvs.srv import SetBool
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose


class ContinuousPick:
    PICK_PATH_DISABLING = 'PICK_PATH_DISABLING'

    def _check_pick_descent_before_execution(self):
        """Plan-only probe from the exact approach joint branch to contact."""
        snapshot = self.transport_target.get('planned_pregrasp') or {}
        if not getattr(self, 'transport_is_pick', False) or snapshot.get('inspection_only'):
            return False
        if snapshot.get('pickup_source') not in ('pallet', 'buffer'):
            return False
        key = (self.operation_id, getattr(self, 'transport_route_generation', 0),
               id(self.transport_trajectory))
        if getattr(self, '_pick_descent_verified', None) == key:
            return False
        if getattr(self, '_pick_descent_pending', None) == key:
            return True
        try:
            z = float(snapshot['top_z_m'])
            if not math.isfinite(z):
                raise ValueError('invalid contact height')
            req = GetCartesianPath.Request()
            req.header.frame_id = 'link_base'
            req.group_name, req.link_name = self.planning_group, self.ik_link_name
            req.start_state.is_diff = True
            req.start_state.joint_state.name = list(self.arm_joint_names)
            req.start_state.joint_state.position = list(self.transport_trajectory.points[-1].positions)
            pose = Pose()
            pose.position.x, pose.position.y = map(float, self.transport_target['pre_place_tcp_xyz_m'][:2])
            pose.position.z = z
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) = map(
                float, self.transport_target['transfer_tcp_quaternion_xyzw'])
            req.waypoints = [pose]
            req.max_step = .005
            # Kinematic-only probe into intentional object contact. Never sent
            # to execution; the approach retains its full collision checks.
            req.avoid_collisions = False
            req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = .1
            self._pick_descent_pending = key
            self.transport_cartesian.call_async(req).add_done_callback(
                self._transport_guard(lambda f: self._pick_descent_result(f, key)))
        except Exception as exc:
            self._fault(f'pickup descent preflight failed: {exc}')
        return True

    def _pick_descent_result(self, future, key):
        if getattr(self, '_pick_descent_pending', None) != key:
            return
        self._pick_descent_pending = None
        try:
            result = future.result()
            trajectory = result.solution.joint_trajectory
            if result.error_code.val != 1 or result.fraction < 1.-1e-6 or not trajectory.points:
                raise ValueError(f'contact descent only {result.fraction:.1%} feasible')
            if list(trajectory.joint_names) != list(self.arm_joint_names):
                raise ValueError('contact descent joint order mismatch')
            for point in trajectory.points:
                if len(point.positions) != len(self.arm_joint_names):
                    raise ValueError('incomplete contact descent joints')
                for name, value in zip(self.arm_joint_names, point.positions):
                    lo, hi, _ = self.transport_joint_limits[name]
                    if not math.isfinite(value) or not lo <= value <= hi:
                        raise ValueError(f'contact descent exceeds {name} limits')
            self._pick_descent_verified = key
            self.get_logger().info('pickup descent kinematic preflight passed; contact still requires force-controlled Servo')
            self._transport_execute()
        except Exception as exc:
            if self._try_pick_grid():
                self.get_logger().warning(f'pickup descent preflight rejected branch: {exc}; trying next waypoint candidate')
                return
            self._fault(f'KINEMATIC_REJECTED: pickup descent preflight: {exc}; no approach executed')

    def _validate_live_pick_recovery(self, check_servo_force=True):
        now = time.monotonic()
        for label, stamp, timeout in (
                ('Servo status', self.servo_status_time, self.status_timeout),
                ('force', self.last_force_time, self.force_timeout),
                ('robot state', self.robot_state_time, self.status_timeout)):
            if stamp is None or not math.isfinite(stamp) or not 0 <= now-stamp <= timeout:
                raise ValueError(f'{label} telemetry stale: timestamp={stamp}, timeout={timeout}s')
        generation = int(self.servo_status.get('enable_generation', -1))
        if self.expected_enable_generation is None or generation < self.expected_enable_generation:
            raise ValueError(f'wrong Servo generation: actual={generation}, expected={self.expected_enable_generation}')
        if self.robot_error != 0 or self.robot_mode != 1 or self.robot_state not in (0, 1, 2):
            raise ValueError(f'robot not healthy: error={self.robot_error}, mode={self.robot_mode}, state={self.robot_state}')
        if self.planning_scene_status.get('attached_item_id'):
            raise ValueError(f'unexpected attached item: {self.planning_scene_status["attached_item_id"]}')
        source = self.active_pickup_snapshot
        current = self._tcp_xyz()
        if (not all(math.isfinite(v) for v in (*current, self.floor_z, self.pregrasp_z)) or
                current[2] <= self.floor_z + self.tolerance or
                current[2] > self.pregrasp_z + .002 or
                math.hypot(current[0]-source['x_m'], current[1]-source['y_m']) > self.xy_tolerance):
            raise ValueError('TCP outside verified pickup column or remaining descent budget')
        if check_servo_force:
            delta = float(self.servo_status.get('force_delta_z_n', float('nan')))
            if not math.isfinite(delta) or abs(delta) >= self.force_threshold:
                raise ValueError(f'contact/load present: delta_fz={delta}; cannot rebaseline recovery')

    def _try_live_pick_singularity_recovery(self, reason):
        """Only a current, verified known-source pickup may switch to steps."""
        source = getattr(self, 'active_pickup_snapshot', None) or {}
        if (self.operation_kind != 'pickup' or
                getattr(self, 'probe_only', False) or getattr(self, 'dry_run', False) or
                source.get('pickup_source') not in ('pallet', 'buffer') or
                'retrieval_target_id' not in source or
                not self._is_servo_singularity_fault(reason)):
            return False
        try:
            self._validate_live_pick_recovery()
        except (ValueError, TypeError, KeyError) as exc:
            self._fault(f'pickup singularity recovery blocked: {exc}; suction remains off')
            return True
        self.raised_pick_active = True
        self.raised_pick_contact_count = 0
        self.raised_place_hold_on_failure = True
        self.live_pick_pause_gate = dict(operation=self.operation_id, source=dict(source),
                                        deadline=time.monotonic()+5., acknowledged=None,
                                        stable_since=None, joints=None, sample=None)
        self._begin_place_singularity_fallback(reason)
        return True

    def _live_pick_pause_tick(self):
        gate = getattr(self, 'live_pick_pause_gate', None)
        if gate is None:
            return False
        if self.operation_id != gate['operation'] or self.state != self.DISABLING_SERVO:
            self.live_pick_pause_gate = None
            return False
        try:
            now = time.monotonic()
            if now >= gate['deadline']:
                raise ValueError(f'pause/stopped-feedback timeout: pause_ack={gate["acknowledged"]}, robot_state={self.robot_state}')
            # The bridge clears its force delta when disabled. Use the raw
            # force baseline captured BEFORE pause instead of that cleared field.
            self._validate_live_pick_recovery(check_servo_force=False)
            if self.active_pickup_snapshot != gate['source']:
                raise ValueError('pickup source changed while pausing Servo')
            # Preserve the original force baseline across the pause.
            delta = self._direct_place_force_delta()
            if delta is None or not math.isfinite(delta) or delta >= self.force_threshold:
                raise ValueError(f'contact/load developed during pause: delta_fz={delta}')
            ack = gate['acknowledged']
            if ack is None:
                return True
            stamp = self.last_joint_state_time
            if (stamp is None or not math.isfinite(stamp) or
                    not 0 <= now-stamp <= self.status_timeout):
                raise ValueError('joint telemetry stale while confirming stopped motion')
            if stamp <= ack or self.robot_state_time <= ack or self.last_force_time <= ack:
                return True
            if self.robot_state not in (0, 2):
                gate['stable_since'], gate['joints'] = None, None
                return True
            joints = tuple(self.latest_joint_positions or ())
            if not joints or not all(math.isfinite(v) for v in joints):
                raise ValueError('invalid joint feedback while confirming stopped motion')
            if gate['sample'] == stamp:
                return True
            gate['sample'] = stamp
            previous = gate['joints']
            if (previous is None or len(previous) != len(joints) or
                    max(abs(a-b) for a, b in zip(previous, joints)) > .001):
                gate['stable_since'], gate['joints'] = now, joints
                return True
            if now-gate['stable_since'] < .25:
                return True
            self.live_pick_pause_gate = None
            self.direct_tcp_z_offset = None  # recapture only after verified stop
            self.get_logger().info('pickup recovery: Servo paused; fresh stopped feedback stable for 0.25 s; beginning exclusive direct-mode handoff')
            self._complete_direct_place_pause()
        except (ValueError, TypeError, KeyError) as exc:
            self._fault(f'pickup recovery pause verification failed: {exc}; suction remains off')
            self.live_pick_pause_gate = None
        return True

    def start_pick_waypoints(self, _request, response):
        # AWAITING_GRASP is the deliberate exception: estimation left the
        # empty tool in contact with an incoming item, but we need to unpack.
        if ((self.state in self.ACTIVE and self.state != self.AWAITING_GRASP) or
                self.manual_gripper_pending):
            response.message = 'supervisor is busy'
            return response
        if self._ft_recovery_blocks_start(response):
            return response
        if self.state == self.FAULT:
            response.message = 'reset the supervisor fault before a pickup approach'
            return response
        if (self.dry_run or not self.transport_joint_limits or not self.pallet_locked or
                self.planning_scene_status.get('attached_item_id') or
                self.motion_status.get('state') != 'PREPARED' or
                self.motion_status.get('transfer_context') != 'known_pick'):
            response.message = 'pickup approach requires an empty tool and prepared known source'
            return response
        if not self.enable_client.service_is_ready():
            response.message = 'Servo pause service unavailable'
            return response
        self.operation_id += 1
        self.transport_is_pick = True
        self.raised_pick_offset_m = 0.
        self.raised_pick_ready = None
        self.raised_pick_active = False
        self.raised_place_hold_on_failure = False
        self.transport_route_generation = getattr(self, 'transport_route_generation', 0) + 1
        self.transport_is_return = False
        self.continuous_return_target_id = None
        self.continuous_return_completed = False
        self.continuous_return_restoring = False
        self.pick_path_completed = False
        self.pick_grid_index = -1
        self.pick_grid_deadline = None
        self.pick_grid_rail = 0
        self.pick_grid_direction = 1
        hint = (self.motion_status.get('planned_pregrasp') or {}).get('inspection_route')
        if hint is not None:
            self.pick_grid_rail, self.pick_grid_index, self.pick_grid_direction = hint
        self.pick_path_target_id = self.motion_status['operation_id']
        self.transport_target = dict(self.motion_status)
        self.transport_scene = None
        self.operation_kind = 'pick_approach'
        self.staging_place_active = False
        self.direct_transfer_succeeded = False
        self.direct_transfer_motion_started = False
        self.place_fallback_used = False
        self.post_retreat_fault = self.fault = ''
        self.transfer_goal_handle = None
        # Moving away invalidates immediate-grasp reuse of the incoming item.
        self.object_info_obtained = False
        self.contact_tcp_z = self.contact_tcp_xyz = self.corrected_object = None
        self.pick_path_restoring = True
        self.pick_path_handoff_started = time.monotonic()
        self.state = self.PICK_PATH_DISABLING
        operation = self.operation_id
        self.enable_client.call_async(SetBool.Request(data=False)).add_done_callback(
            lambda future: self._pick_path_paused(future, operation))
        response.success = True
        response.message = 'pickup approach accepted: pause Servo, lift, cross overhead, pre-pick'
        self.publish_status()
        return response

    def _pick_path_paused(self, future, operation):
        if self.operation_id != operation or self.state != self.PICK_PATH_DISABLING:
            return
        try:
            result = future.result()
            if result is None or not result.success:
                raise ValueError('Servo did not confirm pause')
            # Verify active controllers and mode 1, wait for new samples and
            # the existing uninterrupted post-restore settling interval.
            self._restore_ros2_control_mode()
        except Exception as exc:
            self._fault(f'pickup approach handoff failed: {exc}')

    def _plan_pick_path(self):
        self.pick_path_restoring = False
        now = time.monotonic()
        if (self.planning_scene_status.get('attached_item_id') or not self.pallet_locked or
                self.motion_status.get('operation_id') != self.pick_path_target_id or
                self.motion_status.get('state') != 'PREPARED' or
                self.robot_mode != 1 or self.robot_state not in (0, 2) or self.robot_error != 0 or
                self.robot_state_time is None or now-self.robot_state_time > self.status_timeout or
                self.last_joint_state_time is None or now-self.last_joint_state_time > self.status_timeout or
                self.last_force_time is None or now-self.last_force_time > self.force_timeout):
            self._fault('pickup approach prerequisites changed during handoff')
            return
        if not all(c.service_is_ready() for c in (
                self.transport_fk, self.transport_cartesian, self.state_validity_client)) or not (
                self.transfer_trajectory_client.server_is_ready()):
            self._fault('pickup approach planning/trajectory services unavailable')
            return
        self.transport_seed = tuple(self.latest_joint_positions)
        self.transport_started = now
        self.pick_grid_deadline = now + GRID_SECONDS
        self.transport_feedback_time = 0.
        self.transport_descent_time = None
        self.transport_contact_baseline = None
        self.transport_force_count = 0
        self.transport_terminal = self.transport_cancel_confirmed = False
        self.state = self.TRANSPORT_PLANNING
        self.pick_observation_xyz = None
        if (self.transport_target.get('planned_pregrasp') or {}).get('pickup_source') in ('buffer', 'pallet'):
            try:
                with open(self.return_waypoint_file, encoding='utf-8') as stream:
                    observation = yaml.safe_load(stream)['waypoints']['observation']
                mapping = dict(zip(observation['joint_names'], observation['positions_rad']))
                joints = tuple(float(mapping[n]) for n in self.arm_joint_names)
                if any(not math.isfinite(v) or not self.transport_joint_limits[n][0] <= v <=
                       self.transport_joint_limits[n][1] for n, v in zip(self.arm_joint_names, joints)):
                    raise ValueError('invalid saved observation joints')
                operation = self.operation_id
                self._transport_fk_request(joints, lambda f: self._pick_observation_fk(f, operation))
            except Exception as exc:
                self._fault(f'buffer approach observation waypoint unavailable: {exc}')
            return
        self._transport_fk_request(self.transport_seed, self._transport_start_fk)

    def _pick_observation_fk(self, future, operation):
        if self.state != self.TRANSPORT_PLANNING or self.operation_id != operation:
            return
        try:
            self.pick_observation_xyz, _ = self._transport_pose(future.result())
            self.get_logger().info('observation-side waypoint available; selecting route from current TCP area')
            self._transport_fk_request(self.transport_seed, self._transport_start_fk)
        except Exception as exc:
            self._fault(f'buffer approach observation FK failed: {exc}')

    def _try_pick_grid(self):
        if (not getattr(self, 'transport_is_pick', False) or
                not getattr(self, 'pick_cross_area', False) or
                self.state not in (self.TRANSPORT_PLANNING, self.TRANSPORT_DIAGNOSING, self.TRANSPORT_VALIDATING) or
                getattr(self, 'direct_transfer_motion_started', False) or
                getattr(self, 'transfer_goal_handle', None) is not None):
            return False
        now = time.monotonic()
        if getattr(self, 'pick_grid_deadline', None) is None:
            self.pick_grid_deadline = now + GRID_SECONDS
        index = getattr(self, 'pick_grid_index', -1) + 1
        if now >= self.pick_grid_deadline or index >= len(waypoint_candidates()):
            phase = next_grid(getattr(self, 'pick_grid_rail', 0), getattr(self, 'pick_grid_direction', 1))
            if phase is None:
                return False
            self.pick_grid_rail, self.pick_grid_direction = phase
            self.pick_grid_deadline = now + GRID_SECONDS
            index = 0
        self.pick_grid_index = index
        self.transport_route_generation += 1
        self.state = self.TRANSPORT_PLANNING
        self.transport_started = now
        self._transport_fk_request(self.transport_seed, self._transport_start_fk)
        return True

    def _try_raised_pick_path(self, reason):
        if (getattr(self, 'transport_target', {}).get('planned_pregrasp') or {}).get('inspection_only'):
            return False  # Camera centering requires the requested endpoint.
        if (not getattr(self, 'transport_is_pick', False) or
                self.state not in (self.TRANSPORT_PLANNING, self.TRANSPORT_DIAGNOSING) or
                self.transport_target.get('transfer_context') != 'known_pick' or
                getattr(self, 'direct_transfer_motion_started', False) or
                getattr(self, 'transfer_goal_handle', None) is not None):
            return False
        offset = getattr(self, 'raised_pick_offset_m', 0.) + .005
        if offset*1000 > getattr(self, 'raised_pre_pick_max_mm', 0.) + 1e-6:
            return False
        self.raised_pick_offset_m = offset
        self.transport_route_generation = getattr(self, 'transport_route_generation', 0) + 1
        self.state = self.TRANSPORT_PLANNING
        self.transport_started = time.monotonic()
        self.transport_descent_time = None
        self.get_logger().warning(
            f'{reason}; testing pre-pick +{offset*1000:.0f} mm with full pickup-path validation')
        self._transport_fk_request(self.transport_seed, self._transport_start_fk)
        return True

    def _validated_raised_pick(self, snapshot, xyz):
        record = getattr(self, 'raised_pick_ready', None)
        if record is None:
            return None
        if (record['target_id'] != self.motion_status.get('operation_id') or
                record['retrieval_target_id'] != snapshot.get('retrieval_target_id') or
                self.planning_scene_status.get('attached_item_id') or
                max(abs(a-b) for a,b in zip(xyz, record['xyz'])) > .002 or
                self.last_force_time is None or
                time.monotonic()-self.last_force_time > self.force_timeout):
            raise ValueError('raised pickup handoff changed or force feedback stale')
        return record

    def _raised_pick_contact_check(self, delta, now):
        """Pause steps while confirming contact using distinct force samples."""
        if delta is None or delta < self.force_threshold:
            self.raised_pick_contact_count = 0
            return False
        current_z = self._direct_mode_tcp_xyz()[2]
        if self.pregrasp_z - (current_z + self.direct_tcp_z_offset) < self.minimum_contact_descent:
            self._fault('unexpected force before minimum pickup descent; suction remains off')
            return True
        self.raised_pick_contact_count += 1
        if self.raised_pick_contact_count >= max(2, self.loading_contact_confirm_samples):
            self._release_from_direct_place('guarded incremental pickup contact confirmed', contact_detected=True)
        else:
            self.retreat_target_z = current_z
            self.direct_place_step_completed_at = now
            self.state = self.WAITING_PLACE_STEP_FEEDBACK
        return True
