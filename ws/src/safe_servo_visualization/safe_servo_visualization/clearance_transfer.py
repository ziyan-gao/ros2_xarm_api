"""Clearance routes with one checked cuMotion/Cartesian execution."""
import time
import math
import numpy as np

CLEARANCE_TARGET_MARGIN_M = .010

class ClearanceTransfer:
    def _wait_descent_baseline(self):
        if (not getattr(self, 'direct_moveit_active', False) or
                getattr(self, 'clearance_phase', None) not in ('descend', 'continuous', 'direct', 'departure_lift') or
                getattr(self, 'transport_is_return', False)):
            return False
        key = (self.operation_id, self.transport_route_generation, id(self.transport_trajectory))
        if getattr(self, 'descent_baseline_ready', None) == key:
            return False
        now = time.monotonic()
        force_z = getattr(self, 'latest_force_z', None)
        force_time = getattr(self, 'last_force_time', None)
        if (force_z is None or not math.isfinite(force_z) or force_time is None or
                now-force_time > self.force_timeout):
            self._fault('no fresh Fz sample before Cartesian descent; descent not executed')
            return True
        # Do not wait for a stationary sampling window.  The force sensor is
        # continuously bias-compensated, but retain its small live offset
        # instead of assuming mathematical zero.  Contact detection below uses
        # the change from this value and remains fully active during descent.
        self.transport_contact_baseline = float(force_z)
        self.transport_force_count = 0
        self.descent_baseline_ready = key
        self.descent_baseline_gate = None
        if (self.clearance_phase in ('descend', 'direct', 'departure_lift') or
                (self.clearance_phase == 'continuous' and
                 not getattr(self, 'clearance_has_descent', True))):
            self.transport_descent_time = 0.
        self.get_logger().info(
            f'captured immediate Cartesian descent Fz baseline='
            f'{self.transport_contact_baseline:.3f} N (no settling wait)')
        return False

    def _collect_descent_baseline(self, force_z):
        gate = getattr(self, 'descent_baseline_gate', None)
        if gate is None or self.state != self.TRANSPORT_VALIDATING:
            return
        now = time.monotonic()
        if not math.isfinite(force_z):
            gate['samples'] = []
            return
        if max(abs(a-b) for a,b in zip(self.latest_joint_positions, gate['joints'])) > .001:
            gate['joints'] = tuple(self.latest_joint_positions)
            gate['samples'] = []
            return
        gate['samples'].append((now, float(force_z)))
        gate['samples'] = [(t,f) for t,f in gate['samples'] if now-t <= .4]

    def _descent_baseline_tick(self, now):
        gate = getattr(self, 'descent_baseline_gate', None)
        if gate is None:
            return False
        key = (self.operation_id, self.transport_route_generation, id(self.transport_trajectory))
        if self.state != self.TRANSPORT_VALIDATING or gate['key'] != key:
            self.descent_baseline_gate = None
            return False
        if now-gate['started'] > 3.:
            self.descent_baseline_gate = None
            self._fault('no fresh stable stationary force baseline within 3 s; descent not executed')
            return True
        samples = gate['samples']
        if (len(samples) < 5 or samples[-1][0]-samples[0][0] < .2 or
                now-samples[-1][0] > min(.1, self.force_timeout)):
            return True
        values = [f for _,f in samples]
        if max(values)-min(values) > 1.:
            return True
        self.transport_contact_baseline = float(np.median(values))
        self.transport_force_count = 0
        self.descent_baseline_ready = key
        self.descent_baseline_gate = None
        # Monitor from the first descent sample; do not replace the baseline
        # with an in-motion reading before the first controller feedback.
        self.transport_descent_time = 0.
        self.get_logger().info(f'Cartesian descent Fz baseline={self.transport_contact_baseline:.3f} N')
        self._transport_execute()  # Recheck telemetry, seed, scene and operation.
        return True

    def _begin_clearance_transfer(self, start, q, end, end_q, clearance):
        target = self.transport_target.get('planned_pregrasp') or {}
        slot_pick = (getattr(self, 'transport_is_pick', False) and
                     target.get('pickup_source') == 'buffer' and not target.get('inspection_only'))
        source = getattr(self, 'active_pickup_snapshot', None) or {}
        slot_exit = (not getattr(self, 'transport_is_pick', False) and
                     not getattr(self, 'transport_is_return', False) and
                     source.get('pickup_source') == 'buffer')
        # No Cartesian segments; retained scene obstacles can reject a goal.
        if getattr(self, 'transport_moveit_pipeline_id', '') == 'isaac_ros_cumotion':
            self.clearance_final = (np.asarray(end).copy(), np.asarray(end_q).copy())
            self.clearance_destination = np.asarray(end).copy()
            self.clearance_source_z = float(start[2])
            self.clearance_target_region = 'pallet'
            self.transport_safe_z = self.transport_high_z = max(float(start[2]), float(end[2]))
            self.clearance_phase = 'direct'
            if ((self.transport_scene or {}).get('attached_item_id')
                    and not getattr(self, 'transport_is_pick', False)
                    and not getattr(self, 'transport_is_return', False)):
                self.clearance_source_z = float(start[2]) + getattr(self, 'transport_departure_lift_m', .100)
                if self.clearance_source_z > self._transport_ceiling():
                    raise ValueError('pickup departure lift exceeds workspace ceiling')
                self.clearance_phase = 'departure_lift'
            self._plan_clearance_phase(start, q, self.transport_seed)
            return
        from .transport_path import item_bottom_offset
        source = getattr(self, 'active_pickup_snapshot', None) or {}
        target = self.transport_target.get('planned_pregrasp') or {}
        slot_z = getattr(self, 'transport_slot_clearance_z_m', 0.) or clearance
        source_region = (getattr(self, 'clearance_last_region', None) if getattr(self, 'transport_is_pick', False)
                         else source.get('pickup_source'))
        # Match the pallet-only 100 mm reduction in live_bridge's virtual box.
        pallet_z = clearance - (.100 if getattr(
            self, 'transport_moveit_pipeline_id', '') == 'isaac_ros_cumotion' else 0.)
        source_z = slot_z if source_region == 'buffer' else pallet_z
        destination_z = slot_z if (target.get('pickup_source') == 'buffer' or
                                  self.transport_target.get('transfer_context') == 'staging_store') else pallet_z
        self.clearance_target_region = ('buffer' if (target.get('pickup_source') == 'buffer' or
            self.transport_target.get('transfer_context') == 'staging_store') else 'pallet')
        scene = self.transport_scene
        if not (scene or {}).get('attached_item_id'):
            scene = None
        direct_inspection = (
            bool(getattr(self, 'transport_is_pick', False)) and
            bool(target.get('inspection_only')) and
            self.transport_target.get('target') == 'top_face_view' and
            scene is None)
        same_area_inspection = (
            direct_inspection and not bool(getattr(self, 'pick_cross_area', True)))
        # cuMotion checks the complete gripper/camera sphere model, not only
        # the mathematical TCP.  The measured vacuum-gripper sphere envelope
        # extends 24.5 mm below link_tcp in the downward grasp orientation.
        # Use a configurable 30 mm tool envelope for empty-tool transfers; a
        # carried payload retains the larger fixed/measured downward extent.
        tool_drop = (0.0 if same_area_inspection else
                     float(getattr(self, 'transport_empty_tool_drop_m', .030)))
        configured_drop = max(
            tool_drop,
            float(getattr(self, 'transport_max_payload_drop_m', .300))
            if scene is not None else 0.0)
        source_drop = max(configured_drop, -min(0., item_bottom_offset(q, scene)))
        destination_drop = max(
            configured_drop, -min(0., item_bottom_offset(end_q, scene)))
        source_tcp = source_z + source_drop
        destination_tcp = destination_z + destination_drop
        # Keep the constraint floor independent of commanded target heights.
        # Tracking/settling error must not place a valid lift outside the region.
        planning_floor = min(source_tcp, destination_tcp)
        source_tcp = (max(float(start[2]), source_tcp + CLEARANCE_TARGET_MARGIN_M)
                      if (source_region == 'buffer' or getattr(self, 'transport_moveit_pipeline_id', '')
                          != 'isaac_ros_cumotion') else float(start[2]))
        if same_area_inspection:
            # Within one workspace region a straight Cartesian interpolation
            # cannot exhibit the low-Z arc produced by joint-space planning.
            # TopFaceMotion has already clamped the endpoint to the regional
            # clearance, so bypass cuMotion and move directly to that pose.
            if float(end[2]) < destination_tcp - .001:
                raise ValueError(
                    'top-face inspection target is below regional TCP clearance')
            destination_tcp = float(end[2])
        else:
            # Cross-area inspection retains a 10 mm terminal approach: lift
            # locally, use cuMotion for the free-space crossing, then descend
            # vertically at fixed XY/orientation to the exact camera pose.
            destination_tcp = max(float(end[2]), destination_tcp + CLEARANCE_TARGET_MARGIN_M)
        if (slot_exit and self.transport_target.get('transfer_context') == 'pallet'
                and getattr(self, 'transport_moveit_pipeline_id', '') == 'isaac_ros_cumotion'):
            destination_tcp = float(end[2])
        if max(source_tcp, destination_tcp) > self._transport_ceiling():
            raise ValueError('source/destination clearance exceeds workspace ceiling')
        departure_lift = (scene is not None and source_region != 'buffer' and
                          getattr(self, 'transport_moveit_pipeline_id', '') == 'isaac_ros_cumotion')
        if departure_lift:
            source_tcp = float(start[2]) + getattr(self, 'transport_departure_lift_m', .100)
            if source_tcp > self._transport_ceiling():
                raise ValueError('pickup departure lift exceeds workspace ceiling')
        self.clearance_final = (np.asarray(end).copy(), np.asarray(end_q).copy())
        self.clearance_destination = np.array([end[0], end[1], destination_tcp])
        self.clearance_source_z = source_tcp
        self.clearance_floor_z = min(source_z, destination_z)
        self.transport_safe_z = planning_floor
        self.transport_high_z = max(source_tcp, destination_tcp)
        self.clearance_phase = ('departure_lift' if departure_lift else
                                'inspection_cartesian' if same_area_inspection else 'lift')
        self.get_logger().info(f'clearance TCP: source={source_tcp:.3f} m, destination={destination_tcp:.3f} m')
        if same_area_inspection:
            self.get_logger().info(
                'top-face inspection route: same-area Cartesian motion directly to final pose')
        elif direct_inspection:
            self.get_logger().info(
                'top-face inspection route: cross-area cuMotion target is 10 mm above final pose; '
                'terminal Cartesian approach retained')
        self._plan_clearance_phase(start, q, self.transport_seed)

    def _plan_clearance_phase(self, xyz, q, joints):
        self.transport_seed = tuple(joints)
        self.alternative_seed = tuple(joints)
        self.transport_start_xyz, self.transport_start_q = np.asarray(xyz), np.asarray(q)
        self.transport_started = time.monotonic()
        self.transport_descent_time = None
        self.transport_feedback_time = 0.
        self.transport_contact_baseline = None
        self.descent_baseline_gate = None
        self.descent_baseline_ready = None
        self.transport_force_count = 0
        self.transport_terminal = self.transport_cancel_confirmed = False
        self.transport_route_generation += 1
        self.direct_transfer_motion_started = False
        self.state = self.TRANSPORT_PLANNING
        self.alternative_current_xyz, self.alternative_current_q = np.asarray(xyz), np.asarray(q)
        self.alternative_parts = []
        self.alternative_part_kinds = []
        self.direct_lift_z = None
        self.direct_lift_verified = False
        final, final_q = self.clearance_final
        if self.clearance_phase == 'departure_lift':
            self.transport_end = np.array([xyz[0], xyz[1], self.clearance_source_z])
            self.transport_end_q = np.asarray(q).copy()
            self.direct_lift_z = None  # Joint-space route to a higher endpoint.
            self.alternative_segments = [('moveit', self.transport_end, self.transport_end_q)]
            self.get_logger().info('cuMotion to raised target before planning transport')
            self._alternative_next()
            return
        if self.clearance_phase == 'direct':
            self.transport_end, self.transport_end_q = final, final_q
            self.clearance_has_descent = False
            self.alternative_segments = [('moveit', final, final_q)]
            self.get_logger().info('planning directly to target; pallet clearance waypoints disabled')
            self._alternative_next()
            return
        if (getattr(self, 'transport_moveit_pipeline_id', '') == 'isaac_ros_cumotion' and
                self.clearance_phase != 'inspection_cartesian'):
            self._plan_continuous_clearance(xyz, q)
            return
        if self.clearance_phase == 'lift':
            end = np.array([xyz[0], xyz[1], self.clearance_source_z])
            end_q, kind = q, 'cartesian'
            self.direct_lift_z = float(end[2])
            if abs(end[2]-xyz[2]) < .001:
                self.clearance_phase = 'transfer'
                self._plan_clearance_phase(xyz, q, joints)
                return
        elif self.clearance_phase == 'transfer':
            end, end_q, kind = self.clearance_destination, final_q, 'moveit'
            if getattr(self, 'transport_is_return', False):
                end = final  # Observation is an exact saved joint goal.
        else:
            end, end_q, kind = final, final_q, 'cartesian'
        self.transport_end, self.transport_end_q = np.asarray(end), np.asarray(end_q)
        self.alternative_segments = [(kind, self.transport_end, self.transport_end_q)]
        self._alternative_next()

    def _plan_continuous_clearance(self, xyz, q):
        """Plan all legs from predecessor endpoints; do not move between requests."""
        self.clearance_phase = 'continuous'
        final, final_q = self.clearance_final
        self.transport_end, self.transport_end_q = final, final_q
        self.clearance_has_descent = (
            not getattr(self, 'transport_is_return', False) and
            np.linalg.norm(self.clearance_destination-final) > .001)
        target = self.transport_target.get('planned_pregrasp') or {}
        if getattr(self, 'transport_is_pick', False) and target.get('approach_mode') == 'joint_direct':
            self.clearance_has_descent = False
            self.direct_lift_z = None
            self.alternative_segments = [('moveit', final, final_q)]
            self.get_logger().info('new-item approach: checked direct joint interpolation')
            self._alternative_next()
            return
        if (getattr(self, 'transport_is_pick', False) and not target.get('inspection_only') and
                target.get('pickup_source') == 'pallet'):
            if xyz[2] < final[2] - .001:
                raise ValueError('inspection pose is below pre-pick; cannot move horizontally then downward')
            self.clearance_destination = np.array([final[0], final[1], xyz[2]])
            self.transport_high_z = float(xyz[2])
            self.direct_lift_z = None
            self.clearance_has_descent = xyz[2] - final[2] > .001
            self.alternative_segments = [('cartesian', self.clearance_destination, final_q)]
            if self.clearance_has_descent:
                self.alternative_segments.append(('cartesian', final, final_q))
            self.get_logger().info('inspection approach: checked horizontal Cartesian then downward')
            self._alternative_next()
            return
        segments = []
        if self.clearance_source_z-xyz[2] > .001:
            # Reserve the top half of the clearance margin for the rounded turn.
            self.direct_lift_z = self.clearance_source_z-CLEARANCE_TARGET_MARGIN_M/2
            segments.append(('cartesian', np.array([*xyz[:2], self.clearance_source_z]), q))
        if (not getattr(self, 'transport_is_pick', False) and
                self.transport_target.get('transfer_context') == 'pallet'):
            # After leaving the slot barrier, go directly to pallet pre-place.
            self.clearance_has_descent = False
        destination = self.clearance_destination if self.clearance_has_descent else final
        segments.append(('moveit', destination, final_q))
        if self.clearance_has_descent:
            segments.append(('cartesian', final, final_q))
        self.alternative_segments = segments
        self.get_logger().info('planning complete clearance route; one blended trajectory, no intermediate execution')
        self._alternative_next()

    def _continuous_clearance_geometry_issue(self, xyz, q, t):
        """Classify the turn spatially; sparse neighboring knots are not its bounds."""
        descent = self.transport_descent_time
        distance = float(np.linalg.norm(xyz-self.clearance_destination))
        radius = min(self.transport_radius, CLEARANCE_TARGET_MARGIN_M)
        issue = None
        if descent is not None and t >= descent:
            # The neighboring knot times only limit where blending may occur.
            # Outside the sphere, the incoming path is still free transfer and
            # the outgoing path must already satisfy strict vertical descent.
            if (t < self.clearance_descent_blend_end and distance <= radius and
                    math.isinf(self.clearance_previous_descent_z)):
                return None
            if t >= self.clearance_descent_seam_time:
                if (np.linalg.norm(xyz[:2]-self.transport_end[:2]) > .001 or
                        abs(float(q @ self.transport_end_q)) < math.cos(math.radians(.5)/2) or
                        xyz[2] > min(self.clearance_previous_descent_z,
                                     self.clearance_destination[2])+.001):
                    issue = 'continuous final approach must descend vertically with fixed orientation'
                else:
                    # Once outside the turn, never reopen its relaxed XY region.
                    self.clearance_previous_descent_z = float(xyz[2])
                    return None
        if issue:
            return (f'{issue}; t={t:.4f}s, xyz={np.asarray(xyz).round(6).tolist()}, '
                    f'seam_xyz={self.clearance_destination.round(6).tolist()}, '
                    f'distance={distance*1000:.2f}mm, radius={radius*1000:.2f}mm')
        return None

    def _retry_planning_after_lift(self, code):
        # Some MoveIt adapters collapse planner errors into FAILURE (99999).
        # Never retry IK, communication, execution or hardware failures here.
        if (code not in (-10, -1, 99999) or
                not getattr(self, 'direct_moveit_active', False) or
                getattr(self, 'transport_moveit_pipeline_id', '') != 'isaac_ros_cumotion' or
                self.state != self.TRANSPORT_PLANNING or
                getattr(self, 'direct_transfer_motion_started', False) or
                getattr(self, 'transfer_goal_handle', None) is not None or
                getattr(self, 'clearance_phase', None) != 'direct'):
            return False
        key = (self.operation_id, self.transport_target.get('operation_id'))
        if getattr(self, 'planning_lift_key', None) != key:
            self.planning_lift_key, self.planning_lift_count = key, 0
        if self.planning_lift_count >= 5:
            self._fault('planning still failed after 5 upward recovery steps; no further motion')
            return True
        self.planning_lift_count += 1
        def ready(result):
            xyz, q = self._transport_pose(result.result())
            target_z = float(xyz[2]) + .100
            if target_z > self._transport_ceiling():
                self._fault('planning recovery lift exceeds workspace ceiling; no motion')
                return
            if (getattr(self, 'task_policy_active', False) and
                    getattr(self, 'task_policy_task', None) == 'repack' and
                    getattr(self, 'operation_kind', None) == 'pick_approach'):
                self.planning_sdk_lift_pending = True
                self.planning_sdk_lift_origin = np.asarray(xyz).copy()
                self.direct_target_z = target_z
                self.direct_place_stepping = False
                self.direct_tcp_z_offset = None
                self.get_logger().warning(
                    f'repack planning failed (code={code}); SDK upward 100 mm '
                    f'at slow retreat speed, retry {self.planning_lift_count}/5')
                self._disable_servo_then_direct_retreat()
                return
            self.clearance_source_z = target_z
            self.clearance_phase = 'departure_lift'
            self.get_logger().warning(
                f'cuMotion planning failed (code={code}); checked vertical lift '
                f'100 mm, retry {self.planning_lift_count}/5')
            self._plan_clearance_phase(xyz, q, joints)
        joints = tuple(self.latest_joint_positions)
        self._transport_fk_request(joints, ready)
        return True

    def _planning_sdk_lift_restored(self):
        # Called only after controller restoration and fresh stable telemetry.
        self.state = self.TRANSPORT_PLANNING
        joints = tuple(self.latest_joint_positions)
        def ready(future):
            try:
                xyz, q = self._transport_pose(future.result())
                if (abs(xyz[2]-self.direct_target_z) > .002 or
                        np.linalg.norm(xyz[:2]-self.planning_sdk_lift_origin[:2]) > .003):
                    raise ValueError('SDK planning recovery endpoint mismatch')
                self.planning_sdk_lift_pending = False
                self.clearance_phase = 'direct'
                self._plan_clearance_phase(xyz, q, joints)
            except Exception as exc:
                self._fault(f'SDK planning recovery verification failed: {exc}')
        self._transport_fk_request(joints, ready)

    def _advance_clearance_phase(self, xyz, q):
        phase = getattr(self, 'clearance_phase', None)
        if not getattr(self, 'direct_moveit_active', False) or phase not in ('lift', 'transfer', 'departure_lift'):
            return False
        if phase == 'transfer' and getattr(self, 'transport_is_return', False):
            return False
        if self.transport_verify_error > .002:
            return True  # Wait for settled measured feedback, never plan from desired joints.
        if abs(float(np.asarray(q) @ self.transport_end_q)) < np.cos(np.deg2rad(.5)/2):
            return True
        if phase == 'departure_lift':
            self.clearance_phase = 'direct'
            self._plan_clearance_phase(xyz, q, self.transport_verify_joints)
            return True
        if phase == 'transfer' and np.linalg.norm(np.asarray(xyz)-self.clearance_final[0]) <= .001:
            self.clearance_phase = 'descend'
            return False  # Already at pre-pick/pre-place; do not create an empty trajectory.
        self.clearance_phase = 'transfer' if phase == 'lift' else 'descend'
        self._plan_clearance_phase(xyz, q, self.transport_verify_joints)
        return True
