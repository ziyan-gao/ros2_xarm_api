"""Measured-state clearance -> free-space -> vertical approach sequencing."""
import time
import math
import numpy as np

CLEARANCE_TARGET_MARGIN_M = .010

class ClearanceTransfer:
    def _wait_descent_baseline(self):
        if (not getattr(self, 'direct_moveit_active', False) or
                getattr(self, 'clearance_phase', None) != 'descend' or
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
        from .transport_path import item_bottom_offset
        source = getattr(self, 'active_pickup_snapshot', None) or {}
        target = self.transport_target.get('planned_pregrasp') or {}
        slot_z = getattr(self, 'transport_slot_clearance_z_m', 0.) or clearance
        source_region = (getattr(self, 'clearance_last_region', None) if getattr(self, 'transport_is_pick', False)
                         else source.get('pickup_source'))
        source_z = slot_z if source_region == 'buffer' else clearance
        destination_z = slot_z if (target.get('pickup_source') == 'buffer' or
                                  self.transport_target.get('transfer_context') == 'staging_store') else clearance
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
        source_tcp = max(float(start[2]), source_tcp + CLEARANCE_TARGET_MARGIN_M)
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
        if max(source_tcp, destination_tcp) > self._transport_ceiling():
            raise ValueError('source/destination clearance exceeds workspace ceiling')
        self.clearance_final = (np.asarray(end).copy(), np.asarray(end_q).copy())
        self.clearance_destination = np.array([end[0], end[1], destination_tcp])
        self.clearance_source_z = source_tcp
        self.clearance_floor_z = min(source_z, destination_z)
        self.transport_safe_z = planning_floor
        self.transport_high_z = max(source_tcp, destination_tcp)
        self.clearance_phase = ('inspection_cartesian' if same_area_inspection else 'lift')
        self.get_logger().info(f'clearance TCP: source={source_tcp:.3f} m, destination={destination_tcp:.3f} m; execute lift before free-space planning')
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
        self.direct_lift_z = None
        self.direct_lift_verified = False
        final, final_q = self.clearance_final
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

    def _advance_clearance_phase(self, xyz, q):
        phase = getattr(self, 'clearance_phase', None)
        if not getattr(self, 'direct_moveit_active', False) or phase not in ('lift', 'transfer'):
            return False
        if phase == 'transfer' and getattr(self, 'transport_is_return', False):
            return False
        if self.transport_verify_error > .002:
            return True  # Wait for settled measured feedback, never plan from desired joints.
        if abs(float(np.asarray(q) @ self.transport_end_q)) < np.cos(np.deg2rad(.5)/2):
            return True
        if phase == 'transfer' and np.linalg.norm(np.asarray(xyz)-self.clearance_final[0]) <= .001:
            self.clearance_phase = 'descend'
            return False  # Already at pre-pick/pre-place; do not create an empty trajectory.
        self.clearance_phase = 'transfer' if phase == 'lift' else 'descend'
        self._plan_clearance_phase(xyz, q, self.transport_verify_joints)
        return True
