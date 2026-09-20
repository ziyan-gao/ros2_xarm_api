"""Reuse executed overhead joint paths, never unchecked endpoint trajectories.

All cache hits are planning candidates. Fresh Cartesian connectors, retiming,
and the supervisor's complete collision/FK validation still precede execution.
The cache is process-local and deliberately separates empty and loaded tools.
"""
import copy
import math
import time
from types import SimpleNamespace

import numpy as np
from moveit_msgs.srv import GetCartesianPath
from rclpy.duration import Duration

from .transport_path import quaternion, item_bottom_offset
from .transport_alternatives import pose_message, join_trajectories


def stamp(point):
    return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9


def reverse_path(path):
    """Reverse positions and time; velocity changes sign, acceleration does not."""
    result = copy.deepcopy(path)
    end = stamp(result.points[-1])
    result.points.reverse()
    for point in result.points:
        point.time_from_start = Duration(seconds=max(0., end-stamp(point))).to_msg()
        point.velocities = [-v for v in point.velocities]
        point.effort = []
    return result


def attachment_key(scene):
    if scene is None:
        return ('empty',)
    # IDs may change between operations; geometry and grasp must not.
    orientation = quaternion(scene['attached_item_orientation_in_tcp_xyzw'])
    # q and -q are the same rotation. Use the largest component rather than w
    # alone so half-turns (w == 0) have a deterministic sign too.
    if orientation[int(np.argmax(np.abs(orientation)))] < 0:
        orientation = -orientation
    return ('loaded', tuple(float(v) for v in scene['attached_item_size_m']),
            tuple(float(v) for v in scene['attached_item_center_in_tcp_m']),
            tuple(float(v) for v in orientation))


def overhead_entry(path, poses, floor, scene, key):
    """Extract the longest contiguous overhead knot run from validated FK.

    FK poses are keyed by the retimed knot's timestamp (rounded to nanoseconds).
    Missing/low knots break a run: never join across an unobserved or low section.
    """
    runs, run = [], []
    for i, point in enumerate(path.points):
        pose = poses.get(round(stamp(point), 9))
        if pose is not None and pose[0][2]+item_bottom_offset(pose[1], scene) >= floor+.002:
            run.append(i)
        else:
            if len(run) >= 2:
                runs.append(run)
            run = []
    if len(run) >= 2:
        runs.append(run)
    if not runs:
        return None
    run = max(runs, key=lambda r: stamp(path.points[r[-1]])-stamp(path.points[r[0]]))
    geometry = [poses[round(stamp(path.points[i]), 9)] for i in run]
    if np.linalg.norm(geometry[-1][0][:2]-geometry[0][0][:2]) < .1:
        return None  # A vertical lift is not a reusable crossing.
    result = copy.deepcopy(path)
    result.points = result.points[run[0]:run[-1]+1]
    offset = stamp(result.points[0])
    for point in result.points:
        point.time_from_start = Duration(seconds=max(0., stamp(point)-offset)).to_msg()
        point.effort = []
    return dict(key=key, path=result, poses=copy.deepcopy(geometry))


def compatible_route(entries, key, start, qa, end, qb, floor, ceiling, scene):
    """Prefer short connectors, testing both directions without changing geometry."""
    candidates = []
    match = math.cos(math.radians(.5)/2)
    for entry in reversed(entries):
        if entry['key'] != key:
            continue
        for reverse in (False, True):
            poses = entry['poses'][::-1] if reverse else entry['poses']
            a, b = poses[0], poses[-1]
            if abs(float(quaternion(a[1]) @ qa)) < match or abs(float(quaternion(b[1]) @ qb)) < match:
                continue
            if any(p[0][2] > ceiling or p[0][2]+item_bottom_offset(p[1], scene) < floor+.002
                   for p in poses):
                continue
            # Fixed-orientation transfers must not inherit an old yaw excursion.
            if abs(float(qa @ qb)) >= match and any(abs(float(quaternion(p[1]) @ qa)) < match for p in poses):
                continue
            distance = float(np.linalg.norm(a[0][:2]-start[:2])+np.linalg.norm(b[0][:2]-end[:2]))
            if distance > .6:
                continue
            candidates.append((distance, entry, reverse, poses))
    if not candidates:
        return None
    _, entry, reverse, poses = min(candidates, key=lambda c: c[0])
    return dict(path=reverse_path(entry['path']) if reverse else copy.deepcopy(entry['path']),
                poses=copy.deepcopy(poses), reversed=reverse)


class TransportReuse:
    def _init_transport_reuse(self):
        self.declare_parameter('transport_path_reuse_enabled', True)
        self.transport_path_reuse_enabled = bool(self.get_parameter('transport_path_reuse_enabled').value)
        self.transport_path_cache = []
        self.transport_cache_source = None

    def _begin_reuse_recording(self, trajectory=None):
        """Own FK samples by operation, route generation and JTC trajectory.

        SDK validation passes no trajectory: it uses joint-line sample indices,
        not the JTC spline timestamps, and must never populate this cache.
        """
        self.transport_validated_poses = {}
        self.transport_cache_source = None
        if trajectory is not None and getattr(self, 'transport_path_reuse_enabled', False):
            self.transport_cache_source = dict(
                operation=self.operation_id,
                generation=getattr(self, 'transport_route_generation', 0),
                trajectory=trajectory, executed=False)

    def _reuse_recording_owned(self):
        source = getattr(self, 'transport_cache_source', None)
        return bool(source and not getattr(self, 'transport_sdk_candidate', False) and
                    source['operation'] == self.operation_id and
                    source['generation'] == getattr(self, 'transport_route_generation', 0) and
                    source['trajectory'] is getattr(self, 'transport_trajectory', None))

    def _record_reuse_pose(self, t, xyz, q):
        if self._reuse_recording_owned():
            self.transport_validated_poses[round(t, 9)] = (xyz.copy(), q.copy())

    def _mark_reuse_execution(self):
        # Called only after the JTC action server accepts this operation's goal.
        if self._reuse_recording_owned():
            self.transport_cache_source['executed'] = True

    def _reuse_key(self):
        return (self.planning_group, self.ik_link_name, tuple(self.arm_joint_names),
                tuple(sorted(self.transport_joint_limits.items())), attachment_key(self.transport_scene))

    def _reuse_crossing(self):
        return (not getattr(self, 'transport_is_return', False) and
                (getattr(self, 'pick_cross_area', False) if getattr(self, 'transport_is_pick', False)
                 else getattr(self, 'transport_via_observation', False)))

    def _reuse_is_active(self):
        return (getattr(self, 'reuse_active', False) and
                getattr(self, 'reuse_operation', None) == getattr(self, 'operation_id', None))

    def _try_reuse_transport(self, start_future):
        if (not getattr(self, 'transport_path_reuse_enabled', False) or not self._reuse_crossing() or
                getattr(self, 'reuse_attempted_operation', None) == self.operation_id):
            return False
        self.reuse_attempted_operation = self.operation_id
        candidate = compatible_route(
            self.transport_path_cache, self._reuse_key(), self.transport_start_xyz,
            quaternion(self.transport_start_q), self.transport_end, self.transport_end_q,
            self.transport_clearance, self._transport_ceiling(), self.transport_scene)
        if candidate is None:
            self.get_logger().info('cross-frame path cache: no compatible overhead section; planning normally')
            return False
        self.reuse_operation, self.reuse_active = self.operation_id, True
        self.reuse_start_future, self.reuse_candidate = start_future, candidate
        self.reuse_deadline = time.monotonic()+5.
        self.get_logger().info(
            f'cross-frame path cache: trying {len(candidate["path"].points)} overhead knots '
            f'({"reverse" if candidate["reversed"] else "forward"}); planning fresh connections')
        try:
            # Solve the incoming connector backwards from the exact cached joint
            # boundary, then reverse it. A different branch at the measured start
            # is rejected; we never interpolate across incompatible IK branches.
            first_xyz, first_q = candidate['poses'][0]
            above = np.array([*self.transport_start_xyz[:2],
                              max(first_xyz[2], self.transport_start_xyz[2])])
            self._reuse_cartesian(candidate['path'].points[0].positions,
                                  [(above, first_q), (above, self.transport_start_q),
                                   (self.transport_start_xyz, self.transport_start_q)],
                                  self._reuse_incoming)
        except Exception as exc:
            self._reuse_fallback(str(exc))
        return True

    def _reuse_cartesian(self, seed, poses, callback):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'link_base'
        req.group_name, req.link_name = self.planning_group, self.ik_link_name
        req.start_state.is_diff = True
        req.start_state.joint_state.name = list(self.arm_joint_names)
        req.start_state.joint_state.position = list(seed)
        req.waypoints = [pose_message(xyz, q) for xyz, q in poses]
        req.max_step, req.jump_threshold, req.avoid_collisions = .005, 0., True
        scale = max(.05, min(1., self.motion_speed_percent/100))
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = scale
        self.transport_cartesian.call_async(req).add_done_callback(self._transport_guard(callback))

    def _reuse_result(self, future):
        from .continuous_transport import cartesian_timing_issue
        if time.monotonic() >= self.reuse_deadline:
            raise ValueError('connection planning exceeded 5 s')
        result = future.result()
        if result is None or result.error_code.val != 1 or not 1-1e-6 <= result.fraction <= 1.:
            raise ValueError('incomplete cache connector')
        path = copy.deepcopy(result.solution.joint_trajectory)
        issue = cartesian_timing_issue(path, self.arm_joint_names)
        if issue:
            raise ValueError(issue)
        # Normalize order before testing exact joint seams.
        return join_trajectories([path], self.arm_joint_names)

    def _reuse_incoming(self, future):
        if not self._reuse_is_active():
            return
        try:
            path = reverse_path(self._reuse_result(future))
            if max(abs(a-b) for a, b in zip(path.points[0].positions, self.transport_seed)) > .001:
                raise ValueError('cache connector reaches a different measured-start joint branch')
            # Absorb only the numerical IK residual, then revalidate the actual
            # spline. No unvalidated snap or extra hardware command is issued.
            path.points[0].positions = list(self.transport_seed)
            self.reuse_incoming_path = path
            xyz, q = self.reuse_candidate['poses'][-1]
            above = np.array([*self.transport_end[:2], max(xyz[2], self.transport_end[2])])
            self._reuse_cartesian(self.reuse_candidate['path'].points[-1].positions,
                                  [(above, q), (above, self.transport_end_q),
                                   (self.transport_end, self.transport_end_q)], self._reuse_outgoing)
        except Exception as exc:
            self._reuse_fallback(str(exc))

    def _reuse_outgoing(self, future):
        if not self._reuse_is_active():
            return
        try:
            outgoing = self._reuse_result(future)
            path = join_trajectories([self.reuse_incoming_path, self.reuse_candidate['path'], outgoing],
                                     self.arm_joint_names)
            self.get_logger().info('cross-frame cache connections complete; retiming and rechecking entire path')
            result = SimpleNamespace(error_code=SimpleNamespace(val=1), fraction=1.,
                                     solution=SimpleNamespace(joint_trajectory=path))
            self._transport_planned(SimpleNamespace(result=lambda: result))
        except Exception as exc:
            self._reuse_fallback(str(exc))

    def _reuse_fallback(self, reason):
        if (not self._reuse_is_active() or getattr(self, 'direct_transfer_motion_started', False) or
                getattr(self, 'transfer_goal_handle', None) is not None or
                self.state not in (self.TRANSPORT_PLANNING, self.TRANSPORT_VALIDATING)):
            return False
        self.reuse_active = False
        self.transport_route_generation = getattr(self, 'transport_route_generation', 0)+1
        self.get_logger().info(f'cross-frame cache rejected: {reason}; resuming normal waypoint search')
        self.state = self.TRANSPORT_PLANNING
        self.transport_started = time.monotonic()
        self.transport_descent_time = None
        self.transport_contact_baseline = None
        self.transport_force_count = 0
        if getattr(self, 'transport_is_pick', False):
            self.pick_grid_deadline = self.transport_started+5.
        self._transport_start_fk(self.reuse_start_future)
        return True

    def _reuse_tick(self, now):
        if self._reuse_is_active() and self.state == self.TRANSPORT_PLANNING:
            if now >= self.reuse_deadline:
                self._reuse_fallback('connection planning exceeded 5 s')
            return True  # Do not consume the normal grid budget while connecting.
        return False

    def _remember_overhead_path(self):
        if (not getattr(self, 'transport_path_reuse_enabled', False) or not self._reuse_crossing() or
                not self._reuse_recording_owned() or not self.transport_cache_source['executed']):
            return
        try:
            entry = overhead_entry(self.transport_trajectory, self.transport_validated_poses,
                                   self.transport_clearance, self.transport_scene, self._reuse_key())
            if entry is not None:
                self.transport_path_cache.append(entry)
                del self.transport_path_cache[:-12]
                self.get_logger().info(f'cross-frame path cache: saved verified overhead section ({len(entry["path"].points)} knots)')
        except Exception as exc:
            # Optional optimization must not change a completed operation's result.
            self.get_logger().info(f'cross-frame path cache: not saved ({exc})')
