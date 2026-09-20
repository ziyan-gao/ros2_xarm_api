"""Pre-execution carried-item route alternatives. All services here only plan.

Each candidate includes the lift and descent; nothing moves until the existing
timed-spline collision/FK checks have accepted the complete candidate.
"""
import copy
from functools import lru_cache
import math
import time
from types import SimpleNamespace

import numpy as np
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from rclpy.duration import Duration

from .transport_path import quaternion, rounded_translation_waypoints, buffer_transfer_waypoints
from .transport_path import directed_slerp
from .waypoint_search import GRID_SECONDS, next_grid


# (base-frame X offset, rotation location). Fixed, bounded planning alternatives;
# these are not assumed safe until the complete timed trajectory is checked.
OBSERVATION_ROUTES = ((0., 'via'), (0., 'destination'),
                      (.1, 'via'), (-.1, 'via'),
                      (.1, 'destination'), (-.1, 'destination'))


@lru_cache(maxsize=2)
def waypoint_candidates(expanded=True):
    """Deterministic X/Y rail and yaw-rotation-site search; no tilt detours."""
    routes = [(x, 0., site) for x, site in OBSERVATION_ROUTES]
    if expanded:
        # Keep nominal first, then expand in distance order on a 25 mm grid.
        routes = [(0., 0., 'via'), (0., 0., 'destination')]
        # Integer millimetres avoid floating-point step drift.
        offsets = sorted(((x, y) for x in range(-100, 101, 25)
                          for y in range(-100, 101, 25)),
                         key=lambda xy: (xy[0]**2 + xy[1]**2, xy))
        seen = set(routes)
        for x_mm, y_mm in offsets:
            x, y = x_mm/1000., y_mm/1000.
            for site in ('elbow', 'via', 'destination'):
                candidate = (x, y, site)
                if candidate not in seen:
                    routes.append(candidate)
                    seen.add(candidate)
    return tuple(routes)


def pose_message(xyz, q):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quaternion(q))
    return p


def observation_transfer_waypoint(observation_xyz, high_z, y_offset_mm):
    """Offset in link_base, not tool/pallet axes; preserve transfer clearance Z."""
    observation = np.asarray(observation_xyz, dtype=float)
    if (observation.shape != (3,) or not np.isfinite(observation).all() or
            not math.isfinite(high_z) or not math.isfinite(y_offset_mm)):
        raise ValueError('invalid observation transfer waypoint or offset')
    return np.array([observation[0], observation[1] + y_offset_mm/1000., high_z])




def join_trajectories(parts, names):
    """Keep exact segment seams and rest there; reject discontinuous IK branches."""
    from .continuous_transport import cartesian_timing_issue
    combined = copy.deepcopy(parts[0])
    combined.joint_names, combined.points = list(names), []
    offset = 0.
    for part in parts:
        issue = cartesian_timing_issue(part, names)
        if issue:
            raise ValueError('alternative segment timing invalid: ' + issue)
        part = copy.deepcopy(part)
        indices = [list(part.joint_names).index(n) for n in names]
        for point in part.points:
            for field in ('positions', 'velocities', 'accelerations', 'effort'):
                values = getattr(point, field)
                if values:
                    setattr(point, field, [values[i] for i in indices])
        if combined.points and max(abs(a-b) for a, b in zip(
                combined.points[-1].positions, part.points[0].positions)) > 1e-6:
            raise ValueError('alternative segments have discontinuous joint endpoints')
        for point in (part.points[0], part.points[-1]):
            point.velocities = [0.] * len(names)
            point.accelerations = [0.] * len(names)
        for point in part.points[bool(combined.points):]:
            stamp = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            point.time_from_start = Duration(seconds=offset+stamp).to_msg()
            combined.points.append(point)
        last = combined.points[-1].time_from_start
        offset = last.sec + last.nanosec * 1e-9
    return combined


def blend_translation_segments(segments, initial_q):
    """Group adjacent translations; never absorb a rotation or MoveIt leg."""
    grouped = []
    current_q = quaternion(initial_q)
    i = 0
    while i < len(segments):
        kind, xyz, q = segments[i]
        if kind != 'cartesian' or abs(float(quaternion(q) @ current_q)) < 1-1e-12:
            grouped.append((kind, xyz, q))
            current_q = quaternion(q)
            i += 1
            continue
        destinations = [xyz]
        i += 1
        while i < len(segments):
            next_kind, next_xyz, next_q = segments[i]
            if next_kind != 'cartesian' or abs(float(quaternion(next_q) @ current_q)) < 1-1e-12:
                break
            destinations.append(next_xyz)
            i += 1
        grouped.append(('cartesian_blend', destinations, q) if len(destinations) > 1
                       else (kind, xyz, q))
    return grouped


class TransportAlternatives:
    def _try_slot_yaw_flip(self, now):
        """One planning-only retry after five seconds; never rotate live in place."""
        if (getattr(self, 'alternative_yaw_direction', 1) != -1 or
                getattr(self, 'alternative_rail_index', 0) < 1 or
                now < getattr(self, 'alternative_grid_deadline', math.inf) or
                getattr(self, 'transport_slot_yaw_flipped', False) or
                now < getattr(self, 'transport_slot_yaw_deadline', math.inf) or
                self.state not in (self.TRANSPORT_PLANNING, self.TRANSPORT_DIAGNOSING) or
                getattr(self, 'direct_transfer_motion_started', False) or
                getattr(self, 'transfer_goal_handle', None) is not None or
                getattr(self, 'transport_is_pick', False) or getattr(self, 'transport_is_return', False) or
                getattr(self, 'transport_target', {}).get('transfer_context') != 'staging_store'):
            return False
        pose = self.transport_target.get('staging_yaw180_pose')
        if not pose or len(pose) != 7 or not all(math.isfinite(v) for v in pose):
            return False
        self.transport_slot_yaw_flipped = True
        self.transport_end = np.array(pose[:3], dtype=float)
        self.transport_end_q = quaternion(pose[3:])
        self.transport_target['pre_place_tcp_xyz_m'] = list(pose[:3])
        self.transport_target['transfer_tcp_quaternion_xyzw'] = list(pose[3:])
        self.raised_pre_place_offset_m = 0.
        self.transport_high_z = getattr(self, 'alternative_nominal_high_z', self.transport_high_z)
        self.transport_route_attempt = 0
        self.state = self.TRANSPORT_PLANNING
        self.get_logger().warning('slot transfer: original yaw unresolved after 5 s; retrying yaw +180 at high waypoint (planning only)')
        if not self._try_transport_alternative('slot yaw +180', observation_first=True):
            self._fault('slot yaw fallback could not initialize; item remains held')
        return True

    def _route_candidates(self):
        routes = waypoint_candidates(getattr(self, 'transport_expanded_waypoints_enabled', True))
        if getattr(self, 'transport_target', {}).get('transfer_context') == 'staging_store':
            site = 'elbow' if getattr(self, 'transport_slot_yaw_flipped', False) else 'destination'
            return tuple((x, y, site) for x, y in dict.fromkeys((r[0], r[1]) for r in routes))
        return routes

    @staticmethod
    def _uses_buffer_route(target, pickup_snapshot):
        """Buffer destinations and buffer-origin grasps go via observation XY."""
        source = pickup_snapshot or {}
        return (target.get('transfer_context') == 'staging_store' or
                source.get('pickup_source') == 'buffer' or
                ('pickup_source' not in source and 'staging_slot' in source))

    def _init_transport_alternatives(self):
        self.declare_parameter('transport_moveit_fallback_enabled', False)
        self.transport_moveit_fallback_enabled = bool(
            self.get_parameter('transport_moveit_fallback_enabled').value)
        self.declare_parameter('transport_expanded_waypoints_enabled', True)
        self.transport_expanded_waypoints_enabled = bool(
            self.get_parameter('transport_expanded_waypoints_enabled').value)
        self.declare_parameter('transport_waypoint_search_timeout_sec', 180.0)
        self.transport_waypoint_search_timeout_sec = float(
            self.get_parameter('transport_waypoint_search_timeout_sec').value)
        if (not math.isfinite(self.transport_waypoint_search_timeout_sec) or
                not 1 <= self.transport_waypoint_search_timeout_sec <= 600):
            raise ValueError('transport_waypoint_search_timeout_sec must be in [1, 600]')
        self.declare_parameter('transport_workspace_z_max_mm', 800.0)
        self.transport_workspace_z_max_mm = float(
            self.get_parameter('transport_workspace_z_max_mm').value)
        if (not math.isfinite(self.transport_workspace_z_max_mm) or
                self.transport_workspace_z_max_mm < 0):
            raise ValueError('transport_workspace_z_max_mm must be nonnegative; 0 disables the transfer ceiling')
        self.declare_parameter('raised_pre_pick_max_mm', 30.0)
        self.raised_pre_pick_max_mm = float(self.get_parameter('raised_pre_pick_max_mm').value)
        if not math.isfinite(self.raised_pre_pick_max_mm) or not 0 <= self.raised_pre_pick_max_mm <= 50:
            raise ValueError('raised_pre_pick_max_mm must be in [0, 50]')
        self.declare_parameter('raised_pre_place_max_mm', 30.0)
        self.raised_pre_place_max_mm = float(self.get_parameter('raised_pre_place_max_mm').value)
        if not math.isfinite(self.raised_pre_place_max_mm) or not 0 <= self.raised_pre_place_max_mm <= 50:
            raise ValueError('raised_pre_place_max_mm must be in [0, 50]')
        self.declare_parameter('transport_observation_y_offset_mm', 200.0)
        self.transport_observation_y_offset_mm = float(
            self.get_parameter('transport_observation_y_offset_mm').value)
        if not math.isfinite(self.transport_observation_y_offset_mm):
            raise ValueError('transport_observation_y_offset_mm must be finite')
        self.declare_parameter('continuous_transport_alternatives_enabled', True)
        self.transport_alternatives_enabled = bool(
            self.get_parameter('continuous_transport_alternatives_enabled').value)
        self.declare_parameter('transport_lower_waypoint_search_enabled', True)
        self.transport_lower_waypoint_search_enabled = bool(
            self.get_parameter('transport_lower_waypoint_search_enabled').value)
        self.transport_motion_plan = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.transport_route_attempt = 0
        self.transport_route_generation = 0

    def _try_transport_alternative(self, reason, observation_first=False):
        if (observation_first and (self.transport_route_attempt != 0 or
                                  self.state != self.TRANSPORT_PLANNING)):
            return False
        if ((not getattr(self, 'transport_alternatives_enabled', False) and not observation_first) or
                getattr(self, 'transport_is_return', False) or
                getattr(self, 'transport_is_pick', False) or
                self.state not in (self.TRANSPORT_PLANNING, self.TRANSPORT_VALIDATING,
                                   self.TRANSPORT_DIAGNOSING) or
                getattr(self, 'direct_transfer_motion_started', False) or
                getattr(self, 'transfer_goal_handle', None) is not None or
                not getattr(self, 'transport_scene', None) or
                not self.transport_scene.get('attached_item_id') or
                self.transport_route_attempt >= 2):
            return False
        if self.transport_route_attempt == 0:
            self.alternative_rail_index = 0
            self.alternative_yaw_direction = 1
            self.alternative_grid_deadline = time.monotonic() + GRID_SECONDS
            self.transport_alternative_deadline = time.monotonic() + getattr(
                self, 'transport_waypoint_search_timeout_sec', 180.)
            self.alternative_nominal_high_z = self.transport_high_z
            self.alternative_height_index = 0
            self.alternative_heights = self._lower_waypoint_heights()
        elif time.monotonic() >= getattr(self, 'transport_alternative_deadline', math.inf):
            self.get_logger().warning('transport alternatives reached the total planning budget')
            return False
        candidate_count = len(self._route_candidates())
        heights = getattr(self, 'alternative_heights', [self.transport_high_z])
        height_index = getattr(self, 'alternative_height_index', 0)
        lower_available = height_index + 1 < len(heights)
        grid_expired = time.monotonic() >= getattr(self, 'alternative_grid_deadline', math.inf)
        grid_exhausted = not lower_available and getattr(self, 'alternative_candidate_index', 0) + 1 >= candidate_count
        if self.transport_route_attempt == 1 and (grid_expired or grid_exhausted):
            phase = next_grid(getattr(self, 'alternative_rail_index', 0),
                              getattr(self, 'alternative_yaw_direction', 1))
            if phase is not None:
                self.alternative_rail_index, self.alternative_yaw_direction = phase
                self.alternative_grid_deadline = time.monotonic() + GRID_SECONDS
                self.alternative_candidate_index = -1
                height_index = len(heights)-1
                lower_available = False
                self.get_logger().warning(
                    f'grid ended; rail={self.alternative_rail_index}, yaw direction='
                    f'{self.alternative_yaw_direction:+d}, next grid budget=5 s')
            else:
                if grid_exhausted:
                    self.alternative_grid_deadline = time.monotonic()
                if self._try_slot_yaw_flip(time.monotonic()):
                    return True
                if not getattr(self, 'transport_moveit_fallback_enabled', False):
                    return False
                lower_available = False
                self.alternative_candidate_index = candidate_count - 1
        retry_observation = (
            self.transport_route_attempt == 1 and
            getattr(self, 'alternative_observation', None) is not None and
            (lower_available or self.alternative_candidate_index + 1 < candidate_count))
        if (self.transport_route_attempt == 1 and not retry_observation and
                not getattr(self, 'transport_moveit_fallback_enabled', False)):
            self.get_logger().warning(
                'overhead waypoint search exhausted; sampling-planner fallback disabled; '
                'holding item without executing an unchecked route')
            return False
        if retry_observation:
            if lower_available:
                self.alternative_height_index = height_index + 1
            else:
                self.alternative_height_index = 0
                self.alternative_candidate_index += 1
        else:
            self.transport_route_attempt += 1
            self.alternative_candidate_index = 0
            self.alternative_height_index = 0
            self.alternative_observation = None
        self.transport_high_z = heights[self.alternative_height_index]
        self.alternative_diagnostic_pending = False
        self.transport_route_generation += 1  # Ignore late callbacks from discarded candidates.
        self.transport_started = time.monotonic()
        self.transport_descent_time = None
        self.state = self.TRANSPORT_PLANNING
        self.alternative_parts = []
        self.alternative_seed = tuple(self.transport_seed)
        if getattr(self, 'raised_pre_place_offset_m', 0.) > 0:
            self.transport_end = np.array(self.transport_target['pre_place_tcp_xyz_m'], dtype=float)
            self.raised_pre_place_offset_m = 0.
        route = 'overhead waypoint alternative' if self.transport_route_attempt == 1 else 'MoveIt overhead planning'
        if observation_first:
            self.get_logger().info('buffer transfer: planning lift -> above observation -> above destination -> pre-place')
        else:
            self.get_logger().warning(f'transport candidate rejected: {reason}; trying {route} (planning only)')
        try:
            if self.transport_route_attempt == 1:
                if retry_observation:
                    self._alternative_prepare(self.alternative_observation)
                    return True
                with open(self.return_waypoint_file, encoding='utf-8') as stream:
                    observation = yaml.safe_load(stream)['waypoints']['observation']
                mapping = dict(zip(observation['joint_names'], observation['positions_rad']))
                joints = tuple(float(mapping[n]) for n in self.arm_joint_names)
                if any(not math.isfinite(v) or not self.transport_joint_limits[n][0] <= v <=
                       self.transport_joint_limits[n][1] for n, v in zip(self.arm_joint_names, joints)):
                    raise ValueError('invalid saved observation joints')
                self._transport_fk_request(joints, self._alternative_observation_fk)
            else:
                if not self.transport_motion_plan.service_is_ready():
                    raise ValueError('MoveIt motion planning service unavailable')
                self._alternative_prepare(None)
        except Exception as exc:
            self._alternative_failed(str(exc))
        return True

    def _lower_waypoint_heights(self):
        """Search the full 100 mm range; execution validation stays unchanged."""
        high = float(self.transport_high_z)
        if not getattr(self, 'transport_lower_waypoint_search_enabled', True):
            return [high]
        return [high - step*.01 for step in range(11)]

    def _alternative_failed(self, reason):
        label = getattr(self, 'alternative_segment_label', 'route setup')
        self.get_logger().warning(f'transport {label} rejected: {reason}')
        if not self._try_transport_alternative(reason):
            if self._try_sdk_transport(reason):
                return
            self._fault(f'KINEMATIC_REJECTED: transport alternatives exhausted at {label}: {reason}')

    def _alternative_observation_fk(self, future):
        try:
            xyz, _ = self._transport_pose(future.result())
            self.alternative_observation = np.array(xyz, copy=True)
            self._alternative_prepare(xyz)
        except Exception as exc:
            self._alternative_failed('observation waypoint unavailable: ' + str(exc))

    def _alternative_prepare(self, observation):
        start, end = self.transport_start_xyz, self.transport_end
        qa, qb = self.transport_start_q, self.transport_end_q
        fixed_slot = getattr(self, 'transport_target', {}).get('transfer_context') == 'staging_store'
        if fixed_slot and not getattr(self, 'transport_slot_yaw_flipped', False):
            if abs(float(quaternion(qa) @ quaternion(qb))) < math.cos(math.radians(.5)/2):
                raise ValueError('TCP orientation changed since slot-store capture; prepare a new target')
            qa = qb  # Hold the captured orientation; tolerate only tracking error at start.
        high = self.transport_high_z
        self.get_logger().info(
            f'overhead height trial {getattr(self, "alternative_height_index", 0)+1}/'
            f'{len(getattr(self, "alternative_heights", [high]))}: TCP Z={high*1000:.1f} mm, '
            f'minimum safe TCP Z={self.transport_safe_z*1000:.1f} mm; clearance unchanged')
        if not math.isfinite(high) or high > self._transport_ceiling():
            raise ValueError('alternative overhead height exceeds workspace ceiling')
        above_start, above_end = np.array([*start[:2], high]), np.array([*end[:2], high])
        self.alternative_current_xyz = np.array(start)
        self.alternative_current_q = qa
        self.alternative_segment_number = 0
        self.alternative_segments = [('cartesian', above_start, qa)]
        if observation is not None:
            candidates = self._route_candidates()
            offset, y_offset, rotate_at = candidates[self.alternative_candidate_index]
            elbow, via = buffer_transfer_waypoints(start, end, observation, high, offset)
            if getattr(self, 'alternative_rail_index', 0) == 1:
                elbow[0] = via[0] = -.350 + offset
            elbow[1] += y_offset
            via[1] += y_offset
            self.get_logger().info(
                f'observation route {self.alternative_candidate_index+1}/{len(candidates)}, '
                f'rotate at {rotate_at}; waypoint in link_base: '
                f'xyz=({via[0]:.4f}, {via[1]:.4f}, {via[2]:.4f}) m; '
                f'rail={"base X -350 mm" if getattr(self, "alternative_rail_index", 0) else "observation X -150 mm"}; '
                f'candidate X/Y offsets={offset*1000:.1f}/{y_offset*1000:.1f} mm; '
                'no roll/pitch detour')
            # Keep the pickup orientation during travel to the overhead via.
            # Rotation is a separate checked Cartesian leg, never near the box stack.
            self.get_logger().info(
                f'X-first overhead waypoint: xyz=({elbow[0]:.4f}, {elbow[1]:.4f}, {elbow[2]:.4f}) m')
            overhead = [elbow, via, above_end]
            rotate_index = {'elbow': 0, 'via': 1, 'destination': 2}[rotate_at]
            for index, point in enumerate(overhead):
                self.alternative_segments.append(('cartesian', point, qa if index <= rotate_index else qb))
                if index == rotate_index and (not fixed_slot or getattr(self, 'transport_slot_yaw_flipped', False)):
                    self.alternative_segments.append(('cartesian', point, qb))
        else:
            self.alternative_segments += [('moveit', above_end, qb)]
        self.alternative_segments += [('cartesian', end, qb)]
        if observation is not None:
            self.alternative_segments = blend_translation_segments(self.alternative_segments, qa)
        self._alternative_next()

    def _alternative_next(self):
        try:
            if not self.alternative_segments:
                trajectory = join_trajectories(self.alternative_parts, self.arm_joint_names)
                if getattr(self, 'transport_sdk_candidate', False):
                    self._sdk_validate_trajectory(trajectory)
                    return
                self.get_logger().info('complete alternative planned; validating the full timed path before execution')
                result = SimpleNamespace(error_code=SimpleNamespace(val=1), fraction=1.,
                                         solution=SimpleNamespace(joint_trajectory=trajectory))
                self._transport_planned(SimpleNamespace(result=lambda: result))
                return
            kind, xyz, q = self.alternative_segments.pop(0)
            # Preserve the unexpanded route, not just its final endpoint.
            # A blended segment may include the entire overhead transfer.
            self.alternative_active_segment = copy.deepcopy((kind, xyz, q))
            path = None
            if kind == 'cartesian_blend':
                path = rounded_translation_waypoints(
                    self.alternative_current_xyz, xyz, q, self.transport_safe_z,
                    radius=getattr(self, 'transport_radius', .04))
                xyz = np.asarray(xyz[-1])
            self.alternative_segment_number = getattr(self, 'alternative_segment_number', 0) + 1
            rotation = np.linalg.norm(xyz-self.alternative_current_xyz) < 1e-8
            if (rotation and kind == 'cartesian' and
                    abs(float(quaternion(q) @ quaternion(self.alternative_current_q))) < 1-1e-12):
                path = [(np.array(xyz), directed_slerp(
                    self.alternative_current_q, q, u,
                    getattr(self, 'alternative_yaw_direction', 1)))
                    for u in np.linspace(0., 1., 129)]
            self.alternative_segment_label = (
                f'route {self.transport_route_attempt} candidate '
                f'{getattr(self, "alternative_candidate_index", 0)+1} '
                f'segment {self.alternative_segment_number} ({kind}, '
                f'{"rotation" if rotation else "travel"})')
            self.alternative_destination = (np.array(xyz), quaternion(q))
            if (not path and np.linalg.norm(xyz-self.alternative_current_xyz) < 1e-8 and
                    abs(float(quaternion(q) @ quaternion(self.alternative_current_q))) > 1-1e-12):
                self._alternative_next()
                return
            self.get_logger().info(
                f'planning {self.alternative_segment_label}: '
                f'xyz=({xyz[0]:.4f},{xyz[1]:.4f},{xyz[2]:.4f}) m')
            if kind == 'moveit':
                self._alternative_moveit(xyz, q)
                return
            if kind == 'sdk_joint':
                self._sdk_plan_joint(xyz, q)
                return
            req = GetCartesianPath.Request()
            req.header.frame_id = 'link_base'
            req.group_name, req.link_name = self.planning_group, self.ik_link_name
            req.start_state.is_diff = True  # Retain attached box and the current planning scene.
            req.start_state.joint_state.name = list(self.arm_joint_names)
            req.start_state.joint_state.position = list(self.alternative_seed)
            req.waypoints = ([pose_message(p, orientation) for p, orientation in path]
                             if path else [pose_message(xyz, q)])
            req.max_step, req.jump_threshold, req.avoid_collisions = .005, 0., True
            scale = max(.05, min(1., self.motion_speed_percent/100))
            req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = scale
            self.transport_cartesian_request = copy.deepcopy(req)
            self.transport_cartesian.call_async(req).add_done_callback(
                self._transport_guard(self._alternative_cartesian_received))
        except Exception as exc:
            self._alternative_failed(str(exc))

    def _alternative_cartesian_received(self, future):
        try:
            result = future.result()
            if (result is None or result.error_code.val != 1 or
                    not math.isfinite(result.fraction) or not 1-1e-6 <= result.fraction <= 1.):
                fraction = None if result is None else result.fraction
                if self._partial_is_final_descent(result) and self._try_raised_pre_place():
                    return
                if (result is not None and result.error_code.val == 1 and
                        math.isfinite(result.fraction) and 0. <= result.fraction < 1.):
                    self._diagnose_partial_path(result, alternative=True)
                    return
                code = None if result is None else result.error_code.val
                raise ValueError(f'alternative Cartesian segment incomplete (code={code}, fraction={fraction})')
            self._alternative_segment_ready(result.solution.joint_trajectory)
        except Exception as exc:
            self._alternative_failed(str(exc))

    def _partial_is_final_descent(self, result):
        """Conservatively classify the requested interval, not an exact failure.

        MoveIt reports a fraction, not the failed internal interpolation sample.
        Permit height retries only inside a straight descending suffix, with
        both interval endpoints already below the overhead transition.
        """
        request = getattr(self, 'transport_cartesian_request', None)
        if (result is None or result.error_code.val != 1 or
                not math.isfinite(result.fraction) or not 0. <= result.fraction < 1. or
                request is None or not request.waypoints):
            return False
        points = [np.array([p.position.x, p.position.y, p.position.z])
                  for p in request.waypoints]
        end = points[-1]
        start = np.asarray(self.alternative_current_xyz)
        index = min(len(points)-1, int(result.fraction*len(points)))
        previous = start if index == 0 else points[index-1]
        suffix = [previous] + points[index:]
        column = all(np.linalg.norm(p[:2]-end[:2]) <= .002 for p in suffix)
        descending = all(b[2] <= a[2]+1e-6 for a, b in zip(suffix, suffix[1:]))
        below_transition = max(previous[2], points[index][2]) < self.transport_safe_z-.001
        final = column and descending and below_transition
        if not final:
            self.get_logger().warning(
                f'partial path before confirmed final descent (requested interval '
                f'{index+1}/{len(points)}); skipping pre-place height retries; '
                'diagnosing and trying another overhead route')
        return final

    def _try_raised_pre_place(self):
        """Retry only the final descent, retaining the validated overhead prefix.

        No partial trajectory is executed. The combined candidate still passes
        the normal timed-path collision and geometry checks.
        """
        if (self.transport_route_attempt not in (1, 2) or self.alternative_segments or
                getattr(self, 'transport_is_pick', False) or
                getattr(self, 'transport_is_return', False) or
                getattr(self, 'transport_target', {}).get('transfer_context') != 'pallet' or
                self.state != self.TRANSPORT_PLANNING or
                getattr(self, 'direct_transfer_motion_started', False)):
            return False
        offset = getattr(self, 'raised_pre_place_offset_m', 0.) + .005
        if offset * 1000 > getattr(self, 'raised_pre_place_max_mm', 0.) + 1e-6:
            return False
        original = np.array(self.transport_target['pre_place_tcp_xyz_m'], dtype=float)
        candidate = original + np.array([0., 0., offset])
        if candidate[2] >= self.transport_high_z or candidate[2] > self._transport_ceiling():
            return False
        active = getattr(self, 'alternative_active_segment', None)
        if active is None:
            return False
        kind, points, orientation = copy.deepcopy(active)
        if kind == 'cartesian_blend':
            # Replan from the last fully accepted seed through ALL original
            # overhead waypoints. Never splice in an incomplete solution.
            points = [np.array(point, dtype=float) for point in points]
            if len(points) < 2 or not np.allclose(points[-1][:2], original[:2]):
                return False
            points[-1] = candidate
            retry = (kind, points, orientation)
        elif kind == 'cartesian':
            # A single-point retry is valid only for a vertical final leg.
            if not np.allclose(self.alternative_current_xyz[:2], original[:2], atol=.002, rtol=0):
                return False
            retry = (kind, candidate, orientation)
        else:
            return False
        self.raised_pre_place_offset_m = offset
        self.transport_end = candidate
        self.alternative_segments = [retry]
        self.get_logger().warning(
            f'final descent incomplete; testing pre-place +{offset*1000:.0f} mm (planning only)')
        self._alternative_next()
        return True

    def _alternative_segment_ready(self, trajectory):
        from .continuous_transport import cartesian_timing_issue
        issue = cartesian_timing_issue(trajectory, self.arm_joint_names)
        if issue:
            raise ValueError(issue)
        indices = [list(trajectory.joint_names).index(n) for n in self.arm_joint_names]
        if max(abs(trajectory.points[0].positions[i]-v)
               for i, v in zip(indices, self.alternative_seed)) > 1e-6:
            raise ValueError('alternative planner changed its requested joint start')
        self.alternative_parts.append(copy.deepcopy(trajectory))
        self.alternative_seed = tuple(trajectory.points[-1].positions[i] for i in indices)
        # Use actual segment endpoint FK, not the requested pose, for the next leg.
        self._transport_fk_request(self.alternative_seed, self._alternative_segment_fk)

    def _alternative_segment_fk(self, future):
        try:
            xyz, q = self._transport_pose(future.result())
            target, orientation = self.alternative_destination
            if np.linalg.norm(xyz-target) > .002 or abs(float(q @ orientation)) < math.cos(.005/2):
                raise ValueError('alternative segment endpoint verification failed')
            self.alternative_current_xyz, self.alternative_current_q = xyz, q
            self._alternative_next()
        except Exception as exc:
            self._alternative_failed(str(exc))

    def _transport_ceiling(self):
        limit = getattr(self, 'transport_workspace_z_max_mm', self.servo_bounds_mm[5])
        return math.inf if limit == 0 else limit/1000.

    def _alternative_moveit(self, xyz, q):
        if not getattr(self, 'transport_moveit_fallback_enabled', False):
            raise ValueError('MoveIt sampling-planner fallback is disabled')
        req = GetMotionPlan.Request()
        plan = req.motion_plan_request
        plan.group_name = self.planning_group
        plan.pipeline_id, plan.planner_id = 'ompl', 'RRTConnectkConfigDefault'
        plan.allowed_planning_time, plan.num_planning_attempts = 5., 3
        scale = max(.05, min(1., self.motion_speed_percent/100))
        plan.max_velocity_scaling_factor = plan.max_acceleration_scaling_factor = scale
        plan.start_state.is_diff = True
        plan.start_state.joint_state.name = list(self.arm_joint_names)
        plan.start_state.joint_state.position = list(self.alternative_seed)

        def position(region, center):
            c = PositionConstraint()
            c.header.frame_id, c.link_name, c.weight = 'link_base', self.ik_link_name, 1.
            c.constraint_region.primitives = [region]
            c.constraint_region.primitive_poses = [pose_message(center, [0, 0, 0, 1])]
            return c

        def orientation(tol_xy, tol_z):
            c = OrientationConstraint()
            c.header.frame_id, c.link_name, c.weight = 'link_base', self.ik_link_name, 1.
            c.orientation = pose_message(xyz, q).orientation
            c.parameterization = OrientationConstraint.ROTATION_VECTOR
            c.absolute_x_axis_tolerance = c.absolute_y_axis_tolerance = tol_xy
            c.absolute_z_axis_tolerance = tol_z
            return c

        goal = Constraints()
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.001])
        goal.position_constraints = [position(sphere, xyz)]
        goal.orientation_constraints = [orientation(.002, .002)]
        plan.goal_constraints = [goal]
        # Free orientation only on this overhead leg. Its exact goal orientation
        # is still required before the fixed-orientation Cartesian descent.
        # Rotated carried-item corner clearance is checked on the final spline;
        # the TCP position region alone cannot guarantee item clearance.
        floor, ceiling = self.transport_safe_z, self._transport_ceiling()
        if math.isinf(ceiling):
            # MoveIt requires a finite box to express the lower clearance plane.
            # Ten metres is beyond this fixed-base UF arm's reachable workspace;
            # it is not an execution ceiling. Joint/collision checks still apply.
            ceiling = floor + 10.
        if floor >= ceiling:
            raise ValueError('no overhead workspace for MoveIt transfer')
        region = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[4., 4., ceiling-floor])
        plan.path_constraints.position_constraints = [position(region, [0., 0., (floor+ceiling)/2])]
        plan.path_constraints.orientation_constraints = []
        self.get_logger().info(
            'MoveIt overhead fallback: free path orientation; exact destination orientation retained; '
            f'transfer TCP ceiling={self._transport_ceiling()} m (inf=disabled)')
        self.transport_motion_plan.call_async(req).add_done_callback(
            self._transport_guard(self._alternative_moveit_received))

    def _alternative_moveit_received(self, future):
        try:
            response = future.result()
            result = None if response is None else response.motion_plan_response
            if result is None or result.error_code.val != 1:
                code = None if result is None else result.error_code.val
                raise ValueError(f'MoveIt overhead planning failed (code={code})')
            self._alternative_segment_ready(result.trajectory.joint_trajectory)
        except Exception as exc:
            self._alternative_failed(str(exc))
