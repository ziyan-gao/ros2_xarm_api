"""One planning-only retry for a complete, untimed outbound Cartesian path."""
import copy
import math
import time
from types import SimpleNamespace

import numpy as np


def split_transport_request(request):
    """Retain every pose, splitting at the first/last overhead samples.

    The rounded corners and orientations are deliberately not regenerated.
    Each separately timed part will start/end at rest.
    """
    poses = request.waypoints
    if len(poses) < 3:
        raise ValueError('not enough Cartesian waypoints for staged timing')
    heights = [p.position.z for p in poses]
    if not all(math.isfinite(z) for z in heights):
        raise ValueError('invalid Cartesian waypoint height')
    top = max(heights)
    overhead = [i for i, z in enumerate(heights) if abs(z-top) < 1e-6]
    cuts = sorted({i+1 for i in (overhead[0], overhead[-1]) if 0 < i < len(poses)-1})
    if not cuts:
        cuts = [len(poses)//2]
    parts, start = [], 0
    for end in cuts + [len(poses)]:
        part = copy.deepcopy(request)
        part.waypoints = copy.deepcopy(poses[start:end])
        parts.append(part)
        start = end
    return parts


class StagedTransportTiming:
    def _fallback_staged_transport_timing(self, reason):
        if getattr(self, 'transport_moveit_pipeline_id', '') == 'isaac_ros_cumotion':
            return False  # Free-space SDK/Cartesian fallback is disabled.
        # Return/pick, cache connectors and alternative/SDK routes have their
        # own planners. Never accidentally replay their last segment as a
        # complete outbound route. Execution failures are never retried here.
        if (self.state != self.TRANSPORT_PLANNING or
                getattr(self, 'transport_is_return', False) or
                getattr(self, 'transport_is_pick', False) or
                getattr(self, 'transport_via_observation', False) or
                getattr(self, 'transport_route_attempt', 0) != 0 or
                getattr(self, 'transport_sdk_candidate', False) or
                self._reuse_is_active() or
                getattr(self, 'staged_timing_used', False) or
                getattr(self, 'direct_transfer_motion_started', False) or
                getattr(self, 'transfer_goal_handle', None) is not None):
            return False
        request = getattr(self, 'transport_cartesian_request', None)
        if request is None:
            return False
        self.staged_timing_used = True
        try:
            self.staged_timing_requests = split_transport_request(request)
            self.staged_timing_parts = []
            self.staged_timing_seed = tuple(self.transport_seed)
            self.staged_timing_active = True
            self.transport_started = time.monotonic()
            self.get_logger().warning(
                f'{reason}; planning staged transport timing once '
                f'({len(self.staged_timing_requests)} parts, original waypoints retained; no motion yet)')
            self._staged_timing_next()
        except Exception as exc:
            self._staged_timing_failed(exc)
        return True

    def _staged_timing_failed(self, reason):
        self.staged_timing_active = False
        self._fault(f'staged transport timing failed: {reason}; no automatic motion retry or release')

    def _staged_timing_next(self):
        from .transport_alternatives import join_trajectories
        if not self.staged_timing_requests:
            trajectory = join_trajectories(self.staged_timing_parts, self.arm_joint_names)
            self.staged_timing_active = False
            self.get_logger().info('staged timing complete; validating full trajectory before execution')
            result = SimpleNamespace(error_code=SimpleNamespace(val=1), fraction=1.,
                                     solution=SimpleNamespace(joint_trajectory=trajectory))
            self._transport_planned(SimpleNamespace(result=lambda: result))
            return
        request = self.staged_timing_requests.pop(0)
        request.start_state.joint_state.name = list(self.arm_joint_names)
        request.start_state.joint_state.position = list(self.staged_timing_seed)
        self.staged_timing_endpoint = request.waypoints[-1]
        self.transport_cartesian.call_async(request).add_done_callback(
            self._transport_guard(self._staged_timing_received))

    def _staged_timing_received(self, future):
        from .continuous_transport import cartesian_timing_issue
        try:
            result = future.result()
            if (result is None or result.error_code.val != 1 or
                    not math.isfinite(result.fraction) or not 1-1e-6 <= result.fraction <= 1.):
                raise ValueError('staged Cartesian segment is incomplete')
            trajectory = result.solution.joint_trajectory
            issue = cartesian_timing_issue(trajectory, self.arm_joint_names)
            if issue:
                raise ValueError(issue)
            indices = [list(trajectory.joint_names).index(n) for n in self.arm_joint_names]
            if max(abs(trajectory.points[0].positions[i]-v)
                   for i, v in zip(indices, self.staged_timing_seed)) > 1e-6:
                raise ValueError('staged planner changed its requested joint start')
            for point in trajectory.points:
                for name, i in zip(self.arm_joint_names, indices):
                    lo, hi = self.transport_joint_limits[name][:2]
                    if not lo <= point.positions[i] <= hi:
                        raise ValueError(f'staged trajectory exceeds {name} limits')
            self.staged_timing_parts.append(copy.deepcopy(trajectory))
            self.staged_timing_seed = tuple(trajectory.points[-1].positions[i] for i in indices)
            self._transport_fk_request(self.staged_timing_seed, self._staged_timing_fk)
        except Exception as exc:
            self._staged_timing_failed(exc)

    def _staged_timing_fk(self, future):
        try:
            xyz, q = self._transport_pose(future.result())
            p = self.staged_timing_endpoint
            target = np.array([p.position.x, p.position.y, p.position.z])
            orientation = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            orientation /= np.linalg.norm(orientation)
            if (not np.isfinite(orientation).all() or np.linalg.norm(xyz-target) > .002 or
                    abs(float(q @ orientation)) < math.cos(.005/2)):
                raise ValueError('staged segment endpoint FK mismatch')
            self._staged_timing_next()
        except Exception as exc:
            self._staged_timing_failed(exc)
