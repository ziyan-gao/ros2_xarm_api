"""Slot-local depth geometry. All lengths are metres, angles radians."""
import math
import cv2
import numpy as np


def inspection_position(xyz, observation_z, camera_offset_base, backoff):
    """Back away from the EEF-to-camera mounting offset in base XY only."""
    if not math.isfinite(backoff) or not 0 <= backoff <= .3:
        raise ValueError('inspection backoff must be between 0 and 300 mm')
    result = np.array([xyz[0], xyz[1], observation_z], dtype=float)
    offset = np.asarray(camera_offset_base, dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError('invalid EEF-to-camera mounting offset')
    distance = np.linalg.norm(offset[:2])
    if backoff > 0:
        if distance < 1e-6:
            raise ValueError('camera mounting offset has no usable XY direction')
        result[:2] -= backoff * offset[:2] / distance
    if not np.isfinite(result).all():
        raise ValueError('invalid inspection pose')
    return result


def estimate_slot_top(points, slot, slot_size, size, prior_yaw, *, check_boundary=True,
                      require_footprint_match=True):
    """Fit a top pose; optionally enforce legacy dimension/prior-yaw gates."""
    points = np.asarray(points)
    expected_z = slot[2] + size[2]
    keep = (np.isfinite(points).all(axis=1) &
            (points[:, 0] >= slot[0]) & (points[:, 0] <= slot[0]+slot_size) &
            (points[:, 1] >= slot[1]) & (points[:, 1] <= slot[1]+slot_size) &
            (abs(points[:, 2]-expected_z) <= .025))
    top = points[keep]
    if len(top) < 120:
        raise ValueError('not enough slot top-face depth points')
    z = float(np.median(top[:, 2]))
    top = top[abs(top[:, 2]-z) <= .005]
    if len(top) < 120:
        raise ValueError('unstable top plane')
    plane = np.linalg.lstsq(np.column_stack((top[:, :2], np.ones(len(top)))),
                            top[:, 2], rcond=None)[0]
    if np.linalg.norm(plane[:2]) > math.tan(math.radians(5)):
        raise ValueError('slot item top is not horizontal')
    center, dims, angle = cv2.minAreaRect(top[:, :2].astype(np.float32))
    candidates = []
    for quarter in range(4):
        measured = dims if quarter % 2 == 0 else dims[::-1]
        if require_footprint_match and any(abs(measured[i]-size[i]) > .015 for i in range(2)):
            continue
        yaw = math.radians(angle) + quarter*math.pi/2
        delta = (yaw-prior_yaw+math.pi) % (2*math.pi)-math.pi
        # Dimensions choose the rectangle axis correspondence, not acceptance.
        # Near-square items have ambiguous axes: prefer the recorded direction.
        error = (0. if require_footprint_match or abs(size[0]-size[1]) < .015 else
                 sum((measured[i]-size[i])**2 for i in range(2)))
        candidates.append((error, abs(delta), prior_yaw+delta))
    if not candidates or (require_footprint_match and min(candidates)[1] > math.radians(20)):
        raise ValueError('slot footprint does not match recorded item')
    yaw = min(candidates)[2]
    estimate = np.array([*center, float(np.median(top[:, 2])), yaw])
    if check_boundary:
        validate_slot_boundary(estimate, slot, slot_size, size)
    return estimate


def validate_slot_boundary(estimate, slot, slot_size, size):
    center, yaw = estimate[:2], estimate[3]
    c, s = math.cos(yaw), math.sin(yaw)
    corners = np.array([[center[0]+c*x-s*y, center[1]+s*x+c*y]
                        for x in (-size[0]/2, size[0]/2)
                        for y in (-size[1]/2, size[1]/2)])
    if (np.any(corners < np.asarray(slot[:2])-.005) or
            np.any(corners > np.asarray(slot[:2])+slot_size+.005)):
        raise ValueError('estimated item extends outside selected slot')


def stable_slot_pose(samples, count=20):
    if len(samples) < count:
        return None
    values = np.asarray(samples[-count:])
    if (np.any(np.ptp(values[:, :3], axis=0) > .006) or
            np.ptp(values[:, 3]) > math.radians(3)):
        return None
    return np.median(values, axis=0)
