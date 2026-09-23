"""Read-only top-face diagnostics. Metres, radians, optical-camera convention."""
import math
import cv2
import numpy as np


def transform(position, quaternion):
    q = np.asarray(quaternion, dtype=float)
    p = np.asarray(position, dtype=float)
    if q.shape != (4,) or p.shape != (3,) or not np.isfinite(np.r_[q, p]).all():
        raise ValueError('invalid rigid pose')
    if np.linalg.norm(q) < 1e-8:
        raise ValueError('zero quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = p
    return t


def apply(t, points):
    return np.asarray(points) @ t[:3, :3].T + t[:3, 3]


def top_points(size, base_from_box):
    size = np.asarray(size, dtype=float)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError('box size must be three positive finite metres')
    x, y, z = size / 2
    return apply(base_from_box, [[0, 0, z], [-x, -y, z], [x, -y, z],
                                 [x, y, z], [-x, y, z]])


def project(points_base, camera_from_base, k, d):
    points = apply(camera_from_base, points_base)
    if not np.isfinite(points).all() or np.any(points[:, 2] <= .001):
        raise ValueError('target is behind/at the camera; no valid projection')
    return cv2.projectPoints(points, np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2)


def prompts(pixels, width, height, margin=12):
    """Require the full predicted top face in view; do not clip a wrong target."""
    p = np.asarray(pixels)
    if (not np.isfinite(p).all() or np.any(p < margin) or
            np.any(p[:, 0] >= width-margin) or np.any(p[:, 1] >= height-margin)):
        raise ValueError('predicted top face is outside the image margin; reposition camera first')
    lo, hi = p[1:].min(axis=0), p[1:].max(axis=0)
    if np.min(hi-lo) < 8:
        raise ValueError('projected top face is too small')
    # Inset points reduce sensitivity to corners being slightly displaced.
    positive = np.vstack([p[0], p[0] + .5*(p[1:]-p[0])])
    box = np.r_[np.maximum(lo-8, 0), np.minimum(hi+8, [width-1, height-1])]
    return positive, box


def fit_top(mask, k, d, base_from_camera, prior_points, size, prior_yaw):
    """Intersect the SAM contour rays with the recorded horizontal top plane.

    No measured depth is accepted or used. Height is a prior, not an estimate.
    """
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2 or mask.sum() < 120:
        raise ValueError('fewer than 120 segmented pixels')
    if mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any():
        raise ValueError('segmented top face touches image boundary')
    prior = np.asarray(prior_points, dtype=float)
    if prior.shape != (5, 3) or not np.isfinite(prior).all():
        raise ValueError('invalid recorded top face')
    height = float(prior[0, 2])
    if np.max(abs(prior[:, 2]-height)) > .005:
        raise ValueError('recorded top face is not horizontal')
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour = max(contours, key=cv2.contourArea)
    component = np.zeros(mask.shape, np.uint8)
    cv2.drawContours(component, [contour], -1, 1, cv2.FILLED)
    if (component.astype(bool) & mask).sum() < .9*mask.sum():
        raise ValueError('SAM mask contains multiple disconnected regions')
    pixels = contour.reshape(-1, 2).astype(float)
    rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k, d).reshape(-1, 2)
    directions = np.c_[rays, np.ones(len(rays))] @ base_from_camera[:3, :3].T
    origin = base_from_camera[:3, 3]
    if (not np.isfinite(directions).all() or not np.isfinite(origin).all() or
            np.any(abs(directions[:, 2])/np.linalg.norm(directions, axis=1) < .01)):
        raise ValueError('camera rays are parallel/too grazing to recorded top plane')
    distance = (height-origin[2])/directions[:, 2]
    if not np.isfinite(distance).all() or np.any(distance <= .001):
        raise ValueError('recorded top plane is behind/at the camera')
    xyz = origin + distance[:, None]*directions
    # Do not clip the contour to the prior box: that would bias center/size.
    center, dims, angle = cv2.minAreaRect(xyz[:, :2].astype(np.float32))
    options = []
    for quarter in range(4):
        footprint = np.array(dims if quarter % 2 == 0 else dims[::-1])
        error = float(np.max(abs(footprint-np.asarray(size)[:2])))
        yaw = math.radians(angle) + quarter*math.pi/2
        delta = (yaw-prior_yaw+math.pi) % (2*math.pi)-math.pi
        if error <= .025:
            options.append((abs(delta), error, prior_yaw+delta))
    if not options:
        raise ValueError('segmented footprint differs from recorded size by more than 25 mm')
    _, size_error, yaw = min(options)
    center3 = np.array([*center, height])
    if np.linalg.norm(center3[:2]-prior[0, :2]) > .05:
        raise ValueError('estimated center moved more than 50 mm; target association uncertain')
    return dict(top_center_base_m=center3.tolist(), yaw_rad=float(yaw),
                delta_center_m=(center3-prior[0]).tolist(),
                delta_yaw_deg=float(math.degrees(yaw-prior_yaw)),
                footprint_error_mm=size_error*1000,
                contour_points=len(pixels), recorded_top_z_m=height,
                estimation_method='rgb_mask_recorded_top_plane', uses_measured_depth=False,
                diagnostic_only=True)
