"""Display-only frozen SAM mask projection; no motion or depth estimation."""
import cv2
import numpy as np


def projected_mask(payload, stamp, camera_from_base, k, d, shape):
    age = stamp - float(payload['color_stamp'])
    if not 0 <= age < 10.:
        return None
    contours = []
    polygons = payload['polygons']
    if len(polygons) > 256:
        raise ValueError('too many mask contours')
    for polygon in polygons:
        points = np.asarray(polygon, float)
        if points.ndim != 2 or points.shape[1] != 3 or not 3 <= len(points) <= 10000:
            raise ValueError('invalid mask polygon')
        points = points @ camera_from_base[:3, :3].T + camera_from_base[:3, 3]
        if not np.isfinite(points).all() or np.any(points[:, 2] <= .001):
            return None
        pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), k, d)
        if not np.isfinite(pixels).all() or np.max(abs(pixels)) > 1e6:
            return None
        contours.append(np.rint(pixels).astype(np.int32))
    mask = np.zeros(shape[:2], np.uint8)
    if contours:
        cv2.fillPoly(mask, contours, 1)  # even-odd fill preserves holes
    return mask.astype(bool)
