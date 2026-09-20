"""Shared physical pallet placement records for test and policy execution."""
import math

from .transport_path import rotate


def pallet_record(values):
    """Keep physical (not padded) source geometry and its virtual key."""
    dx, dy, dz = map(float, values[6:9])
    if values[5] > .5:
        dx, dy = dy, dx
    return dict(item_id=int(values[1]), corner_mm=list(values[2:5]),
                size_mm=[dx, dy, dz], virtual_corner_mm=list(values[12:15]))


def retrieval_values(record, sequence, translation, q, clearance):
    x, y, z = record['corner_mm']
    dx, dy, dz = record['size_mm']
    top = rotate(q, [(x+dx/2)/1000, (y+dy/2)/1000, (z+dz)/1000])
    center = [float(translation[i]+top[i]) for i in range(3)]
    qx, qy, qz, qw = q
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    # Same top-centre convention as policy unpacking. Pallet must be level.
    if abs(float(rotate(q, [0, 0, 1])[2])-1.) > 1e-4:
        raise ValueError('pickup requires a horizontal, locked pallet')
    rpy = record.get('release_tcp_rpy_rad')
    if (not isinstance(rpy, (list, tuple)) or len(rpy) != 3 or
            not all(math.isfinite(float(v)) for v in rpy)):
        raise ValueError('recorded pallet release TCP orientation is missing; do not guess pickup yaw')
    # Tool orientation and object orientation are different. Dimensions here
    # describe the footprint in pallet axes, so object yaw remains pallet yaw.
    return [float(sequence), *center, *map(float, rpy),
            dx/1000, dy/1000, dz/1000, .030, yaw,
            max(float(translation[2])+clearance, center[2]+.050), 0.0]
