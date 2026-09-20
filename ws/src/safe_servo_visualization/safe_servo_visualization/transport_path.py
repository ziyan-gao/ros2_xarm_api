"""Pure reusable lift/transfer/descent geometry; no ROS or hardware commands."""
import math
import numpy as np


def quaternion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('invalid transport orientation')
    return q / np.linalg.norm(q)


def rotate(q, v):
    q = quaternion(q)
    v = np.asarray(v, dtype=float)
    return v + 2 * np.cross(q[:3], np.cross(q[:3], v) + q[3] * v)


def slerp(a, b, u):
    a, b = quaternion(a), quaternion(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > 0.9995:
        return quaternion(a + u * (b - a))
    angle = math.acos(np.clip(dot, -1, 1))
    return (math.sin((1-u)*angle)*a + math.sin(u*angle)*b) / math.sin(angle)


def directed_slerp(a, b, u, yaw_direction=1):
    """Explicit base-Z half-turn direction; preserve other endpoint rotations."""
    a, b = quaternion(a), quaternion(b)
    if (abs(float(a @ b)) < 1e-5 and
            np.linalg.norm(rotate(a, (0, 0, 1))-rotate(b, (0, 0, 1))) < 1e-5):
        angle = (1 if yaw_direction >= 0 else -1)*math.pi*u/2
        c, s = math.cos(angle), math.sin(angle)
        x, y, z, w = a
        return np.array([c*x-s*y, c*y+s*x, c*z+s*w, c*w-s*z])
    return slerp(a, b, u)


def item_bottom_offset(q, scene):
    if scene is None:
        return 0.0  # Empty-tool return: clearance is specified for the TCP.
    size = np.asarray(scene['attached_item_size_m'], dtype=float)
    center = np.asarray(scene['attached_item_center_in_tcp_m'], dtype=float)
    iq = scene['attached_item_orientation_in_tcp_xyzw']
    if size.shape != (3,) or center.shape != (3,) or not np.isfinite([size, center]).all() or np.any(size <= 0):
        raise ValueError('invalid carried-item geometry')
    return min(rotate(q, center + rotate(iq, size * np.array([x,y,z]) / 2))[2]
               for x in (-1,1) for y in (-1,1) for z in (-1,1))


def vertical_retreat_waypoints(start, orientation, clearance_z, step=0.005):
    """Empty-tool handoff: raise at fixed XY/orientation, never move downward."""
    start = np.asarray(start, float)
    q = quaternion(orientation)
    if (start.shape != (3,) or not np.isfinite(start).all() or
            not math.isfinite(clearance_z) or not math.isfinite(step) or step <= 0):
        raise ValueError('invalid vertical retreat geometry')
    end = np.array([start[0], start[1], max(float(start[2]), clearance_z)])
    count = max(1, math.ceil(abs(end[2]-start[2])/step))
    samples = [(start+(end-start)*u, q) for u in np.linspace(0, 1, count+1)[1:]]
    return samples, end


def rounded_translation_waypoints(start, destinations, orientation, safe_z,
                                  radius=0.04, step=0.005):
    """Blend a fixed-orientation polyline; only round corners above safe_z.

    Interior vertices are virtual corners, not mandatory stop/exact-pass points.
    Quintic Bezier bends have zero endpoint curvature and tangent to each line.
    The control hull stays above safe_z; low approach/departure columns stay exact.
    """
    points = [np.asarray(p, float) for p in [start, *destinations]]
    q = quaternion(orientation)
    if (len(points) < 2 or any(p.shape != (3,) for p in points) or
            not np.isfinite(points).all() or
            not np.isfinite([safe_z, radius, step]).all() or radius <= 0 or step <= 0):
        raise ValueError('invalid rounded translation geometry')
    unique = [points[0]]
    for p in points[1:]:
        if np.linalg.norm(p-unique[-1]) > 1e-9:
            unique.append(p)
    if len(unique) < 2:
        return []
    samples = []
    def line(a, b):
        length = np.linalg.norm(b-a)
        if length > 1e-9:
            samples.extend((a+(b-a)*u, q) for u in
                           np.linspace(0, 1, max(1, math.ceil(length/step))+1)[1:])
    cursor = unique[0]
    for a, b, c in zip(unique, unique[1:], unique[2:]):
        incoming, outgoing = b-a, c-b
        li, lo = np.linalg.norm(incoming), np.linalg.norm(outgoing)
        incoming, outgoing = incoming/li, outgoing/lo
        r = min(radius, li/4, lo/4)
        # Vertical bends may not start lateral motion below the clearance plane.
        drop = max(0., incoming[2], -outgoing[2])
        if b[2] < safe_z or float(incoming @ outgoing) < -1+1e-6:
            r = 0.  # Low corner / reversal: retain exact geometry and allow a stop.
        elif drop > 1e-9:
            r = min(r, max(0., b[2]-safe_z)/drop)
        if r < 1e-8 or float(incoming @ outgoing) > 1-1e-10:
            line(cursor, b)
            cursor = b
            continue
        entry, leave = b-incoming*r, b+outgoing*r
        line(cursor, entry)
        controls = [entry, entry+incoming*r/4, entry+incoming*r/2,
                    leave-outgoing*r/2, leave-outgoing*r/4, leave]
        polygon_length = sum(np.linalg.norm(y-x) for x, y in zip(controls, controls[1:]))
        for u in np.linspace(0, 1, max(4, math.ceil(polygon_length/step))+1)[1:]:
            position = sum(math.comb(5,i)*(1-u)**(5-i)*u**i*v
                           for i,v in enumerate(controls))
            samples.append((position, q))
        cursor = leave
    line(cursor, unique[-1])
    return samples


def buffer_transfer_waypoints(start, end, observation, high_z, candidate_offset=0.):
    """Shared observation-side rail for loaded transfers and empty approaches."""
    values = np.asarray([start, end, observation], dtype=float)
    if values.shape != (3, 3) or not np.isfinite(values).all() or not np.isfinite([high_z, candidate_offset]).all():
        raise ValueError('invalid buffer transfer waypoints')
    rail_x = values[2, 0] - .150 + candidate_offset
    return np.array([rail_x, values[0, 1], high_z]), np.array([rail_x, values[1, 1], high_z])


def pickup_needs_observation(current_xyz, target_xyz, source):
    """Route across the fixed staging table/pallet areas, not within one area.

    Staging uses base-frame X [-375,375], Y [180,680] mm. Include the
    100 mm camera inspection backoff plus 5 mm tracking tolerance.
    Unknown source types retain the conservative rail route.
    """
    start, end = np.asarray(current_xyz, float), np.asarray(target_xyz, float)
    if start.shape != (3,) or end.shape != (3,) or not np.isfinite([*start, *end]).all():
        raise ValueError('invalid pickup routing positions')
    at_staging = (-.480 <= start[0] <= .480 and .075 <= start[1] <= .785)
    if source == 'pallet':
        return at_staging
    if source == 'buffer':
        return not at_staging
    return True


def grid_pick_waypoints(start, qa, end, qb, clearance, observation, candidate, step=.005,
                       rail_x=None, yaw_direction=1):
    """Complete empty-tool route; preserve final pose and avoid an unnecessary lift."""
    start, end = np.asarray(start, float), np.asarray(end, float)
    qa, qb = quaternion(qa), quaternion(qb)
    high = max(float(clearance)+.002, start[2], end[2])
    dx, dy, site = candidate
    elbow, via = buffer_transfer_waypoints(start, end, observation, high, dx)
    if rail_x is not None:
        if not math.isfinite(rail_x):
            raise ValueError('invalid alternate rail X')
        elbow[0] = via[0] = rail_x + dx
    elbow[1] += dy
    via[1] += dy
    above_start, above_end = np.array([*start[:2], high]), np.array([*end[:2], high])
    rotation_index = {'elbow': 0, 'via': 1, 'destination': 2}[site]
    knots = [(above_start, qa)]
    for i, xyz in enumerate((elbow, via, above_end)):
        knots.append((xyz, qa if i <= rotation_index else qb))
        if i == rotation_index:
            knots.append((xyz, qb))
    knots.append((end, qb))
    samples = []
    previous, orientation = start, qa
    for xyz, q in knots:
        angle = 2*math.acos(min(1., abs(float(orientation @ q))))
        count = max(1, math.ceil(np.linalg.norm(xyz-previous)/step), math.ceil(angle/.025))
        samples.extend((previous+(xyz-previous)*u, directed_slerp(orientation,q,u,yaw_direction))
                       for u in np.linspace(0,1,count+1)[1:])
        previous, orientation = xyz, q
    return samples, float(clearance), high


def pick_waypoints(current_xyz, current_q, pre_pick_xyz, pre_pick_q,
                   clearance_z, radius=0.04, step=0.005, observation_xyz=None):
    """Empty-tool approach: lift here, cross overhead, descend to pre-pick.

    All positions and clearance_z are absolute link_base metres. Neither
    contact nor gripper commands are part of this path. Orientation changes
    only overhead. Close/coincident XY targets still go via the upper level.
    """
    start, end = np.asarray(current_xyz, float), np.asarray(pre_pick_xyz, float)
    qa, qb = quaternion(current_q), quaternion(pre_pick_q)
    if (start.shape != (3,) or end.shape != (3,) or
            not np.isfinite([*start, *end, clearance_z, radius, step]).all() or
            radius <= 0 or step <= 0):
        raise ValueError('invalid pickup approach geometry')
    if observation_xyz is not None:
        safe = max(float(clearance_z), start[2], end[2])
        high = safe + radius + .002
        elbow, via = buffer_transfer_waypoints(start, end, observation_xyz, high)
        above_start, above_end = np.array([*start[:2], high]), np.array([*end[:2], high])
        samples = rounded_translation_waypoints(start, [above_start, elbow, via], qa, safe, radius, step)
        angle = 2*math.acos(min(1., abs(float(qa @ qb))))
        if angle > 1e-8:
            samples.extend((via.copy(), slerp(qa, qb, u**3*(10-15*u+6*u*u)))
                           for u in np.linspace(0, 1, max(1, math.ceil(angle/.025))+1)[1:])
        samples.extend(rounded_translation_waypoints(via, [above_end, end], qb, safe, radius, step))
        return samples, safe, high
    if np.linalg.norm(end[:2]-start[:2]) >= .01:
        return transport_waypoints(start, qa, end, qb, clearance_z, None,
                                   radius, step, allow_tilt_change=True)
    # No rounded lateral corners fit here. Keep the same vertical/overhead
    # contract, allowing the retimer to stop smoothly at the upper knots.
    height = max(start[2], end[2], float(clearance_z) + .002)
    above_start = np.array([*start[:2], height])
    above_end = np.array([*end[:2], height])
    samples = []
    for p, q, a, b in ((start, above_start, qa, qa),
                       (above_start, above_end, qa, qb),
                       (above_end, end, qb, qb)):
        angle = 2*math.acos(min(1., abs(float(a @ b))))
        count = max(1, math.ceil(np.linalg.norm(q-p)/step), math.ceil(angle/.025))
        if np.linalg.norm(q-p) < 1e-10 and angle < 1e-8:
            continue
        samples.extend((p+(q-p)*u, slerp(a, b, u**3*(10-15*u+6*u*u)))
                       for u in np.linspace(0, 1, count+1)[1:])
    if not samples:
        raise ValueError('pickup approach contains no motion')
    return samples, height, height


def transport_waypoints(start, start_q, end, end_q, clearance_z, scene,
                        radius=0.04, step=0.005, allow_tilt_change=False):
    """C2 rounded corners above the item-bottom clearance, vertical end legs.

    Yaw changes only on the elevated straight segment. A 2 mm extra margin
    covers numerical sampling of the carried-item orientation envelope.
    """
    start, end = np.asarray(start, float), np.asarray(end, float)
    start_q, end_q = quaternion(start_q), quaternion(end_q)
    if not np.isfinite([*start, *end, clearance_z, radius, step]).all() or radius <= 0 or step <= 0:
        raise ValueError('invalid continuous transport geometry')
    if not allow_tilt_change and float(rotate(start_q, [0,0,1]) @ rotate(end_q, [0,0,1])) < math.cos(math.radians(2)):
        raise ValueError('pickup and placement tool tilt differ by more than 2 degrees')
    delta = end[:2] - start[:2]
    distance = float(np.linalg.norm(delta))
    if distance < 0.01:
        raise ValueError('insufficient lateral distance for a rounded transfer')
    direction = np.array([* (delta / distance), 0.0])
    r = min(radius, distance / 4)
    bottom = min(item_bottom_offset(slerp(start_q, end_q, u), scene)
                 for u in np.linspace(0, 1, 101))
    safe_z = max(float(clearance_z) - bottom + 0.002, start[2], end[2])
    high_z = safe_z + r
    a = np.array([*start[:2], safe_z])
    b = a + direction*r + np.array([0,0,r])
    d = np.array([*end[:2], safe_z])
    c = d - direction*r + np.array([0,0,r])
    samples = []
    def line(p, q, qa, qb):
        angle = 2*math.acos(min(1, abs(float(quaternion(qa) @ quaternion(qb)))))
        count = max(1, math.ceil(np.linalg.norm(q-p)/step), math.ceil(angle/0.025))
        for u in np.linspace(0, 1, count+1)[1:]:
            samples.append((p+(q-p)*u, slerp(qa, qb, u**3*(10-15*u+6*u*u))))
    def bend(p, q, incoming, outgoing, orientation):
        controls = [p, p+incoming*r/4, p+incoming*r/2,
                    q-outgoing*r/2, q-outgoing*r/4, q]
        for u in np.linspace(0, 1, max(4, math.ceil(2*r/step))+1)[1:]:
            position = sum(math.comb(5,i)*(1-u)**(5-i)*u**i*v
                           for i,v in enumerate(controls))
            samples.append((position, orientation))
    line(start, a, start_q, start_q)
    bend(a, b, np.array([0,0,1]), direction, start_q)
    line(b, c, start_q, end_q)
    bend(c, d, direction, np.array([0,0,-1]), end_q)
    line(d, end, end_q, end_q)
    return samples, safe_z, high_z
