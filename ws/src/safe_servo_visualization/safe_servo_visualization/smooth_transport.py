"""Quintic JTC interpolation with continuous acceleration and optional jerk cap.

Feasible planner times are retained; only violating intervals and neighboring
transitions are lengthened. Positions are retained, shared velocities are
reduced consistently, and accelerations minimize integrated squared jerk.
Endpoints and existing full stops have zero
acceleration. Adjacent segments share acceleration (C2). Jerk is not necessarily
continuous; its software cap is opt-in. Geometry must be collision/FK checked afterwards.
"""
import math
import copy
import time

import numpy as np
from numpy.polynomial import polynomial as poly
from rclpy.duration import Duration


def coefficients(p0, p1, v0, v1, dt, a0=0., a1=0.):
    delta = p1 - p0
    return np.array([p0, dt*v0, dt*dt*a0/2,
                     10*delta-dt*(6*v0+4*v1)+dt*dt*(-1.5*a0+.5*a1),
                     -15*delta+dt*(8*v0+7*v1)+dt*dt*(1.5*a0-a1),
                     6*delta-3*dt*(v0+v1)+dt*dt*(-.5*a0+.5*a1)])


def knot_accelerations(points, times):
    """O(N) tridiagonal minimum-jerk solve, with fixed positions/velocities.

    For each segment integral jerk^2 is quadratic in its two accelerations.
    Its Hessian (up to a common factor) is [[9,-3],[-3,9]]/dt.
    Sharing each knot variable removes the artificial acceleration reset.
    """
    n, joints = len(points), len(points[0].positions)
    diagonal = np.zeros(n)
    off = np.zeros(n-1)
    rhs = np.zeros((n, joints))
    basis = (np.array([-9., 36., -30.]), np.array([3., -24., 30.]))
    for i in range(n-1):
        dt = times[i+1]-times[i]
        diagonal[i:i+2] += 9/dt
        off[i] = -3/dt
        for j in range(joints):
            c = coefficients(points[i].positions[j], points[i+1].positions[j],
                             points[i].velocities[j], points[i+1].velocities[j], dt)
            for side in (0, 1):
                product = poly.polymul(basis[side], poly.polyder(c, 3))
                rhs[i+side, j] -= sum(product / np.arange(1, len(product)+1))/dt**3
    fixed = [i for i, p in enumerate(points)
             if i in (0, n-1) or max(map(abs, p.velocities)) < 1e-7]
    for i in fixed:
        diagonal[i], rhs[i] = 1., 0.
        if i: off[i-1] = 0.
        if i < n-1: off[i] = 0.
    for i in range(1, n):
        factor = off[i-1]/diagonal[i-1]
        diagonal[i] -= factor*off[i-1]
        rhs[i] -= factor*rhs[i-1]
    result = np.zeros_like(rhs)
    result[-1] = rhs[-1]/diagonal[-1]
    for i in range(n-2, -1, -1):
        result[i] = (rhs[i]-off[i]*result[i+1])/diagonal[i]
    if not np.isfinite(result).all():
        raise ValueError('non-finite optimized knot accelerations')
    return result


def extrema(c):
    roots = poly.polyroots(poly.polyder(c))
    return [0., 1.] + [float(r.real) for r in roots
                       if abs(r.imag) < 1e-8 and 0 < r.real < 1]


def derivative_caps(vmax, speed_cap, acceleration, jerk, scale):
    caps = [(1, min(vmax, speed_cap)*scale), (2, acceleration*scale)]
    if jerk is not None:
        caps.append((3, jerk*scale))
    return caps


def segment_requirements(points, times, accelerations, names, limits,
                         speed_cap, acceleration, jerk, scale):
    """Analytic derivative extrema; the suggested dilation is rechecked, not trusted."""
    required = np.ones(len(points)-1)
    worst = {'factor': 1., 'segment': None, 'joint': None, 'derivative': None}
    for i, (old, point) in enumerate(zip(points, points[1:])):
        dt = times[i+1]-times[i]
        for name, p0, p1, v0, v1, a0, a1 in zip(names, old.positions, point.positions,
                old.velocities, point.velocities, accelerations[i], accelerations[i+1]):
            c = coefficients(p0, p1, v0, v1, dt, a0, a1)
            if not np.isfinite(c).all():
                raise ValueError('non-finite quintic interpolation')
            lo, hi, vmax = limits[name]
            if any(not lo-1e-7 <= poly.polyval(u, c) <= hi+1e-7 for u in extrema(c)):
                raise ValueError(f'controller interpolation exceeds {name} position limits')
            for order, cap in derivative_caps(vmax, speed_cap, acceleration, jerk, scale):
                derivative = poly.polyder(c, order)
                peak = max(abs(poly.polyval(u, derivative)) for u in extrema(derivative))/dt**order
                if not math.isfinite(peak):
                    raise ValueError('non-finite transport derivative')
                factor = (peak/cap)**(1/order)
                required[i] = max(required[i], factor)
                if factor > worst['factor']:
                    worst = dict(factor=float(factor), segment=i, joint=name,
                                 derivative=('velocity', 'acceleration', 'jerk')[order-1])
    return required, worst


def smooth_and_sample(trajectory, limits, speed_cap, acceleration, jerk, scale, diagnostics=None):
    """Preserve feasible segment times; repair violations locally, then validate JTC spline."""
    if (not all(math.isfinite(x) and x > 0 for x in (speed_cap, acceleration, scale)) or
            (jerk is not None and (not math.isfinite(jerk) or jerk <= 0))):
        raise ValueError('invalid transport derivative limits')
    # Do not expose a partially repaired trajectory on failure.
    points = copy.deepcopy(trajectory.points)
    times = [p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in points]
    if (len(points) < 2 or times[0] != 0 or
            not all(math.isfinite(t) for t in times) or
            any(b <= a for a, b in zip(times, times[1:]))):
        raise ValueError('transport timing must start at zero and increase strictly')
    original_dt = np.diff(times)
    original_ns = np.array([p.time_from_start.sec*1000000000 +
                            p.time_from_start.nanosec for p in points], dtype=np.int64)
    original_velocity = np.array([p.velocities for p in points], dtype=float)
    positions = np.array([p.positions for p in points], dtype=float)
    expected = (len(points), len(trajectory.joint_names))
    if (original_velocity.shape != expected or positions.shape != expected or
            not np.isfinite(original_velocity).all() or not np.isfinite(positions).all()):
        raise ValueError('invalid transport joint data')
    segment_ns = np.diff(original_ns).copy()
    deadline = time.monotonic() + 10.
    initial_worst = None
    uniform_ns = None
    strategy = 'unchanged'
    for iteration in range(20):
        if time.monotonic() > deadline:
            raise ValueError('local transport retiming timed out')
        stamp_ns = np.concatenate(([0], np.cumsum(segment_ns)))
        sent_times = [float(t)*1e-9 for t in stamp_ns]
        factors = (segment_ns*1e-9)/original_dt
        # Interpolate the local time scale to each shared knot. Choosing the
        # slower neighbor here biases every dense interval's endpoint velocity
        # and creates artificial jerk even with a smooth time-scale envelope.
        interior = (factors[:-1]*original_dt[1:] + factors[1:]*original_dt[:-1]) / (
            original_dt[:-1]+original_dt[1:])
        knot_factors = np.maximum(1., np.concatenate(([factors[0]], interior, [factors[-1]])))
        for i, point in enumerate(points):
            point.velocities = list(original_velocity[i]/knot_factors[i])
        accelerations = knot_accelerations(points, sent_times)
        required, worst = segment_requirements(points, sent_times, accelerations,
                                               trajectory.joint_names, limits,
                                               speed_cap, acceleration, jerk, scale)
        if initial_worst is None:
            initial_worst = worst
            # A comparison candidate, not the default: local transitions must
            # never make the result slower than a verified uniform dilation.
            uniform_ns = np.ceil(np.diff(original_ns)*max(1., worst['factor'])*1.00001)
        if np.max(required) <= 1.+1e-8:
            break
        # Repair only violating intervals. Neighbors are rechecked next pass,
        # since shared derivatives can change when one interval is lengthened.
        violating = required > 1.+1e-8
        proposed = np.array(factors)
        proposed[violating] *= required[violating]*1.02
        # Smoothly bring neighboring intervals into the slower region instead
        # of introducing an abrupt time-scale edge at one dense waypoint.
        centers = (np.asarray(times[:-1])+np.asarray(times[1:]))/2
        width = .5
        envelope = np.ones_like(proposed)
        residual = proposed-envelope
        while np.max(residual) > 1e-8:
            index = int(np.argmax(residual))
            r = np.minimum(1., np.abs(centers-centers[index])/width)
            bump = (1-r)**3*(1+3*r+6*r*r)
            envelope += residual[index]*bump
            residual = proposed-envelope
        new_ns = np.ceil(original_dt*envelope*1e9)
        strategy = 'local'
        if np.sum(new_ns) >= np.sum(uniform_ns):
            new_ns = uniform_ns
            strategy = 'uniform-shorter'
        if not np.isfinite(new_ns).all() or np.sum(new_ns)*1e-9 > 400.:
            raise ValueError('transport validation exceeds the bounded sample budget')
        segment_ns = new_ns.astype(np.int64)
    else:
        raise ValueError('local transport retiming did not converge; no trajectory sent')
    for i, point in enumerate(points):
        point.time_from_start = Duration(nanoseconds=int(stamp_ns[i])).to_msg()
        point.accelerations = list(accelerations[i])
    if diagnostics is not None:
        diagnostics.update(original_duration=times[-1], duration=sent_times[-1],
                           repaired_segments=int(np.count_nonzero(segment_ns != np.diff(original_ns))),
                           total_segments=len(segment_ns), iterations=iteration+1,
                           max_segment_factor=float(np.max(factors)),
                           initial_worst=initial_worst, strategy=strategy)
    # Reconstruct with the quantized times, exactly as the controller does.
    sent_times = [p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in points]
    checks = [(tuple(points[0].positions), 0.)]
    peak_jerk = 0.
    for i, (old, point) in enumerate(zip(points, points[1:])):
        dt = sent_times[i+1]-sent_times[i]
        row = [coefficients(p0, p1, v0, v1, dt, a0, a1) for p0, p1, v0, v1, a0, a1 in
               zip(old.positions, point.positions, old.velocities, point.velocities,
                   old.accelerations, point.accelerations)]
        for name, c in zip(trajectory.joint_names, row):
            lo, hi, vmax = limits[name]
            if any(not lo-1e-7 <= poly.polyval(u, c) <= hi+1e-7 for u in extrema(c)):
                raise ValueError(f'controller interpolation exceeds {name} position limits')
            for order, cap in derivative_caps(vmax, speed_cap, acceleration, jerk, scale):
                derivative = poly.polyder(c, order)
                peak = max(abs(poly.polyval(u, derivative)) for u in extrema(derivative))/dt**order
                if not math.isfinite(peak) or peak > cap*(1+1e-7):
                    raise ValueError('quantized transport exceeds derivative limits')
            jerk_poly = poly.polyder(c, 3)
            segment_jerk = max(abs(poly.polyval(u, jerk_poly)) for u in extrema(jerk_poly))/dt**3
            if not math.isfinite(segment_jerk):
                raise ValueError('non-finite transport jerk')
            peak_jerk = max(peak_jerk, segment_jerk)
        # Bound displacement per checked interval, even between extrema.
        bound = max(sum(abs(poly.polyder(c))) for c in row)
        steps = max(1, math.ceil(dt/.02), math.ceil(bound/.02))
        if len(checks)+steps > 20000:
            raise ValueError('transport validation exceeds the bounded sample budget')
        for u in np.linspace(0, 1, steps+1)[1:]:
            checks.append((tuple(poly.polyval(u, c) for c in row), sent_times[i]+dt*u))
    trajectory.points = points
    if diagnostics is not None:
        diagnostics.update(jerk_enforced=jerk is not None, peak_jerk=float(peak_jerk))
    return checks, sent_times[-1], sent_times[-1]/times[-1]
