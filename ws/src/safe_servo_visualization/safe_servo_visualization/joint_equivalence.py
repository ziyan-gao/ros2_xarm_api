"""Bounded equivalent-angle selection shared by planning and SDK fallbacks."""
import math


def nearest_equivalent_joints(goal, seed, names, limits, *, minimize_wrist=False):
    """Choose bounded 2*pi-equivalent angles nearest the preceding arm state."""
    if len(goal) != len(names) or len(seed) != len(names):
        raise ValueError('invalid IK joint layout')
    chosen = []
    for index, (name, value, previous) in enumerate(zip(names, goal, seed)):
        lower, upper = limits[name][:2]
        if (not all(math.isfinite(v) for v in
                    (value, previous, lower, upper)) or
                not lower <= value <= upper or
                not lower <= previous <= upper):
            raise ValueError(f'IK/seed exceeds joint limits: {name}')
        first = math.ceil((lower-value)/(2*math.pi))
        last = math.floor((upper-value)/(2*math.pi))
        candidates = [value+2*math.pi*k for k in range(first, last+1)]
        chosen.append(min(
            candidates, key=lambda candidate:
            (abs(candidate) if minimize_wrist and index == len(names)-1
             else abs(candidate-previous), abs(candidate-value))))
    return tuple(chosen)
