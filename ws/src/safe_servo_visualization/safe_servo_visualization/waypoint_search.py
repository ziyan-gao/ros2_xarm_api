"""Shared bounded cross-area search order, independent of ROS."""
GRID_SECONDS = 5.0


def next_grid(rail, direction):
    """Observation then alternate rail, clockwise then counterclockwise."""
    if rail == 0:
        return 1, direction
    if direction == 1:
        return 0, -1
    return None
