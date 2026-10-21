"""Geometry checks independent of the UAV mission and state layout."""


def segment_intersects_box(start, end, lower, upper):
    """Does a straight segment touch a closed, axis-aligned box?

    Coordinates and bounds have the same dimension. Bounds may be infinite.
    Intersect each axis's allowed segment-parameter interval with [0, 1].
    No tolerance or safety margin is added.
    """
    enter, leave = 0.0, 1.0
    for axis in range(len(start)):
        delta = end[axis] - start[axis]
        if delta == 0:
            if start[axis] < lower[axis] or start[axis] > upper[axis]:
                return False
            continue
        first = (lower[axis] - start[axis]) / delta
        last = (upper[axis] - start[axis]) / delta
        enter = max(enter, min(first, last))
        leave = min(leave, max(first, last))
        if enter > leave:
            return False
    return True
