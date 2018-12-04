#!/usr/bin/env python


def dist(a, b):
    """Determine the distance between between two points."""
    from math import sqrt
    return sqrt((a.y - b.y)**2 + (a.x - b.x)**2)


def is_within_dist(a, b, tol):
    """Determine if the dist between two points is within the specified tolerance.

    Args:
        p1: The first point to compare
        p2: The second point to compare
        dist: The tolerance
    Returns:
        `True` if the points are within the tolerance
    """
    return dist(a, b) < tol
