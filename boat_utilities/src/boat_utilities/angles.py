#!/usr/bin/env python

# NOTE: All operations in this module use degrees CCW from East, as in a unit circle
# TODO: Switch to rads
__UNITS = 360.0


def cosd(angle):
    """Calculate the cosine of the following angle, in degrees."""
    from math import cos, radians
    return cos(radians(angle))


def sind(angle):
    """Calculate the sine of the following angle, in degrees."""
    from math import sin, radians
    return sin(radians(angle))


def tand(angle):
    """Calculate the tangent of the following angle, in degrees."""
    from math import tan, radians
    return tan(radians(angle))


def atan2d(y, x):
    """Calculate the arctan of the following angle, in degrees."""
    from math import atan2, degrees
    return degrees(atan2(y, x))


def normalize(angle):
    """Normalize the angle to be 0 to 360."""
    from math import fmod
    return fmod(fmod(angle, __UNITS) + __UNITS, __UNITS)


def normalize_signed(angle):
    """Normalize the angle to be -180 to 180."""
    angle = normalize(angle)
    if angle > __UNITS/2:
        angle -= __UNITS
    return angle


def is_within_bounds(val, lower, upper):
    """Determine whether lower < val < upper.

    Args:
        val The value to check
        lower The lower bound
        upper The upper bound
    Returns:
        `True` if the value is between the specified bounds
    """
    val = normalize(val)
    lower = normalize(lower)
    upper = normalize(upper)

    if val > upper:
        val -= __UNITS
    if lower > upper:
        lower -= __UNITS

    return lower <= val and val <= upper


def is_on_right(angle, ref):
    """Determine whether the specified angle is within the 180 degrees to the right of the reference.
    
    Args:
        angle The angle to check
        ref The reference angle to compare against
    Returns:
        `True` if `angle` is within 180 degrees to the right of `ref`
    """
    angle = normalize(angle)
    ref = normalize(ref)
    ref_opp = normalize(ref + __UNITS/2)

    # If the range (ref, ref_opp) doesn't cross 0, we check if ref_opp < angle < ref
    if ref > __UNITS/2:
        return is_within_bounds(angle, ref, ref_opp)
    else:
        return not is_within_bounds(angle, ref, ref_opp)


def is_on_left(angle, ref):
    """Determine whether the specified angle is within the 180 degrees to the left of the reference.
    
    Args:
        angle The angle to check
        ref The reference angle to compare against
    Returns:
        `True` if `angle` is within 180 degrees to the left of `ref`
    """
    angle = normalize(angle)
    ref = normalize(ref)
    ref_opp = normalize(ref + __UNITS/2)

    # If the range (ref, ref_opp) doesn't cross 0, we check if !(ref_opp < angle < ref)
    if ref > __UNITS/2:
        return not is_within_bounds(angle, ref, ref_opp)
    else:
        return is_within_bounds(angle, ref, ref_opp)
