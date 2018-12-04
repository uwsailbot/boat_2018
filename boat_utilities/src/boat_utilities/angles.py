#!/usr/bin/env python

from enum import Enum

# NOTE: All operations in this module use degrees CCW from East, as in a unit circle
# TODO: Switch to rads
__UNITS = 360.0


class ComparisonResult(Enum):
    """The result of comparing two angles using `get_side()`."""
    EQUAL=0
    LEFT=1
    RIGHT=2
    OPPOSITE=3


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


def opposite(angle):
    """Calculate the opposite angle, normalized to be 0 to 360."""
    return normalize(angle + __UNITS/2)


def opposite_signed(angle):
    """Calculate the opposite angle, normalized to be -180 to 180."""
    return normalize_signed(angle + __UNITS/2)


def is_within_bounds(val, lower, upper, inclusive=False):
    """Determine whether lower < val < upper.

    Args:
        val The value to check
        lower The lower bound
        upper The upper bound
        inclusive Whether to include exact equality with the limits (Default: False)
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

    if inclusive:
        return lower <= val and val <= upper
    else:
        return lower < val < upper


def is_on_right(angle, ref, inclusive=False):
    """Determine whether the specified angle is within the 180 degrees to the right of the reference.

    Note: It is recommended to use `get_side()` in most cases instead.

    Args:
        angle The angle to check
        ref The reference angle to compare against
        inclusive Whether to include exact equality with the reference and opposite angles (Default: False)
    Returns:
        `True` if `angle` is within 180 degrees to the right of `ref`
    """
    angle = normalize(angle)
    ref = normalize(ref)
    ref_opp = normalize(ref + __UNITS/2)

    # We check if ref_opp < angle < ref
    return is_within_bounds(angle, ref_opp, ref, inclusive)


def is_on_left(angle, ref, inclusive=False):
    """Determine whether the specified angle is within the 180 degrees to the left of the reference.

    Note: It is recommended to use `get_side()` in most cases instead.

    Args:
        angle The angle to check
        ref The reference angle to compare against
        inclusive Whether to include exact equality with the reference and opposite angles (Default: False)
    Returns:
        `True` if `angle` is within 180 degrees to the left of `ref`
    """
    angle = normalize(angle)
    ref = normalize(ref)
    ref_opp = normalize(ref + __UNITS/2)

    # We check if ref < angle < ref_opp
    return is_within_bounds(angle, ref, ref_opp, inclusive)


def get_side(angle, ref):
    """Determine on which side of a reference the specified angle lies.

    Args:
        angle The angle to check
        ref The reference angle to compare against
    Returns:
        The corresponding `ComparisonResult` enum value
    """
    if angle == ref:
        return ComparisonResult.EQUAL

    if is_on_left(angle, ref, False):
        return ComparisonResult.LEFT

    if is_on_right(angle, ref, False):
        return ComparisonResult.RIGHT

    return ComparisonResult.OPPOSITE
