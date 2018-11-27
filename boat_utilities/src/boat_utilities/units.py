#!/usr/bin/env python
"""Conversions to/from SI units."""

# Mass
LB_TO_KG = 0.453592

# Length
INCHES_TO_METERS = 0.0254
FEET_TO_METERS = 0.3048
YARDS_TO_METERS = 0.9144
MILES_TO_METERS = 1609.34
NAUT_MILES_TO_METERS = 1852

# Speed
MPH_TO_MPS = 0.44704
KNOTS_TO_MPS = 0.514444

# Coordinates
# NOTE: This is always accurate for latitude. However, longitude varies with latitude,
# meaning this is only accurate for longitude at 0deg N
DEGREES_TO_METERS = 111319.492188


def from_lbs(lbs):
    """Convert pounds to kilograms."""
    return lbs * LB_TO_KG


def to_lbs(kilograms):
    """Convert kilograms to pounds."""
    return kilograms / LB_TO_KG


def from_inches(inches):
    """Convert inches to meters."""
    return inches * INCHES_TO_METERS


def to_inches(meters):
    """Convert meters to inches."""
    return meters / INCHES_TO_METERS


def from_feet(feet):
    """Convert feet to meters."""
    return feet * FEET_TO_METERS


def to_feet(meters):
    """Convert meters to feet."""
    return meters / FEET_TO_METERS


def from_yards(yards):
    """Convert yards to meters."""
    return yards * YARDS_TO_METERS


def to_yards(meters):
    """Convert meters to yards."""
    return meters / YARDS_TO_METERS


def from_miles(miles):
    """Convert miles to meters."""
    return miles * MILES_TO_METERS


def to_miles(meters):
    """Convert meters to miles."""
    return meters / MILES_TO_METERS


def from_naut_miles(naut_miles):
    """Convert nautical miles to meters."""
    return naut_miles * NAUT_MILES_TO_METERS


def to_naut_miles(meters):
    """Convert meters to nautical miles."""
    return meters / NAUT_MILES_TO_METERS


def from_mph(mph):
    """Convert miles/hour to meters/second."""
    return mph * MPH_TO_MPS


def to_mph(m_per_sec):
    """Convert meters/second to miles/hour."""
    return m_per_sec / MPH_TO_MPS


def from_knots(knots):
    """Convert knots to meters/second."""
    return knots * KNOTS_TO_MPS


def to_knots(m_per_sec):
    """Convert meters/second to knots."""
    return m_per_sec / KNOTS_TO_MPS


def from_degrees(degrees):
    """Convert degrees lat/long to meters."""
    return degrees * DEGREES_TO_METERS


def to_degrees(meters):
    """Convert meters to degrees lat/long."""
    return meters / DEGREES_TO_METERS
