"""
Flight scenario definitions.

A "flight" is described in ordinary terms (airspeed, altitude, attitude,
duration) rather than the raw state vector the integrator needs.
build_initial_state() does that conversion.

Add a new flight by copying an existing dict and changing the numbers.
All angles are in degrees here for readability; they get converted to
radians inside build_initial_state().
"""

import math
import numpy as np


TRIMMED_STRAIGHT_LEVEL_50MPS = {
    "airspeed_mps": 50.0,     # true airspeed at t=0, assumed aligned with body x-axis
    "altitude_m": 1000.0,     # positive = above ground (converted to NED sign internally)
    "bank_angle_deg": 0.0,    # phi
    "pitch_angle_deg": 0.0,   # theta (0 deg is a reasonable approx for trimmed level flight)
    "heading_deg": 0.0,       # psi
    "north_m": 0.0,           # starting NED position
    "east_m": 0.0,
    "t0_s": 0.0,
    "tf_s": 10.0,
    "h_s": 0.01,              # 100 Hz
}

CLIMBING_TURN_60MPS = {
    "airspeed_mps": 60.0,
    "altitude_m": 500.0,
    "bank_angle_deg": 15.0,
    "pitch_angle_deg": 5.0,
    "heading_deg": 0.0,
    "north_m": 0.0,
    "east_m": 0.0,
    "t0_s": 0.0,
    "tf_s": 20.0,
    "h_s": 0.01,
}


def build_initial_state(flight):
    """
    Convert a flight dict (human terms) into the 12-element state vector x0
    expected by flat_earth_eom.py:

        [u, v, w, p, q, r, phi, theta, psi, p1_N, p2_E, p3_D]

    Assumes the given airspeed is aligned with the body x-axis at t=0
    (i.e. zero initial alpha/beta) and zero initial angular rates -
    a reasonable starting point for a trim condition. If you want a
    non-zero initial alpha, add it to the flight dict and split
    airspeed into u/w components here.
    """
    u0_bf_mps = flight["airspeed_mps"]
    v0_bf_mps = 0.0
    w0_bf_mps = 0.0
    p0_bf_mps = 0.0
    q0_bf_mps = 0.0
    r0_bf_mps = 0.0

    phi0_rad = math.radians(flight["bank_angle_deg"])
    theta0_rad = math.radians(flight["pitch_angle_deg"])
    psi0_rad = math.radians(flight["heading_deg"])

    p1_0_n_m = flight["north_m"]
    p2_0_n_m = flight["east_m"]
    p3_0_n_m = -flight["altitude_m"]  # NED: negative-down = above ground

    return np.array([
        u0_bf_mps, v0_bf_mps, w0_bf_mps,
        p0_bf_mps, q0_bf_mps, r0_bf_mps,
        phi0_rad, theta0_rad, psi0_rad,
        p1_0_n_m , p2_0_n_m , p3_0_n_m])
