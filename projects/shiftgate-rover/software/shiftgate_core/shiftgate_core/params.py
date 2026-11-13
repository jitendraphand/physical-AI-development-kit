"""Physical and grid parameters shared by CAD, firmware comments, and sims.

Units are SI (metres, kilograms, seconds) unless a name ends with `_MM`.
"""

from __future__ import annotations

# --- Arena ---
GRID_N = 5
CELL_SIZE_M = 0.180
WALL_THICKNESS_M = 0.004
WALL_HEIGHT_M = 0.050
# 5 cells + 6 wall thicknesses ≈ 0.924 m square, fits on a 1 m foam board.
ARENA_OUTER_M = GRID_N * CELL_SIZE_M + (GRID_N + 1) * WALL_THICKNESS_M

# --- Robot (matches the printed chassis) ---
CHASSIS_LENGTH_M = 0.118
CHASSIS_WIDTH_M = 0.090
CHASSIS_HEIGHT_M = 0.022
WHEEL_RADIUS_M = 0.024
WHEEL_WIDTH_M = 0.008
WHEEL_SEPARATION_M = 0.102  # wheel centre to wheel centre
CASTER_OFFSET_M = 0.038  # caster ahead of axle, along +x in the body frame

# N20 6 V ~100 RPM with 3 PPR magnetic encoder (typical GA12-N20).
MOTOR_NO_LOAD_RPM = 100.0
ENCODER_PPR = 3
GEAR_RATIO = 150.0  # pulses per wheel revolution ≈ PPR * GEAR_RATIO

# --- Sensors (body frame, +x forward, +y left) ---
US_MAX_RANGE_M = 0.40
US_MIN_RANGE_M = 0.02
IR_MAX_RANGE_M = 0.18  # calibrated ~one cell; module itself is 0.02–0.30 m
IR_MIN_RANGE_M = 0.02
# A cell is "occupied ahead" if a return is closer than this (centre to wall).
CELL_BLOCKED_THRESHOLD_M = 0.12

# --- Planning ---
# Extra cost added to an edge that has not been witnessed recently:
#   cost = 1.0 + (1 - trust) * TRUST_COST_WEIGHT
TRUST_COST_WEIGHT = 0.35
TRUST_HALF_LIFE_STEPS = 40  # trust *= 0.5 every 40 sense steps without a visit
GATE_CANDIDATE_WIDTH_CELLS = 1

# --- Start / package cells of the stock curriculum maze ---
START_CELL = (0, 0)
PACKAGE_CELL = (4, 4)
