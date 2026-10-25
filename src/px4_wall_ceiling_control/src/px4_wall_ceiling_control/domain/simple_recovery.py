"""Shared demo recovery contract, used by decision, mux and mode manager."""
import math

ENTRY_TILT_DEG = 10.
MAX_TILT_DEG = 10.
MAX_RATE_RAD_S = math.radians(20.)
HEIGHT_MARGIN_M = .20
SPEED_MARGIN_MPS = .25
STABLE_TIME_S = .50


def attitude_within_margins(tilt, rate):
    """Captured RECOVER handover depends on attitude, not range convergence."""
    return (all(math.isfinite(v) for v in (tilt, rate))
            and abs(tilt) <= math.radians(MAX_TILT_DEG)+1.e-9
            and abs(rate) <= MAX_RATE_RAD_S)


def within_margins(tilt, rate, height_error, vertical_speed, horizontal_speed):
    values = (tilt, rate, height_error, vertical_speed, horizontal_speed)
    return (all(math.isfinite(v) for v in values)
            and abs(tilt) <= math.radians(MAX_TILT_DEG)+1.e-9
            and abs(rate) <= MAX_RATE_RAD_S
            and abs(height_error) <= HEIGHT_MARGIN_M+1.e-9
            and abs(vertical_speed) <= SPEED_MARGIN_MPS
            and abs(horizontal_speed) <= SPEED_MARGIN_MPS)
