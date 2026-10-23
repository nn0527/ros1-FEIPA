"""Bounded departure height control. Thrust is collective, not measured lift.

Near a vertical wall the vehicle still needs mechanical support. This controller
preserves wall pressure until measured attitude provides vertical authority; it
must not be interpreted as a promise of free-flight lift at 90 degrees.
"""
import math


def bounded(value, low, high):
    return max(low, min(high, value))


def smooth(value):
    value = bounded(value, 0., 1.)
    return value**3 * (10. + value * (-15. + 6. * value))


class DepartureHeightController:
    def __init__(self, height_kp=.5, velocity_kp=.3, velocity_ki=.12,
                 integral_limit=.06, correction_limit=.12, up_speed=.10,
                 down_speed=.15, max_thrust=.85, thrust_slew=.50,
                 compensation_start_deg=80.):
        values = (height_kp, velocity_kp, velocity_ki, integral_limit,
                  correction_limit, up_speed, down_speed, max_thrust, thrust_slew)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('departure control gains and limits must be finite and positive')
        if not .1 < max_thrust <= 1. or not 0 < compensation_start_deg < 90:
            raise ValueError('invalid departure thrust/authority limit')
        self.height_kp, self.velocity_kp, self.velocity_ki = height_kp, velocity_kp, velocity_ki
        self.integral_limit, self.correction_limit = integral_limit, correction_limit
        self.up_speed, self.down_speed = up_speed, down_speed
        self.max_thrust, self.thrust_slew = max_thrust, thrust_slew
        self.authority_start = math.cos(math.radians(compensation_start_deg))
        self.active = False
        self.clear_integral()

    def clear_integral(self):
        self.integral = 0.
        self.integral_active = False
        self.last_source_stamp = None
        self.integral_budget = 0.

    def begin(self, height, base, actual_thrust, now):
        if not all(math.isfinite(v) for v in (height, base, actual_thrust, now)):
            raise ValueError('nonfinite departure seed')
        if not .1 <= base < self.max_thrust or not .1-1e-6 <= actual_thrust <= self.max_thrust+1e-6:
            raise ValueError('departure seed outside collective bounds')
        self.reference, self.base = height, base
        self.wall_pressure = self.value = bounded(actual_thrust, .1, self.max_thrust)
        self.at = now
        self.active = True
        self.clear_integral()
        self.target_speed = self.correction = 0.
        self.support_weight = 0.
        self.reference_tracking = False
        self.saturated = False
        self.slew_limited = False
        self.requested = actual_thrust
        self.reason = 'departure support initialized from final output'

    def reanchor(self, height):
        """Change only the height origin, preserving captured pressure/output."""
        if not self.active or not math.isfinite(height):
            raise ValueError('invalid departure height reanchor')
        self.reference = height
        self.clear_integral()

    def begin_support(self, base, actual_thrust, now):
        """Initialize thrust support without a height measurement or target."""
        self.begin(0., base, actual_thrust, now)
        self.reference = self.target_speed = math.nan

    def update_support(self, vertical_fraction, now, actual_thrust):
        """Attitude feedforward only; no local position or linear velocity."""
        if not self.active or not all(math.isfinite(x) for x in (vertical_fraction, now, actual_thrust)):
            raise ValueError('invalid departure support feedback')
        if now < self.at or not .1-1e-6 <= actual_thrust <= self.max_thrust+1e-6:
            raise ValueError('invalid departure support time/output')
        self.clear_integral()
        self.reference = self.target_speed = math.nan
        self.reference_tracking = False
        self.correction = 0.
        if now == self.at:
            return self.value
        dt = min(.05, now-self.at)
        self.at = now
        fraction = bounded(vertical_fraction, 0., 1.)
        requested, _, weight, unlimited = self._requested(fraction, None, 0.)
        step = self.thrust_slew*dt
        actual = bounded(actual_thrust, .1, self.max_thrust)
        self.value = bounded(requested, max(.1, actual-step), min(self.max_thrust, actual+step))
        self.requested, self.support_weight = requested, weight
        self.slew_limited = abs(requested-self.value) > 1e-9
        self.saturated = unlimited > self.max_thrust or self.slew_limited
        self.reason = 'attitude-only departure support; waiting for range'
        return self.value

    def update_range(self, height, velocity, vertical_fraction, now, actual_thrust,
                     height_kp, velocity_kp):
        """Range PD: no cascade, integral, velocity admission or reanchoring."""
        values = (height, vertical_fraction, now, actual_thrust, height_kp, velocity_kp)
        if velocity is not None:
            values += (velocity,)
        if (not self.active or not all(math.isfinite(x) for x in values)
                or now < self.at or not .1-1e-6 <= actual_thrust <= self.max_thrust+1e-6):
            raise ValueError('invalid range PD feedback/time/output')
        self.clear_integral()
        self.reference_tracking = False
        self.target_speed = 0. if velocity is not None else math.nan
        if now == self.at:
            return self.value
        dt = min(.05, now-self.at)
        self.at = now
        correction = height_kp*(self.reference-height)
        if velocity is not None:
            correction -= velocity_kp*velocity
        correction = bounded(correction, -self.correction_limit, self.correction_limit)
        fraction = bounded(vertical_fraction, 0., 1.)
        requested, _, weight, unlimited = self._requested(
            fraction, None, 0., height_correction=correction, velocity_weight=0.)
        step = self.thrust_slew*dt
        actual = bounded(actual_thrust, .1, self.max_thrust)
        self.value = bounded(requested, max(.1, actual-step), min(self.max_thrust, actual+step))
        self.requested, self.correction, self.support_weight = requested, correction, weight
        self.slew_limited = abs(requested-self.value) > 1e-9
        self.saturated = unlimited > self.max_thrust or abs(correction) >= self.correction_limit or self.slew_limited
        self.reason = 'range PD' if velocity is not None else 'range P; waiting for measured range velocity'
        return self.value

    def _requested(self, vertical_fraction, velocity_error, integral, correction_weight=1.,
                   height_correction=None, velocity_weight=1., velocity_kp=None):
        gain = self.velocity_kp if velocity_kp is None else velocity_kp
        correction = (gain*velocity_error+integral if velocity_error is not None else 0.)
        if height_correction is not None:
            correction = height_correction+velocity_weight*(correction-height_correction)
        correction = bounded(correction,
                             -self.correction_limit, self.correction_limit)*correction_weight
        demand = max(0., self.base+correction)
        support = bounded(demand/max(vertical_fraction, .001), .1, self.max_thrust)
        full = min(1., max(self.authority_start+.01, demand/self.max_thrust))
        weight = smooth((vertical_fraction-self.authority_start)/(full-self.authority_start))
        return (self.wall_pressure+weight*(support-self.wall_pressure), correction, weight,
                demand/max(vertical_fraction, .001))

    def update(self, height, velocity, vertical_fraction, now, source_stamp,
               actual_thrust, integrate=True, correction_weight=1.,
               height_only_kp=None, velocity_weight=1., velocity_kp=None,
               allow_reference_tracking=True):
        if not self.active:
            raise ValueError('departure controller has no seed')
        values = (height, vertical_fraction, now, source_stamp, actual_thrust, correction_weight, velocity_weight)
        if velocity is not None:
            values += (velocity,)
        elif height_only_kp is None:
            raise ValueError('missing velocity requires explicit height-only control')
        if height_only_kp is not None and (not math.isfinite(height_only_kp) or height_only_kp <= 0):
            raise ValueError('invalid height-to-thrust gain')
        if velocity_kp is not None and (not math.isfinite(velocity_kp) or velocity_kp <= 0):
            raise ValueError('invalid velocity-to-thrust gain')
        if not all(math.isfinite(v) for v in values) or source_stamp <= 0:
            raise ValueError('invalid departure feedback')
        if not .1-1e-6 <= actual_thrust <= self.max_thrust+1e-6:
            raise ValueError('actual collective outside departure bounds')
        actual_thrust = bounded(actual_thrust, .1, self.max_thrust)
        if not 0. <= correction_weight <= 1. or not 0. <= velocity_weight <= 1.:
            raise ValueError('invalid departure correction weight')
        if correction_weight < 1.:
            self.clear_integral()
            integrate = False
        if now < self.at:
            self.clear_integral()
            raise ValueError('departure clock moved backwards')
        if now == self.at:
            return self.value
        dt = min(.05, now-self.at)
        self.at = now
        # Low-rate range control needs its full elapsed interval. Keep the
        # original budget for legacy/other entry points.
        budget_limit = .5 if height_only_kp is not None else .05
        self.integral_budget = min(budget_limit, self.integral_budget+dt)
        vertical_fraction = bounded(vertical_fraction, 0., 1.)
        self.target_speed = (bounded(self.height_kp*(self.reference-height),
                                     -self.down_speed, self.up_speed) if velocity is not None else math.nan)
        if correction_weight == 0.:
            self.target_speed = 0.
        error = self.target_speed-velocity if velocity is not None else None
        height_correction = (height_only_kp*(self.reference-height) if height_only_kp is not None else None)
        # Decide authority before a negative correction can lower the full-lift
        # threshold. While the wall carries weight, neither accumulated height
        # error nor the velocity P term may demand a later thrust reduction.
        neutral_weight = self._requested(vertical_fraction, None, 0.)[2]
        feedback_weight = self._requested(
            vertical_fraction, error, self.integral, correction_weight,
            height_correction, velocity_weight, velocity_kp)[2]
        was_supported = self.reference_tracking and self.support_weight < 1.-1e-9
        self.reference_tracking = allow_reference_tracking and (
            neutral_weight < 1.-1e-9 or feedback_weight < 1.-1e-9 or was_supported)
        if self.reference_tracking:
            # Include the first full-authority sample in the reanchor so the
            # handoff starts at the hover feedforward without stored error.
            self.reanchor(height)
            self.target_speed = 0.
            error = -velocity if velocity is not None else None
            height_correction = 0. if height_only_kp is not None else None
            correction_weight = 0.
            integrate = False
        if velocity is None or velocity_weight < 1.:
            self.clear_integral()
            integrate = False
        previous_source = self.last_source_stamp
        fresh_sample = previous_source is None or source_stamp > previous_source
        source_gap = 0. if previous_source is None else source_stamp-previous_source
        # Budget wall time separately from sample time: a 20 Hz sensor may
        # integrate its full interval, while a burst of queued frames cannot.
        source_dt = min(max(0., source_gap), self.integral_budget) if source_gap <= .5 else 0.
        if source_gap > .5:
            # A fresh frame after a gap starts a new integration interval; never
            # integrate the missing time or preserve unverified compensation.
            self.integral = 0.
        if fresh_sample:
            self.last_source_stamp = source_stamp
            self.integral_budget = 0. if previous_source is None or source_gap > .5 else max(0., self.integral_budget-source_dt)
        requested, correction, weight, unlimited = self._requested(
            vertical_fraction, error, self.integral, correction_weight, height_correction, velocity_weight, velocity_kp)
        self.integral_active = False
        self.saturated = unlimited > self.max_thrust or abs(correction) >= self.correction_limit-1e-9
        if integrate and fresh_sample and source_dt > 0 and vertical_fraction >= math.cos(math.radians(30.)):
            trial = bounded(self.integral+self.velocity_ki*error*source_dt,
                            -self.integral_limit, self.integral_limit)
            proposed, trial_correction, _, trial_unlimited = self._requested(
                vertical_fraction, error, trial, correction_weight, velocity_kp=velocity_kp)
            direction = trial-self.integral
            outward = ((trial_unlimited > self.max_thrust or trial_correction >= self.correction_limit
                        or requested > actual_thrust+self.thrust_slew*dt+.005) and direction > 0)
            outward |= ((trial_unlimited < .1 or trial_correction <= -self.correction_limit
                         or requested < actual_thrust-self.thrust_slew*dt-.005) and direction < 0)
            if abs(trial) < abs(self.integral)-1e-12:
                outward = False
            self.saturated |= outward
            if not outward:
                self.integral_active = abs(direction) > 1e-12
                self.integral = trial
                requested, correction, weight, unlimited = self._requested(
                    vertical_fraction, error, trial, correction_weight, velocity_kp=velocity_kp)
        step = self.thrust_slew*dt
        self.value = bounded(requested, max(.1, actual_thrust-step),
                             min(self.max_thrust, actual_thrust+step))
        self.requested, self.correction, self.support_weight = requested, correction, weight
        self.slew_limited = abs(requested-self.value) > 1e-9
        self.saturated |= self.slew_limited
        self.reason = ('wall support required at current tilt' if weight < 1.-1e-9 else
                       'departure height support limited' if self.saturated else 'departure height support active')
        return self.value
