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

    def begin(self, height, base, actual_thrust, now):
        if not all(math.isfinite(v) for v in (height, base, actual_thrust, now)):
            raise ValueError('nonfinite departure seed')
        if not .1 <= base < self.max_thrust or not .1 <= actual_thrust <= self.max_thrust:
            raise ValueError('departure seed outside collective bounds')
        self.reference, self.base = height, base
        self.wall_pressure = self.value = actual_thrust
        self.at = now
        self.active = True
        self.clear_integral()
        self.target_speed = self.correction = 0.
        self.support_weight = 0.
        self.saturated = False
        self.requested = actual_thrust
        self.reason = 'departure support initialized from final output'

    def _requested(self, vertical_fraction, velocity_error, integral):
        correction = bounded(self.velocity_kp*velocity_error+integral,
                             -self.correction_limit, self.correction_limit)
        demand = max(0., self.base+correction)
        support = bounded(demand/max(vertical_fraction, .001), .1, self.max_thrust)
        full = min(1., max(self.authority_start+.01, demand/self.max_thrust))
        weight = smooth((vertical_fraction-self.authority_start)/(full-self.authority_start))
        return (self.wall_pressure+weight*(support-self.wall_pressure), correction, weight,
                demand/max(vertical_fraction, .001))

    def update(self, height, velocity, vertical_fraction, now, source_stamp,
               actual_thrust, integrate=True):
        if not self.active:
            raise ValueError('departure controller has no seed')
        values = (height, velocity, vertical_fraction, now, source_stamp, actual_thrust)
        if not all(math.isfinite(v) for v in values) or source_stamp <= 0:
            raise ValueError('invalid departure feedback')
        if not .1 <= actual_thrust <= self.max_thrust:
            raise ValueError('actual collective outside departure bounds')
        if now < self.at:
            self.clear_integral()
            raise ValueError('departure clock moved backwards')
        if now == self.at:
            return self.value
        dt = min(.05, now-self.at)
        self.at = now
        vertical_fraction = bounded(vertical_fraction, 0., 1.)
        self.target_speed = bounded(self.height_kp*(self.reference-height),
                                    -self.down_speed, self.up_speed)
        error = self.target_speed-velocity
        previous_source = self.last_source_stamp
        fresh_sample = previous_source is None or source_stamp > previous_source
        source_dt = 0. if previous_source is None else bounded(source_stamp-previous_source, 0., dt)
        if fresh_sample:
            self.last_source_stamp = source_stamp
        requested, correction, weight, unlimited = self._requested(vertical_fraction, error, self.integral)
        self.integral_active = False
        self.saturated = unlimited > self.max_thrust or abs(correction) >= self.correction_limit-1e-9
        if integrate and fresh_sample and source_dt > 0 and vertical_fraction >= math.cos(math.radians(30.)):
            trial = bounded(self.integral+self.velocity_ki*error*source_dt,
                            -self.integral_limit, self.integral_limit)
            proposed, trial_correction, _, trial_unlimited = self._requested(vertical_fraction, error, trial)
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
                requested, correction, weight, unlimited = self._requested(vertical_fraction, error, trial)
        step = self.thrust_slew*dt
        self.value = bounded(requested, max(.1, actual_thrust-step),
                             min(self.max_thrust, actual_thrust+step))
        self.requested, self.correction, self.support_weight = requested, correction, weight
        self.saturated |= abs(requested-self.value) > 1e-9
        self.reason = ('wall support required at current tilt' if weight < 1.-1e-9 else
                       'departure height support limited' if self.saturated else 'departure height support active')
        return self.value
