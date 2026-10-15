"""Bounded horizontal velocity feedback in ENU; no direct motor control."""
import math

from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw


class HorizontalVelocityController:
    def __init__(self, kp=1.5, max_acceleration=.30, target_acceleration=.10,
                 ki=.30, integral_limit=.20):
        for value in (kp, max_acceleration, target_acceleration, ki, integral_limit):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('horizontal gains and acceleration limits must be finite and positive')
        self.kp = kp
        self.max_acceleration = max_acceleration
        self.target_acceleration = target_acceleration
        self.ki, self.integral_limit = ki, integral_limit
        self.reset()

    def reset(self):
        self.target = (0., 0.)
        self.acceleration = (0., 0.)
        self.clear_integral()

    def clear_integral(self):
        self.integral = (0., 0.)
        self.integral_active = False
        self.saturated = False

    @staticmethod
    def _bounded(vector, limit):
        norm = math.hypot(*vector)
        scale = min(1., limit/norm) if norm else 1.
        return tuple(value*scale for value in vector)

    def attitude(self, desired_velocity, measured_velocity, yaw, dt, equilibrium_axis=(0.,0.,1.),
                 integrate=False, integral_dt=0., max_tilt=None, applied_axis=None):
        values = (*desired_velocity, *measured_velocity, *equilibrium_axis, yaw, dt)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('nonfinite horizontal control input')
        dt = min(.05, max(0., dt))
        delta = tuple(a-b for a,b in zip(desired_velocity, self.target))
        length = math.hypot(*delta)
        fraction = min(1., self.target_acceleration*dt/length) if length else 1.
        self.target = tuple(old+fraction*change for old,change in zip(self.target,delta))
        if equilibrium_axis[2] <= 0:
            raise ValueError('equilibrium must have positive vertical thrust authority')
        if not math.isfinite(integral_dt) or (max_tilt is not None and not 0 < max_tilt < math.pi/2):
            raise ValueError('invalid integral interval or tilt limit')
        if applied_axis is not None and not all(math.isfinite(v) for v in applied_axis):
            raise ValueError('nonfinite applied thrust axis')
        error = tuple(target-measured for target,measured in zip(self.target,measured_velocity))
        proportional = tuple(self.kp*v for v in error)
        baseline = tuple(9.80665*v/equilibrium_axis[2] for v in equilibrium_axis[:2])
        self.integral_active = False
        applied_saturated = bool(applied_axis is not None and max_tilt is not None
                                 and applied_axis[2] <= math.cos(max_tilt)+1e-9)
        self.saturated = applied_saturated
        if integrate and integral_dt > 0:
            trial = tuple(i+self.ki*e*min(.10,integral_dt) for i,e in zip(self.integral,error))
            bounded = self._bounded(trial,self.integral_limit)
            self.saturated |= math.hypot(*trial) > self.integral_limit
            delta = tuple(a-b for a,b in zip(bounded,self.integral))
            old = tuple(p+i for p,i in zip(proportional,self.integral))
            new = tuple(p+i for p,i in zip(proportional,bounded))
            # Reject outward motion at either disk boundary; inward release is
            # allowed even while the proportional term keeps output saturated.
            constraints = [(old,new,self.max_acceleration)]
            if max_tilt is not None:
                constraints.append((tuple(b+a for b,a in zip(baseline,old)),
                                    tuple(b+a for b,a in zip(baseline,new)),
                                    9.80665*math.tan(max_tilt)))
            blocked = any(math.hypot(*n) > limit and math.hypot(*n) > math.hypot(*o)+1e-12
                          for o,n,limit in constraints)
            blocked |= applied_saturated and sum(a*d for a,d in zip(applied_axis[:2],delta)) > 1e-12
            # A sign reversal may saturate P in the opposite direction. Still
            # release stored compensation instead of trapping the old bias.
            if math.hypot(*bounded) < math.hypot(*self.integral)-1e-12:
                blocked = False
            self.saturated |= blocked
            if not blocked:
                self.integral = bounded
                self.integral_active = any(abs(v)>1e-12 for v in delta)
        acceleration = tuple(p+i for p,i in zip(proportional,self.integral))
        self.saturated |= math.hypot(*acceleration) >= self.max_acceleration-1e-12
        self.acceleration = self._bounded(acceleration,self.max_acceleration)
        horizontal = tuple(b+a for b,a in zip(baseline,self.acceleration))
        if max_tilt is not None:
            limit = 9.80665*math.tan(max_tilt)
            self.saturated |= math.hypot(*horizontal) >= limit-1e-12
            horizontal = self._bounded(horizontal,limit)
        # Callers use separate controller instances for approach and recovery history.
        axis = (*horizontal,9.80665)
        norm = math.sqrt(sum(value*value for value in axis))
        return attitude_from_axis_yaw(tuple(value/norm for value in axis),yaw)
