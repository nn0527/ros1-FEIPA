"""Bounded C2 reference replanning; this does not control/reset PX4 rate I.

Angles/rates use radians. The reference may retract towards the measured
attitude. A quintic is checked over its whole interval using Bernstein hulls,
not just at controller samples. Replanning preserves position/velocity/
acceleration, and the final unchanged goal is reached in finite time.
"""
import math

from .geometry import (clamp, quaternion_conjugate, quaternion_multiply,
                       quaternion_normalize, rotate_vector)


def _bezier(points, u):
    values = list(points)
    while len(values) > 1:
        values = [(1. - u) * a + u * b for a, b in zip(values, values[1:])]
    return values[0]


def _derivative(points, duration):
    return [(len(points)-1) * (b-a) / duration for a, b in zip(points, points[1:])]


def _bounded(points, low, high, depth=12):
    # Convex hull containment proves the whole Bezier curve is bounded.
    eps = 1.e-9
    if min(points) >= low-eps and max(points) <= high+eps:
        return True
    if points[0] < low-eps or points[0] > high+eps or points[-1] < low-eps or points[-1] > high+eps:
        return False
    if depth == 0:
        return False  # Conservative rejection, never accept an unproven curve.
    left, right, row = [points[0]], [points[-1]], list(points)
    while len(row) > 1:
        row = [(a+b)*.5 for a, b in zip(row, row[1:])]
        left.append(row[0]); right.append(row[-1])
    return (_bounded(left, low, high, depth-1)
            and _bounded(list(reversed(right)), low, high, depth-1))


class QuinticSegment:
    def __init__(self, p, v, a, goal, duration, end_rate=0., end_accel=0.):
        if not all(math.isfinite(x) for x in (p, v, a, goal, duration, end_rate, end_accel)) or duration <= 0.:
            raise ValueError('invalid quintic boundary')
        self.duration = duration
        self.points = (p, p+v*duration/5., p+2.*v*duration/5.+a*duration**2/20.,
                       goal-2.*end_rate*duration/5.+end_accel*duration**2/20.,
                       goal-end_rate*duration/5., goal)
        self.velocities = _derivative(self.points, duration)
        self.accelerations = _derivative(self.velocities, duration)

    def evaluate(self, t):
        u = clamp(t/self.duration, 0., 1.)
        return tuple(_bezier(c, u) for c in (self.points, self.velocities, self.accelerations))

    def within_bounds(self, position_min, position_max, max_rate, max_accel):
        return (_bounded(self.points, position_min, position_max)
                and _bounded(self.velocities, -max_rate, max_rate)
                and _bounded(self.accelerations, -max_accel, max_accel))


class BrakingTrajectory:
    def __init__(self, angle, rate=0., accel=0., max_angle=math.pi/2.,
                 max_rate=math.radians(56.25), max_accel=math.radians(180.),
                 min_duration=.4, max_duration=3., replan_interval=.10):
        values = (angle, rate, accel, max_angle, max_rate, max_accel,
                  min_duration, max_duration, replan_interval)
        if (not all(math.isfinite(x) for x in values) or not 0. <= angle <= max_angle
                or min(max_angle, max_rate, max_accel, min_duration, replan_interval) <= 0.
                or max_duration < min_duration or abs(rate) > max_rate+1.e-9
                or abs(accel) > max_accel+1.e-9):
            raise ValueError('invalid braking trajectory state or limits')
        self.angle, self.rate, self.accel = angle, rate, accel
        self.max_angle, self.max_rate, self.max_accel = max_angle, max_rate, max_accel
        self.min_duration, self.max_duration = min_duration, max_duration
        self.replan_interval = replan_interval
        self.segment = None
        self.elapsed = self.since_replan = 0.
        self.duration = 0.
        self.goal = angle
        self.done = False
        self.replan_count = 0
        self.replan_rejected = False
        self.capture = False

    def _plan(self, goal):
        duration = self.min_duration
        while True:
            segment = QuinticSegment(self.angle, self.rate, self.accel, goal, duration)
            if segment.within_bounds(0., self.max_angle, self.max_rate, self.max_accel):
                self.segment, self.duration, self.goal = segment, duration, goal
                self.elapsed = self.since_replan = 0.
                self.replan_count += 1
                self.replan_rejected = False
                self.capture = False
                return True
            if duration >= self.max_duration:
                self.replan_rejected = True
                return False
            duration = min(self.max_duration, duration*1.12)

    def _capture_acceleration(self, goal):
        # A reference approaching the speed cap with outward acceleration may
        # have no single rest-end quintic to a distant retract goal. First
        # unwind acceleration with constant jerk, preserving all three initial
        # derivatives and checking the same full-interval bounds.
        if abs(self.accel) < 1.e-8:
            return False
        duration = .05
        while duration >= .001:
            endpoint = self.angle+self.rate*duration+self.accel*duration**2/3.
            velocity = self.rate+self.accel*duration*.5
            segment = QuinticSegment(self.angle, self.rate, self.accel, endpoint,
                                      duration, end_rate=velocity)
            if segment.within_bounds(0., self.max_angle, self.max_rate, self.max_accel):
                continuation = BrakingTrajectory(endpoint, velocity, 0., self.max_angle,
                    self.max_rate, self.max_accel, self.min_duration, self.max_duration, self.replan_interval)
                if continuation._plan(goal):
                    self.segment, self.duration = segment, duration
                    self.elapsed = self.since_replan = 0.
                    self.capture = True
                    self.replan_rejected = False
                    return True
            duration *= .5
        return False

    def step(self, dt, goal):
        if not all(math.isfinite(x) for x in (dt, goal)) or dt < 0.:
            raise ValueError('invalid braking step')
        if dt == 0.:
            return self.angle
        dt = min(dt, .05)
        goal = clamp(goal, 0., self.max_angle)
        tolerance = 1.e-8 if self.elapsed >= self.duration else math.radians(.25)
        changed = abs(goal-self.goal) > tolerance or (goal == 0. and self.goal != 0.)
        capture_finished = self.capture and self.elapsed >= self.duration
        if (self.segment is None or capture_finished
                or (not self.capture and changed and self.since_replan+1.e-9 >= self.replan_interval)):
            if not self._plan(goal) and (self.segment is None or capture_finished):
                if capture_finished or not self._capture_acceleration(goal):
                    raise RuntimeError('no bounded braking splice')
            # If replanning is infeasible, finish the previous bounded segment;
            # the adapter reports rejection and the absolute timeout still runs.
        consumed = min(dt, max(0., self.duration-self.elapsed))
        self.elapsed = min(self.duration, self.elapsed+consumed)
        self.since_replan += consumed
        self.angle, self.rate, self.accel = self.segment.evaluate(self.elapsed)
        self.done = bool(not self.capture and self.elapsed >= self.duration and self.goal == 0. and goal == 0.)
        if self.capture and dt-consumed > 1.e-10:
            # The capture endpoint has nonzero velocity. Consume the remainder
            # in the next segment instead of inserting a sample-time pause.
            return self.step(dt-consumed, goal)
        return self.angle


def allowed_rate(remaining, decel, delay, margin, max_rate):
    if (not all(math.isfinite(x) for x in (remaining, decel, delay, margin, max_rate))
            or decel <= 0. or min(delay, margin, max_rate) < 0.):
        raise ValueError('invalid stopping envelope')
    distance = max(0., remaining-margin)
    return min(max_rate, max(0., math.sqrt((decel*delay)**2+2.*decel*distance)-decel*delay))


def braking_goal(remaining, closing_rate, allowed, attitude_gain, rate_damping, max_lead, max_angle):
    if (not all(math.isfinite(x) for x in (remaining, closing_rate, allowed, attitude_gain,
                                         rate_damping, max_lead, max_angle))
            or attitude_gain <= 0. or min(allowed, rate_damping, max_lead, max_angle) < 0.):
        raise ValueError('invalid braking goal')
    lead = clamp((allowed-rate_damping*(closing_rate-allowed))/attitude_gain, -max_lead, max_lead)
    return clamp(remaining-lead, 0., max_angle)


class ReturnAxis:
    """Signed twist coordinate about the actual published return axis.

    q maps body FLU to world ENU. Body rates must be converted before calling
    closing_rate when an odometry provider uses world-frame twist.
    """
    def __init__(self, hover, seed):
        self.hover = quaternion_normalize(hover)
        q = self._relative(seed)
        norm = math.sqrt(sum(x*x for x in q[:3]))
        self.axis = tuple(x/norm for x in q[:3]) if norm > 1.e-8 else (0., 1., 0.)

    def _relative(self, q):
        relative = quaternion_multiply(quaternion_conjugate(self.hover), quaternion_normalize(q))
        return tuple(-x for x in relative) if relative[3] < 0. else relative

    def angle(self, q):
        relative = self._relative(q)
        return 2.*math.atan2(sum(x*y for x, y in zip(relative[:3], self.axis)), relative[3])

    def quaternion(self, angle):
        return quaternion_normalize(quaternion_multiply(self.hover,
            tuple(x*math.sin(angle*.5) for x in self.axis)+(math.cos(angle*.5),)))

    def closing_rate(self, q, body_rate):
        # Exact derivative of the signed twist coordinate, also off the plane.
        relative = self._relative(q)
        derivative = tuple(.5*x for x in quaternion_multiply(relative, tuple(body_rate)+(0.,)))
        s = sum(x*y for x, y in zip(relative[:3], self.axis))
        ds = sum(x*y for x, y in zip(derivative[:3], self.axis))
        norm = relative[3]**2+s*s
        if norm < 1.e-8:
            raise ValueError('singular return axis')
        return -2.*(relative[3]*ds-s*derivative[3])/norm
