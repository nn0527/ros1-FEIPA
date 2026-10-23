"""Measured-rate governor for the final part of normal wall detachment."""
import math

from px4_wall_ceiling_control.domain.detach_braking import (
    BrakingTrajectory, ReturnAxis, allowed_rate, braking_goal)
from px4_wall_ceiling_control.domain.geometry import (
    clamp, quaternion_angle, quaternion_conjugate, rotate_vector)


class WallDetachBrakingMixin:
    def _init_detach_braking(self, get_param):
        self.detach_brake_enabled = bool(get_param('~detach_brake_enabled', False))
        self.detach_brake_trigger = math.radians(float(get_param('~detach_brake_target_deg', 30.)))
        self.detach_brake_decel = math.radians(float(get_param('~detach_brake_decel_deg_s2', 30.)))
        self.detach_brake_delay = float(get_param('~detach_brake_delay_s', .15))
        self.detach_brake_margin = math.radians(float(get_param('~detach_brake_margin_deg', 5.)))
        self.detach_brake_max_rate = math.radians(float(get_param('~detach_brake_target_rate_deg_s', 56.25)))
        # Share the existing mux's limit; a slower configured return must not
        # make the internal reference run ahead of what can be published.
        mux_rate = math.radians(1.875*abs(float(get_param('~wall_angle_deg', 90.)))/self.detach_time_s)
        self.detach_brake_max_rate = min(self.detach_brake_max_rate, mux_rate)
        self.detach_brake_max_accel = math.radians(float(get_param('~detach_brake_target_accel_deg_s2', 180.)))
        self.detach_brake_max_lead = math.radians(float(get_param('~detach_brake_max_lead_deg', 10.)))
        self.detach_brake_attitude_gain = float(get_param('~detach_brake_attitude_gain', 4.))
        self.detach_brake_rate_damping = float(get_param('~detach_brake_rate_damping', 1.))
        if (not all(math.isfinite(v) and v > 0. for v in (
                self.detach_brake_trigger, self.detach_brake_decel, self.detach_brake_delay,
                self.detach_brake_margin, self.detach_brake_max_rate, self.detach_brake_max_accel,
                self.detach_brake_max_lead, self.detach_brake_attitude_gain, self.detach_brake_rate_damping))
                or self.detach_brake_trigger > math.pi/2.
                or self.detach_brake_max_lead > self.detach_max_tracking_error_rad
                or self.detach_brake_margin >= self.detach_brake_trigger):
            raise ValueError('invalid detach braking limits')
        self._reset_detach_braking()

    def _reset_detach_braking(self):
        self.detach_brake_planner = self.detach_brake_axis = None
        self.detach_brake_target = None
        self.detach_brake_history = []
        self.detach_brake_fault = self.detach_brake_reason = ''
        self.detach_brake_paused = False
        self.detach_brake_start_progress = 0.
        self.detach_brake_triggered = False

    @staticmethod
    def _brake_q(output):
        return tuple(getattr(output.orientation, k) for k in 'xyzw')

    def _detach_target_quaternion(self):
        if getattr(self, 'detach_brake_target', None) is not None:
            return self.detach_brake_target
        return self._quat_slerp(self.q_detach_start, self.q_hover, self._smoothstep5(self._detach_progress()))

    def _brake_progress(self):
        p = self.detach_brake_planner
        if p is None:
            return None
        if p.done and not self.detach_brake_paused:
            return 1.
        fraction = clamp(1.-p.angle/max(self.detach_brake_seed_angle, 1.e-6), 0., .999)
        return min(.999, self.detach_brake_start_progress+(1.-self.detach_brake_start_progress)*fraction)

    def _advance_detach_braking(self, now, dt):
        if not getattr(self, 'detach_brake_enabled', False):
            return False
        planner = self.detach_brake_planner
        output = self._valid_wall_output(now)
        measured = self._current_quaternion() if self._odom_fresh(now) else None
        if planner is None and measured is not None:
            measured_tilt = math.acos(clamp(rotate_vector(measured, (0., 0., 1.))[2], -1., 1.))
            # Crossing is measured, inclusive, and latched for this departure.
            # Publication history seeds continuity; it never triggers braking.
            if measured_tilt <= self.detach_brake_trigger + 1.e-12:
                self.detach_brake_triggered = True
        if output is None or measured is None:
            self.detach_brake_paused = True
            self.detach_brake_reason = '回平减速等待新鲜实测与最终发布反馈'
            return True
        sent = self._brake_q(output)
        stamp = output.header.stamp.to_sec()
        history = self.detach_brake_history
        if not history or stamp > history[-1][0]:
            history.append((stamp, sent))
            del history[:-3]
        elif stamp < history[-1][0]:
            self.detach_brake_paused = True
            self.detach_brake_reason = '回平减速拒绝倒序发布反馈'
            return True
        if planner is None:
            if not self.detach_brake_triggered:
                self.detach_brake_paused = False
                self.detach_brake_reason = ''
                self.detach_brake_target = None
                return False
            # Do not invent a zero starting velocity when publication history
            # is missing. Keep the actual target until a derivative is observed.
            if len(history) < 2 or not 1.e-4 <= history[-1][0]-history[-2][0] <= .20:
                self.detach_brake_target = sent
                self.detach_brake_reason = '回平减速等待目标速度历史'
                return True
            axis = ReturnAxis(self.q_hover, sent)
            angle = axis.angle(sent)
            rates = []
            for (t0, q0), (t1, q1) in zip(history, history[1:]):
                if 1.e-4 <= t1-t0 <= .20:
                    rates.append((axis.angle(q1)-axis.angle(q0))/(t1-t0))
            velocity = clamp(rates[-1], -self.detach_brake_max_rate, self.detach_brake_max_rate)
            acceleration = (2.*(rates[-1]-rates[-2])/(history[-1][0]-history[-3][0])
                            if len(rates) == 2 else 0.)
            acceleration = clamp(acceleration, -self.detach_brake_max_accel, self.detach_brake_max_accel)
            # At the hard publication rate cap the outgoing derivative is zero,
            # even if a backward difference still sees the approach to the cap.
            if abs(velocity) >= self.detach_brake_max_rate-1.e-6 and velocity*acceleration > 0.:
                acceleration = 0.
            self.detach_brake_start_progress = self._detach_progress()
            self.detach_brake_seed_angle = angle
            self.detach_brake_axis = axis
            try:
                planner = BrakingTrajectory(angle, velocity, acceleration,
                    max_angle=max(angle, quaternion_angle(self.q_detach_start, self.q_hover)),
                    max_rate=self.detach_brake_max_rate, max_accel=self.detach_brake_max_accel)
            except ValueError as error:
                self.detach_brake_fault = 'detach braking seed invalid: '+str(error)
                return True
            self.detach_brake_planner = planner
            self.detach_brake_target = sent
            self.detach_brake_paused = False
            self.detach_brake_reason = '回平减速插值接入：继承已发布目标及导数'
            return True  # Exact published target on the engagement tick.
        self.detach_brake_paused = False
        axis = self.detach_brake_axis
        omega = self.odom.twist.twist.angular
        body_rate = (omega.x, omega.y, omega.z)
        if not getattr(self, 'odom_twist_in_child_frame', False):
            body_rate = rotate_vector(quaternion_conjugate(measured), body_rate)
        try:
            remaining = axis.angle(measured)
            closing = axis.closing_rate(measured, body_rate)
            # Account for the active interpolation horizon and time required
            # to reverse the currently moving reference, as well as base delay.
            delay = (self.detach_brake_delay + planner.replan_interval
                     + max(0., planner.duration-planner.elapsed)
                     + abs(planner.rate)/planner.max_accel)
            allowed = allowed_rate(remaining, self.detach_brake_decel, delay,
                                   self.detach_brake_margin, self.detach_brake_max_rate)
            goal = braking_goal(remaining, closing, allowed, self.detach_brake_attitude_gain,
                                self.detach_brake_rate_damping, self.detach_brake_max_lead, planner.max_angle)
            # Once measured motion has settled inside the original cone, finish
            # to exactly hover; completion does not depend on horizontal/height gates.
            if self._detach_rotation_pose_valid():
                goal = 0.
            previous_state = (planner.angle, planner.rate, planner.accel)
            angle = planner.step(dt, goal)
            target = axis.quaternion(angle)
            old_error = quaternion_angle(self.detach_brake_target, measured)
            new_error = quaternion_angle(target, measured)
            if new_error > self.detach_max_tracking_error_rad+1.e-6 and new_error > old_error+1.e-8:
                # Tracking safety takes priority over a smooth nominal splice.
                # Restore kinematics and discard the frozen segment time so a
                # later valid sample cannot cause a catch-up jump.
                planner.angle, planner.rate, planner.accel = previous_state
                planner.segment = None
                planner.done = False
                self.detach_brake_paused = self.detach_tracking_limited = True
                self.detach_brake_reason = '回平减速目标受原跟踪误差门限制'
                return True
            self.detach_brake_target = target
            self.detach_tracking_limited = False
            self.detach_brake_reason = ('回平减速：实测 %.1f°，回平转速 %.1f°/s，允许 %.1f°/s，目标 %.1f°，重规划 %d%s' %
                (math.degrees(remaining), math.degrees(closing), math.degrees(allowed), math.degrees(angle),
                 planner.replan_count, '（保留上一可行曲线）' if planner.replan_rejected else ''))
        except (ValueError, RuntimeError) as error:
            self.detach_brake_fault = 'detach braking trajectory invalid: '+str(error)
        return True
