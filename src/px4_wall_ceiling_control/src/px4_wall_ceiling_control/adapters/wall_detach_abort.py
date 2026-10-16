"""Post-departure ABORT admission and height-holding RECOVER bridge.

The entry gate is Euler pitch, not total tilt or a declaration of stable flight.
Hard feedback/output/kill interlocks are still enforced by the existing nodes.
"""
import math
import rospy

from px4_wall_ceiling_control.msg import WallPerchStatus as W
from px4_wall_ceiling_control.domain.recovery_attitude_limits import limit_terminal_attitude


class WallDetachAbortMixin:
    def _init_detach_abort(self):
        self.detach_abort_route_enabled = bool(rospy.get_param('~detach_abort_route_enabled', False))
        self.detach_abort_max_pitch_deg = float(rospy.get_param('~detach_abort_max_pitch_deg', 20.))
        if not math.isfinite(self.detach_abort_max_pitch_deg) or not 0. < self.detach_abort_max_pitch_deg < 90.:
            raise ValueError('detach_abort_max_pitch_deg must be finite and in (0, 90)')
        self._reset_detach_abort()

    def _reset_detach_abort(self):
        self.detach_abort_recovery = False
        self.detach_rotation_succeeded = False
        self.detach_abort_original_reason = ''
        self.detach_abort_recovery_q = None
        self.detach_abort_recovery_at = None
        self.detach_abort_height_pending = False

    def _detach_abort_pitch(self, now):
        if not self._odom_fresh(now):
            return None
        q = self._current_quaternion()
        if q is None or not all(math.isfinite(v) for v in q):
            return None
        pitch = math.degrees(self._euler_from_quaternion(q)[1])
        return pitch if math.isfinite(pitch) else None

    def _detach_abort_gate(self, now):
        pitch = self._detach_abort_pitch(now)
        return pitch is not None and abs(pitch) <= self.detach_abort_max_pitch_deg + 1.e-9

    def _route_detach_abort(self, new_state, now, reason):
        if not getattr(self, 'detach_abort_route_enabled', False):
            return new_state, reason
        from_detach = self.state == W.STATE_DETACH_ROTATE
        bridge = self.state == W.STATE_RECOVER and self.detach_abort_recovery
        if not ((from_detach and new_state in (W.STATE_RECOVER, W.STATE_ABORT))
                or (bridge and new_state == W.STATE_ABORT)):
            return new_state, reason
        if from_detach:
            self.detach_rotation_succeeded = new_state == W.STATE_RECOVER
            self.detach_abort_original_reason = reason
        if self._detach_abort_gate(now):
            self.detach_abort_recovery = False
            label = 'detach rotation completed; enter ABORT control' if self.detach_rotation_succeeded else 'detach failure; enter ABORT control'
            return W.STATE_ABORT, '%s; pitch=%.2f deg <= %.2f deg; %s' % (
                label, abs(self._detach_abort_pitch(now)), self.detach_abort_max_pitch_deg,
                self.detach_abort_original_reason)
        if not bridge:
            self.detach_abort_recovery = True
            self.detach_abort_recovery_q = self._current_quaternion() if self._odom_fresh(now) else None
            self.detach_abort_recovery_at = now
            self.detach_abort_height_pending = True
            self.cancel_requested = self.detach_requested = False
            if hasattr(self, '_reset_recovery_horizontal'):
                self._reset_recovery_horizontal('脱离失败：保高回平后进入ABORT', clear_departure=True)
            controller = getattr(self, 'detach_vertical_controller', None)
            if controller is not None:
                controller.clear_integral()
            self._capture_detach_abort_height(now)
        return W.STATE_RECOVER, self._detach_abort_recovery_reason(now)

    def _capture_detach_abort_height(self, now):
        # Never repeatedly move the reference with the falling/rising aircraft.
        if not self.detach_abort_height_pending:
            return
        if (getattr(self, 'detach_vertical_fault', '')
                or not getattr(self, 'detach_feedback_valid', False)
                or not self._detach_vertical_feedback_ok(now)):
            return
        sample = self.detach_feedback_snapshot
        controller = self.detach_vertical_controller
        if sample is None or not controller.active:
            return
        old = controller.reference
        controller.reanchor(sample.height)
        self.detach_altitude_m = self.recovery_altitude_m = sample.height
        self.recovery_wait_for_level = False
        self.vertical_reference_source = 'detach_abort_recover_entry'
        self.detach_vertical_last_feedback = (sample.source, sample.height, sample.velocity)
        self.detach_abort_height_pending = False
        self.detach_height_reanchor_note = '脱离异常回平高度基准 %.3f→%.3f m' % (old, sample.height)

    def _detach_abort_recovery_reason(self, now):
        pitch = self._detach_abort_pitch(now)
        angle = ('等待新鲜有效姿态' if pitch is None else
                 '实际俯仰 %.2f°，等待绝对值≤%.2f°' % (pitch, self.detach_abort_max_pitch_deg))
        age = self._state_age(now) if self.state == W.STATE_RECOVER else 0.
        overdue = '; 回平等待已超过原恢复时限，不绕过俯仰入口门槛' if age >= self.recover_timeout_s else ''
        return '脱离异常RECOVER保高回平；%s；%s%s' % (angle, self.detach_abort_original_reason, overdue)

    def _update_detach_abort_recovery(self, now):
        self._capture_detach_abort_height(now)
        if self._detach_abort_gate(now):
            self._enter_state(W.STATE_ABORT, now, self.detach_abort_original_reason)
        else:
            self.last_reason = self._detach_abort_recovery_reason(now)

    def _detach_abort_recovery_attitude(self, now):
        if not self._odom_fresh(now):
            return None
        actual = self._current_quaternion()
        if actual is None:
            return None
        if self.detach_abort_recovery_q is None:
            self.detach_abort_recovery_q = actual
            self.detach_abort_recovery_at = now
        dt = min(.05, max(0., (now-self.detach_abort_recovery_at).to_sec()))
        self.detach_abort_recovery_at = now
        self.detach_abort_recovery_q = limit_terminal_attitude(
            self.q_hover, self.detach_abort_recovery_q, dt,
            math.radians(10.), math.radians(20.), 2.)
        return self.detach_abort_recovery_q
