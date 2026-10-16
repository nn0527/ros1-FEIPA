#!/usr/bin/env python3
"""Publication-bound near-level limiter regression; no vehicle commands are sent."""
import math
import threading
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate as Candidate, OperatorCommand, WallPerchStatus as Wall,
)
import test_candidate_latency as fixture


def q_rpy(roll=0., pitch=0., yaw=0.):
    r, p, y = (math.radians(value) / 2. for value in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def q_tuple(target):
    return tuple(getattr(target.orientation, key) for key in 'xyzw')


def axis_heading(q):
    # Independent algebra, deliberately not the production limiter/helper.
    norm = math.sqrt(sum(value*value for value in q))
    x, y, z, w = (value/norm for value in q)
    return (2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)), math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def steps_degrees(before, after):
    a, yaw_a = axis_heading(before)
    b, yaw_b = axis_heading(after)
    axis = math.acos(max(-1., min(1., sum(x*y for x, y in zip(a, b)))))
    heading = math.atan2(math.sin(yaw_b-yaw_a), math.cos(yaw_b-yaw_a))
    return math.degrees(axis), math.degrees(heading)


class DetachNearLevelLatchTest(unittest.TestCase):
    def setUp(self):
        self.fx = fixture.CandidateLatencyTest()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.node
        self.fx.accept(self.fx.candidate())
        self.n.owner = fixture.Mux.OWNER_WALL
        self.n.output_enabled = True
        self.sequence = 1
        self.fx.advance()

    def measured(self, q=None, source_age=0., receipt_age=0.):
        self.n.odom.pose.pose.orientation = Quaternion(*tuple(q or q_rpy()))
        self.n.odom.header.stamp = self.fx.now-rospy.Duration.from_sec(source_age)
        self.n.odom_at = self.fx.now-rospy.Duration.from_sec(receipt_age)
        self.n.flight_state_at = self.fx.now

    def seed(self, q=None, thrust=.52, age=.02, maneuver='run-a', source=Candidate.SOURCE_WALL):
        target = AttitudeTarget(orientation=Quaternion(*tuple(q or q_rpy())), thrust=thrust)
        self.n._remember_publication(target, source, self.fx.now-rospy.Duration.from_sec(age), maneuver_id=maneuver)

    def command(self, q=None, horizontal=True, phase=Wall.STATE_DETACH_ROTATE, thrust=.52):
        self.sequence += 1
        result = self.fx.candidate(phase=phase, sequence=self.sequence, acquire=False)
        result.attitude_setpoint = Quaternion(*tuple(q or q_rpy()))
        result.horizontal_velocity_active = horizontal
        result.thrust_normalized = thrust
        self.fx.accept(result)
        return result

    def publish(self, candidate):
        self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_ATTITUDE_THRUST,
                         self.n.wall_candidate_rejection)
        return self.n.attitude_publisher.publish.call_args.args[0]

    def assert_bounded(self, before, target, dt):
        axis, heading = steps_degrees(before, q_tuple(target))
        self.assertLessEqual(axis, 10.*dt+1e-6)
        self.assertLessEqual(abs(heading), 20.*dt+1e-6)
        return axis, heading

    def latch(self):
        self.seed(q_rpy(pitch=4.))
        self.measured(q_rpy(roll=2.))
        target = self.publish(self.command(q_rpy(pitch=-4., yaw=30.)))
        self.assertEqual(self.n.wall_near_level_maneuver, 'run-a')
        return target

    def test_first_qualifying_candidate_uses_axis_and_heading_rates_immediately(self):
        before = q_rpy(pitch=4., yaw=10.)
        self.seed(before)
        self.measured(q_rpy(roll=3.))
        target = self.publish(self.command(q_rpy(pitch=-4., yaw=100.), thrust=.70))
        axis, heading = self.assert_bounded(before, target, .02)
        self.assertAlmostEqual(axis, .2, places=6)
        self.assertAlmostEqual(heading, .4, places=6)
        self.assertAlmostEqual(target.thrust, .53)
        self.assertEqual(self.n.wall_near_level_maneuver, 'run-a')
        diag = self.n.last_wall_command['publication_slew']
        self.assertEqual(diag['rate_mode'], 'detach_near_level')
        self.assertEqual(diag['latched_maneuver_id'], 'run-a')
        self.assertAlmostEqual(diag['thrust_axis_step_deg'], axis, places=6)
        self.assertAlmostEqual(diag['heading_step_deg'], heading, places=6)

    def test_all_three_unprojected_cones_are_required(self):
        for measured, previous, requested in ((6., 0., 0.), (0., 6., 0.), (0., 0., 6.)):
            with self.subTest(measured=measured, previous=previous, requested=requested):
                self.n.wall_near_level_maneuver = None
                self.fx.advance()
                self.seed(q_rpy(pitch=previous))
                self.measured(q_rpy(pitch=measured))
                self.n._publish_candidate(self.fx.now, self.command(q_rpy(pitch=requested)))
                self.assertIsNone(self.n.wall_near_level_maneuver)

    def test_stale_source_or_receipt_cannot_establish_latch(self):
        for source_age, receipt_age in ((.6, 0.), (0., .6)):
            with self.subTest(source_age=source_age, receipt_age=receipt_age):
                self.n.wall_near_level_maneuver = None
                self.fx.advance()
                self.seed(q_rpy(pitch=4.))
                self.measured(source_age=source_age, receipt_age=receipt_age)
                self.n._publish_candidate(self.fx.now, self.command(q_rpy(pitch=-4.)))
                self.assertIsNone(self.n.wall_near_level_maneuver)

    def test_old_maneuver_and_prestream_history_cannot_establish_latch(self):
        for maneuver, source in (('old-run', Candidate.SOURCE_WALL), ('run-a', Candidate.SOURCE_NONE)):
            with self.subTest(maneuver=maneuver, source=source):
                self.n.wall_near_level_maneuver = None
                self.fx.advance()
                self.seed(q_rpy(pitch=4.), maneuver=maneuver, source=source)
                self.measured()
                self.n._publish_candidate(self.fx.now, self.command(q_rpy(pitch=-4.)))
                self.assertIsNone(self.n.wall_near_level_maneuver)

    def test_measurement_fallback_cannot_establish_latch(self):
        self.measured(q_rpy(pitch=3.))
        self.n.last_published_control = None
        self.n.last_published_maneuver_id = None
        echo = AttitudeTarget(orientation=Quaternion(*q_rpy()), thrust=.52)
        echo.header.stamp = self.fx.now
        self.n._on_fcu_attitude_target(echo)
        self.publish(self.command(q_rpy(pitch=-4.)))
        self.assertIsNone(getattr(self.n, 'wall_near_level_maneuver', None))
        self.fx.advance()
        self.measured()
        self.publish(self.command(q_rpy(pitch=-4.)))
        self.assertEqual(self.n.wall_near_level_maneuver, 'run-a')

    def test_hloop_toggling_and_measured_excursion_cannot_reenable_fast_slew(self):
        target = self.latch()
        for horizontal, tilt in ((False, 2.), (True, 9.), (False, 7.), (True, 1.)):
            self.fx.advance()
            self.measured(q_rpy(roll=tilt))
            before = q_tuple(target)
            target = self.publish(self.command(q_rpy(roll=-4., yaw=-90.), horizontal=horizontal))
            self.assert_bounded(before, target, .02)
            self.assertEqual(self.n.wall_near_level_maneuver, 'run-a')
            self.assertEqual(self.n.last_wall_command['publication_slew']['rate_mode'], 'detach_near_level')

    def test_final_candidate_expiry_does_not_commit_latch_or_history(self):
        self.seed(q_rpy(pitch=4.))
        self.measured()
        before = self.n.last_published_control
        candidate = self.command(q_rpy(pitch=-4.))
        original = self.n._limit_wall_publication
        def expire(*args, **kwargs):
            result = original(*args, **kwargs)
            self.fx.advance(.21)
            return result
        with mock.patch.object(self.n, '_limit_wall_publication', side_effect=expire):
            self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
        self.assertIsNone(getattr(self.n, 'wall_near_level_maneuver', None))
        self.assertIs(self.n.last_published_control, before)
        self.n.attitude_publisher.publish.assert_not_called()

    def test_feedback_expiry_before_publish_does_not_commit_latch(self):
        self.seed(q_rpy(pitch=4.))
        self.measured(source_age=.49)
        candidate = self.command(q_rpy(pitch=-4.))
        original = self.n._limit_wall_publication
        def expire(*args, **kwargs):
            result = original(*args, **kwargs)
            self.fx.advance(.02)
            return result
        with mock.patch.object(self.n, '_limit_wall_publication', side_effect=expire):
            self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
        self.assertIsNone(getattr(self.n, 'wall_near_level_maneuver', None))
        self.n.attitude_publisher.publish.assert_not_called()

    def test_publisher_exception_cannot_commit_latch(self):
        self.seed(q_rpy(pitch=4.))
        self.measured()
        before = self.n.last_published_control
        self.n.attitude_publisher.publish.side_effect = RuntimeError('simulated transport failure')
        with self.assertRaises(RuntimeError):
            self.n._publish_candidate(self.fx.now, self.command(q_rpy(pitch=-4.)))
        self.assertIsNone(getattr(self.n, 'wall_near_level_maneuver', None))
        self.assertIs(self.n.last_published_control, before)

    def assert_late_permission_rejection(self, permission, phase):
        self.fx.advance()
        self.n.owner = fixture.Mux.OWNER_WALL
        self.n.output_enabled = self.n.wall_output_enabled = True
        self.n.kill_guard_enabled = False
        self.n.wall_near_level_maneuver = None
        self.seed(q_rpy(pitch=4.))
        self.measured()
        if phase == Wall.STATE_RECOVER:
            self.publish(self.command(q_rpy(pitch=-4.)))
            self.fx.advance()
            self.measured()
        before = self.n.last_published_control
        previous_latch = self.n.wall_near_level_maneuver
        previous_count = self.n.attitude_publisher.publish.call_count
        candidate = self.command(q_rpy(pitch=-4.), phase=phase)
        original = self.n._limit_wall_publication
        with mock.patch.object(self.n, '_rc_ready', return_value=True) as rc_ready:
            def revoke(*args, **kwargs):
                result = original(*args, **kwargs)
                if permission.startswith('kill_'):
                    self.n.kill_guard_enabled = True
                    self.n.kill_status_at = self.fx.now
                    self.n.kill_status = {'valid': permission != 'kill_invalid',
                                          'active': permission == 'kill_active'}
                elif permission == 'owner':
                    self.n.owner = fixture.Mux.OWNER_NONE
                elif permission == 'output_disabled':
                    self.n.output_enabled = False
                elif permission == 'wall_output_disabled':
                    self.n.wall_output_enabled = False
                elif permission == 'rc':
                    rc_ready.return_value = False
                else:
                    raise AssertionError('unknown permission test')
                return result
            with mock.patch.object(self.n, '_limit_wall_publication', side_effect=revoke):
                self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
        self.assertEqual(self.n.attitude_publisher.publish.call_count, previous_count)
        self.assertIs(self.n.last_published_control, before)
        self.assertEqual(self.n.wall_near_level_maneuver, previous_latch)

    def test_permission_revoked_before_first_publish_cannot_commit_latch(self):
        for permission in ('kill_active', 'kill_invalid', 'owner', 'output_disabled', 'wall_output_disabled', 'rc'):
            with self.subTest(permission=permission):
                self.assert_late_permission_rejection(permission, Wall.STATE_DETACH_ROTATE)

    def test_latched_recover_rechecks_permission_before_publish(self):
        for permission in ('kill_active', 'kill_invalid', 'owner', 'output_disabled', 'wall_output_disabled', 'rc'):
            with self.subTest(permission=permission):
                self.assert_late_permission_rejection(permission, Wall.STATE_RECOVER)

    def test_offboard_exit_callback_cannot_race_latch_commit(self):
        self.seed(q_rpy(pitch=4.))
        self.measured()
        candidate = self.command(q_rpy(pitch=-4.))
        inside_publish = threading.Event()
        release_publish = threading.Event()
        callback_started = threading.Event()
        callback_finished = threading.Event()
        results, errors = [], []
        def transport(_target):
            inside_publish.set()
            if not release_publish.wait(2.):
                raise RuntimeError('test did not release transport')
        def publish_worker():
            try:
                results.append(self.n._publish_candidate(self.fx.now, candidate))
            except BaseException as error:
                errors.append(error)
        def mode_callback():
            callback_started.set()
            try:
                self.n._on_state(State(connected=True, armed=True, mode='POSCTL'))
            except BaseException as error:
                errors.append(error)
            finally:
                callback_finished.set()
        self.n.attitude_publisher.publish.side_effect = transport
        publisher = threading.Thread(target=publish_worker)
        callback = threading.Thread(target=mode_callback)
        publisher.start()
        try:
            self.assertTrue(inside_publish.wait(2.))
            callback.start()
            self.assertTrue(callback_started.wait(2.))
            self.assertFalse(callback_finished.wait(.02), 'mode callback must serialize with publication')
        finally:
            release_publish.set()
            publisher.join(2.)
            if callback.ident is not None:
                callback.join(2.)
        self.assertFalse(publisher.is_alive())
        self.assertFalse(callback.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(results, [fixture.Mux.OUTPUT_ATTITUDE_THRUST])
        self.assertIsNone(self.n.wall_near_level_maneuver)
        self.assertIsNone(self.n.last_published_control)
        self.assertEqual(self.n.flight_state.mode, 'POSCTL')

    def test_heading_wrap_uses_short_signed_step(self):
        before = q_rpy(pitch=1., yaw=179.8)
        self.seed(before)
        self.measured(before)
        target = self.publish(self.command(q_rpy(pitch=-1., yaw=-179.)))
        _, heading = self.assert_bounded(before, target, .02)
        self.assertAlmostEqual(heading, .4, places=6)

    def test_detach_to_recover_retains_actual_published_history(self):
        target = self.latch()
        for phase in (Wall.STATE_DETACH_ROTATE, Wall.STATE_RECOVER, Wall.STATE_RECOVER):
            self.fx.advance()
            self.measured()
            before, thrust = q_tuple(target), target.thrust
            # Deliberately contradictory unsent helper history.
            self.n.recovery_previous_target = q_rpy(roll=-5., yaw=-120.)
            self.n.recovery_previous_at = self.fx.now
            target = self.publish(self.command(q_rpy(roll=-4., yaw=-90.), phase=phase, thrust=.59))
            self.assert_bounded(before, target, .02)
            self.assertLessEqual(abs(target.thrust-thrust), .010001)

    def test_latched_outside_cone_fallback_rejects_instead_of_projecting(self):
        self.latch()
        self.fx.advance(.6)
        self.measured(q_rpy(pitch=12.))
        echo = AttitudeTarget(orientation=Quaternion(*q_rpy()), thrust=.52)
        echo.header.stamp = self.fx.now
        self.n._on_fcu_attitude_target(echo)
        before = self.n.attitude_publisher.publish.call_count
        self.assertEqual(self.n._publish_candidate(self.fx.now, self.command(horizontal=False)), fixture.Mux.OUTPUT_NONE)
        self.assertEqual(self.n.attitude_publisher.publish.call_count, before)
        self.assertEqual(self.n.wall_near_level_maneuver, 'run-a')

    def test_stale_feedback_and_restoration_preserve_latch(self):
        self.latch()
        self.fx.advance()
        self.measured(source_age=.6)
        self.n._publish_candidate(self.fx.now, self.command(q_rpy(roll=-4.), horizontal=False))
        self.assertEqual(self.n.wall_near_level_maneuver, 'run-a')
        before = self.n.last_published_control[0]
        self.fx.advance()
        self.measured()
        target = self.publish(self.command(q_rpy(roll=-4.), horizontal=False))
        self.assert_bounded(before, target, .04)
        self.assertEqual(self.n.last_wall_command['publication_slew']['rate_mode'], 'detach_near_level')

    def test_offboard_exit_and_retirement_clear_latch_and_maneuver_history(self):
        self.latch()
        self.n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.assertIsNone(self.n.wall_near_level_maneuver)
        self.assertIsNone(self.n.last_published_maneuver_id)
        self.assertIsNone(self.n.last_published_control)

    def test_ownership_exit_clears_latch(self):
        self.latch()
        self.fx.advance()
        self.fx.accept(self.fx.candidate(phase=Wall.STATE_IDLE, sequence=3, acquire=False, keep=False))
        self.n.interlock_active = False
        self.n.interlock_clear_since = None
        self.n.handoff_dwell_s = .5
        self.n._operator_mode = lambda now: OperatorCommand.MODE_MANUAL
        with mock.patch.object(rospy, 'logwarn'), mock.patch.object(rospy, 'loginfo'):
            self.n._update_owner(self.fx.now)
        self.assertNotEqual(self.n.owner, fixture.Mux.OWNER_WALL)
        self.assertIsNone(self.n.wall_near_level_maneuver)

    def test_new_maneuver_clears_previous_latch(self):
        self.latch()
        self.fx.advance()
        self.fx.accept(self.fx.candidate(phase=Wall.STATE_IDLE, sequence=3, acquire=False, keep=False))
        self.n.owner = fixture.Mux.OWNER_NONE
        self.fx.advance()
        self.fx.accept(self.fx.candidate(maneuver='run-b'))
        self.assertIsNone(self.n.wall_near_level_maneuver)
        self.assertIsNone(self.n.last_published_maneuver_id)

    def test_abort_retains_emergency_immediate_leveling(self):
        self.latch()
        self.seed(q_rpy(pitch=70.), thrust=.8)
        self.fx.advance()
        self.measured(q_rpy(pitch=70.))
        target = self.publish(self.command(q_rpy(pitch=60.), phase=Wall.STATE_ABORT, thrust=.49))
        self.assertLessEqual(fixture.tilt(target), 5.000001)
        self.assertAlmostEqual(target.thrust, .78)
        self.assertTrue(self.n.last_wall_command['publication_slew']['abort_level_immediate'])

    def test_recorded_braking_entry_then_explicit_synthetic_50hz_continuation(self):
        # Actual ROS CSV entry from 20261016T164250Z_9b84dc/signals.csv.
        # CSV publication 1792169037.8603194, prior publication 1792169037.7616181.
        # This replays saved input snapshots, not absent intervening high-rate data.
        previous = (0., 0., .6130940858087712, .7900098999039867)
        measured = (-.0005993920646420997, .017466643815310073,
                    -.6073569391952838, -.7942367609155884)
        requested = (-.011485084989596288, -.02534578227002202,
                     .6137783336406075, .7889878587768393)
        self.seed(previous, thrust=.5155335068702698, age=.0987013)
        self.measured(measured, source_age=.0601843)
        target = self.publish(self.command(requested, thrust=.505366325378418))
        self.assert_bounded(previous, target, .05)  # Shared dt cap, as in deployment.
        self.assertEqual(self.n.wall_near_level_maneuver, 'run-a')
        # Explicit synthetic repeated target after the real first-entry fixture.
        for phase in [Wall.STATE_DETACH_ROTATE]*5+[Wall.STATE_RECOVER]*5:
            self.fx.advance()
            self.measured(measured)
            before = q_tuple(target)
            target = self.publish(self.command(requested, phase=phase, thrust=.505366325378418))
            self.assert_bounded(before, target, .02)


if __name__ == '__main__':
    unittest.main()
