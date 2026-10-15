"""Failsafe fallbacks retain OFFBOARD intention; explicitly reassert pilot mode."""
import unittest
from unittest import mock
import rospy
from mavros_msgs.msg import State, EstimatorStatus
from px4_wall_ceiling_control.msg import OperatorCommand
import test_entry_cancellation as cancellation_tests
from test_flight_handover import Manager

class OffboardExitCleanupTests(unittest.TestCase):
    def setUp(self):
        fixture=cancellation_tests.EntryCancellationTests();fixture.setUp()
        self.f=fixture.f;self.n=fixture.n;self.helper=fixture
        self.n.phase=Manager.ACTIVE
        self.n._offboard_exit_request_ready=mock.Mock(return_value=True)
        self.n.last_request_at=None
    def tick(self,dt=0):self.helper.tick(dt)
    def heartbeat(self,mode='POSCTL'):self.helper.heartbeat(mode)
    def leave(self):
        self.heartbeat();self.tick()
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.assertTrue(self.n.offboard_exit_cleanup)
    def submit(self,success=True):
        self.n.set_mode=mock.Mock(return_value=type('Response',(),{'mode_sent':success})())
        with mock.patch.object(rospy.Time,'now',return_value=self.f.now),mock.patch('time.monotonic',return_value=self.f.clock),mock.patch.object(rospy,'wait_for_service'):
            self.n._send_mode_request('POSCTL',attempt=self.n.takeover_attempt)
    def test_fallback_retains_context_cancels_old_action_and_stops_prestream(self):
        self.leave();self.assertEqual(self.n.selected_mode,OperatorCommand.MODE_WALL)
        self.assertIsNone(self.n.offboard_request)
        self.n.cancel_pub.publish.assert_called_once()
        self.assertFalse(self.n.prestream_pub.publish.call_args[0][0].data)
    def test_explicit_same_posctl_request_required_despite_two_heartbeats(self):
        self.leave();self.heartbeat();self.heartbeat();self.tick(7)
        self.n._request_mode.assert_called_with('POSCTL')
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
    def test_submission_and_two_later_heartbeats_plus_settle_are_required(self):
        self.leave();self.heartbeat();self.heartbeat();self.submit()
        self.assertEqual(self.n.cancel_entry_pilot_samples,0)
        self.tick(7);self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.heartbeat();self.tick();self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.heartbeat();self.tick();self.assertEqual(self.n.phase,Manager.LOCKED)
    def test_rejected_submission_never_finishes(self):
        self.leave();self.submit(False);self.heartbeat();self.heartbeat();self.tick(9)
        self.assertIsNone(self.n.pilot_intent_request_at)
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
    def test_old_rpc_worker_cannot_set_confirmation_or_send(self):
        self.leave();self.n.set_mode=mock.Mock()
        with mock.patch.object(rospy.Time,'now',return_value=self.f.now),mock.patch.object(rospy,'wait_for_service'):
            self.n._send_mode_request('POSCTL',attempt=self.n.takeover_attempt-1)
            self.n._send_mode_request('OFFBOARD',attempt=self.n.takeover_attempt-1)
        self.n.set_mode.assert_not_called()
    def test_renewed_offboard_invalidates_confirmation_and_needs_recovery(self):
        self.leave();self.submit();self.heartbeat('OFFBOARD')
        self.assertIsNone(self.n.pilot_intent_request_at)
        self.n._offboard_exit_request_ready.return_value=False
        self.n._request_mode.reset_mock();self.tick(9)
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.n._request_mode.assert_not_called()
    def test_safe_renewed_offboard_requests_pilot_mode_after_stability(self):
        self.leave();self.heartbeat('OFFBOARD');self.tick();self.tick(.31)
        self.n._request_mode.assert_called_once_with('POSCTL')
    def test_stale_state_or_third_mode_does_not_send_mode_request(self):
        self.leave();self.heartbeat('AUTO.LAND');self.tick(8)
        self.n._request_mode.assert_not_called()
        self.heartbeat();self.n.flight_at=rospy.Time.from_sec(8);self.tick()
        self.n._request_mode.assert_not_called()
    def test_new_request_cannot_escape_cleanup_and_old_request_must_release(self):
        self.leave();self.submit();self.n._requested_mode.return_value=OperatorCommand.MODE_CEILING
        self.heartbeat();self.heartbeat();self.tick(7)
        self.assertEqual(self.n.phase,Manager.LOCKED)
        self.tick();self.assertEqual(self.n.phase,Manager.LOCKED)
        self.n._requested_mode.return_value=None;self.tick()
        self.assertEqual(self.n.phase,Manager.IDLE)
    def test_kill_stops_output_and_cleanup_survives(self):
        self.leave();self.n.kill_guard_enabled=True
        self.n.kill_status={'valid':True,'active':True};self.n.kill_status_at=self.f.now
        self.tick(9);self.n._request_mode.assert_not_called()
        self.assertTrue(self.n.offboard_exit_cleanup)
        self.assertFalse(self.n.prestream_pub.publish.call_args[0][0].data)
    def test_real_same_mode_readiness_needs_rc_and_posctl_estimates(self):
        self.leave();n=self.n;n._offboard_exit_request_ready=Manager._offboard_exit_request_ready.__get__(n)
        n.rc_timeout_s=.2;n.rc_channels=[1500]*4;n.rc_at=self.f.now
        n.estimator_at=self.f.now;n.estimator_timeout_s=.5
        n.estimator=EstimatorStatus(attitude_status_flag=True,velocity_vert_status_flag=True,
            velocity_horiz_status_flag=True,pos_horiz_rel_status_flag=True)
        self.assertTrue(n._offboard_exit_request_ready(self.f.now))
        n.estimator.const_pos_mode_status_flag=True;self.assertFalse(n._offboard_exit_request_ready(self.f.now))
        n.estimator.const_pos_mode_status_flag=False;n.rc_at=rospy.Time.from_sec(8)
        self.assertFalse(n._offboard_exit_request_ready(self.f.now))
    def test_mode_worker_rechecks_readiness_after_service_wait(self):
        self.leave();self.n._offboard_exit_request_ready.return_value=False;self.submit()
        self.n.set_mode.assert_not_called()

    def test_late_offboard_requires_live_rc_even_with_recovery_ready(self):
        self.leave();n=self.n;n._offboard_exit_request_ready=Manager._offboard_exit_request_ready.__get__(n)
        self.heartbeat('OFFBOARD');n.rc_timeout_s=.2;n.rc_channels=[1500]*4;n.rc_at=self.f.now
        n._entry_cancel_handover_ready=mock.Mock(return_value=True)
        self.assertTrue(n._offboard_exit_request_ready(self.f.now))
        n.rc_at=rospy.Time.from_sec(8)
        self.assertFalse(n._offboard_exit_request_ready(self.f.now))

if __name__=='__main__':unittest.main()
