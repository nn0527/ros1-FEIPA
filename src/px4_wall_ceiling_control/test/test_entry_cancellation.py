"""Canceled OFFBOARD transactions, delayed acknowledgements and bounded dropouts."""
import unittest
from unittest import mock
import rospy
from mavros_msgs.msg import State, EstimatorStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from px4_wall_ceiling_control.msg import OperatorCommand
import test_manual_wall_approach as fixtures
from test_flight_handover import Manager

class EntryCancellationTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.ManualApproachTests();self.f.setUp();self.n=self.f.node
        self.n.wall_auto_approach=True;self.n.wall_manual_approach=False
        self.n.wall_pilot_mode='POSCTL';self.n.flight.mode='POSCTL'
        self.n.wall.front_ready=self.n.wall.sensor_health_fresh=True
        self.n.phase=Manager.ENTERING;self.n.selected_mode=OperatorCommand.MODE_WALL
        self.n.takeover_attempt=4;self.n.offboard_request=(4,OperatorCommand.MODE_WALL,self.f.clock)
        self.n.enter_timeout_s=6.;self.n.request_in_flight=False
        self.n.last_offboard_finished_at=None
    def tick(self,dt=0):
        self.f.clock+=dt;self.f.tick()
    def cancel(self):
        self.n._requested_mode.return_value=None;self.tick()
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
    def heartbeat(self,mode='POSCTL'):
        with mock.patch.object(rospy.Time,'now',return_value=self.f.now):
            self.n._on_flight(State(connected=True,armed=True,mode=mode))
    def test_twenty_ms_drop_does_not_cancel_or_authorize_unready_output(self):
        self.n.output_mode=0;self.tick()
        self.assertEqual(self.n.phase,Manager.ENTERING)
        self.n.flight.mode='OFFBOARD';self.tick(.02)
        self.assertEqual(self.n.phase,Manager.ENTERING)
        with mock.patch.object(rospy.Time,'now',return_value=self.f.now):self.n._on_output(UInt8(data=2))
        self.tick(.02);self.assertEqual(self.n.phase,Manager.ACTIVE)
    def test_sustained_dropout_cancels_and_keeps_prestream(self):
        self.n.output_mode=0;self.tick();self.tick(.13)
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.assertTrue(self.n.prestream_pub.publish.call_args[0][0].data)
        self.assertIsNone(self.n.offboard_request)
        self.assertEqual(self.n.selected_mode,OperatorCommand.MODE_WALL)
    def test_stale_output_has_no_grace(self):
        self.n.output_at=rospy.Time.from_sec(8);self.tick()
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
    def test_hard_preflight_fault_has_no_grace(self):
        self.n._preflight_ready.return_value=False;self.tick()
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
    def test_late_offboard_never_starts_approach_and_returns_only_when_safe(self):
        self.cancel();self.n._entry_cancel_handover_ready=mock.Mock(return_value=False)
        self.heartbeat('OFFBOARD');self.tick(.2)
        self.n._request_mode.assert_not_called()
        self.n._entry_cancel_handover_ready.return_value=True
        self.tick(.1);self.tick(.31)
        self.n._request_mode.assert_called_once_with('POSCTL')
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
    def test_two_fresh_pilot_confirmations_and_settle_window_required(self):
        self.cancel();self.heartbeat();self.tick(7.)
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.heartbeat();self.tick()
        self.assertEqual(self.n.phase,Manager.LOCKED)
    def test_rpc_in_flight_prevents_cancel_completion_and_new_actions(self):
        self.cancel();self.n.request_in_flight=True
        self.heartbeat();self.heartbeat();self.tick(9.)
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.n._request_mode.assert_not_called()
        self.n.request_in_flight=False;self.n.last_offboard_finished_at=self.f.clock
        self.tick();self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.tick(6.1);self.assertEqual(self.n.phase,Manager.LOCKED)
    def test_pilot_switch_to_third_mode_is_respected(self):
        self.cancel();self.heartbeat('STABILIZED');self.tick(10.)
        self.n._request_mode.assert_not_called()
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
    def test_stale_pilot_heartbeat_cannot_finish_cancel(self):
        self.cancel();self.heartbeat();self.heartbeat()
        self.n.flight_at=rospy.Time.from_sec(8);self.tick(8.)
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
    def test_even_later_offboard_reopens_cleanup_from_tombstone(self):
        self.cancel();self.heartbeat();self.heartbeat();self.tick(7.)
        self.assertEqual(self.n.phase,Manager.LOCKED)
        self.n._entry_cancel_handover_ready=mock.Mock(return_value=False)
        self.heartbeat('OFFBOARD');self.tick(.1)
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.assertEqual(self.n.selected_mode,OperatorCommand.MODE_WALL)
    def test_unrelated_offboard_while_waiting_is_cleaned_up(self):
        self.n.phase=Manager.WAITING_WALL;self.n.offboard_request=None
        self.heartbeat('OFFBOARD');self.tick()
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
    def test_preownership_return_has_real_pose_estimator_and_rc_checks(self):
        n=self.n;n._entry_cancel_handover_ready=Manager._entry_cancel_handover_ready.__get__(n)
        n.flight.mode='OFFBOARD';n._rc_fresh=mock.Mock(return_value=True)
        n.estimator=EstimatorStatus(attitude_status_flag=True,velocity_vert_status_flag=True,
            velocity_horiz_status_flag=True,pos_horiz_rel_status_flag=True)
        n.estimator_at=self.f.now;n.estimator_timeout_s=.5
        n.odom=Odometry();n.odom.pose.pose.orientation.w=1
        n.odom_at=self.f.now;n.odom_timeout_s=.5;n.odom_twist_in_child_frame=False
        n.handover_max_vertical_speed_mps=.2
        self.assertTrue(n._entry_cancel_handover_ready(self.f.now))
        n.estimator.const_pos_mode_status_flag=True
        self.assertFalse(n._entry_cancel_handover_ready(self.f.now))
        n.estimator.const_pos_mode_status_flag=False;n._rc_fresh.return_value=False
        self.assertFalse(n._entry_cancel_handover_ready(self.f.now))
    def test_existing_behavior_owner_keeps_full_recovery_gates(self):
        self.n.owner=2;self.n._handover_ready.return_value=False
        self.assertFalse(self.n._entry_cancel_handover_ready(self.f.now))
        self.n._handover_ready.assert_called_once_with(self.f.now)


    def test_pending_cancel_rejects_new_action_and_old_rpc_worker(self):
        self.cancel()
        self.n._requested_mode.return_value=OperatorCommand.MODE_CEILING
        self.tick(1.)
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.assertEqual(self.n.selected_mode,OperatorCommand.MODE_WALL)
        self.n.set_mode=mock.Mock()
        with mock.patch.object(rospy.Time,'now',return_value=self.f.now),mock.patch.object(rospy,'wait_for_service'):
            self.n._send_mode_request('OFFBOARD',attempt=4)
        self.n.set_mode.assert_not_called()

    def test_emergency_stop_immediately_stops_output_but_retains_cleanup(self):
        self.n.kill_guard_enabled=True
        self.n.kill_status={'valid':True,'active':True};self.n.kill_status_at=self.f.now
        self.tick()
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.assertFalse(self.n.prestream_pub.publish.call_args[0][0].data)
        self.n._request_mode.assert_not_called()

if __name__=='__main__':unittest.main()
