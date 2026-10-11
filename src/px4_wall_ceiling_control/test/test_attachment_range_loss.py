#!/usr/bin/env python3
import importlib.util,pathlib,unittest
from px4_wall_ceiling_control.msg import WallPerchStatus as S
spec=importlib.util.spec_from_file_location('hold_test',pathlib.Path(__file__).with_name('test_hold_range_loss.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class AttachmentRangeLossTest(m.HoldRangeLossTest):
 def setUp(self):
  super().setUp();self.node.ignore_range_loss_during_attachment=True
 def test_loss_ignored_only_in_established_hold(self):
  for state in [S.STATE_IDLE,S.STATE_FRONT_WALL_DETECT,S.STATE_STABILIZE_HOVER,S.STATE_SLOW_APPROACH,S.STATE_FLIP_TO_WALL,S.STATE_WALL_CAPTURE,S.STATE_WALL_HOLD,S.STATE_WALL_PIN]:
   self.node.state=state;self.assertTrue(self.node._safety_ok(self.now))
  for state in [S.STATE_DETACH_ROTATE,S.STATE_RECOVER]:
   self.node.state=state;self.assertFalse(self.node._safety_ok(self.now))
 def test_disable_parameter_restores_loss_protection(self):
  self.node.ignore_range_loss_during_attachment=False;self.node.ignore_range_loss_during_hold=False
  for state in [S.STATE_FLIP_TO_WALL,S.STATE_WALL_CAPTURE,S.STATE_WALL_HOLD]:
   self.node.state=state;self.assertFalse(self.node._safety_ok(self.now))
 def test_stale_data_does_not_confirm_contact(self):
  self.node.state=S.STATE_WALL_CAPTURE;self.node.top_contact_since=self.now;self.node.top_contact_last_sample_at=self.now
  self.node.top_capture_near_samples=0;self.node.top_capture_window_samples=3
  self.assertFalse(self.node._top_contact_ready(self.now))
 def test_acquire_loss_waits_without_abort_or_flip(self):
  node=self.node;node.cancel_requested=False;node.rearm_required=False
  node.manual_approach=True;node.fault_count=0
  node._mode_requested=lambda now:True;node._entry_height_ready=lambda:True
  node._bench_ranges_ready=lambda now:False;node._flip_ready=lambda now:True
  entered=[];node._enter_state=lambda state,now,reason:entered.append(state)
  node.state=S.STATE_IDLE;node._update_state_machine(self.now)
  self.assertEqual(entered,[]);self.assertIn("接管等待",node.last_reason)
  node.state=S.STATE_SLOW_APPROACH;node._update_state_machine(self.now)
  self.assertEqual(entered,[]);self.assertEqual(node.fault_count,0)
  neutral,quaternion,thrust,progress=node._candidate(self.now)
  self.assertTrue(neutral);self.assertIsNone(quaternion)
 def test_acquire_requires_current_valid_health_not_cached_range(self):
  from px4_wall_ceiling_control.msg import SensorHealth
  node=self.node;node._sensor_health_fresh=lambda now:True
  node._front_fresh=lambda now:True;node._top_fresh=lambda now:True
  node.sensor_health=SensorHealth(front_valid=True,ceiling_valid=False)
  self.assertFalse(node._bench_ranges_ready(self.now))
  node.sensor_health.ceiling_valid=True
  self.assertTrue(node._bench_ranges_ready(self.now))
 def test_valid_manual_distance_still_starts_flip(self):
  from px4_wall_ceiling_control.msg import SensorHealth
  node=self.node;node.state=S.STATE_IDLE;node.cancel_requested=False
  node.rearm_required=False;node.manual_approach=True;node.fault_count=0
  node._mode_requested=lambda now:True;node._entry_height_ready=lambda:True
  node._sensor_health_fresh=lambda now:True
  node._front_fresh=lambda now:True;node._top_fresh=lambda now:True
  node.sensor_health=SensorHealth(front_valid=True,ceiling_valid=True)
  node._flip_ready=lambda now:True
  entered=[]
  def enter(state,now,reason):
   node.state=state;entered.append(state)
  node._enter_state=enter;node._update_state_machine(self.now)
  self.assertEqual(entered,[S.STATE_FRONT_WALL_DETECT,S.STATE_FLIP_TO_WALL])
 def test_non_range_safety_still_blocks_acquire_and_flip(self):
  for state in [S.STATE_IDLE,S.STATE_SLOW_APPROACH,S.STATE_FLIP_TO_WALL,S.STATE_WALL_CAPTURE]:
   self.node.state=state
   for name in ["_flight_ready","_odom_fresh","_rates_safe","_vertical_speed_safe"]:
    original=getattr(self.node,name);setattr(self.node,name,lambda *args:False)
    self.assertFalse(self.node._safety_ok(self.now),(state,name))
    setattr(self.node,name,original)
 def test_approach_far_sample_enters_abort_only_before_flip(self):
  import rospy
  node=self.node;node._mode_requested=lambda now:True;node.cancel_requested=False
  node.rearm_required=False;node.fault_count=0;node.sensor_timeout_s=.6
  node._capture_attitude_references=lambda:None
  entered=[];node._enter_state=lambda state,now,reason:entered.append((state,reason))
  for state in [S.STATE_IDLE,S.STATE_FRONT_WALL_DETECT,S.STATE_STABILIZE_HOVER,S.STATE_SLOW_APPROACH]:
   node.state=state;node.approach_far_sample=(self.now,1.301)
   node._update_state_machine(self.now)
   self.assertEqual(entered[-1][0],S.STATE_ABORT)
   self.assertIn("1.301 m exceeds 1.300 m",entered[-1][1])
  self.assertEqual(node.fault_count,4)
  for state in [S.STATE_FLIP_TO_WALL,S.STATE_WALL_CAPTURE,S.STATE_WALL_HOLD,S.STATE_DETACH_ROTATE,S.STATE_RECOVER]:
   node.state=state;self.assertFalse(node._approach_stage())
 def test_front_limit_boundary_and_invalid_or_stale_samples(self):
  from sensor_msgs.msg import Range
  import rospy
  node=self.node;node.state=S.STATE_SLOW_APPROACH;node.sensor_timeout_s=.6
  node._front_fresh=lambda now:True;node._sensor_health_fresh=lambda now:True
  from px4_wall_ceiling_control.msg import SensorHealth
  node.sensor_health=SensorHealth(front_valid=True)
  node.front_range=Range(min_range=.01,max_range=40,range=1.3)
  self.assertIsNone(node._approach_far_distance(self.now))
  node.front_range.range=1.31;self.assertAlmostEqual(node._approach_far_distance(self.now),1.31)
  node.front_range.range=float("nan");self.assertIsNone(node._approach_far_distance(self.now))
  node.front_range.range=1.31;node._front_fresh=lambda now:False
  self.assertIsNone(node._approach_far_distance(self.now))
  node.approach_far_sample=(rospy.Time.from_sec(9),1.5)
  self.assertIsNone(node._approach_far_distance(self.now))
if __name__=='__main__':unittest.main()
