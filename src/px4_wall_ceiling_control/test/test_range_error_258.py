#!/usr/bin/env python3
import importlib.util,pathlib,unittest
from types import SimpleNamespace
p=pathlib.Path(__file__).resolve().parents[1]/'scripts/decisions/wall_perch_decision_node.py'
spec=importlib.util.spec_from_file_location('range258',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Range258Test(unittest.TestCase):
 def setUp(self):
  self.n=m.WallPerchDecisionNode.__new__(m.WallPerchDecisionNode)
  self.n.ignore_range_error_258=True
  self.n.sensor_health=SimpleNamespace(ceiling_valid=False,ceiling_error_code=258,front_valid=False,front_error_code=258)
  self.n._sensor_health_fresh=lambda now:True
  self.n._top_fresh=lambda now:True
  self.n._front_fresh=lambda now:True
 def test_258_tolerated_without_changing_health(self):
  for side in ['front','ceiling']:self.assertTrue(self.n._range_valid_for_safety(side,None));self.assertFalse(getattr(self.n.sensor_health,side+'_valid'))
 def test_other_errors_rejected(self):
  for code in [255,-1,-2,-8,0]:
   self.n.sensor_health.ceiling_error_code=code;self.assertFalse(self.n._range_valid_for_safety('ceiling',None))
 def test_stale_range_rejected(self):
  self.n._top_fresh=lambda now:False;self.assertFalse(self.n._range_valid_for_safety('ceiling',None))
 def test_stale_health_rejected(self):
  self.n._sensor_health_fresh=lambda now:False;self.assertFalse(self.n._range_valid_for_safety('ceiling',None))
 def test_disabled_restores_original_check(self):
  self.n.ignore_range_error_258=False;self.assertFalse(self.n._range_valid_for_safety('ceiling',None))
if __name__=='__main__':unittest.main()
