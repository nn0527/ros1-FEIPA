import pathlib,sys,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/"src"))
from px4_wall_ceiling_control.adapters.kill_switch import KillSwitchConfig,kill_clear
from types import SimpleNamespace


class Time:
    def __init__(self,value):self.value=value
    def __sub__(self,other):return Time(self.value-other.value)
    def to_sec(self):return self.value


class KillTests(unittest.TestCase):
    def test_actual_channel_seven_calibration(self):
        config=KillSwitchConfig(7,.75,1000,2000,1)
        self.assertFalse(config.report([1500]*6+[999],True)["active"])
        self.assertTrue(config.report([1500]*6+[1999],True)["active"])

    def test_missing_channel_stale_rc_and_invalid_pwm_fail_closed(self):
        config=KillSwitchConfig(7)
        for channels,fresh in [([1500],True),([1500]*7,False),([1500]*6+[0],True)]:
            self.assertFalse(config.report(channels,fresh)["valid"])

    def test_reverse_and_negative_threshold_follow_fcu_polarity(self):
        self.assertTrue(KillSwitchConfig(1,.75,1000,2000,-1).report([1000],True)["active"])
        self.assertTrue(KillSwitchConfig(1,-.25).report([1000],True)["active"])
        self.assertFalse(KillSwitchConfig(1,-.25).report([2000],True)["active"])

    def test_unassigned_channel_is_explicitly_reported(self):
        self.assertEqual(KillSwitchConfig(0).report([],False),{"valid":True,"active":False,"channel":0})

    def test_kill_guard_rejects_active_unknown_stale_and_clock_reset(self):
        node=SimpleNamespace(kill_guard_enabled=True,kill_status_at=Time(10),kill_status={"valid":True,"active":False})
        self.assertTrue(kill_clear(node,Time(10.1)))
        self.assertFalse(kill_clear(node,Time(10.4)))
        self.assertFalse(kill_clear(node,Time(9.9)))
        node.kill_status["active"]=True
        self.assertFalse(kill_clear(node,Time(10.1)))
        node.kill_status=None
        self.assertFalse(kill_clear(node,Time(10.1)))

if __name__=="__main__":unittest.main()
