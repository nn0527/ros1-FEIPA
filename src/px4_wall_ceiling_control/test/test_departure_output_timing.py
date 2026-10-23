"""Actual publication intervals, including processing, bound final thrust slew."""
import importlib.util
import os
import sys
import unittest

if importlib.util.find_spec('test_candidate_latency') is None:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_candidate_latency as fixtures


class DepartureOutputTimingTests(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.CandidateLatencyTest()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.n=self.fixture.node
        # A real maneuver acquires ownership in approach before returning.
        self.fixture.accept(self.fixture.candidate(sequence=1))

    def publish_after_processing(self, seconds):
        original=self.n._attitude_target

        def delayed(*args,**kwargs):
            self.fixture.advance(seconds)
            return original(*args,**kwargs)

        self.n._attitude_target=delayed

    def candidate(self, sequence=1):
        candidate=self.fixture.candidate(phase=fixtures.Wall.STATE_ABORT,sequence=sequence+1)
        candidate.recovery_support_valid=True
        candidate.recovery_thrust_limit=.85
        self.fixture.accept(candidate)
        return candidate

    def test_processing_time_is_counted_between_actual_publications(self):
        self.fixture.advance(.02)
        self.fixture.seed_publication(thrust=.50,age=.01)
        candidate=self.candidate()
        began=self.fixture.now
        self.publish_after_processing(.01)
        self.assertEqual(self.n._publish_candidate(began,candidate),fixtures.Mux.OUTPUT_ATTITUDE_THRUST)
        published=self.n.attitude_publisher.publish.call_args.args[0]
        self.assertAlmostEqual(published.header.stamp.to_sec(),10.03)
        self.assertAlmostEqual(published.thrust,.51,places=7)
        self.assertAlmostEqual(self.n.wall_publication_slew['dt_s'],.02,places=7)

    def test_processing_does_not_halve_slew_at_fifty_hz(self):
        self.fixture.advance(.01)
        self.fixture.seed_publication(thrust=.50)
        previous=.50
        self.publish_after_processing(.01)
        for index in range(1,11):
            self.fixture.now=fixtures.rospy.Time.from_sec(10.+index*.02)
            candidate=self.candidate(sequence=index)
            self.assertEqual(self.n._publish_candidate(self.fixture.now,candidate),fixtures.Mux.OUTPUT_ATTITUDE_THRUST)
            published=self.n.attitude_publisher.publish.call_args.args[0]
            self.assertLessEqual(abs(published.thrust-previous),.01000001)
            previous=published.thrust
        self.assertAlmostEqual(previous,.60,places=7)

    def test_delayed_processing_keeps_existing_fifty_ms_maximum_budget(self):
        self.fixture.advance(.02)
        self.fixture.seed_publication(thrust=.50,age=.02)
        candidate=self.candidate()
        began=self.fixture.now
        self.publish_after_processing(.08)
        self.assertEqual(self.n._publish_candidate(began,candidate),fixtures.Mux.OUTPUT_ATTITUDE_THRUST)
        published=self.n.attitude_publisher.publish.call_args.args[0]
        self.assertLessEqual(published.thrust-.50,.02500001)
        self.assertAlmostEqual(self.n.wall_publication_slew['dt_s'],.05,places=7)

    def test_processing_that_expires_candidate_still_cannot_publish(self):
        self.fixture.advance(.02)
        self.fixture.seed_publication(thrust=.50,age=.02)
        candidate=self.candidate()
        began=self.fixture.now
        self.publish_after_processing(.21)
        self.assertEqual(self.n._publish_candidate(began,candidate),fixtures.Mux.OUTPUT_NONE)
        self.n.attitude_publisher.publish.assert_not_called()


if __name__=='__main__':
    unittest.main()
