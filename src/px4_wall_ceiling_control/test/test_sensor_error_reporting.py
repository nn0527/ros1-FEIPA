#!/usr/bin/env python3
"""Sensor errors, recovery and timeout must identify the affected channel."""
import importlib.util
import json
import pathlib
import unittest
from unittest.mock import patch, Mock

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import String

ROOT = pathlib.Path(__file__).resolve().parents[1]
def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts/sensors' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

manager = load('sensor_range_manager_node')
lidar = load('lidar_distance_node')

class SensorErrorReportingTest(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(10))
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        with patch.object(rospy, 'get_param', side_effect=lambda key, default: default), \
             patch.object(rospy, 'Publisher', return_value=Mock()), \
             patch.object(rospy, 'Subscriber'), patch.object(rospy, 'Timer'):
            self.node = manager.SensorRangeManagerNode()

    def sample(self, distance):
        msg = Range(); msg.range = distance; msg.min_range = .03; msg.max_range = 40
        return msg

    def diag(self, channel, code, kind='sensor'):
        self.node._on_diagnostic(channel, String(data=json.dumps({
            'valid': False, 'error_code': code, 'error_type': kind, 'error': 'test error'})))

    def health(self):
        return self.node._build_health(rospy.Time.now())

    def test_device_error_255_recovery_and_channel_independence(self):
        self.node._on_front(self.sample(1)); self.node._on_ceiling(self.sample(.04))
        self.diag('front', 255); self.node._on_front(self.sample(float('nan')))
        status = self.health()
        self.assertFalse(status.front_valid); self.assertEqual(status.front_error_code, 255)
        self.assertEqual(status.front_error_type, 'sensor'); self.assertTrue(status.ceiling_valid)
        self.assertEqual(status.ceiling_error_code, 0)
        self.node._on_front(self.sample(1))
        self.assertTrue(self.health().front_valid); self.assertEqual(self.health().front_error_reason, '')

    def test_raw_then_diagnostic_order_preserves_sensor_code(self):
        self.node._on_ceiling(self.sample(float('nan'))); self.diag('ceiling', 258)
        self.assertEqual(self.health().ceiling_error_code, 258)

    def test_timeout_replaces_stale_device_error(self):
        self.node._on_front(self.sample(float('nan'))); self.diag('front', 255)
        self.now.return_value = rospy.Time.from_sec(11)
        self.assertEqual(self.health().front_error_code, -1)

    def test_no_data_nonfinite_and_range_limits(self):
        self.assertEqual(self.health().front_error_code, -2)
        self.node._on_front(self.sample(float('inf')))
        self.assertEqual(self.health().front_error_code, -3)
        self.node._on_ceiling(self.sample(.01))
        self.assertEqual(self.health().ceiling_error_code, -4)

    def test_uart_error_is_available_without_range_samples(self):
        self.diag('front', -8, 'serial')
        self.now.return_value = rospy.Time.from_sec(12)
        self.assertEqual(self.health().front_error_code, -8)
        self.assertFalse(self.health().front_valid)

    def test_malformed_diagnostic_and_unknown_device_code(self):
        self.node._on_diagnostic('front', String(data='not json'))
        self.assertEqual(self.health().front_error_code, -9)
        self.diag('front', 999)
        self.assertEqual(self.health().front_error_code, 999)

    def test_driver_preserves_native_code_and_invalid_range(self):
        with self.assertRaises(lidar.LidarInputError) as caught:
            lidar.LidarDistanceNode.decode_l1_ascii(b'E=255\r\n')
        self.assertEqual(caught.exception.code, 255)
        self.assertIn('weak reflection', str(caught.exception))
        self.assertAlmostEqual(lidar.LidarDistanceNode.decode_l1_ascii(b'D=0.045m,520#'), .045)
        node = lidar.LidarDistanceNode.__new__(lidar.LidarDistanceNode)
        node.input_format = 'l1_ascii'; node.range_publisher = Mock(); node.diagnostic_publisher = Mock()
        node._make_range = lambda distance, frame, stamp: self.sample(distance)
        node.frame_id = 'front'
        node._report_diagnostic(False, 255, 'sensor', 'weak reflection', b'E=255')
        detail = json.loads(node.diagnostic_publisher.publish.call_args[0][0].data)
        self.assertEqual(detail['error_code'], 255); self.assertEqual(detail['raw_frame'], 'E=255')
        self.assertTrue(__import__('math').isnan(node.range_publisher.publish.call_args[0][0].range))

if __name__ == '__main__':
    unittest.main()
