"""Startup regression tests: no flight commands and no live ROS dependency."""
import importlib.util
import pathlib
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
spec = importlib.util.spec_from_file_location('startup_server', str(ROOT / 'scripts/operator/wall_console_server.py'))
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class MasterStartupTests(unittest.TestCase):
    def runtime(self, folder):
        r = object.__new__(server.RemoteRuntime)
        r.state = server.ConsoleState()
        r.log_dir = pathlib.Path(folder)
        r.children = {}
        r.stop_requested = False
        r.spawn = mock.Mock()
        return r

    def test_empty_graph_is_a_live_master_and_never_spawns(self):
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(server, 'master_system_state', return_value=[[], [], []]):
            r = self.runtime(folder)
            self.assertEqual(r.nodes(), set())
            self.assertTrue(r.master_alive())
            r.ensure_ros_master()
            r.spawn.assert_not_called()

    def test_nodes_are_read_directly_without_launching_rosnode_cli(self):
        state = [[['a', ['/a']]], [['b', ['/b', '/a']]], [['c', ['/c']]]]
        with mock.patch.object(server, 'master_system_state', return_value=state), mock.patch.object(server.subprocess, 'run') as run:
            self.assertEqual(server.RemoteRuntime.nodes(), {'/a', '/b', '/c'})
            run.assert_not_called()

    def test_failed_probe_is_bounded_and_does_not_escape(self):
        with mock.patch.object(server.xmlrpc.client, 'ServerProxy', side_effect=TimeoutError('slow master')):
            self.assertIsNone(server.master_system_state())
            self.assertEqual(server.RemoteRuntime.nodes(), set())
        transport = server.MasterTransport()
        self.assertEqual(transport.make_connection('localhost:11311').timeout, 1)

    def test_retired_roscore_pid_does_not_mask_live_master(self):
        with tempfile.TemporaryDirectory() as folder:
            r = self.runtime(folder)
            r.children['roscore'] = SimpleNamespace(poll=lambda: 1)
            r.state.launch_error = 'roscore 已退出，请查看启动日志'
            r.master_alive = mock.Mock(return_value=True)
            r.update_process_errors({'enabled': True, 'nodes': []})
            self.assertEqual(r.state.launch_error, '')
            self.assertNotIn('roscore', r.children)

    def test_missing_master_retains_exit_error_and_other_errors_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            r = self.runtime(folder)
            r.children['roscore'] = SimpleNamespace(poll=lambda: 1)
            r.master_alive = mock.Mock(return_value=False)
            r.update_process_errors({'enabled': False, 'nodes': []})
            self.assertEqual(r.state.launch_error, 'roscore 已退出，请查看启动日志')
            r.state.launch_error = '雷达串口错误'
            r.master_alive.return_value = True
            r.update_process_errors({'enabled': True, 'nodes': []})
            self.assertEqual(r.state.launch_error, '雷达串口错误')

    @unittest.skipUnless(sys.platform.startswith('linux'), 'fcntl tests run inside Docker')
    def test_race_loser_adopts_winning_master(self):
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(server.time, 'sleep'):
            r = self.runtime(folder)
            r.master_alive = mock.Mock(side_effect=[False, False, True])
            r.master_listening = mock.Mock(return_value=False)
            r.spawn.side_effect = lambda *args: r.children.update(roscore=SimpleNamespace(poll=lambda: 1))
            r.ensure_ros_master()
            r.spawn.assert_called_once_with('roscore', ['roscore'])
            self.assertNotIn('roscore', r.children)

    @unittest.skipUnless(sys.platform.startswith('linux'), 'fcntl tests run inside Docker')
    def test_busy_unresponsive_master_is_not_replaced(self):
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(server.time, 'sleep'), mock.patch.object(server.time, 'monotonic', side_effect=[0, .1, 11]):
            r = self.runtime(folder)
            r.master_alive = mock.Mock(return_value=False)
            r.master_listening = mock.Mock(return_value=True)
            with self.assertRaisesRegex(ValueError, '无响应'):
                r.ensure_ros_master()
            r.spawn.assert_not_called()

    @unittest.skipUnless(sys.platform.startswith('linux'), 'fcntl tests run inside Docker')
    def test_cancel_never_starts_master(self):
        with tempfile.TemporaryDirectory() as folder:
            r = self.runtime(folder)
            r.master_alive = mock.Mock(return_value=False)
            r.stop_requested = True
            with self.assertRaisesRegex(ValueError, '取消'):
                r.ensure_ros_master()
            r.spawn.assert_not_called()


if __name__ == '__main__':
    unittest.main()
