"""Real Linux socket/singleton tests; no ROS or vehicle commands."""
import json
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import tempfile
import time
import unittest

PACKAGE = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE))


def fake_service(path, marker):
    from px4_wall_ceiling_control.shared_bridge import serve
    from px4_wall_ceiling_control.wall_console import ConsoleState
    class Runtime:
        def __init__(self):
            with open(marker, "a") as log: log.write("runtime\n")
            self.state = ConsoleState()
        def wall(self): return {"message": "simulated wall"}
        def start(self, config): raise RuntimeError("simulated startup fault")
    serve(Runtime, path)


@unittest.skipUnless(sys.platform.startswith("linux"), "Pi Unix socket tests run on Linux")
class ServiceTests(unittest.TestCase):
    def test_singleton_two_clients_pause_control_disconnect(self):
        with tempfile.TemporaryDirectory() as folder:
            path, marker = str(Path(folder)/"bridge.sock"), str(Path(folder)/"instances")
            args = [sys.executable, __file__, "--fixture", path, marker]
            worker = subprocess.Popen(args)
            clients = []
            try:
                for _ in range(100):
                    if Path(path).exists(): break
                    time.sleep(.05)
                duplicate = subprocess.run(args, timeout=3)
                self.assertEqual(duplicate.returncode, 0)
                self.assertEqual(Path(marker).read_text(), "runtime\n")
                def connect(name):
                    conn = socket.socket(socket.AF_UNIX)
                    conn.connect(path)
                    clients.append(conn)
                    conn.sendall((json.dumps({"client": name*48, "label": name})+"\n").encode())
                    return conn, conn.makefile("r")
                a, ar = connect("a")
                b, br = connect("b")
                def reply(conn, reader, action, config=None):
                    conn.sendall((json.dumps({"id": action, "action": action, "config": config})+"\n").encode())
                    for _ in range(100):
                        message = json.loads(reader.readline())
                        if message.get("id") == action: return message
                    self.fail("no reply")
                self.assertTrue(reply(a, ar, "acquire")["ok"])
                failure = reply(a, ar, "start", {})
                self.assertFalse(failure["ok"])
                self.assertIn("simulated startup fault", failure["error"])
                # The next request must work on the same connection after a failure.
                self.assertTrue(reply(a, ar, "wall")["ok"])
                self.assertFalse(reply(b, br, "wall")["ok"])
                self.assertTrue(reply(a, ar, "push", {"enabled": False})["ok"])
                # Following the paused metadata there must be no more snapshots.
                for _ in range(50):
                    message = json.loads(ar.readline())
                    if message["type"] == "session" and not message["data"]["push"]: break
                for _ in range(2): self.assertEqual(json.loads(ar.readline())["type"], "session")
                self.assertFalse(reply(a, ar, "wall")["ok"])
                for _ in range(50):
                    if json.loads(br.readline())["type"] == "snapshot": break
                else: self.fail("other client stopped")
                self.assertTrue(reply(a, ar, "push", {"enabled": True})["ok"])
                self.assertTrue(reply(a, ar, "wall")["ok"])
                a.shutdown(socket.SHUT_RDWR)
                ar.close(); a.close()
                time.sleep(.2)
                self.assertFalse(reply(b, br, "acquire")["ok"])
                ar = None
                br.close()
            finally:
                for conn in clients: conn.close()
                worker.terminate()
                worker.wait(timeout=5)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--fixture": fake_service(sys.argv[2], sys.argv[3])
    else: unittest.main()
