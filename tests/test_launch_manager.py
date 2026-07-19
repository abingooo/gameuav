import os
import socket
import sys
import signal
import threading
import time
import unittest
import yaml
from unittest.mock import patch

from agent.launch_manager.manager import LaunchManager, ModuleRuntimeError


def write_config(path):
    config = {
        "modules": {
            "roscore": {
                "type": "process",
                "command": [
                    sys.executable,
                    "-c",
                    (
                        "import os,signal,sys,time;"
                        "from xmlrpc.server import SimpleXMLRPCServer;"
                        "uri=os.environ.get('ROS_MASTER_URI','http://localhost:11311');"
                        "host_port=uri.split('://',1)[-1].split('/',1)[0];"
                        "host,port=(host_port.rsplit(':',1) if ':' in host_port else ('localhost','11311'));"
                        "ready_at=time.monotonic()+0.25;"
                        "server=SimpleXMLRPCServer((host,int(port)),logRequests=False,allow_none=True);"
                        "server.register_function(lambda caller,key: [1,'run_id','test-run-id'] if time.monotonic() >= ready_at else [-1,'not ready',0], 'getParam');"
                        "signal.signal(signal.SIGINT, lambda s,f: sys.exit(0));"
                        "server.serve_forever()"
                    ),
                ],
            },
            "sleepy": {
                "type": "process",
                "command": [
                    sys.executable,
                    "-c",
                    "import signal,time; signal.signal(signal.SIGINT, lambda s,f: exit(0)); time.sleep(60)",
                ],
            },
            "auto_sleepy": {
                "type": "process",
                "autostart": True,
                "command": [
                    sys.executable,
                    "-c",
                    "import signal,time; signal.signal(signal.SIGINT, lambda s,f: exit(0)); time.sleep(60)",
                ],
            },
            "fake_launch": {
                "type": "launch",
                "file": "launch/fake.launch",
                "args": {"mode": "default"},
                "allowed_args": {"mode": ["default", "test"]},
                "pre_start": {
                    "command": [sys.executable, "-c", "print('{mode}')"],
                    "timeout": 2,
                },
            },
            "failed_pre_start": {
                "type": "process",
                "command": [sys.executable, "-c", "import time; time.sleep(60)"],
                "pre_start": {
                    "command": [sys.executable, "-c", "raise SystemExit(7)"],
                    "timeout": 2,
                },
            },
        }
    }
    path.write_text(yaml.safe_dump(config), encoding="utf-8")


class LaunchManagerTest(unittest.TestCase):
    def make_manager(self, tmp_path):
        config_path = tmp_path / "modules.yaml"
        write_config(config_path)
        with socket.socket() as sock:
            sock.bind(("localhost", 0))
            ros_master_uri = "http://localhost:%d" % sock.getsockname()[1]
        bin_dir = tmp_path / "bin"
        bin_dir.mkdir()
        roslaunch = bin_dir / "roslaunch"
        roslaunch.write_text(
            "#!/bin/sh\n"
            "trap 'exit 0' INT TERM\n"
            "sleep 60 &\n"
            "wait $!\n",
            encoding="utf-8",
        )
        roslaunch.chmod(0o755)
        return LaunchManager(
            config_path=str(config_path),
            workspace_root=str(tmp_path),
            ros_setup="",
            workspace_setup="missing_setup.bash",
            log_dir=str(tmp_path / "logs"),
            ros_master_uri=ros_master_uri,
            env_extra={"PATH": "%s:%s" % (bin_dir, os.environ.get("PATH", ""))},
        )

    def test_start_status_stop_process(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))

            started = manager.start("sleepy")
            self.assertEqual(started["status"], "running")
            self.assertTrue(started["pid"])

            stopped = manager.stop("sleepy", timeout=2.0)
            self.assertEqual(stopped["status"], "exited")
            self.assertIn(stopped["returncode"], (0, -signal.SIGINT))

    def test_autostart_processes_marked_modules(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))

            results = manager.autostart()
            self.assertEqual(set(results), {"auto_sleepy"})
            self.assertEqual(results["auto_sleepy"]["status"], "running")
            self.assertEqual(manager.status("sleepy")["status"], "stopped")

            stopped = manager.stop("auto_sleepy", timeout=2.0)
            self.assertEqual(stopped["status"], "exited")

    def test_autostart_orders_roscore_before_launch_modules(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            manager.modules["roscore"]["autostart"] = True
            manager.modules["fake_launch"]["autostart"] = True
            manager.modules = {
                "fake_launch": manager.modules["fake_launch"],
                "auto_sleepy": manager.modules["auto_sleepy"],
                "roscore": manager.modules["roscore"],
                **{
                    name: config
                    for name, config in manager.modules.items()
                    if name not in {"fake_launch", "auto_sleepy", "roscore"}
                },
            }

            results = manager.autostart()

            self.assertEqual(next(iter(results)), "roscore")
            manager.stop_all(timeout=2.0)

    def test_stop_all_stops_roscore_last(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            manager.start("roscore")
            manager.start("sleepy")
            manager.start("auto_sleepy")
            stop_order = []
            original_stop = manager.stop

            def record_stop(name, timeout=8.0):
                stop_order.append(name)
                return original_stop(name, timeout=timeout)

            with patch.object(manager, "stop", side_effect=record_stop):
                results = manager.stop_all(timeout=2.0)

            self.assertEqual(stop_order[-1], "roscore")
            self.assertEqual(set(results), {"roscore", "sleepy", "auto_sleepy"})
            self.assertTrue(all(result["status"] == "exited" for result in results.values()))

    def test_roscore_start_waits_for_stable_run_id(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            started_at = time.monotonic()

            started = manager.start("roscore")

            self.assertEqual(started["status"], "running")
            self.assertGreaterEqual(time.monotonic() - started_at, 0.2)
            manager.stop("roscore", timeout=2.0)

    def test_refuses_to_start_roscore_over_external_master(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            with patch.object(manager, "_ros_master_run_id", return_value="external-run-id"):
                with self.assertRaisesRegex(ModuleRuntimeError, "outside this agent"):
                    manager.start("roscore")

            self.assertEqual(manager.status("roscore")["status"], "stopped")

    def test_reject_unknown_module(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            with self.assertRaises(ModuleRuntimeError):
                manager.start("not_allowed")

    def test_reject_unlisted_launch_arg(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            with self.assertRaises(ModuleRuntimeError):
                manager.start("fake_launch", args={"unsafe": "1"})

    def test_reject_disallowed_launch_arg_value(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            with self.assertRaises(ModuleRuntimeError):
                manager.start("fake_launch", args={"mode": "prod"})

    def test_build_whitelisted_launch_command(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            command = manager._build_command(
                "fake_launch", manager.modules["fake_launch"], {"mode": "test"}
            )
            self.assertEqual(command, ["roslaunch", "--wait", "launch/fake.launch", "mode:=test"])

            pre_start = manager._build_pre_start_command(
                "fake_launch", manager.modules["fake_launch"], {"mode": "test"}
            )
            self.assertEqual(pre_start, [sys.executable, "-c", "print('test')"])

    def test_launch_start_keeps_roscore_independent(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))

            started = manager.start("fake_launch", args={"mode": "test"})
            self.assertEqual(started["status"], "running")
            self.assertEqual(manager.status("roscore")["status"], "running")

            stopped = manager.stop("fake_launch", timeout=2.0)
            self.assertEqual(stopped["status"], "exited")
            self.assertEqual(manager.status("roscore")["status"], "running")

            manager.stop("roscore", timeout=2.0)

    def test_refuses_to_stop_roscore_while_launch_is_running(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            manager.start("fake_launch")

            with self.assertRaisesRegex(ModuleRuntimeError, "fake_launch"):
                manager.stop("roscore", timeout=2.0)

            manager.stop("fake_launch", timeout=2.0)
            manager.stop("roscore", timeout=2.0)

    def test_concurrent_start_is_idempotent(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            manager.start("roscore")
            original_pre_start = manager._run_pre_start
            barrier = threading.Barrier(3)
            results = []
            errors = []

            def slow_pre_start(*args, **kwargs):
                time.sleep(0.15)
                return original_pre_start(*args, **kwargs)

            def start_launch():
                barrier.wait()
                try:
                    results.append(manager.start("fake_launch"))
                except Exception as exc:
                    errors.append(exc)

            threads = [threading.Thread(target=start_launch) for _ in range(2)]
            with patch.object(manager, "_run_pre_start", side_effect=slow_pre_start):
                for thread in threads:
                    thread.start()
                barrier.wait()
                for thread in threads:
                    thread.join(timeout=5.0)

            self.assertFalse(errors)
            self.assertEqual(len(results), 2)
            self.assertEqual(results[0]["pid"], results[1]["pid"])
            manager.stop("fake_launch", timeout=2.0)
            manager.stop("roscore", timeout=2.0)

    def test_failed_pre_start_prevents_module_start(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            with self.assertRaisesRegex(ModuleRuntimeError, "pre_start failed"):
                manager.start("failed_pre_start")
            self.assertEqual(manager.status("failed_pre_start")["status"], "stopped")

    def test_limits_module_log_size_and_retains_tail(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            manager = self.make_manager(Path(tmp))
            manager.max_module_log_bytes = 32
            manager.retained_log_tail_bytes = 8
            log_path = Path(tmp) / "oversized.log"
            log_path.write_bytes(b"0123456789" * 10)

            manager._limit_log_size(str(log_path))

            content = log_path.read_bytes()
            self.assertIn(b"log truncated", content)
            self.assertTrue(content.endswith(b"23456789"))


if __name__ == "__main__":
    unittest.main()
