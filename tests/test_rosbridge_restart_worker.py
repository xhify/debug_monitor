import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rosbridge_restart_worker import RosbridgeRestartConfig, RosbridgeRestartWorker


class FakeResult:
    def __init__(self, returncode=0, stdout="", stderr="") -> None:
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


class RosbridgeRestartWorkerTests(unittest.TestCase):
    def test_restart_uses_batch_ssh_and_only_restarts_rosbridge(self) -> None:
        calls: list[list[str]] = []
        worker = RosbridgeRestartWorker(
            RosbridgeRestartConfig(host="192.168.0.14", port=9090),
            process_runner=lambda args, **_kwargs: calls.append(args) or FakeResult(),
            port_probe=lambda _host, _port, _timeout: True,
        )

        result = worker.restart()

        self.assertTrue(result["ok"])
        self.assertEqual(
            calls[0][:5],
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5"],
        )
        self.assertEqual(calls[0][-2], "wheeltec@192.168.0.14")
        self.assertIn("rosnode kill /rosbridge_websocket", calls[0][-1])
        self.assertIn(
            "roslaunch rosbridge_server rosbridge_websocket.launch "
            "port:=9090 address:=0.0.0.0",
            calls[0][-1],
        )
        self.assertNotIn("turn_on_wheeltec_robot.launch", calls[0][-1])

    def test_restart_reports_ssh_failure_without_probing_port(self) -> None:
        probes: list[tuple[str, int, float]] = []
        worker = RosbridgeRestartWorker(
            RosbridgeRestartConfig(host="robot.local", port=9090),
            process_runner=lambda _args, **_kwargs: FakeResult(
                returncode=255,
                stderr="Permission denied (publickey).",
            ),
            port_probe=lambda host, port, timeout: probes.append((host, port, timeout)) or True,
        )

        with self.assertRaisesRegex(RuntimeError, "Permission denied"):
            worker.restart()

        self.assertEqual(probes, [])

    def test_restart_times_out_when_port_never_opens(self) -> None:
        clock_values = iter([0.0, 0.0, 0.5, 1.1])
        worker = RosbridgeRestartWorker(
            RosbridgeRestartConfig(
                host="robot.local",
                port=19090,
                timeout_s=1.0,
                probe_interval_s=0.0,
            ),
            process_runner=lambda _args, **_kwargs: FakeResult(),
            port_probe=lambda _host, _port, _timeout: False,
            monotonic_clock=lambda: next(clock_values),
            sleep=lambda _seconds: None,
        )

        with self.assertRaisesRegex(TimeoutError, "19090"):
            worker.restart()

    def test_missing_rosbridge_node_is_allowed_when_start_succeeds(self) -> None:
        worker = RosbridgeRestartWorker(
            RosbridgeRestartConfig(host="robot.local", port=9090),
            process_runner=lambda _args, **_kwargs: FakeResult(
                stdout="ERROR: Unknown node(s): /rosbridge_websocket"
            ),
            port_probe=lambda _host, _port, _timeout: True,
        )

        result = worker.restart()

        self.assertTrue(result["ok"])


if __name__ == "__main__":
    unittest.main()
