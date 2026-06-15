import json
import os
import socket
import sys
import threading
import unittest
from pathlib import Path


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from hr23_radar_client import Hr23RadarClient, Hr23RadarError


class FakeJsonLinesServer:
    def __init__(self, responses: list[dict[str, object]]) -> None:
        self._responses = list(responses)
        self.requests: list[dict[str, object]] = []
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen()
        self.host, self.port = self._listener.getsockname()
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self._listener.close()
        self._thread.join(timeout=2.0)

    def _serve(self) -> None:
        for response in self._responses:
            try:
                connection, _address = self._listener.accept()
            except OSError:
                return
            with connection:
                payload = b""
                while not payload.endswith(b"\n"):
                    chunk = connection.recv(4096)
                    if not chunk:
                        break
                    payload += chunk
                self.requests.append(json.loads(payload.decode("utf-8")))
                connection.sendall(
                    json.dumps(response, ensure_ascii=False).encode("utf-8") + b"\n"
                )


class Hr23RadarClientTests(unittest.TestCase):
    def test_status_prepare_start_and_stop_use_one_json_line_connection_each(self) -> None:
        responses = [
            {"ok": True, "state": "idle", "packetCount": 2, "totalBytes": 64},
            {"ok": True, "state": "prepared"},
            {"ok": True, "state": "recording"},
            {"ok": True, "state": "stopped", "packetCount": 5, "totalBytes": 160},
        ]
        with FakeJsonLinesServer(responses) as server:
            client = Hr23RadarClient(server.host, server.port, timeout=1.0)
            self.assertEqual(client.status()["state"], "idle")
            self.assertEqual(
                client.prepare(
                    session_id="session_20260612_153000",
                    output_dir=Path("recordings/session_20260612_153000/raw/hr23_radar"),
                    prepare_cmd_send_epoch_s=1781188199.9,
                    prepare_cmd_send_perf_s=12345.5,
                    metadata={"experimentNote": "直线测试"},
                    recording_start_epoch_s=1781188198.4,
                    recording_start_perf_s=12344.0,
                )["state"],
                "prepared",
            )
            self.assertEqual(client.start()["state"], "recording")
            self.assertEqual(client.stop()["state"], "stopped")

        self.assertEqual([request["cmd"] for request in server.requests], ["status", "prepare", "start", "stop"])
        prepare_request = server.requests[1]
        self.assertEqual(prepare_request["sessionId"], "session_20260612_153000")
        self.assertEqual(prepare_request["metadata"]["source"], "debug_monitor")
        self.assertEqual(prepare_request["metadata"]["experimentNote"], "直线测试")
        self.assertEqual(prepare_request["timeBase"]["master"], "debug_monitor")
        self.assertEqual(prepare_request["timeBase"]["recordingStartEpochS"], 1781188198.4)
        self.assertEqual(prepare_request["timeBase"]["recordingStartPerfS"], 12344.0)
        self.assertEqual(prepare_request["timeBase"]["prepareCmdSendEpochS"], 1781188199.9)
        self.assertEqual(prepare_request["timeBase"]["prepareCmdSendPerfS"], 12345.5)

    def test_prepare_falls_back_to_prepare_send_time_base(self) -> None:
        with FakeJsonLinesServer([{"ok": True, "state": "prepared"}]) as server:
            Hr23RadarClient(server.host, server.port, timeout=1.0).prepare(
                session_id="session_fallback",
                output_dir=Path("recordings/session_fallback/raw/hr23_radar"),
                prepare_cmd_send_epoch_s=1781188201.25,
                prepare_cmd_send_perf_s=54321.75,
            )
        time_base = server.requests[0]["timeBase"]
        self.assertEqual(time_base["recordingStartEpochS"], 1781188201.25)
        self.assertEqual(time_base["recordingStartPerfS"], 54321.75)

    def test_ok_false_raises_error_with_protocol_context(self) -> None:
        response = {
            "ok": False,
            "state": "recording",
            "error": "busy",
            "message": "already recording",
        }
        with FakeJsonLinesServer([response]) as server:
            client = Hr23RadarClient(server.host, server.port, timeout=1.0)
            with self.assertRaises(Hr23RadarError) as context:
                client.start()

        message = str(context.exception)
        self.assertIn("cmd=start", message)
        self.assertIn("state=recording", message)
        self.assertIn("error=busy", message)
        self.assertIn("message=already recording", message)

    def test_invalid_json_raises_hr23_radar_error(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        host, port = listener.getsockname()

        def serve_invalid_json() -> None:
            connection, _address = listener.accept()
            with connection:
                connection.recv(4096)
                connection.sendall(b"not-json\n")
            listener.close()

        thread = threading.Thread(target=serve_invalid_json, daemon=True)
        thread.start()
        with self.assertRaises(Hr23RadarError) as context:
            Hr23RadarClient(host, port, timeout=1.0).status()
        thread.join(timeout=2.0)
        self.assertIn("cmd=status", str(context.exception))
        self.assertIn("invalid JSON", str(context.exception))

    def test_empty_response_error_includes_command_context(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        host, port = listener.getsockname()

        def close_without_response() -> None:
            connection, _address = listener.accept()
            with connection:
                connection.recv(4096)
            listener.close()

        thread = threading.Thread(target=close_without_response, daemon=True)
        thread.start()
        with self.assertRaises(Hr23RadarError) as context:
            Hr23RadarClient(host, port, timeout=1.0).stop()
        thread.join(timeout=2.0)
        self.assertIn("cmd=stop", str(context.exception))
        self.assertIn("empty response", str(context.exception))


if __name__ == "__main__":
    unittest.main()
