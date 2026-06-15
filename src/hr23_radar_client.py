"""HR2.3 Radar Recorder TCP JSON Lines client."""

from __future__ import annotations

import json
import socket
from pathlib import Path


class Hr23RadarError(RuntimeError):
    """Raised when an HR2.3 recorder request fails."""


class Hr23RadarClient:
    """Send one JSON Lines command per short-lived TCP connection."""

    def __init__(self, host: str = "127.0.0.1", port: int = 7070, timeout: float = 2.0) -> None:
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)

    def request(self, payload: dict) -> dict:
        cmd = str(payload.get("cmd", ""))
        encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8") + b"\n"
        try:
            with socket.create_connection((self.host, self.port), timeout=self.timeout) as connection:
                connection.settimeout(self.timeout)
                connection.sendall(encoded)
                response_line = self._read_line(connection)
        except (OSError, TimeoutError) as exc:
            raise Hr23RadarError(
                self._error_message(cmd=cmd, error=type(exc).__name__, message=str(exc))
            ) from exc
        if not response_line:
            raise Hr23RadarError(
                self._error_message(
                    cmd=cmd,
                    error="empty response",
                    message="connection closed before JSON line",
                )
            )

        try:
            response = json.loads(response_line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise Hr23RadarError(
                self._error_message(cmd=cmd, error="invalid JSON", message=str(exc))
            ) from exc
        if not isinstance(response, dict):
            raise Hr23RadarError(
                self._error_message(cmd=cmd, error="invalid JSON", message="response is not an object")
            )
        if response.get("ok") is not True:
            raise Hr23RadarError(
                self._error_message(
                    cmd=cmd,
                    state=response.get("state", ""),
                    error=response.get("error", ""),
                    message=response.get("message", ""),
                )
            )
        return response

    def status(self) -> dict:
        return self.request({"cmd": "status"})

    def prepare(
        self,
        session_id: str,
        output_dir: Path,
        prepare_cmd_send_epoch_s: float,
        prepare_cmd_send_perf_s: float,
        metadata: dict | None = None,
    ) -> dict:
        request_metadata = {
            "experimentNote": "",
            "operator": "",
            "source": "debug_monitor",
        }
        if metadata:
            request_metadata.update(metadata)
        request_metadata["source"] = "debug_monitor"
        return self.request({
            "cmd": "prepare",
            "sessionId": session_id,
            "outputDir": str(Path(output_dir)),
            "timeBase": {
                "master": "debug_monitor",
                "prepareCmdSendEpochS": float(prepare_cmd_send_epoch_s),
                "prepareCmdSendPerfS": float(prepare_cmd_send_perf_s),
            },
            "metadata": request_metadata,
        })

    def start(self) -> dict:
        return self.request({"cmd": "start"})

    def stop(self) -> dict:
        return self.request({"cmd": "stop"})

    @staticmethod
    def _read_line(connection: socket.socket) -> bytes:
        response = bytearray()
        while True:
            chunk = connection.recv(4096)
            if not chunk:
                break
            newline_index = chunk.find(b"\n")
            if newline_index >= 0:
                response.extend(chunk[:newline_index])
                break
            response.extend(chunk)
        return bytes(response)

    @staticmethod
    def _error_message(
        cmd: object = "",
        state: object = "",
        error: object = "",
        message: object = "",
    ) -> str:
        return f"cmd={cmd} state={state} error={error} message={message}"
