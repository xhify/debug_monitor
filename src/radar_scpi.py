"""Radar software SCPI TCP client."""

from __future__ import annotations

import socket


DEFAULT_RADAR_HOST = "127.0.0.1"
DEFAULT_RADAR_PORT = 5026
DEFAULT_TIMEOUT_SECONDS = 2.0
IDN_COMMAND = "*IDN?"
START_COMMAND = "MEMMory:RECord:STARt"
STOP_COMMAND = "MEMM:REC:STOP"


class RadarScpiError(RuntimeError):
    """Raised when the radar SCPI endpoint is unavailable or unexpected."""


class RadarScpiClient:
    """Small SCPI text client for the radar control software."""

    def __init__(
        self,
        host: str = DEFAULT_RADAR_HOST,
        port: int = DEFAULT_RADAR_PORT,
        timeout: float = DEFAULT_TIMEOUT_SECONDS,
        socket_factory=socket,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self._socket_factory = socket_factory

    def identify(self) -> str:
        response = self._query(IDN_COMMAND)
        self._validate_identity(response)
        return response

    def start_recording(self, timestamp: str) -> str:
        filename = f"{self._format_filename_timestamp(timestamp)}.bin"
        sock = self._connect()
        try:
            sock.sendall(self._encode_command(IDN_COMMAND))
            data = sock.recv(512)
            if not data:
                raise RadarScpiError("雷达无响应")
            self._validate_identity(data.decode("ascii", errors="replace").strip())
            sock.sendall(self._encode_command(f"{START_COMMAND} {filename}"))
        except OSError as exc:
            raise RadarScpiError(f"雷达通信失败: {exc}") from exc
        finally:
            sock.close()
        return filename

    def stop_recording(self) -> None:
        self._send(STOP_COMMAND)

    def _query(self, command: str) -> str:
        sock = self._connect()
        try:
            sock.sendall(self._encode_command(command))
            data = sock.recv(512)
        except OSError as exc:
            raise RadarScpiError(f"雷达通信失败: {exc}") from exc
        finally:
            sock.close()
        if not data:
            raise RadarScpiError("雷达无响应")
        return data.decode("ascii", errors="replace").strip()

    def _send(self, command: str) -> None:
        sock = self._connect()
        try:
            sock.sendall(self._encode_command(command))
        except OSError as exc:
            raise RadarScpiError(f"雷达通信失败: {exc}") from exc
        finally:
            sock.close()

    def _connect(self):
        try:
            sock = self._socket_factory.create_connection((self.host, self.port), self.timeout)
            sock.settimeout(self.timeout)
            return sock
        except OSError as exc:
            raise RadarScpiError(f"雷达连接失败: {exc}") from exc

    @staticmethod
    def _encode_command(command: str) -> bytes:
        return f"{command}\n".encode("ascii")

    @staticmethod
    def _validate_identity(response: str) -> None:
        if not response.startswith("PHASELOCK"):
            raise RadarScpiError(f"雷达识别失败: {response}")

    @staticmethod
    def _format_filename_timestamp(timestamp: str) -> str:
        parts = timestamp.split("_")
        if len(parts) == 2 and len(parts[0]) == 8 and len(parts[1]) == 6:
            return (
                f"{parts[0][0:4]}_{parts[0][4:6]}_{parts[0][6:8]}_"
                f"{parts[1][0:2]}_{parts[1][2:4]}_{parts[1][4:6]}"
            )
        return timestamp
