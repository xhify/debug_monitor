import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from radar_scpi import RadarScpiClient, RadarScpiError


class FakeSocket:
    def __init__(self, response: bytes = b"PHASELOCK,radar\n") -> None:
        self.response = response
        self.sent: list[bytes] = []
        self.timeout: float | None = None
        self.closed = False

    def settimeout(self, timeout: float) -> None:
        self.timeout = timeout

    def sendall(self, data: bytes) -> None:
        self.sent.append(data)

    def recv(self, _size: int) -> bytes:
        return self.response

    def close(self) -> None:
        self.closed = True


class FakeSocketFactory:
    def __init__(self, response: bytes = b"PHASELOCK,radar\n") -> None:
        self.response = response
        self.instances: list[FakeSocket] = []
        self.connections: list[tuple[tuple[str, int], float]] = []

    def create_connection(self, address: tuple[str, int], timeout: float) -> FakeSocket:
        self.connections.append((address, timeout))
        sock = FakeSocket(self.response)
        self.instances.append(sock)
        return sock


class RadarScpiClientTests(unittest.TestCase):
    def test_identify_sends_idn_and_requires_phaselock_response(self) -> None:
        factory = FakeSocketFactory(response=b"PHASELOCK,DebugRadar\n")
        client = RadarScpiClient(socket_factory=factory)

        response = client.identify()

        self.assertEqual(response, "PHASELOCK,DebugRadar")
        self.assertEqual(factory.connections, [(("127.0.0.1", 5026), 2.0)])
        self.assertEqual(factory.instances[0].sent, [b"*IDN?\n"])
        self.assertTrue(factory.instances[0].closed)

    def test_identify_rejects_non_phaselock_response(self) -> None:
        factory = FakeSocketFactory(response=b"OTHER,tool\n")
        client = RadarScpiClient(socket_factory=factory)

        with self.assertRaises(RadarScpiError):
            client.identify()

    def test_start_recording_uses_timestamp_bin_filename(self) -> None:
        factory = FakeSocketFactory()
        client = RadarScpiClient(socket_factory=factory)

        filename = client.start_recording("20260424_153000")

        self.assertEqual(filename, "2026_04_24_15_30_00.bin")
        self.assertEqual(
            factory.instances[0].sent,
            [b"*IDN?\n", b"MEMMory:RECord:STARt 2026_04_24_15_30_00.bin\n"],
        )

    def test_stop_recording_sends_short_scpi_command(self) -> None:
        factory = FakeSocketFactory()
        client = RadarScpiClient(socket_factory=factory)

        client.stop_recording()

        self.assertEqual(factory.instances[0].sent, [b"MEMM:REC:STOP\n"])


if __name__ == "__main__":
    unittest.main()
