import os
import struct
import sys
import unittest


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from imu_protocol import (  # noqa: E402
    ImuSample,
    YesenseParser,
    assign_batch_host_times,
    sample_to_csv_row,
    yesense_checksum,
)


HEADER = b"\x59\x53"


def block(block_id: int, payload: bytes) -> bytes:
    return bytes([block_id, len(payload)]) + payload


def vec3_payload(x: int, y: int, z: int) -> bytes:
    return struct.pack("<3i", x, y, z)


def build_packet(sequence: int, payload: bytes) -> bytes:
    body = struct.pack("<HB", sequence, len(payload)) + payload
    ck1, ck2 = yesense_checksum(body)
    return HEADER + body + bytes([ck1, ck2])


class ImuProtocolTests(unittest.TestCase):
    def test_yesense_checksum_accumulates_ck1_and_ck2(self) -> None:
        self.assertEqual(yesense_checksum(bytes([1, 2, 3])), (6, 10))

    def test_parser_decodes_supported_blocks_with_official_scaling(self) -> None:
        payload = b"".join(
            [
                block(0x01, struct.pack("<h", 2534)),
                block(0x10, vec3_payload(1_000_000, -2_000_000, 3_000_000)),
                block(0x20, vec3_payload(4_000_000, -5_000_000, 6_000_000)),
                block(0x31, vec3_payload(7000, -8000, 9000)),
                block(0x40, vec3_payload(10_000_000, 20_000_000, -30_000_000)),
                block(0x41, struct.pack("<4i", 1_000_000, 0, -1_000_000, 500_000)),
                block(0x51, struct.pack("<I", 123456)),
                block(0x52, struct.pack("<I", 654321)),
            ]
        )
        parser = YesenseParser()

        samples = parser.feed(build_packet(42, payload))

        self.assertEqual(len(samples), 1)
        sample = samples[0]
        self.assertEqual(sample.sequence, 42)
        self.assertAlmostEqual(sample.temperature_c, 25.34)
        self.assertEqual(sample.accel, (1.0, -2.0, 3.0))
        self.assertEqual(sample.gyro, (4.0, -5.0, 6.0))
        self.assertEqual(sample.mag, (7.0, -8.0, 9.0))
        self.assertEqual(sample.euler, (10.0, 20.0, -30.0))
        self.assertEqual(sample.quat, (1.0, 0.0, -1.0, 0.5))
        self.assertEqual(sample.device_time, 123456)
        self.assertEqual(sample.sync_time, 654321)

    def test_parser_keeps_partial_frames_and_ignores_noise(self) -> None:
        payload = block(0x10, vec3_payload(1, 2, 3))
        packet = build_packet(7, payload)
        parser = YesenseParser()

        self.assertEqual(parser.feed(b"\x00\x99" + packet[:5]), [])
        samples = parser.feed(packet[5:])

        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].sequence, 7)
        self.assertEqual(samples[0].accel, (0.000001, 0.000002, 0.000003))

    def test_parser_returns_multiple_samples_from_one_chunk(self) -> None:
        first = build_packet(1, block(0x51, struct.pack("<I", 100)))
        second = build_packet(2, block(0x51, struct.pack("<I", 200)))
        parser = YesenseParser()

        samples = parser.feed(first + second)

        self.assertEqual([sample.sequence for sample in samples], [1, 2])
        self.assertEqual([sample.device_time for sample in samples], [100, 200])

    def test_parser_drops_bad_checksum_packet(self) -> None:
        packet = bytearray(build_packet(3, block(0x51, struct.pack("<I", 100))))
        packet[-1] ^= 0xFF
        parser = YesenseParser()

        self.assertEqual(parser.feed(bytes(packet)), [])

    def test_assign_batch_host_times_spreads_samples_over_read_window(self) -> None:
        samples = [ImuSample(host_time=0.0, sequence=i) for i in range(3)]

        assign_batch_host_times(samples, 10.0, 10.2)

        self.assertEqual([sample.host_time for sample in samples], [10.0, 10.1, 10.2])

    def test_sample_to_csv_row_uses_stable_field_names(self) -> None:
        sample = ImuSample(
            host_time=1.25,
            sequence=9,
            accel=(1.0, 2.0, 3.0),
            gyro=(4.0, 5.0, 6.0),
            euler=(7.0, 8.0, 9.0),
            temperature_c=25.0,
            device_time=11,
            sync_time=12,
        )

        row = sample_to_csv_row(sample)

        self.assertEqual(row["host_time"], "1.250000")
        self.assertEqual(row["sequence"], "9")
        self.assertEqual(row["temperature_c"], "25.000000")
        self.assertEqual(row["pitch"], "7.000000")
        self.assertEqual(row["roll"], "8.000000")
        self.assertEqual(row["yaw"], "9.000000")
        self.assertEqual(row["mag_x"], "")


if __name__ == "__main__":
    unittest.main()

