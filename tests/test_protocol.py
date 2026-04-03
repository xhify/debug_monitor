import os
import struct
import sys
import unittest


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from protocol import (  # noqa: E402
    FRAME_ID_PARAM,
    HEADER1,
    HEADER2,
    ParamFrame,
    compute_xor_checksum,
    parse_param_frame,
)


class ProtocolParsingTests(unittest.TestCase):
    def test_parse_param_frame_accepts_40_byte_param_frame(self) -> None:
        payload = struct.pack(
            "<9f",
            80.0, 0.6, 20.0,
            81.0, 0.7, 21.0,
            100.0, 0.8, 0.02,
        )
        frame = bytes([HEADER1, HEADER2, FRAME_ID_PARAM]) + payload
        raw = frame + bytes([compute_xor_checksum(frame)])

        parsed = parse_param_frame(raw)

        self.assertIsNotNone(parsed)
        expected = ParamFrame(
            A_kp=80.0,
            A_ki=0.6,
            A_kd=20.0,
            B_kp=81.0,
            B_ki=0.7,
            B_kd=21.0,
            rc_speed=100.0,
            limt_max_speed=0.8,
            smooth_MotorStep=0.02,
        )
        for field in expected.__dataclass_fields__:
            self.assertAlmostEqual(
                getattr(parsed, field),
                getattr(expected, field),
                places=5,
            )


if __name__ == "__main__":
    unittest.main()
