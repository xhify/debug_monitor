import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from recording_clock import RecordingClock


class RecordingClockTests(unittest.TestCase):
    def test_session_id_exists(self) -> None:
        clock = RecordingClock()

        self.assertTrue(clock.session_id)
        self.assertTrue(clock.started_at_iso)

    def test_elapsed_s_is_monotonic(self) -> None:
        clock = RecordingClock()

        first = clock.elapsed_s()
        time.sleep(0.01)
        second = clock.elapsed_s()

        self.assertGreaterEqual(second, first)

    def test_now_record_fields_include_session_metadata(self) -> None:
        clock = RecordingClock(session_id="session_test")

        fields = clock.now_record_fields()

        self.assertEqual(fields["session_id"], "session_test")
        self.assertIn("recv_time_epoch_s", fields)
        self.assertIn("session_elapsed_s", fields)


if __name__ == "__main__":
    unittest.main()
