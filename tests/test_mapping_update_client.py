import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from mapping_update_client import MappingUpdateClient, MappingUpdateError


class CompletedProcess:
    def __init__(self, returncode: int = 0, stdout: str = "", stderr: str = "") -> None:
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


class MappingUpdateClientTests(unittest.TestCase):
    def test_freeze_and_resume_send_expected_rosparam_values_over_ssh(self) -> None:
        calls = []

        def runner(args, capture_output, text, check, timeout):
            calls.append((args, capture_output, text, check, timeout))
            return CompletedProcess(stdout="ok\n")

        client = MappingUpdateClient(ssh_host="wheeltec14", timeout=3.5, runner=runner)

        frozen = client.set_map_update_enabled(False)
        resumed = client.set_map_update_enabled(True)

        self.assertFalse(frozen["enabled"])
        self.assertTrue(resumed["enabled"])
        self.assertEqual(
            calls[0][0],
            [
                "ssh",
                "wheeltec14",
                "source /opt/ros/noetic/setup.bash && rosparam set /mapping/map_update_enable false",
            ],
        )
        self.assertEqual(
            calls[1][0],
            [
                "ssh",
                "wheeltec14",
                "source /opt/ros/noetic/setup.bash && rosparam set /mapping/map_update_enable true",
            ],
        )
        self.assertEqual(calls[0][4], 3.5)

    def test_nonzero_rosparam_command_raises_clear_error(self) -> None:
        def runner(args, capture_output, text, check, timeout):
            return CompletedProcess(returncode=1, stderr="unknown parameter")

        client = MappingUpdateClient(runner=runner)

        with self.assertRaises(MappingUpdateError) as ctx:
            client.set_map_update_enabled(False)

        self.assertIn("/mapping/map_update_enable", str(ctx.exception))
        self.assertIn("unknown parameter", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
