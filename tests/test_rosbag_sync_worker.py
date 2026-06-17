import os
import sys
import unittest
import shutil
import uuid
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rosbag_sync_worker import RosbagSyncWorker


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


@contextmanager
def temp_dir():
    TEST_TMP_ROOT.mkdir(exist_ok=True)
    path = TEST_TMP_ROOT / f"rosbag_sync_{uuid.uuid4().hex}"
    path.mkdir()
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


class RosbagSyncWorkerTests(unittest.TestCase):
    def test_rsync_success_does_not_call_scp(self) -> None:
        calls = []

        def runner(args, **kwargs):
            calls.append(args)
            return SimpleNamespace(returncode=0, stdout="ok", stderr="")

        with temp_dir() as tmp:
            worker = RosbagSyncWorker(
                host="192.168.0.100",
                remote_dir="/bags/session_1",
                local_dir=tmp / "session_1",
                process_runner=runner,
            )
            results = []
            worker.finished.connect(results.append)

            worker.run()

        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], "rsync")
        self.assertEqual(results[0]["method"], "rsync")

    def test_rsync_failure_falls_back_to_scp(self) -> None:
        calls = []

        def runner(args, **kwargs):
            calls.append(args)
            if args[0] == "rsync":
                return SimpleNamespace(returncode=23, stdout="", stderr="failed")
            return SimpleNamespace(returncode=0, stdout="copied", stderr="")

        with temp_dir() as tmp:
            worker = RosbagSyncWorker(
                host="192.168.0.100",
                remote_dir="/bags/session_1",
                local_dir=tmp / "session_1",
                process_runner=runner,
            )
            progress = []
            results = []
            worker.progress.connect(progress.append)
            worker.finished.connect(results.append)

            worker.run()

        self.assertEqual([call[0] for call in calls], ["rsync", "scp"])
        self.assertIn("降级为 scp", "\n".join(progress))
        self.assertEqual(results[0]["method"], "scp")

    def test_scp_fallback_copies_remote_contents_into_target_dir(self) -> None:
        calls = []

        def runner(args, **kwargs):
            calls.append(args)
            if args[0] == "rsync":
                raise FileNotFoundError("rsync")
            return SimpleNamespace(returncode=0, stdout="copied", stderr="")

        with temp_dir() as tmp:
            local_dir = tmp / "session_1"
            worker = RosbagSyncWorker(
                host="192.168.0.14",
                remote_dir="/bags/session_1",
                local_dir=local_dir,
                process_runner=runner,
            )
            results = []
            worker.finished.connect(results.append)

            worker.run()

        self.assertEqual(
            calls[1],
            [
                "scp",
                "-r",
                "wheeltec@192.168.0.14:/bags/session_1/.",
                str(local_dir),
            ],
        )
        self.assertEqual(results[0]["local_dir"], str(local_dir))

    def test_rsync_missing_falls_back_to_scp_and_scp_failure_reports_error(self) -> None:
        calls = []

        def runner(args, **kwargs):
            calls.append(args)
            if args[0] == "rsync":
                raise FileNotFoundError("rsync")
            return SimpleNamespace(returncode=1, stdout="", stderr="scp failed")

        with temp_dir() as tmp:
            worker = RosbagSyncWorker(
                host="192.168.0.100",
                remote_dir="/bags/session_1",
                local_dir=tmp / "session_1",
                process_runner=runner,
            )
            errors = []
            worker.error.connect(errors.append)

            worker.run()

        self.assertEqual([call[0] for call in calls], ["rsync", "scp"])
        self.assertIn("scp failed", errors[0])


if __name__ == "__main__":
    unittest.main()
