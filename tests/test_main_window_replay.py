import os
import csv
import gc
import json
import shutil
import sys
import unittest
import uuid
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from imu_protocol import ImuSample
from main_window import MainWindow
from protocol import DataFrame
from ros_bridge_worker import RosImuReading, RosSnapshot


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


@contextmanager
def temp_dir():
    TEST_TMP_ROOT.mkdir(exist_ok=True)
    path = TEST_TMP_ROOT / f"main_window_{uuid.uuid4().hex}"
    path.mkdir()
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


def make_encoder_frame() -> DataFrame:
    return DataFrame(
        t_raw_A=1.0,
        t_raw_B=2.0,
        m_raw_A=3.0,
        m_raw_B=4.0,
        final_A=5.0,
        final_B=6.0,
        target_A=7.0,
        target_B=8.0,
        output_A=9,
        output_B=10,
        afc_output_A=11.0,
        afc_output_B=12.0,
    )


def make_imu_sample(sequence: int) -> ImuSample:
    return ImuSample(
        host_time=sequence / 1000.0,
        sequence=sequence,
        accel=(1.0, 2.0, 3.0),
        gyro=(4.0, 5.0, 6.0),
        euler=(7.0, 8.0, 9.0),
        sync_time=sequence * 1000,
    )


class FakeRadarClient:
    def __init__(self, fail_start: bool = False) -> None:
        self.fail_start = fail_start
        self.identify_calls = 0
        self.started: list[str] = []
        self.stopped = 0

    def identify(self) -> str:
        self.identify_calls += 1
        return "PHASELOCK,FakeRadar"

    def start_recording(self, timestamp: str) -> str:
        if self.fail_start:
            raise RuntimeError("radar unavailable")
        self.started.append(timestamp)
        return "2026_04_24_15_30_00.bin"

    def stop_recording(self) -> None:
        self.stopped += 1


class FakeHr23RadarClient:
    def __init__(self, fail_stop: bool = False) -> None:
        self.fail_stop = fail_stop
        self.calls: list[tuple[str, object]] = []

    def status(self) -> dict[str, object]:
        self.calls.append(("status", None))
        return {
            "ok": True,
            "state": "idle",
            "packetCount": 3,
            "totalBytes": 96,
            "lastPacketUtc": "2026-06-12T15:30:00Z",
        }

    def prepare(self, **kwargs) -> dict[str, object]:
        self.calls.append(("prepare", kwargs))
        return {"ok": True, "state": "prepared"}

    def start(self) -> dict[str, object]:
        self.calls.append(("start", None))
        return {"ok": True, "state": "recording"}

    def stop(self) -> dict[str, object]:
        self.calls.append(("stop", None))
        if self.fail_stop:
            raise RuntimeError("recorder stop timeout")
        return {
            "ok": True,
            "state": "stopped",
            "packetCount": 8,
            "totalBytes": 256,
            "firstPacketUtc": "2026-06-12T15:30:01Z",
            "lastPacketUtc": "2026-06-12T15:30:02Z",
            "rawFileClosedUtc": "2026-06-12T15:30:03Z",
            "files": {
                "raw": "raw.dat",
                "packets": "packets.csv",
                "events": "events.csv",
                "metadata": "metadata.json",
            },
            "time": {"durationS": 1.0},
        }


def make_radar_big_frame(offset: int = 0) -> bytes:
    import numpy as np

    header = bytearray(16)
    header[8] = offset & 0xFF
    header[9] = (offset >> 8) & 0xFF
    iq_values = np.arange(65120, dtype=">i2")
    body = iq_values.tobytes()
    footer = bytes(4)
    return bytes(header) + body + footer


def allow_summary_source_checks(window: MainWindow) -> None:
    window._check_summary_sources = lambda: {
        "mock": {
            "status": "ok",
            "estimated_hz": 100.0,
            "has_header_stamp": True,
            "messages_received": 3,
            "notes": "test source ready",
        }
    }


class MainWindowReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_main_window_uses_shorter_default_height(self) -> None:
        window = MainWindow()
        screen = window.screen() or self.app.primaryScreen()
        available = screen.availableGeometry()

        self.assertLessEqual(window.size().width(), available.width())
        self.assertLessEqual(window.size().height(), available.height())

    def test_main_window_starts_centered_on_available_screen(self) -> None:
        window = MainWindow()
        screen = window.screen() or self.app.primaryScreen()
        available = screen.availableGeometry()
        geometry = window.frameGeometry()

        self.assertEqual(geometry.center().x(), available.center().x())
        self.assertEqual(geometry.center().y(), available.center().y())

    def test_shown_main_window_frame_fits_available_screen(self) -> None:
        window = MainWindow()
        window.show()
        self.app.processEvents()
        screen = window.screen() or self.app.primaryScreen()
        available = screen.availableGeometry()
        frame = window.frameGeometry()

        self.assertLessEqual(frame.width(), available.width())
        self.assertLessEqual(frame.height(), available.height())

    def test_monitor_panels_move_into_tabs_with_param_default(self) -> None:
        window = MainWindow()

        self.assertEqual(window._monitor_tabs.count(), 3)
        self.assertEqual(window._monitor_tabs.tabText(0), "固件参数")
        self.assertIs(window._monitor_tabs.currentWidget(), window._param_panel)
        self.assertIs(window._command_panel.parentWidget(), window._right_sidebar)

    def test_main_window_exposes_encoder_and_imu_module_switches(self) -> None:
        window = MainWindow()

        self.assertEqual(window._module_stack.currentWidget(), window._encoder_page)
        self.assertEqual(window._summary_module_btn.text(), "汇总")
        self.assertEqual(window._encoder_module_btn.text(), "编码器")
        self.assertEqual(window._imu_module_btn.text(), "IMU")
        self.assertEqual(window._ros_module_btn.text(), "ROS")
        self.assertEqual(window._ros_imu_module_btn.text(), "ROS IMU")
        self.assertEqual(window._rosbag_module_btn.text(), "ROSBag")
        self.assertEqual(window._localization_module_btn.text(), "定位精度")

    def test_main_window_switches_to_localization_module(self) -> None:
        window = MainWindow()

        window._localization_module_btn.click()

        self.assertEqual(window._module_stack.currentWidget(), window._localization_panel)
        self.assertTrue(window._localization_module_btn.isChecked())
        self.assertFalse(window._encoder_module_btn.isChecked())

    def test_main_window_switches_to_imu_module(self) -> None:
        window = MainWindow()

        window._imu_module_btn.click()

        self.assertEqual(window._module_stack.currentWidget(), window._imu_panel)
        self.assertTrue(window._imu_module_btn.isChecked())
        self.assertFalse(window._encoder_module_btn.isChecked())

    def test_imu_module_has_two_independent_device_connections_and_shared_record_button(self) -> None:
        window = MainWindow()
        imu_panel = window._imu_panel

        self.assertEqual(set(imu_panel._devices.keys()), {"A", "B"})
        self.assertEqual(imu_panel._devices["A"].group.title(), "IMU A")
        self.assertEqual(imu_panel._devices["B"].group.title(), "IMU B")
        self.assertEqual(imu_panel._record_btn.text(), "开始记录")

    def test_summary_module_switches_to_summary_page(self) -> None:
        window = MainWindow()

        window._summary_module_btn.click()

        self.assertEqual(window._module_stack.currentWidget(), window._summary_page)
        self.assertTrue(window._summary_module_btn.isChecked())

    def test_ros_module_switches_to_ros_page(self) -> None:
        window = MainWindow()

        window._ros_module_btn.click()

        self.assertEqual(window._module_stack.currentWidget(), window._ros_panel)
        self.assertTrue(window._ros_module_btn.isChecked())

    def test_ros_imu_module_switches_to_ros_imu_page(self) -> None:
        window = MainWindow()

        window._ros_imu_module_btn.click()

        self.assertEqual(window._module_stack.currentWidget(), window._ros_imu_panel)
        self.assertTrue(window._ros_imu_module_btn.isChecked())

    def test_rosbag_module_switches_to_rosbag_page(self) -> None:
        window = MainWindow()

        window._rosbag_module_btn.click()

        self.assertEqual(window._module_stack.currentWidget(), window._rosbag_panel)
        self.assertTrue(window._rosbag_module_btn.isChecked())

    def test_ros_snapshot_only_updates_visible_ros_page(self) -> None:
        window = MainWindow()
        ros_updates = 0
        ros_imu_updates = 0

        def count_ros(_snapshot) -> None:
            nonlocal ros_updates
            ros_updates += 1

        def count_ros_imu(_snapshot) -> None:
            nonlocal ros_imu_updates
            ros_imu_updates += 1

        window._ros_panel.update_snapshot = count_ros
        window._ros_imu_panel.update_snapshot = count_ros_imu
        snapshot = RosSnapshot(
            frame_count=1,
            imu=RosImuReading(frame_count=1),
            active_imu=RosImuReading(frame_count=1),
        )

        window._on_ros_snapshot(snapshot)
        self.assertEqual((ros_updates, ros_imu_updates), (0, 0))

        window._ros_imu_module_btn.click()
        window._on_ros_snapshot(snapshot)
        self.assertEqual((ros_updates, ros_imu_updates), (0, 1))

        window._ros_module_btn.click()
        window._on_ros_snapshot(snapshot)
        self.assertEqual((ros_updates, ros_imu_updates), (1, 1))

    def test_ros_panel_pid_control_signal_reaches_worker(self) -> None:
        window = MainWindow()
        commands: list[tuple[float, bool, bool]] = []
        window._ros_worker.publish_line_follow_control = (
            lambda linear_x, forward, backward: commands.append((linear_x, forward, backward))
        )
        window._ros_panel.set_connected(True)
        window._ros_panel._pid_linear_x_spin.setValue(0.2)

        window._ros_panel._pid_forward_btn.click()

        self.assertEqual(commands, [(0.2, True, False)])

    def test_ros_panel_connect_passes_selected_data_topics_to_worker(self) -> None:
        window = MainWindow()
        requests: list[tuple[str, int, list[str]]] = []
        window._ros_worker.open_bridge = lambda host, port, topics=None: requests.append((host, port, list(topics or [])))
        window._ros_panel._host_edit.setText("192.168.0.14")
        window._ros_panel._port_spin.setValue(9090)
        window._ros_panel._topic_checkboxes["/odom"].setChecked(True)
        window._ros_panel._topic_checkboxes["/imu"].setChecked(True)

        window._ros_panel._connect_btn.click()

        self.assertEqual(requests, [("192.168.0.14", 9090, ["/odom", "/imu"])])

    def test_ros_panel_pid_launch_buttons_publish_launch_manager_commands(self) -> None:
        window = MainWindow()
        commands: list[str] = []
        window._ros_worker.publish_launch_manager_command = commands.append
        window._ros_panel.set_connected(True)

        window._ros_panel._pid_launch_start_btn.click()
        window._ros_panel._pid_launch_stop_btn.click()

        self.assertEqual(
            commands,
            [
                "start pid_control simple_follower pid_control.launch",
                "stop pid_control",
            ],
        )

    def test_ros_panel_radar_calibration_launch_buttons_publish_launch_manager_commands(self) -> None:
        window = MainWindow()
        commands: list[str] = []
        window._ros_worker.publish_launch_manager_command = commands.append
        window._ros_panel.set_connected(True)

        window._ros_panel._radar_calibration_launch_start_btn.click()
        window._ros_panel._radar_calibration_launch_stop_btn.click()

        self.assertEqual(
            commands,
            [
                "restart pid_control simple_follower pid_control_lidar_assisted.launch "
                "imu_topic:=/active_imu lidar_odom_topic:=/Odometry",
                "stop pid_control",
            ],
        )

    def test_localization_panel_fastlio_launch_buttons_publish_launch_manager_commands(self) -> None:
        window = MainWindow()
        commands: list[str] = []
        window._localization_panel._worker.publish_launch_manager_command = commands.append
        window._localization_panel._set_connected(True)

        window._localization_panel._fastlio_launch_start_btn.click()
        window._localization_panel._fastlio_launch_stop_btn.click()

        self.assertEqual(
            commands,
            [
                "start fastlio fast_lio mapping_c16.launch",
                "stop fastlio",
            ],
        )

    def test_summary_recording_writes_encoder_and_imu_files_to_one_directory(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        with temp_dir() as tmp:
            window._summary_note_edit.setPlainText("室内地面，低速直线，PWM 3000")
            session_dir = window._start_summary_recording(
                base_dir=tmp,
                timestamp="20260420_170000",
            )
            window._buffer.append(make_encoder_frame())
            window._imu_panel._on_sample("A", make_imu_sample(1))
            window._imu_panel._on_sample("B", make_imu_sample(2))

            window._stop_summary_recording(save=True)

            self.assertEqual(session_dir.name, "session_20260420_170000")
            self.assertTrue((session_dir / "encoder.csv").exists())
            self.assertTrue((session_dir / "imu_A.csv").exists())
            self.assertTrue((session_dir / "imu_B.csv").exists())
            self.assertTrue((session_dir / "imu_session.json").exists())
            self.assertTrue((session_dir / "imu_merged_aligned.csv").exists())
            self.assertTrue((session_dir / "session.json").exists())
            with (session_dir / "session.json").open("r", encoding="utf-8") as fh:
                metadata = json.load(fh)
            self.assertEqual(metadata["note"], "室内地面，低速直线，PWM 3000")
            self.assertEqual(metadata["files"]["encoder"], "encoder.csv")

    def test_summary_module_exposes_serial_and_ros_source_selectors(self) -> None:
        window = MainWindow()

        encoder_source = window._summary_rows["encoder"]["source_combo"]
        imu_source = window._summary_rows["imu_A"]["source_combo"]

        self.assertGreaterEqual(encoder_source.findData("serial"), 0)
        self.assertGreaterEqual(encoder_source.findData("ros_odom"), 0)
        self.assertGreaterEqual(imu_source.findData("serial"), 0)
        self.assertGreaterEqual(imu_source.findData("ros_imu"), 0)

    def test_summary_ros_odom_status_counts_only_odom_topic_messages(self) -> None:
        window = MainWindow()
        window._summary_rows["encoder"]["source_combo"].setCurrentIndex(
            window._summary_rows["encoder"]["source_combo"].findData("ros_odom")
        )
        window._on_ros_connection_changed(True)

        for topic in ("/odom", "/imu", "/active_imu", "/PowerVoltage", "/wheeltec/akm_state"):
            window._on_ros_message({"topic": topic, "message": {}, "recv_time_epoch_s": 1.0})

        window._update_summary_status()

        self.assertEqual(window._summary_rows["encoder"]["frame_label"].text(), "1")

    def test_summary_page_defaults_to_recordings_directory(self) -> None:
        window = MainWindow()

        self.assertEqual(window._summary_save_dir_edit.text(), r"D:\debug_monitor\recordings")

    def test_summary_page_defaults_all_recordable_sources_checked(self) -> None:
        window = MainWindow()

        self.assertTrue(window._summary_source_checks)
        self.assertFalse(window._summary_source_checks["hr23_radar"].isChecked())
        self.assertTrue(all(
            check.isChecked()
            for source_id, check in window._summary_source_checks.items()
            if source_id != "hr23_radar"
        ))

    def test_summary_page_exposes_rosbag_sync_checkbox(self) -> None:
        window = MainWindow()

        self.assertEqual(window._summary_rosbag_sync_cb.text(), "同步车端 rosbag 录制")
        self.assertTrue(window._summary_rosbag_sync_cb.isChecked())
        self.assertTrue(window._summary_source_enabled("rosbag_raw"))
        self.assertIn("rosbag_raw", window._selected_summary_sources())

        window._summary_rosbag_sync_cb.setChecked(False)

        self.assertFalse(window._summary_source_enabled("rosbag_raw"))
        self.assertNotIn("rosbag_raw", window._selected_summary_sources())

    def test_launch_manager_status_updates_rosbag_panel(self) -> None:
        window = MainWindow()

        payload = {
            "rosbag": {"active": True, "session_id": "session_1", "current_size_bytes": 1024},
            "rosbag_library": {
                "bag_dir": "/home/wheeltec/bags",
                "sessions": [{"session_id": "session_1", "size_bytes": 1024}],
            },
        }
        window._on_launch_manager_status(payload)

        self.assertEqual(window._latest_rosbag_status.session_id, "session_1")
        self.assertEqual(window._rosbag_panel._status_values["session_id"].text(), "session_1")
        self.assertEqual(window._rosbag_panel._session_table.rowCount(), 1)

    def test_launch_manager_status_updates_rosbag_panel_from_nested_data(self) -> None:
        window = MainWindow()

        payload = {
            "level": "state",
            "message": "ok",
            "data": {
                "running": [],
                "detail": {},
                "rosbag": {"active": True, "session_id": "session_nested", "topics": ["/imu"]},
                "rosbag_library": {"sessions": [{"session_id": "session_nested"}]},
            },
        }
        window._on_launch_manager_status(payload)

        self.assertEqual(window._latest_rosbag_status.session_id, "session_nested")
        self.assertEqual(window._rosbag_panel._status_values["session_id"].text(), "session_nested")
        self.assertEqual(window._rosbag_panel._session_table.rowCount(), 1)

    def test_launch_manager_status_shows_protocol_errors(self) -> None:
        window = MainWindow()

        window._on_launch_manager_status({"level": "error", "message": "bad command", "data": {}})
        self.assertIn("bad command", window._status_label.text())

        window._on_launch_manager_status({"data": {"last_command": {"ok": False, "error": "delete denied"}}})
        self.assertIn("delete denied", window._status_label.text())

    def test_summary_recording_sends_rosbag_start_and_stop_and_writes_metadata(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        commands = []
        window._ros_worker.request_rosbag_start = commands.append
        window._ros_worker.request_rosbag_stop = lambda session_id: commands.append({"action": "stop", "session_id": session_id})

        with temp_dir() as tmp:
            session_dir = window._start_summary_recording(base_dir=tmp, timestamp="20260612_153000")
            window._on_launch_manager_status(
                {
                    "rosbag": {
                        "active": True,
                        "session_id": "session_20260612_153000",
                        "remote_dir": "/home/wheeltec/bags/session_20260612_153000",
                        "current_size_bytes": 4096,
                        "bag_files": ["fastlio_0.bag.active"],
                        "duration_s": 2.5,
                    }
                }
            )
            window._stop_summary_recording(save=True)

            with (session_dir / "session.json").open("r", encoding="utf-8") as handle:
                metadata = json.load(handle)

        self.assertEqual(commands[0]["action"], "start_rosbag")
        self.assertEqual(commands[0]["session_id"], "session_20260612_153000")
        self.assertEqual(commands[1], {"action": "stop", "session_id": "session_20260612_153000"})
        self.assertTrue(metadata["rosbag"]["enabled"])
        self.assertEqual(metadata["rosbag"]["session_id"], "session_20260612_153000")
        self.assertEqual(metadata["files"]["rosbag_manifest"], "raw/rosbag_manifest.json")

    def test_summary_recording_sends_rosbag_start_for_each_session(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        commands = []
        window._ros_worker.request_rosbag_start = lambda config: commands.append(dict(config))
        window._ros_worker.request_rosbag_stop = lambda session_id: commands.append(
            {"action": "stop", "session_id": session_id}
        )

        with temp_dir() as tmp:
            window._start_summary_recording(base_dir=tmp, timestamp="20260612_153000")
            window._stop_summary_recording(save=True)
            window._start_summary_recording(base_dir=tmp, timestamp="20260612_153100")
            window._stop_summary_recording(save=True)

        start_commands = [command for command in commands if command.get("action") == "start_rosbag"]
        stop_commands = [command for command in commands if command.get("action") == "stop"]
        self.assertEqual(
            [command["session_id"] for command in start_commands],
            ["session_20260612_153000", "session_20260612_153100"],
        )
        self.assertEqual(
            [command["session_id"] for command in stop_commands],
            ["session_20260612_153000", "session_20260612_153100"],
        )

    def test_summary_recording_uses_unique_rosbag_session_id_when_directory_gets_suffix(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        commands = []
        window._ros_worker.request_rosbag_start = lambda config: commands.append(dict(config))
        window._ros_worker.request_rosbag_stop = lambda session_id: commands.append(
            {"action": "stop", "session_id": session_id}
        )

        with temp_dir() as tmp:
            first_dir = window._start_summary_recording(base_dir=tmp, timestamp="20260612_153000")
            window._stop_summary_recording(save=True)
            second_dir = window._start_summary_recording(base_dir=tmp, timestamp="20260612_153000")
            window._stop_summary_recording(save=True)

        self.assertEqual(first_dir.name, "session_20260612_153000")
        self.assertEqual(second_dir.name, "session_20260612_153000_001")
        start_commands = [command for command in commands if command.get("action") == "start_rosbag"]
        self.assertEqual(
            [command["session_id"] for command in start_commands],
            ["session_20260612_153000", "session_20260612_153000_001"],
        )

    def test_summary_record_button_sends_rosbag_start_for_repeated_recordings(self) -> None:
        window = MainWindow()
        window._apply_summary_check_results({"mock": {"status": "ok"}})
        commands = []
        stop_tasks = []
        window._ros_worker.request_rosbag_start = lambda config: commands.append(dict(config))
        window._ros_worker.request_rosbag_stop = lambda session_id: commands.append(
            {"action": "stop", "session_id": session_id}
        )
        window._run_background_task = lambda function, _finished, _error: stop_tasks.append(function)

        with temp_dir() as tmp:
            window._summary_save_dir_edit.setText(str(tmp))
            window._summary_record_btn.click()
            window._summary_record_btn.click()
            result = stop_tasks.pop(0)()
            window._on_summary_stop_finished(result)
            window._summary_record_btn.click()
            window._summary_record_btn.click()
            result = stop_tasks.pop(0)()
            window._on_summary_stop_finished(result)

        start_commands = [command for command in commands if command.get("action") == "start_rosbag"]
        self.assertEqual(len(start_commands), 2)
        self.assertNotEqual(start_commands[0]["session_id"], start_commands[1]["session_id"])

    def test_summary_page_defaults_trajectory_topic_to_odometry(self) -> None:
        window = MainWindow()

        self.assertEqual(window._trajectory_topic_combo.currentData(), "/Odometry")
        self.assertFalse(window._trajectory_topic_custom_edit.isEnabled())

    def test_summary_page_exposes_check_sources_button(self) -> None:
        window = MainWindow()

        self.assertEqual(window._summary_check_btn.text(), "检查可记录数据源")

    def test_summary_page_exposes_radar_source_directory_and_xml_fields(self) -> None:
        window = MainWindow()

        self.assertTrue(hasattr(window, "_summary_radar_source_dir_edit"))
        self.assertTrue(hasattr(window, "_summary_radar_xml_path_edit"))

    def test_summary_page_exposes_hr23_radar_source_unchecked_by_default(self) -> None:
        window = MainWindow()

        self.assertIn("hr23_radar", window._summary_source_checks)
        self.assertFalse(window._summary_source_checks["hr23_radar"].isChecked())
        self.assertEqual(window._summary_hr23_host_edit.text(), "127.0.0.1")
        self.assertEqual(window._summary_hr23_port_edit.text(), "7070")
        self.assertEqual(window._summary_hr23_test_btn.text(), "测试")

    def test_hr23_radar_test_button_displays_status_fields(self) -> None:
        window = MainWindow()
        fake = FakeHr23RadarClient()
        window._hr23_radar_client_factory = lambda **_kwargs: fake

        window._test_summary_hr23_connection()

        self.assertEqual(fake.calls, [("status", None)])
        self.assertIn("idle", window._summary_hr23_state_label.text())
        self.assertIn("3", window._summary_hr23_packets_label.text())
        self.assertIn("96", window._summary_hr23_bytes_label.text())
        self.assertIn("2026-06-12T15:30:00Z", window._summary_hr23_last_packet_label.text())

    def test_summary_recording_controls_hr23_and_writes_separate_metadata(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        fake = FakeHr23RadarClient()
        window._hr23_radar_client_factory = lambda **_kwargs: fake
        window._summary_source_checks["hr23_radar"].setChecked(True)

        with temp_dir() as tmp:
            session_dir = window._start_summary_recording(
                base_dir=tmp,
                timestamp="20260612_153000",
            )
            self.assertTrue((session_dir / "raw" / "hr23_radar").is_dir())
            prepare_kwargs = fake.calls[0][1]
            self.assertIn("recording_start_epoch_s", prepare_kwargs)
            self.assertIn("recording_start_perf_s", prepare_kwargs)
            self.assertEqual(
                prepare_kwargs["recording_start_epoch_s"],
                window._summary_recording_start_epoch_s,
            )
            self.assertEqual(
                prepare_kwargs["recording_start_perf_s"],
                window._summary_recording_start_perf_s,
            )
            window._stop_summary_recording(save=True)

            self.assertEqual([call[0] for call in fake.calls], ["prepare", "start", "stop"])
            self.assertEqual(prepare_kwargs["session_id"], "session_20260612_153000")
            self.assertEqual(prepare_kwargs["output_dir"], session_dir / "raw" / "hr23_radar")

            with (session_dir / "session.json").open("r", encoding="utf-8") as handle:
                metadata = json.load(handle)
            hr23 = metadata["devices"]["hr23_radar"]
            self.assertTrue(hr23["enabled"])
            self.assertEqual(hr23["capture_dir"], "raw/hr23_radar")
            self.assertEqual(hr23["packetCount"], 8)
            self.assertEqual(hr23["totalBytes"], 256)
            self.assertEqual(hr23["firstPacketUtc"], "2026-06-12T15:30:01Z")
            self.assertEqual(metadata["files"]["hr23_radar_dir"], "raw/hr23_radar")
            self.assertEqual(metadata["devices"]["radar"]["enabled"], False)

    def test_hr23_stop_failure_writes_error_and_skips_packaging(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        fake = FakeHr23RadarClient(fail_stop=True)
        window._hr23_radar_client_factory = lambda **_kwargs: fake
        window._summary_source_checks["hr23_radar"].setChecked(True)

        with temp_dir() as tmp:
            session_dir = window._start_summary_recording(
                base_dir=tmp,
                timestamp="20260612_154000",
            )
            with self.assertRaisesRegex(RuntimeError, "HR2.3 stop 失败"):
                window._stop_summary_recording(save=True)

            with (session_dir / "session.json").open("r", encoding="utf-8") as handle:
                metadata = json.load(handle)
            self.assertIn("recorder stop timeout", metadata["devices"]["hr23_radar"]["stop_error"])
            self.assertFalse((session_dir / f"{session_dir.name}.zip").exists())

    def test_background_task_retains_worker_until_completion(self) -> None:
        window = MainWindow()
        finished: list[object] = []
        errors: list[str] = []

        window._run_background_task(lambda: "ok", finished.append, errors.append)
        gc.collect()

        self.assertTrue(window._summary_background_workers)

        for _ in range(100):
            self.app.processEvents()
            if (finished or errors) and not window._summary_background_workers:
                break

        self.assertEqual(finished, ["ok"])
        self.assertFalse(errors)
        self.assertFalse(window._summary_background_workers)

    def test_summary_source_check_timeout_recovers_buttons_and_ignores_late_result(self) -> None:
        window = MainWindow()
        generation = 1
        window._summary_check_generation = generation
        window._set_summary_recording_state("CHECKING", "CHECKING")

        window._on_summary_source_check_timeout(generation)

        self.assertEqual(window._summary_recording_state, "ERROR")
        self.assertTrue(window._summary_check_btn.isEnabled())
        self.assertTrue(window._summary_record_btn.isEnabled())
        self.assertIn("超时", window._summary_check_status_label.text())

        window._on_summary_source_check_finished({"mock": {"status": "ok"}}, generation=generation)

        self.assertEqual(window._summary_recording_state, "ERROR")
        self.assertEqual(window._summary_last_check_results, {})

    def test_check_summary_sources_reports_offline_when_ros_and_serial_are_unavailable(self) -> None:
        window = MainWindow()
        window._sample_ros_topic = lambda **kwargs: SimpleNamespace(
            status="offline",
            estimated_hz=0.0,
            has_header_stamp=False,
            messages_received=0,
            notes="ROS 未连接",
        )

        result = window._check_summary_sources()

        self.assertIn("serial_encoder", result)
        self.assertEqual(result["serial_encoder"]["status"], "offline")
        self.assertEqual(result["imu_A"]["status"], "offline")
        self.assertEqual(result["imu_B"]["status"], "offline")
        self.assertIn("fastlio_odometry", result)
        self.assertEqual(result["fastlio_odometry"]["status"], "offline")
        self.assertEqual(result["radar_bin"]["status"], "skipped")
        self.assertIn("offline 12", window._summary_check_status_label.text())
        self.assertIn("skipped 1", window._summary_check_status_label.text())
        self.assertIn("检查完成", window._summary_check_status_label.text())

    def test_start_summary_recording_runs_source_check_first(self) -> None:
        window = MainWindow()
        calls = []

        def wrapped():
            calls.append("checked")
            return {
                "mock": {
                    "status": "ok",
                    "estimated_hz": 100.0,
                    "has_header_stamp": True,
                    "messages_received": 3,
                    "notes": "test source ready",
                }
            }

        window._check_summary_sources = wrapped

        with temp_dir() as tmp:
            window._start_summary_recording(base_dir=tmp, timestamp="20260608_131500")
            window._stop_summary_recording(save=False)

        self.assertEqual(calls, ["checked"])

    def test_start_summary_recording_blocks_when_checked_source_is_offline(self) -> None:
        window = MainWindow()
        window._check_summary_sources = lambda: {
            "fastlio_odometry": {
                "status": "offline",
                "estimated_hz": 0.0,
                "has_header_stamp": False,
                "messages_received": 0,
                "notes": "ROS 未连接",
            }
        }

        with temp_dir() as tmp:
            with self.assertRaises(RuntimeError):
                window._start_summary_recording(base_dir=tmp, timestamp="20260608_140000")

            self.assertFalse(any(tmp.iterdir()))

    def test_summary_recording_can_use_ros_odom_and_received_ros_imu_sources(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        window._summary_rows["encoder"]["source_combo"].setCurrentIndex(
            window._summary_rows["encoder"]["source_combo"].findData("ros_odom")
        )
        window._summary_rows["imu_A"]["source_combo"].setCurrentIndex(
            window._summary_rows["imu_A"]["source_combo"].findData("ros_imu")
        )
        window._summary_rows["imu_B"]["source_combo"].setCurrentIndex(
            window._summary_rows["imu_B"]["source_combo"].findData("ros_imu")
        )

        with temp_dir() as tmp:
            session_dir = window._start_summary_recording(
                base_dir=tmp,
                timestamp="20260423_110000",
            )
            window._on_ros_snapshot(
                RosSnapshot(
                    frame_count=1,
                    last_topic="/odom",
                    linear_x=0.2,
                    angular_z=0.03,
                    pose_x=1.0,
                    pose_y=2.0,
                )
            )
            window._on_ros_snapshot(
                RosSnapshot(
                    frame_count=2,
                    last_topic="/imu",
                    imu=RosImuReading(frame_count=1, accel_z=9.8, yaw_deg=12.0),
                )
            )

            window._stop_summary_recording(save=True)

            self.assertTrue((session_dir / "ros_odom.csv").exists())
            self.assertTrue((session_dir / "ros_imu.csv").exists())
            self.assertTrue((session_dir / "ros_active_imu.csv").exists())
            self.assertFalse((session_dir / "encoder.csv").exists())
            self.assertFalse((session_dir / "imu_A.csv").exists())
            with (session_dir / "session.json").open("r", encoding="utf-8") as fh:
                metadata = json.load(fh)
            self.assertEqual(metadata["devices"]["encoder"]["source"], "ros_odom")
            self.assertEqual(metadata["devices"]["encoder"]["topic"], "/odom")
            self.assertEqual(metadata["devices"]["imu_A"]["source"], "ros_imu")
            self.assertEqual(metadata["files"]["ros_odom"], "ros_odom.csv")
            self.assertEqual(metadata["files"]["ros_imu"], "ros_imu.csv")
            self.assertEqual(metadata["files"]["ros_active_imu"], "ros_active_imu.csv")

    def test_summary_recording_writes_fastlio_and_akm_topic_files_from_ros_messages(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)

        with temp_dir() as tmp:
            session_dir = window._start_summary_recording(
                base_dir=tmp,
                timestamp="20260608_130000",
            )
            window._on_ros_message(
                {
                    "topic": "/Odometry",
                    "message_type": "nav_msgs/Odometry",
                    "recv_time_epoch_s": 10.0,
                    "message": {
                        "header": {"stamp": {"secs": 1, "nsecs": 0}, "frame_id": "odom"},
                        "child_frame_id": "base_link",
                        "pose": {"pose": {"position": {"x": 1.0, "y": 2.0, "z": 0.0}, "orientation": {"z": 0.0, "w": 1.0}}},
                        "twist": {"twist": {"linear": {"x": 0.4}, "angular": {"z": 0.1}}},
                    },
                }
            )
            window._on_ros_message(
                {
                    "topic": "/wheeltec/akm_state",
                    "message_type": "turn_on_wheeltec_robot/AkmState",
                    "recv_time_epoch_s": 10.01,
                    "message": {
                        "header": {"stamp": {"secs": 1, "nsecs": 10_000_000}, "frame_id": "base_link"},
                        "seq_id": 1,
                        "control_tick_us": 1000,
                        "dt_us": 10000,
                        "left_wheel_speed": 3.0,
                        "right_wheel_speed": 3.1,
                        "steering_angle": 0.2,
                    },
                }
            )

            window._stop_summary_recording(save=True)

            self.assertTrue((session_dir / "trajectory_odometry.csv").exists())
            self.assertTrue((session_dir / "fastlio_odometry.csv").exists())
            self.assertTrue((session_dir / "akm_state.csv").exists())
            self.assertTrue((session_dir / "raw" / "trajectory_odometry.csv").exists())
            self.assertTrue((session_dir / "raw" / "fastlio_odometry.csv").exists())
            self.assertTrue((session_dir / "raw" / "akm_state.csv").exists())

    def test_summary_ros_message_gate_drops_messages_outside_recording_window(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)

        with temp_dir() as tmp:
            session_dir = window._start_summary_recording(base_dir=tmp, timestamp="20260609_101500")
            start_epoch = window._summary_recording_start_epoch_s
            self.assertIsNotNone(start_epoch)

            base_message = {
                "header": {"stamp": {"secs": 10, "nsecs": 0}, "frame_id": "odom"},
                "pose": {"pose": {"position": {"x": 1.0}, "orientation": {"w": 1.0}}},
                "twist": {"twist": {"linear": {"x": 0.2}, "angular": {"z": 0.3}}},
            }
            window._on_ros_message({"topic": "/odom", "message": base_message, "recv_time_epoch_s": start_epoch - 0.5})
            window._on_ros_message({"topic": "/odom", "message": base_message, "recv_time_epoch_s": start_epoch + 0.5})
            window._summary_recording_stop_epoch_s = start_epoch + 1.0
            window._summary_recording_gate_enabled = False
            window._on_ros_message({"topic": "/odom", "message": base_message, "recv_time_epoch_s": start_epoch + 1.5})

            window._stop_summary_recording(save=True)

            with (session_dir / "ros_odom.csv").open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            with (session_dir / "session.json").open("r", encoding="utf-8") as handle:
                metadata = json.load(handle)

            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["recv_time"], str(start_epoch + 0.5))
            self.assertEqual(metadata["dropped_pre_start_ros_messages"], 1)
            self.assertEqual(metadata["dropped_post_stop_ros_messages"], 1)

    def test_stop_summary_recording_sets_stop_boundary_before_packaging(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)

        with temp_dir() as tmp:
            session_dir = window._start_summary_recording(base_dir=tmp, timestamp="20260609_102000")
            self.assertTrue(window._summary_recording_gate_enabled)

            observed = {}
            main_window_module = sys.modules["main_window"]
            original_package = main_window_module.build_summary_package
            main_window_module.build_summary_package = lambda _session_dir: observed.update(
                gate=window._summary_recording_gate_enabled,
                stop_epoch=window._summary_recording_stop_epoch_s,
            ) or {}
            try:
                window._stop_summary_recording(save=True)
            finally:
                main_window_module.build_summary_package = original_package

            self.assertFalse(observed["gate"])
            self.assertIsNotNone(observed["stop_epoch"])
            with (session_dir / "session.json").open("r", encoding="utf-8") as handle:
                metadata = json.load(handle)
            self.assertEqual(metadata["recording_stop_epoch_s"], observed["stop_epoch"])
            self.assertAlmostEqual(
                metadata["duration_s"],
                metadata["recording_stop_epoch_s"] - metadata["recording_start_epoch_s"],
                places=3,
            )

    def test_async_stop_detaches_ui_state_before_background_finalize(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)

        with temp_dir() as tmp:
            window._summary_note_edit.setPlainText("async stop")
            session_dir = window._start_summary_recording(base_dir=tmp, timestamp="20260609_110000")
            window._buffer.append(make_encoder_frame())
            window._imu_panel._on_sample("A", make_imu_sample(1))

            captured = {}
            window._run_background_task = lambda function, _finished, _error: captured.setdefault("function", function)
            window._imu_panel.stop_recording = lambda save: (_ for _ in ()).throw(
                AssertionError("background stop must not call ImuPanel.stop_recording")
            )

            window._stop_summary_recording_async(save=True)

            self.assertIn("function", captured)
            window._summary_files_metadata = lambda: (_ for _ in ()).throw(
                AssertionError("background stop must not read summary UI state")
            )
            result = captured["function"]()

            self.assertEqual(result, session_dir)
            self.assertTrue((session_dir / "encoder.csv").exists())
            self.assertTrue((session_dir / "imu_A.csv").exists())
            self.assertTrue((session_dir / "imu_session.json").exists())

    def test_radar_sync_checkbox_is_enabled_only_after_successful_connection_test(self) -> None:
        window = MainWindow()
        radar = FakeRadarClient()
        window._radar_client = radar

        self.assertFalse(window._summary_radar_sync_cb.isEnabled())

        window._test_summary_radar_connection()

        self.assertEqual(radar.identify_calls, 1)
        self.assertTrue(window._summary_radar_sync_cb.isEnabled())
        self.assertIn("PHASELOCK", window._summary_radar_status_label.text())

    def test_summary_page_does_not_expose_mapping_freeze_control(self) -> None:
        window = MainWindow()

        self.assertFalse(hasattr(window, "_summary_mapping_freeze_btn"))
        self.assertFalse(hasattr(window, "_toggle_summary_mapping_freeze"))

    def test_summary_recording_starts_and_stops_radar_when_sync_is_checked(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        radar = FakeRadarClient()
        window._radar_client = radar
        window._test_summary_radar_connection()
        window._summary_radar_sync_cb.setChecked(True)

        with temp_dir() as tmp:
            window._start_summary_recording(base_dir=tmp, timestamp="20260424_153000")
            window._stop_summary_recording(save=True)

        self.assertEqual(radar.started, ["20260424_153000"])
        self.assertEqual(radar.stopped, 1)

    def test_summary_recording_parses_radar_outputs_into_raw_radar_directory(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        radar = FakeRadarClient()
        window._radar_client = radar
        window._test_summary_radar_connection()
        window._summary_radar_sync_cb.setChecked(True)

        with temp_dir() as tmp:
            radar_output_dir = tmp / "radar_source"
            radar_output_dir.mkdir()
            radar_filename = "2026_04_24_15_30_00.bin"
            (radar_output_dir / radar_filename).write_bytes(make_radar_big_frame())
            radar_xml = tmp / "radar_config.xml"
            radar_xml.write_text(
                """
                <配置>
                  <扫描时间s>0.000352</扫描时间s>
                  <扫频周期s>0.05</扫频周期s>
                </配置>
                """,
                encoding="utf-8",
            )
            window._summary_radar_source_dir_edit.setText(str(radar_output_dir))
            window._summary_radar_xml_path_edit.setText(str(radar_xml))

            session_dir = window._start_summary_recording(base_dir=tmp, timestamp="20260424_153000")
            window._stop_summary_recording(save=True)

            self.assertTrue((session_dir / "raw" / "radar" / "radar_recording.bin").exists())
            self.assertTrue((session_dir / "raw" / "radar" / "radar_config.xml").exists())
            self.assertTrue((session_dir / "raw" / "radar" / "radar_sweeps.csv").exists())
            self.assertTrue((session_dir / "raw" / "radar" / "radar_complex.npz").exists())

    def test_summary_recording_does_not_touch_radar_when_sync_is_unchecked(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        radar = FakeRadarClient()
        window._radar_client = radar

        with temp_dir() as tmp:
            window._start_summary_recording(base_dir=tmp, timestamp="20260424_153000")
            window._stop_summary_recording(save=True)

        self.assertEqual(radar.started, [])
        self.assertEqual(radar.stopped, 0)

    def test_summary_recording_aborts_when_checked_radar_start_fails(self) -> None:
        window = MainWindow()
        allow_summary_source_checks(window)
        radar = FakeRadarClient(fail_start=True)
        window._radar_client = radar
        window._summary_radar_sync_cb.setEnabled(True)
        window._summary_radar_sync_cb.setChecked(True)
        stopped = []
        window._ros_worker.request_rosbag_stop = stopped.append

        with temp_dir() as tmp:
            with self.assertRaises(RuntimeError):
                window._start_summary_recording(base_dir=tmp, timestamp="20260424_153000")

            self.assertFalse(any(tmp.iterdir()))
        self.assertEqual(stopped, ["session_20260424_153000"])
        self.assertFalse(window._is_summary_recording())
        self.assertFalse(window._buffer.recording)

    def test_recording_default_filename_includes_both_motor_pid_values(self) -> None:
        window = MainWindow()
        window._command_panel._a_kp_spin.setValue(4000)
        window._command_panel._a_ki_spin.setValue(175)
        window._command_panel._a_kd_spin.setValue(0)
        window._command_panel._b_kp_spin.setValue(4100)
        window._command_panel._b_ki_spin.setValue(200)
        window._command_panel._b_kd_spin.setValue(1)

        filename = window._recording_default_name("20260415_181442")

        self.assertEqual(
            filename,
            "debug_data_Akp4000_Aki175_Akd0_Bkp4100_Bki200_Bkd1_20260415_181442.csv",
        )

    def test_switching_to_replay_enables_replay_mode(self) -> None:
        window = MainWindow()
        window._set_replay_loaded_for_test(
            time_values=[0.0, 0.1],
            rows=[
                {
                    "final_a": 1.0,
                    "final_b": 2.0,
                    "target_a": 1.0,
                    "target_b": 2.0,
                    "output_a": 10,
                    "output_b": 20,
                },
                {
                    "final_a": 1.1,
                    "final_b": 2.1,
                    "target_a": 1.0,
                    "target_b": 2.0,
                    "output_a": 11,
                    "output_b": 21,
                },
            ],
        )
        window._set_data_mode_for_test("replay")

        self.assertEqual(window.current_data_mode(), "replay")

    def test_replay_refresh_keeps_plot_showing_first_10_seconds_before_cursor_reaches_10s(self) -> None:
        window = MainWindow()
        window._set_replay_loaded_for_test(
            time_values=[0.0, 2.0, 4.0, 6.0, 8.0, 10.0],
            rows=[
                {"final_a": 1.0, "final_b": 2.0, "target_a": 1.0, "target_b": 2.0, "output_a": 10, "output_b": 20},
                {"final_a": 1.1, "final_b": 2.1, "target_a": 1.0, "target_b": 2.0, "output_a": 11, "output_b": 21},
                {"final_a": 1.2, "final_b": 2.2, "target_a": 1.0, "target_b": 2.0, "output_a": 12, "output_b": 22},
                {"final_a": 1.3, "final_b": 2.3, "target_a": 1.0, "target_b": 2.0, "output_a": 13, "output_b": 23},
                {"final_a": 1.4, "final_b": 2.4, "target_a": 1.0, "target_b": 2.0, "output_a": 14, "output_b": 24},
                {"final_a": 1.5, "final_b": 2.5, "target_a": 1.0, "target_b": 2.0, "output_a": 15, "output_b": 25},
            ],
        )
        window._set_data_mode_for_test("replay")
        window._plot_panel._speed_plot.setXRange(0.0, 1.0, padding=0.0)
        window._replay_current_time = 2.0

        window._on_refresh()

        x_min, x_max = window._plot_panel._speed_plot.getPlotItem().viewRange()[0]
        self.assertAlmostEqual(x_min, 0.0, places=6)
        self.assertAlmostEqual(x_max, 10.0, places=6)

    def test_replay_refresh_keeps_plot_following_current_time_with_10_second_window(self) -> None:
        window = MainWindow()
        window._set_replay_loaded_for_test(
            time_values=[0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0],
            rows=[
                {"final_a": 1.0, "final_b": 2.0, "target_a": 1.0, "target_b": 2.0, "output_a": 10, "output_b": 20},
                {"final_a": 1.1, "final_b": 2.1, "target_a": 1.0, "target_b": 2.0, "output_a": 11, "output_b": 21},
                {"final_a": 1.2, "final_b": 2.2, "target_a": 1.0, "target_b": 2.0, "output_a": 12, "output_b": 22},
                {"final_a": 1.3, "final_b": 2.3, "target_a": 1.0, "target_b": 2.0, "output_a": 13, "output_b": 23},
                {"final_a": 1.4, "final_b": 2.4, "target_a": 1.0, "target_b": 2.0, "output_a": 14, "output_b": 24},
                {"final_a": 1.5, "final_b": 2.5, "target_a": 1.0, "target_b": 2.0, "output_a": 15, "output_b": 25},
                {"final_a": 1.6, "final_b": 2.6, "target_a": 1.0, "target_b": 2.0, "output_a": 16, "output_b": 26},
            ],
        )
        window._set_data_mode_for_test("replay")
        window._plot_panel._speed_plot.setXRange(0.0, 1.0, padding=0.0)
        window._replay_current_time = 12.0

        window._on_refresh()

        x_min, x_max = window._plot_panel._speed_plot.getPlotItem().viewRange()[0]
        self.assertAlmostEqual(x_min, 2.0, places=6)
        self.assertAlmostEqual(x_max, 12.0, places=6)
