import os
import sys
import unittest

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from ros_bridge_worker import RosImuReading, RosSnapshot
from widgets.ros_imu_panel import RosImuPanel


class RosImuPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def tearDown(self) -> None:
        for widget in QApplication.topLevelWidgets():
            widget.close()
            widget.deleteLater()
        self.app.processEvents()

    def test_connect_button_emits_host_and_port(self) -> None:
        panel = RosImuPanel()
        requests: list[tuple[str, int]] = []
        panel.connect_requested.connect(lambda host, port: requests.append((host, port)))
        panel._host_edit.setText("192.168.0.14")
        panel._port_spin.setValue(9090)

        panel._connect_btn.click()

        self.assertEqual(requests, [("192.168.0.14", 9090)])

    def test_update_snapshot_refreshes_both_imu_value_columns(self) -> None:
        panel = RosImuPanel()

        panel.update_snapshot(
            RosSnapshot(
                imu=RosImuReading(
                    frame_count=3,
                    accel_x=1.0,
                    accel_y=2.0,
                    accel_z=9.8,
                    gyro_x=0.1,
                    gyro_y=0.2,
                    gyro_z=0.3,
                    roll_deg=10.0,
                    pitch_deg=20.0,
                    yaw_deg=30.0,
                ),
                active_imu=RosImuReading(
                    frame_count=4,
                    accel_x=-1.0,
                    accel_y=-2.0,
                    accel_z=-9.8,
                    gyro_x=-0.1,
                    gyro_y=-0.2,
                    gyro_z=-0.3,
                    roll_deg=-10.0,
                    pitch_deg=-20.0,
                    yaw_deg=-30.0,
                ),
            )
        )
        panel._flush_pending_snapshot()

        self.assertEqual(panel._labels["imu"]["title"].text(), "IMU")
        self.assertEqual(panel._labels["active_imu"]["title"].text(), "活动 IMU")
        self.assertEqual(panel._labels["imu"]["accel_z"].text(), "9.8000")
        self.assertEqual(panel._labels["imu"]["yaw_deg"].text(), "30.00")
        self.assertEqual(panel._labels["imu"]["frame_count"].text(), "3")
        self.assertEqual(panel._labels["active_imu"]["accel_z"].text(), "-9.8000")
        self.assertEqual(panel._labels["active_imu"]["yaw_deg"].text(), "-30.00")
        self.assertEqual(panel._labels["active_imu"]["frame_count"].text(), "4")

    def test_update_snapshot_appends_dual_imu_plot_series(self) -> None:
        panel = RosImuPanel()

        panel.update_snapshot(
            RosSnapshot(
                imu=RosImuReading(frame_count=1, accel_z=9.7, gyro_z=0.1, yaw_deg=5.0),
                active_imu=RosImuReading(frame_count=1, accel_z=9.8, gyro_z=0.2, yaw_deg=6.0),
            )
        )
        panel.update_snapshot(
            RosSnapshot(
                imu=RosImuReading(frame_count=2, accel_z=9.9, gyro_z=0.3, yaw_deg=7.0),
                active_imu=RosImuReading(frame_count=2, accel_z=10.0, gyro_z=0.4, yaw_deg=8.0),
            )
        )

        time_arr, data = panel._buffer.snapshot()

        self.assertEqual(len(time_arr), 2)
        self.assertAlmostEqual(data["imu_accel_z"][-1], 9.9)
        self.assertAlmostEqual(data["active_imu_accel_z"][-1], 10.0)
        self.assertAlmostEqual(data["imu_yaw_deg"][-1], 7.0)
        self.assertAlmostEqual(data["active_imu_yaw_deg"][-1], 8.0)

    def test_high_rate_updates_are_coalesced_before_plot_refresh(self) -> None:
        panel = RosImuPanel()
        refresh_count = 0

        def count_refresh() -> None:
            nonlocal refresh_count
            refresh_count += 1

        panel._refresh_plot = count_refresh

        for frame_count in range(1, 21):
            panel.update_snapshot(
                RosSnapshot(
                    imu=RosImuReading(frame_count=frame_count, accel_z=9.8),
                    active_imu=RosImuReading(frame_count=frame_count, accel_z=9.9),
                )
            )

        self.assertEqual(refresh_count, 0)

        panel._flush_pending_snapshot()

        self.assertEqual(refresh_count, 1)
        self.assertEqual(panel._labels["imu"]["frame_count"].text(), "20")
        self.assertEqual(panel._labels["active_imu"]["frame_count"].text(), "20")

    def test_plot_view_follows_latest_10_seconds(self) -> None:
        panel = RosImuPanel()
        original_append = panel._buffer.append
        timestamps = iter([100.0, 102.0, 104.0, 106.0, 108.0, 110.0, 112.0, 114.0, 116.0])

        def append_with_timestamp(snapshot: RosSnapshot) -> float:
            return original_append(snapshot, timestamp=next(timestamps))

        panel._buffer.append = append_with_timestamp
        panel._acc_plot.setXRange(0.0, 1.0, padding=0.0)

        for frame_count in range(1, 10):
            panel.update_snapshot(
                RosSnapshot(
                    imu=RosImuReading(frame_count=frame_count, accel_z=9.8),
                    active_imu=RosImuReading(frame_count=frame_count, accel_z=9.9),
                )
            )

        panel._flush_pending_snapshot()

        x_min, x_max = panel._acc_plot.getPlotItem().viewRange()[0]
        self.assertAlmostEqual(x_min, 6.0, places=3)
        self.assertAlmostEqual(x_max, 16.0, places=3)

    def test_update_snapshot_does_not_plot_non_imu_only_updates(self) -> None:
        panel = RosImuPanel()

        panel.update_snapshot(RosSnapshot(frame_count=1, linear_x=0.2, voltage=25.0))

        time_arr, _data = panel._buffer.snapshot()
        self.assertEqual(len(time_arr), 0)

    def test_clear_button_resets_buffer(self) -> None:
        panel = RosImuPanel()
        panel.update_snapshot(RosSnapshot(imu=RosImuReading(frame_count=1, accel_z=9.8)))

        panel._clear_btn.click()

        time_arr, _data = panel._buffer.snapshot()
        self.assertEqual(len(time_arr), 0)


if __name__ == "__main__":
    unittest.main()
