import os
import struct
import sys
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from sar_bp import (
    BpGrid,
    RadarParams,
    SarBpRunConfig,
    back_projection,
    back_projection_auto,
    back_projection_time_domain,
    calculate_one_sweep_points,
    image_magnitude_db,
    _grid_from_range,
    read_xkbd_bin,
    reshape_sweeps,
    sync_motion_trajectory,
)


class SarBpProtocolTests(unittest.TestCase):
    def test_calculates_default_one_sweep_points(self) -> None:
        self.assertEqual(calculate_one_sweep_points(), 510)

    def test_read_xkbd_bin_extracts_payload_as_big_endian_iq_and_offset(self) -> None:
        raw = bytearray(130260)
        int16_values = [0] * 65130
        int16_values[6:12] = [1, -2, 32767, -32768, 100, -100]
        for index, value in enumerate(int16_values):
            struct.pack_into(">h", raw, index * 2, value)
        raw[8] = 1
        raw[9] = 0

        path = os.path.join(os.path.dirname(__file__), "..", ".test_tmp", "xkbd_one_frame.bin")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as handle:
            handle.write(raw)

        result = read_xkbd_bin(path, one_sweep_points=510)

        np.testing.assert_array_equal(result.complex_samples[:3], np.array([1 - 2j, 32767 - 32768j, 100 - 100j]))
        self.assertEqual(result.row_count, 1)
        self.assertEqual(result.offset_points, 431)

    def test_reshape_sweeps_discards_offset_and_incomplete_tail(self) -> None:
        samples = np.arange(14, dtype=np.complex64)

        matrix = reshape_sweeps(samples, nr=4, offset_points=2)

        np.testing.assert_array_equal(
            matrix,
            np.array([[2, 6, 10], [3, 7, 11], [4, 8, 12], [5, 9, 13]], dtype=np.complex64),
        )


class SarBpMotionTests(unittest.TestCase):
    def test_sync_motion_trajectory_integrates_encoder_and_applies_lever_arm(self) -> None:
        odom = pd.DataFrame(
            {
                "time_s": [10.0, 11.0, 12.0],
                "motor_a_left_speed": [1.0, 1.0, 1.0],
                "motor_b_right_speed": [1.0, 1.0, 1.0],
            }
        )
        imu = pd.DataFrame(
            {
                "pair_time": [20.0, 21.0, 22.0],
                "imu_accel_x": [0.0, 0.0, 0.0],
                "imu_accel_y": [0.0, 0.0, 0.0],
                "imu_accel_z": [0.0, 0.0, 0.0],
                "active_imu_accel_x": [0.0, 0.0, 0.0],
                "active_imu_accel_y": [0.0, 0.0, 0.0],
                "active_imu_accel_z": [0.0, 0.0, 0.0],
                "imu_roll_deg": [0.0, 0.0, 0.0],
                "imu_pitch_deg": [0.0, 0.0, 0.0],
                "imu_yaw_deg": [0.0, 0.0, 0.0],
                "active_imu_roll_deg": [0.0, 0.0, 0.0],
                "active_imu_pitch_deg": [0.0, 0.0, 0.0],
                "active_imu_yaw_deg": [0.0, 0.0, 0.0],
            }
        )

        trajectory = sync_motion_trajectory(odom, imu, radar_lever_arm_m=(0.1, 0.2, 0.3))

        np.testing.assert_allclose(trajectory.time_s, [0.0, 1.0, 2.0])
        np.testing.assert_allclose(trajectory.radar_xyz_sample[:, 0], [-0.9, 0.1, 1.1])
        np.testing.assert_allclose(trajectory.radar_xyz_sample[:, 1], [0.2, 0.2, 0.2])
        np.testing.assert_allclose(trajectory.radar_xyz_sample[:, 2], [0.3, 0.3, 0.3])


class SarBpImagingTests(unittest.TestCase):
    def test_back_projection_returns_image_with_peak_at_point_target(self) -> None:
        params = RadarParams(range_bandwidth_hz=600e6, sweep_time_s=408e-6, fs_hz=1.25e6, ddc_hz=0.0)
        radar_xyz = np.array([[-0.1, 0.0, 0.0], [0.1, 0.0, 0.0]], dtype=np.float64)
        grid = BpGrid(
            x_m=np.array([0.0], dtype=np.float64),
            y_m=np.array([0.75, 1.0, 1.25], dtype=np.float64),
            z_m=0.0,
        )
        nfft = 512
        range_bin_spacing = params.range_bin_spacing_m(nfft)
        target_bin = int(round(np.sqrt(1.0**2 + 0.1**2) / range_bin_spacing))
        data = np.zeros((32, 2), dtype=np.complex128)
        profiles = np.zeros((nfft, 2), dtype=np.complex128)
        profiles[target_bin, :] = 1.0
        data[:, :] = np.fft.ifft(profiles, axis=0)[:32, :]

        image = back_projection(data, radar_xyz, grid, params=params, range_fft_size=nfft, window=False)

        self.assertEqual(image.shape, (3, 1))
        self.assertEqual(int(np.argmax(np.abs(image[:, 0]))), 1)

    def test_time_domain_back_projection_focuses_matlab_style_point_target(self) -> None:
        params = RadarParams(
            carrier_hz=18.8e9,
            range_bandwidth_hz=600e6,
            sweep_time_s=408e-6,
            fs_hz=1.25e6,
            ddc_hz=0.0,
        )
        nr = 32
        radar_xyz = np.array([[-0.2, -20.0, 0.0], [0.0, -20.0, 0.0], [0.2, -20.0, 0.0]], dtype=np.float64)
        target = np.array([0.0, 16.0, 0.0], dtype=np.float64)
        tr = np.arange(nr, dtype=np.float64) / params.fs_hz
        data = np.zeros((nr, radar_xyz.shape[0]), dtype=np.complex128)
        for pulse_index, radar_pos in enumerate(radar_xyz):
            distance = np.linalg.norm(target - radar_pos)
            phase = 4.0 * np.pi / 299_792_458.0 * (params.carrier_hz + params.chirp_slope_hz_per_s * tr) * distance
            data[:, pulse_index] = np.exp(1j * phase)
        grid = BpGrid(
            x_m=np.array([0.0], dtype=np.float64),
            y_m=np.array([15.5, 16.0, 16.5], dtype=np.float64),
            z_m=0.0,
        )

        image = back_projection_time_domain(data, radar_xyz, grid, params=params)

        self.assertEqual(image.shape, (3, 1))
        self.assertEqual(int(np.argmax(np.abs(image[:, 0]))), 1)

    def test_back_projection_auto_falls_back_to_cpu_when_gpu_is_unavailable(self) -> None:
        params = RadarParams(range_bandwidth_hz=600e6, sweep_time_s=408e-6, fs_hz=1.25e6, ddc_hz=0.0)
        radar_xyz = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
        grid = BpGrid(x_m=np.array([0.0]), y_m=np.array([1.0]), z_m=0.0)
        data = np.ones((8, 1), dtype=np.complex128)

        image = back_projection_auto(data, radar_xyz, grid, params=params, range_fft_size=16, use_gpu=True)

        self.assertEqual(image.shape, (1, 1))

    def test_image_magnitude_db_normalizes_peak_to_zero_db(self) -> None:
        image = np.array([[1 + 0j, 10 + 0j], [0 + 0j, 5 + 0j]], dtype=np.complex128)

        db = image_magnitude_db(image, dynamic_range_db=40.0)

        self.assertAlmostEqual(float(db[0, 1]), 0.0)
        self.assertGreaterEqual(float(db.min()), -40.0)

    def test_image_magnitude_db_handles_all_zero_image(self) -> None:
        db = image_magnitude_db(np.zeros((2, 2), dtype=np.complex128))

        np.testing.assert_array_equal(db, np.zeros((2, 2), dtype=np.float64))


class SarBpConfigTests(unittest.TestCase):
    def test_run_config_uses_in_file_grid_ranges_instead_of_cli_strings(self) -> None:
        config = SarBpRunConfig(x_grid_m=(-1.0, 1.0, 5), y_grid_m=(0.2, 0.6, 3))

        np.testing.assert_allclose(_grid_from_range(config.x_grid_m), [-1.0, -0.5, 0.0, 0.5, 1.0])
        np.testing.assert_allclose(_grid_from_range(config.y_grid_m), [0.2, 0.4, 0.6])


if __name__ == "__main__":
    unittest.main()
