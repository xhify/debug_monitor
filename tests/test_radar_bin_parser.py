import csv
import json
import os
import shutil
import sys
import unittest
import uuid
from contextlib import contextmanager
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from radar_bin_parser import parse_radar_recording, parse_radar_xml


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


@contextmanager
def temp_dir():
    TEST_TMP_ROOT.mkdir(exist_ok=True)
    path = TEST_TMP_ROOT / f"radar_parser_{uuid.uuid4().hex}"
    path.mkdir()
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


def make_big_frame(offset: int = 0) -> bytes:
    header = bytearray(16)
    header[8] = offset & 0xFF
    header[9] = (offset >> 8) & 0xFF
    iq_values = np.arange(65120, dtype=">i2")
    body = iq_values.tobytes()
    footer = bytes(4)
    return bytes(header) + body + footer


class RadarBinParserTests(unittest.TestCase):
    def test_parse_radar_xml_reads_chinese_tags(self) -> None:
        with temp_dir() as tmp:
            xml_path = tmp / "radar_config.xml"
            xml_path.write_text(
                """
                <配置>
                  <扫描起始频率Hz>77000000000</扫描起始频率Hz>
                  <扫描截止频率Hz>81000000000</扫描截止频率Hz>
                  <扫描步进频率Hz>1000000</扫描步进频率Hz>
                  <扫描时间s>0.000352</扫描时间s>
                  <扫频周期s>0.05</扫频周期s>
                  <触发数>2</触发数>
                  <连续单次触发间隔ms>20</连续单次触发间隔ms>
                </配置>
                """,
                encoding="utf-8",
            )

            parsed = parse_radar_xml(xml_path)

            self.assertEqual(parsed["scan_time_s"], 0.000352)
            self.assertEqual(parsed["sweep_period_s"], 0.05)
            self.assertEqual(parsed["trigger_count"], 2)

    def test_parse_radar_recording_outputs_csv_npz_and_metadata(self) -> None:
        with temp_dir() as tmp:
            xml_path = tmp / "radar_config.xml"
            xml_path.write_text(
                """
                <配置>
                  <扫描时间s>0.000352</扫描时间s>
                  <扫频周期s>0.05</扫频周期s>
                </配置>
                """,
                encoding="utf-8",
            )
            bin_path = tmp / "radar_recording.bin"
            bin_path.write_bytes(make_big_frame(offset=0))
            output_dir = tmp / "out"

            result = parse_radar_recording(
                bin_path=bin_path,
                xml_path=xml_path,
                output_dir=output_dir,
                session_id="session_radar",
                radar_start_session_elapsed_s=1.5,
                host_start_epoch_s=10.0,
                host_stop_epoch_s=10.5,
                radar_stop_session_elapsed_s=2.75,
            )

            self.assertEqual(result["one_sweep_points"], 440)
            with (output_dir / "radar_sweeps.csv").open("r", encoding="utf-8", newline="") as handle:
                sweeps = list(csv.DictReader(handle))
            self.assertGreater(len(sweeps), 0)
            self.assertEqual(sweeps[0]["session_id"], "session_radar")
            npz = np.load(output_dir / "radar_complex.npz")
            self.assertEqual(npz["complex_data"].shape[1], 440)
            with (output_dir / "radar_metadata.json").open("r", encoding="utf-8") as handle:
                metadata = json.load(handle)
            self.assertEqual(metadata["one_sweep_points"], 440)
            self.assertEqual(metadata["radar_stop_session_elapsed_s"], 2.75)
            self.assertTrue((output_dir / "radar_timeline.csv").exists())
            with (output_dir / "radar_timeline.csv").open("r", encoding="utf-8", newline="") as handle:
                timeline = list(csv.DictReader(handle))
            self.assertEqual(timeline[-1]["event"], "stop")
            self.assertEqual(timeline[-1]["session_elapsed_s"], "2.75")


if __name__ == "__main__":
    unittest.main()
