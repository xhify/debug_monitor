import json
import os
import shutil
import sys
import unittest
import uuid
from pathlib import Path

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from protocol import CMD_SET_PID_BOTH, ParamFrame, build_pid_command
from widgets.command_panel import CommandPanel


class CommandPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        base_dir = Path(__file__).resolve().parent / "_tmp_command_panel"
        base_dir.mkdir(exist_ok=True)
        self._tmpdir = base_dir / f"case_{uuid.uuid4().hex}"
        self._tmpdir.mkdir()

    def tearDown(self) -> None:
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def test_sync_mode_disables_b_inputs_and_uses_a_values(self) -> None:
        panel = CommandPanel(settings_path=self._tmpdir / "pid_config.json")
        sent_commands: list[bytes] = []
        panel.command_ready.connect(sent_commands.append)

        panel._pid_mode_sync.setChecked(True)
        panel._a_kp_spin.setValue(11.0)
        panel._a_ki_spin.setValue(2.0)
        panel._a_kd_spin.setValue(3.0)
        panel._b_kp_spin.setValue(44.0)

        self.assertFalse(panel._b_kp_spin.isEnabled())
        self.assertFalse(panel._b_ki_spin.isEnabled())
        self.assertFalse(panel._b_kd_spin.isEnabled())

        panel._send_pid_both_btn.click()

        self.assertEqual(
            sent_commands[-1],
            build_pid_command(CMD_SET_PID_BOTH, 11.0, 2.0, 3.0),
        )

    def test_fill_params_does_not_overwrite_editable_pid_inputs(self) -> None:
        panel = CommandPanel(settings_path=self._tmpdir / "pid_config.json")
        panel._pid_mode_independent.setChecked(True)
        panel._a_kp_spin.setValue(1.1)
        panel._a_ki_spin.setValue(1.2)
        panel._a_kd_spin.setValue(1.3)
        panel._b_kp_spin.setValue(2.1)
        panel._b_ki_spin.setValue(2.2)
        panel._b_kd_spin.setValue(2.3)

        frame = ParamFrame(
            A_kp=10.0,
            A_ki=20.0,
            A_kd=30.0,
            B_kp=40.0,
            B_ki=50.0,
            B_kd=60.0,
            rc_speed=70.0,
            limt_max_speed=80.0,
            smooth_MotorStep=90.0,
        )
        panel.fill_params(frame)

        self.assertAlmostEqual(panel._a_kp_spin.value(), 1.1)
        self.assertAlmostEqual(panel._a_ki_spin.value(), 1.2)
        self.assertAlmostEqual(panel._a_kd_spin.value(), 1.3)
        self.assertAlmostEqual(panel._b_kp_spin.value(), 2.1)
        self.assertAlmostEqual(panel._b_ki_spin.value(), 2.2)
        self.assertAlmostEqual(panel._b_kd_spin.value(), 2.3)

        panel._load_device_a_btn.click()
        panel._load_device_b_btn.click()

        self.assertAlmostEqual(panel._a_kp_spin.value(), 10.0)
        self.assertAlmostEqual(panel._a_ki_spin.value(), 20.0)
        self.assertAlmostEqual(panel._a_kd_spin.value(), 30.0)
        self.assertAlmostEqual(panel._b_kp_spin.value(), 40.0)
        self.assertAlmostEqual(panel._b_ki_spin.value(), 50.0)
        self.assertAlmostEqual(panel._b_kd_spin.value(), 60.0)

    def test_pid_state_persists_to_json_and_restores(self) -> None:
        settings_path = self._tmpdir / "pid_config.json"
        panel = CommandPanel(settings_path=settings_path)
        panel._pid_mode_independent.setChecked(True)
        panel._a_kp_spin.setValue(101.0)
        panel._a_ki_spin.setValue(102.0)
        panel._a_kd_spin.setValue(103.0)
        panel._b_kp_spin.setValue(201.0)
        panel._b_ki_spin.setValue(202.0)
        panel._b_kd_spin.setValue(203.0)

        saved = json.loads(settings_path.read_text(encoding="utf-8"))
        self.assertEqual(saved["pid_mode"], "independent")
        self.assertEqual(saved["motor_a"]["kp"], 101.0)
        self.assertEqual(saved["motor_b"]["kd"], 203.0)

        restored = CommandPanel(settings_path=settings_path)
        self.assertTrue(restored._pid_mode_independent.isChecked())
        self.assertAlmostEqual(restored._a_kp_spin.value(), 101.0)
        self.assertAlmostEqual(restored._a_ki_spin.value(), 102.0)
        self.assertAlmostEqual(restored._a_kd_spin.value(), 103.0)
        self.assertAlmostEqual(restored._b_kp_spin.value(), 201.0)
        self.assertAlmostEqual(restored._b_ki_spin.value(), 202.0)
        self.assertAlmostEqual(restored._b_kd_spin.value(), 203.0)
