# Dual Motor PID Persistence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add dual-motor PID editing, sync/independent send modes, and automatic local persistence without letting parameter queries overwrite the editable values.

**Architecture:** Keep command building in `protocol.py` unchanged and evolve `CommandPanel` into a stateful widget with separate editable A/B PID state plus last-known device PID state. Persist the editable state in a small JSON file under `settings/`, while `ParamPanel` remains the display for device-reported values.

**Tech Stack:** Python 3.11, PySide6, json, unittest

---

### Task 1: Add regression tests for the new PID widget behavior

**Files:**
- Create: `D:\radar\car\debug_monitor\tests\test_command_panel.py`

- [ ] **Step 1: Write the failing tests**

```python
def test_sync_mode_disables_b_inputs_and_uses_a_values():
    ...

def test_fill_params_does_not_overwrite_editable_pid_inputs():
    ...

def test_pid_state_persists_to_json_and_restores():
    ...
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_command_panel -v`
Expected: FAIL because the widget has one shared PID input set and no persistence.

- [ ] **Step 3: Implement only the minimum surface needed to satisfy the test names**

```python
class CommandPanel(QWidget):
    def current_pid_values_int(self) -> tuple[int, int, int]:
        ...
```

- [ ] **Step 4: Run the test again**

Run: `python -m unittest tests.test_command_panel -v`
Expected: still FAIL, but now on missing dual-motor state details.

- [ ] **Step 5: Commit**

```bash
git add tests/test_command_panel.py
git commit -m "test: cover dual motor pid widget behavior"
```

### Task 2: Add a tiny persistence helper for PID settings

**Files:**
- Create: `D:\radar\car\debug_monitor\src\pid_settings.py`
- Test: `D:\radar\car\debug_monitor\tests\test_command_panel.py`

- [ ] **Step 1: Implement JSON load/save helpers**

```python
DEFAULT_PID_SETTINGS = {
    "pid_mode": "sync",
    "motor_a": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
    "motor_b": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
}
```

- [ ] **Step 2: Save under the project settings directory**

```python
SETTINGS_DIR = Path.cwd() / "settings"
SETTINGS_FILE = SETTINGS_DIR / "pid_config.json"
```

- [ ] **Step 3: Verify the persistence test still fails only on widget integration**

Run: `python -m unittest tests.test_command_panel -v`
Expected: FAIL on `CommandPanel` not yet using the helper.

- [ ] **Step 4: Commit**

```bash
git add src/pid_settings.py tests/test_command_panel.py
git commit -m "feat: add pid settings persistence helper"
```

### Task 3: Rebuild the PID section in CommandPanel

**Files:**
- Modify: `D:\radar\car\debug_monitor\src\widgets\command_panel.py`
- Modify: `D:\radar\car\debug_monitor\src\main_window.py`
- Modify: `D:\radar\car\debug_monitor\tests\test_main_window_replay.py`
- Test: `D:\radar\car\debug_monitor\tests\test_command_panel.py`

- [ ] **Step 1: Replace the single PID input set with A/B groups**

```python
self._pid_mode_sync = QRadioButton("两轮同步")
self._pid_mode_independent = QRadioButton("独立配置")
self._a_kp_spin = ...
self._b_kp_spin = ...
```

- [ ] **Step 2: Make sync mode disable B edits without clearing values**

```python
for widget in (self._b_kp_spin, self._b_ki_spin, self._b_kd_spin):
    widget.setEnabled(not sync_mode)
```

- [ ] **Step 3: Add send buttons for sync / A / B**

```python
sync_btn.clicked.connect(self._send_pid_both_from_a)
send_a_btn.clicked.connect(self._send_pid_a)
send_b_btn.clicked.connect(self._send_pid_b)
```

- [ ] **Step 4: Keep recording filename compatibility**

```python
def current_pid_values_int(self) -> tuple[int, int, int]:
    return int(self._a_kp_spin.value()), int(self._a_ki_spin.value()), int(self._a_kd_spin.value())
```

- [ ] **Step 5: Run focused tests**

Run: `python -m unittest tests.test_command_panel tests.test_main_window_replay -v`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add src/widgets/command_panel.py src/main_window.py tests/test_main_window_replay.py tests/test_command_panel.py
git commit -m "feat: add dual motor pid editing modes"
```

### Task 4: Separate editable PID state from device-reported PID state

**Files:**
- Modify: `D:\radar\car\debug_monitor\src\widgets\command_panel.py`
- Test: `D:\radar\car\debug_monitor\tests\test_command_panel.py`

- [ ] **Step 1: Store the latest device-reported PID values separately**

```python
self._device_pid = {
    "motor_a": {"kp": frame.A_kp, "ki": frame.A_ki, "kd": frame.A_kd},
    "motor_b": {"kp": frame.B_kp, "ki": frame.B_ki, "kd": frame.B_kd},
}
```

- [ ] **Step 2: Change `fill_params()` so it no longer overwrites editable PID inputs**

```python
def fill_params(self, frame: ParamFrame) -> None:
    self._device_pid = ...
    self._rc_speed_spin.setValue(frame.rc_speed)
    ...
```

- [ ] **Step 3: Add explicit load buttons for device A/B PID**

```python
load_a_btn.clicked.connect(self._load_device_pid_a)
load_b_btn.clicked.connect(self._load_device_pid_b)
```

- [ ] **Step 4: Run focused tests**

Run: `python -m unittest tests.test_command_panel -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/widgets/command_panel.py tests/test_command_panel.py
git commit -m "feat: separate device pid from editable pid state"
```

### Task 5: Wire in automatic persistence and verify the full suite

**Files:**
- Modify: `D:\radar\car\debug_monitor\src\widgets\command_panel.py`
- Modify: `D:\radar\car\debug_monitor\tests\test_command_panel.py`

- [ ] **Step 1: Save on PID edits and mode changes**

```python
spin.valueChanged.connect(self._save_pid_settings)
self._pid_mode_sync.toggled.connect(self._save_pid_settings)
```

- [ ] **Step 2: Load persisted settings during widget initialization**

```python
settings = load_pid_settings()
self._apply_pid_settings(settings)
```

- [ ] **Step 3: Run the targeted suite**

```bash
$env:PYTHONPATH='src'; $env:QT_QPA_PLATFORM='offscreen'; python -m unittest tests.test_command_panel tests.test_main_window_replay tests.test_analysis_panel -v
```

- [ ] **Step 4: Run the broader verification suite**

```bash
$env:PYTHONPATH='src'; $env:QT_QPA_PLATFORM='offscreen'; python -m unittest tests.test_data_buffer tests.test_command_panel tests.test_main_window_replay tests.test_analysis_panel tests.test_recording_session tests.test_replay_data tests.test_analytics -v
```

- [ ] **Step 5: Commit**

```bash
git add src/widgets/command_panel.py src/pid_settings.py tests/test_command_panel.py
git commit -m "feat: persist dual motor pid settings locally"
```
