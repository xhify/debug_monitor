# ROSBag UI Polish Design

## 目标

优化 ROSBag 管理界面的两个交互点：

1. 模式下拉框在保留英文模式名的同时显示中文解释，降低选择成本。
2. 永久删除不再要求手动输入 session_id，只使用带风险提示的二次确认弹窗。

本次只调整 Windows 上位机界面行为，不改变车端 rosbag 命令协议、不改变 topic preset 内容、不改变远端删除命令。

## 模式选择

`RosbagPanel` 的模式下拉框继续使用现有英文 key 作为内部值：

- `control`
- `fastlio`
- `trajectory_environment`
- `fallback_no_fastlio`
- `full`
- `custom`

下拉框可见文本改为“英文 key + 中文解释”：

- `control - 底盘控制与基础定位`
- `fastlio - FAST-LIO 定位常用`
- `trajectory_environment - 轨迹与环境地图`
- `fallback_no_fastlio - 无 FAST-LIO 备用记录`
- `full - 全量诊断记录`
- `custom - 手动选择 topic`

每个选项的 `userData` 仍保存英文 key。模式切换逻辑读取 `currentData()`，而不是依赖可见文本，因此：

- `ROSBAG_TOPIC_PRESETS` 查找仍使用英文 key。
- `prefix` 自动填充仍使用英文 key。
- `current_config()` 输出不变。
- 现有通过 `findData("fastlio")` 选择模式的测试和调用方式保持有效。

## 永久删除确认

永久删除入口从 `QInputDialog.getText` 改为 `QMessageBox.question`。

确认弹窗显示以下信息：

- `session_id`
- session 状态
- session 大小
- 远程目录
- “永久删除不可恢复，未同步数据会丢失”的风险提示

用户点击“是”后才发送删除请求。用户点击“否”或关闭弹窗时不发送请求。

`_request_delete` 保留现有录制保护：

- 当前车端正在录制且 `session_id` 相同，不允许删除。
- 列表中的 session 状态为 `recording`，不允许删除。

为了减少牵连范围，`delete_requested` 信号暂时保持现有签名 `Signal(str, str)`。删除确认不再依赖用户输入文本，第二个参数继续传递 `session_id` 作为兼容值。后续如果要清理接口，可单独把信号改成 `Signal(str)` 并同步主窗口和测试。

## 测试

更新 `tests/test_rosbag_panel.py`：

- 验证模式下拉框可见文本包含中文解释。
- 验证 `findData("fastlio")`、preset topic 勾选、`current_config()` 仍正常。
- 调整永久删除测试，不再验证输入文本匹配。
- 验证 stopped session 可以直接发出删除信号。
- 验证当前正在录制或列表状态为 `recording` 的 session 仍被拒绝。

## 非目标

- 不新增模式。
- 不调整各模式 topic preset。
- 不改变远程 rosbag JSON 命令。
- 不改变同步、移入回收站、查看详情的行为。
