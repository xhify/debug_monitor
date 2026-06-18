# Application Icon Design

## 目标

使用用户提供的机器人调试图标作为 Debug Monitor 的默认应用图标，并同时准备 Windows `.ico` 资源，便于后续打包 `.exe` 时复用。

## 资源文件

新增资源目录：

- `src/assets/app_icon.png`
- `src/assets/app_icon.ico`

`app_icon.png` 保存用户提供的原始 PNG 图标，用于 PySide6 运行时加载。`app_icon.ico` 由同一 PNG 生成，包含常用 Windows 图标尺寸，供 PyInstaller 或其他打包工具使用。

推荐 `.ico` 尺寸：

- 16x16
- 32x32
- 48x48
- 64x64
- 128x128
- 256x256

## 运行时加载

在 `src/main.py` 中加载应用图标：

- 创建 `QApplication` 后设置 `app.setWindowIcon(icon)`。
- 创建 `MainWindow` 后设置 `window.setWindowIcon(icon)`。

图标路径通过 helper 解析，兼容两种运行方式：

- 源码运行：资源路径相对于 `src/main.py`。
- PyInstaller 打包运行：如果存在 `sys._MEIPASS`，优先从 `_MEIPASS/assets/app_icon.png` 读取。

如果图标文件缺失或加载失败，程序继续启动，不阻断主窗口显示。

## 打包约定

本次不新增正式打包脚本，但生成的 `.ico` 可直接用于后续 Windows 打包：

```powershell
pyinstaller --icon src/assets/app_icon.ico src/main.py
```

如果未来新增 `.spec` 文件，应把 `src/assets` 作为数据资源加入，并把 `src/assets/app_icon.ico` 作为 Windows executable icon。

## 测试

新增或更新测试覆盖：

- 源码运行路径下能解析到 `src/assets/app_icon.png`。
- 创建的 `QIcon` 非空。
- `.ico` 文件存在，且可由 Qt 作为图标加载。

测试不启动完整 GUI 主循环，只验证资源路径和 `QIcon` 加载。

## 非目标

- 不修改窗口布局。
- 不修改 rosbag、串口、记录、回放等业务功能。
- 不新增正式 PyInstaller `.spec` 文件。
- 不替换已有临时 `dist/` 或 `.test_tmp/` 产物。
