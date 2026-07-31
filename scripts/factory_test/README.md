# W2MLaserTOY 工厂测试工具

运行入口：

```powershell
python vendor\ble_cat_laser_toy\scripts\factory_test\main.py
```

兼容旧入口：

```powershell
python vendor\ble_cat_laser_toy\scripts\factory_test_tool.py
```

模块说明：

- `main.py`: 程序入口。
- `main_window.py`: 主窗口 UI、测试流程、CSV/Excel 保存。
- `ble_worker.py`: BLE 扫描、连接、控制命令发送、日志通知。
- `camera_dialog.py`: 摄像头二维码扫描窗口。
- `protocol.py`: CTRL 帧组包/解析、MANUFACTURER_DATA 格式化和匹配。
- `constants.py`: 协议常量、默认结果文件路径。
- `deps.py`: 可选三方依赖导入和错误记录。
- `models.py`: 共享数据结构。

依赖：

```powershell
pip install PyQt5 bleak openpyxl opencv-python pyzbar
```
