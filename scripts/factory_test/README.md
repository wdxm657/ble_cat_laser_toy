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
- `qr_generator.py`: 连接成功后生成 `300x300` 的 MANUFACTURER_DATA 二维码图片。
- `make_qr_print_md.py`: 将二维码图片按 A4 纵向每行 10 种、每种只排版一次到 Markdown；图片固定为 `70x70`，不显示文字。
- `constants.py`: 协议常量、默认结果文件路径。
- `deps.py`: 可选三方依赖导入和错误记录。
- `models.py`: 共享数据结构。

依赖：

```powershell
pip install PyQt5 bleak openpyxl opencv-python pyzbar qrcode pillow
```

设备连接成功后，工具会将二维码 PNG 保存到 `scripts/factory_test/qrcodes/`。
同时会将连接设备的唯一 `MANUFACTURER_DATA` 记录到 `scripts/factory_test/manufacturer_data_records.csv`，重复连接同一个 `MANUFACTURER_DATA` 不会重复生成二维码或写入 CSV。二维码文件名以生成时间开头，按文件名排序即可得到生成顺序。CSV 字段为：生成时间、设备名称、MAC/地址、MANUFACTURER_DATA。

生成 A4 打印用 Markdown：

```powershell
python scripts\factory_test\make_qr_print_md.py
```

默认输出 `scripts/factory_test/qr_print.md`。也可以指定图片目录和输出文件：

```powershell
python scripts\factory_test\make_qr_print_md.py `
  --input-dir scripts\factory_test\qrcodes `
  --output scripts\factory_test\qr_print.md
```
