# B80 FCT 上位机

安装依赖：

```bash
python -m pip install pyserial bleak PyQt5
```

运行：

```bash
python -m scripts.fct_test.main
```

固件串口协议：`115200 8N1`，帧格式为：

```text
55 AA version type cmd seq payload_len(u16 LE) payload crc16(u16 LE)
```

GPIO 编号依次为 `PC0, PC1, PC2, PC3, PB7, PB6, PB5, PB4, PC6, PD7`。
