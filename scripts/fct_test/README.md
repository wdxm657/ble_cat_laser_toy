# B80 FCT 上位机

## 安装和运行

安装依赖：

```bash
python -m pip install pyserial bleak PyQt5
```

运行：

```bash
python -m scripts.fct_test.main
```

操作顺序：

1. 选择设备串口和 `115200` 波特率。
2. 点击“打开串口”。
3. 使用 GPIO 单路按钮或“全部 GPIO 开/关”按钮控制输出。
4. 点击“读取电池 ADC”和“读取 NTC ADC”获取最新缓存值。
5. 点击“读取 UID”和“读取固件版本”获取设备信息。
6. 上位机启动后常开 BLE 扫描，并在“BLE 设备”页面展示所有名称为 `W2MLaserTOY` 的设备及 Manufacturer Specific Data。
7. 读取 UID 后，上位机会用 UID 匹配当前 BLE 设备列表中的 Manufacturer Specific Data，匹配设备会高亮显示，状态区域同步显示匹配结果。
8. 按键和 USB 插拔事件会自动显示在状态区域。

上位机会在终端打印原始 `TX ...` 和 `RX ...` 十六进制日志，便于串口调试。

## 串口接口

- 默认参数：`115200 8N1`

## 帧格式

所有多字节数值均为小端序：

```text
55 AA version type cmd seq payload_len_lo payload_len_hi payload... crc_lo crc_hi
```

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| `55 AA` | 2 | 固定帧头 |
| `version` | 1 | 当前为 `01` |
| `type` | 1 | `01` 命令，`02` 响应，`03` 设备事件 |
| `cmd` | 1 | 命令或事件编号 |
| `seq` | 1 | 上位机命令序号或设备事件序号 |
| `payload_len` | 2 | 负载长度，小端序 |
| `payload` | N | 负载数据 |
| `crc16` | 2 | Modbus CRC16，小端序 |

CRC 计算范围为 `version` 到 `payload` 的全部字节，不包含帧头 `55 AA` 和 CRC 本身。帧总长度为 `10 + payload_len`。

## 命令协议

| 命令 | 名称 | TX 负载 | RX 负载 |
| ---: | --- | --- | --- |
| `0x10` | GPIO 单路控制 | `gpio_id level` | `status` |
| `0x11` | GPIO 全部控制 | `level` | `status` |
| `0x20` | 读取 Flash UID | 无 | `status uid[16]` |
| `0x21` | 读取电池 ADC | 无 | `status battery_mv_u16` |
| `0x22` | 读取 NTC ADC | 无 | `status ntc_mv_u16` |
| `0x23` | 读取固件版本 | 无 | `status major minor patch` |
| `0x30` | 进入低功耗 | 无 | `status` |
| `0x40` | 读取状态 | 无 | 固件返回 `0x80` ADC 状态事件 |

`status` 是响应状态码，固定放在所有响应负载的第 1 个字节。只有 `status=00` 时，后续负载字段才表示有效数据。

状态码定义：

| 值 | 含义 |
| ---: | --- |
| `00` | 成功 |
| `01` | 负载长度错误 |
| `02` | 不支持的命令 |
| `03` | 参数错误 |

## 设备事件

事件帧的 `type` 为 `03`：

| 事件 | 名称 | 负载 |
| ---: | --- | --- |
| `0x80` | ADC 状态 | `battery_mv_u16 ntc_mv_u16 key_state` |
| `0x81` | 按键 | `key_state`，`01` 按下，`00` 松开 |
| `0x82` | 单路 GPIO | `gpio_id level` |
| `0x83` | UID 事件 | `uid[16]` |
| `0x84` | USB 检测 | `usb_state`，`01` 插入，`00` 拔出 |

ADC 不自动上传。电池 ADC、NTC ADC 通过按钮主动读取。按键和 USB 检测均使用约 20 ms 去抖，只在状态变化时上报。

## GPIO 对照表

| `gpio_id` | 芯片 GPIO | Alias |
| ---: | --- | --- |
| 0 | `PC0` | `1A` |
| 1 | `PC1` | `1B` |
| 2 | `PC2` | `1C` |
| 3 | `PC3` | `1D` |
| 4 | `PB7` | `2D` |
| 5 | `PB6` | `2C` |
| 6 | `PB5` | `2B` |
| 7 | `PB4` | `2A` |
| 8 | `PC6` | `Laser` |
| 9 | `PD7` | `5v+` |

协议中的 `level=1` 表示开启，`level=0` 表示关闭。

## BLE UID 匹配

上位机只展示名称为 `W2MLaserTOY` 的 BLE 设备。Bleak 会把 Manufacturer Specific Data 拆成 Company ID 和数据体，上位机会按小端序把 Company ID 拼回前两个字节后再显示和匹配。

示例：

```text
Bleak: company_id=0x5042 data=32 30 33 33 37 17 00 F7 03 44 56 03 01 78
完整 Manufacturer Specific Data: 42 50 32 30 33 33 37 17 00 F7 03 44 56 03 01 78
```

读取到的 UID 若完整包含于上述 Manufacturer Specific Data，则对应表格行高亮。

## TX/RX 示例

### GPIO 单路控制：打开 `gpio_id=0`

```text
TX 12B: 55 AA 01 01 10 00 02 00 00 01 53 EF
RX 12B: 55 AA 01 03 82 02 02 00 00 01 15 5D
RX 11B: 55 AA 01 02 10 00 01 00 00 9A 21
```

第一帧 RX 是 GPIO 状态事件，第二帧 RX 是命令响应。

### GPIO 全部打开

```text
TX 11B: 55 AA 01 01 11 01 01 00 01 67 EE
RX 11B: 55 AA 01 02 11 01 01 00 00 A6 1D
```

### 读取 UID

```text
TX 10B: 55 AA 01 01 20 02 00 00 96 0A
RX 27B: 55 AA 01 02 20 02 11 00 00 50 32 00 42 33 37 30 33 F7 03 17 00 03 01 44 2D C9 88
```

### 读取电池 ADC

```text
TX 10B: 55 AA 01 01 21 03 00 00 C6 36
RX 13B: 55 AA 01 02 21 03 03 00 00 D8 0E 28 4C
```

上例电池值为 `0x0ED8 = 3800 mV`。

### 读取 NTC ADC

```text
TX 10B: 55 AA 01 01 22 04 00 00 77 B3
RX 13B: 55 AA 01 02 22 04 03 00 00 72 06 65 9D
```

上例 NTC 值为 `0x0672 = 1650 mV`。

### 读取固件版本

```text
TX 10B: 55 AA 01 01 23 06 00 00 D7 8F
RX 14B: 55 AA 01 02 23 06 04 00 00 01 00 00 4C EB
```

上例响应负载为 `00 01 00 00`：`status=00` 表示成功，版本号为 `1.0.0`。

### 按键按下和 USB 插入

```text
RX 11B: 55 AA 01 03 81 00 01 00 01 A7 ED
RX 11B: 55 AA 01 03 84 01 01 00 01 6A 11
```

### 进入低功耗

```text
TX 10B: 55 AA 01 01 30 05 00 00 23 0B
RX 11B: 55 AA 01 02 30 05 01 00 00 1B 2A
```
