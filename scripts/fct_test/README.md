# B80 FCT 上位机

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
5. 上位机进入 APP 后会常开 BLE 扫描，并在“BLE 设备”页面展示所有名称为 `W2MLaserTOY` 的设备及 Manufacturer Specific Data。Bleak 会把 Manufacturer Data 拆成 Company ID 和数据体，上位机会按小端序把 Company ID 拼回前两个字节后再显示和匹配。
6. 点击“读取 UID”后，上位机会用 UID 匹配当前 BLE 设备列表中的 Manufacturer Specific Data，匹配设备会高亮显示，状态区域同步显示匹配结果。
7. 按键和 USB 插拔事件会自动显示在状态区域。

上位机会在终端打印原始 `TX ...` 和 `RX ...` 十六进制日志，便于串口调试。

# B80 FCT 量产测试

## 1. 串口接口

- 默认参数：`115200 8N1`
- 不使用回环串口 `PD5/PD6`

## 2. 数据帧格式

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

CRC 计算范围为 `version` 到 `payload` 的全部字节，不包含 `55 AA` 和 CRC 本身。帧总长度为 `10 + payload_len`。

## 3. 命令协议

### GPIO 单路控制：`0x10`

负载为 `gpio_id level`：

- `gpio_id`：GPIO 对照表中的编号，范围 `0~9`
- `level`：`00` 关闭，`01` 开启

成功时返回 `status=00`，并额外发送一个 `0x82` GPIO 事件。

### GPIO 全部控制：`0x11`

负载只有一个字节：

```text
level
```

一次设置全部 10 路 GPIO，仅返回一帧响应，不发送 10 条单路命令。

### 读取 Flash UID：`0x20`

无负载。响应负载为：

```text
status uid[16]
```

### 读取电池 ADC：`0x21`

无负载。响应负载为：

```text
status battery_mv_u16
```

电池电压按 `app_adc_dbg.c` 的分压比例计算，即 ADC 电压乘以 `6.6`，单位为 mV。

### 读取 NTC ADC：`0x22`

无负载。响应负载为：

```text
status ntc_mv_u16
```

NTC 值为平均后的 NTC ADC 引脚电压，单位为 mV。

### 进入低功耗：`0x30`

无负载。固件先返回成功响应，然后进入深度睡眠。


## 4. 响应状态码

响应负载的第一个字节为状态：

| 值 | 含义 |
| ---: | --- |
| `00` | 成功 |
| `01` | 负载长度错误 |
| `02` | 不支持的命令 |
| `03` | 参数错误 |

## 5. 设备事件

事件帧的 `type` 为 `03`：

| 事件编号 | 名称 | 负载 |
| ---: | --- | --- |
| `80` | ADC 状态 | `battery_mv_u16 ntc_mv_u16 key_state` |
| `81` | 按键 | `key_state`，`01` 按下，`00` 松开 |
| `82` | 单路 GPIO | `gpio_id level` |
| `83` | UID 事件 | `uid[16]` |
| `84` | USB 检测 | `usb_state`，`01` 插入，`00` 拔出 |

按键和 USB 检测均使用约 20 ms 去抖，只在状态变化时上报。USB 检测脚为 `GPIO_PA2`，高电平表示插入。

## 6. TX/RX 示例

下面示例中的 UID 是演示数据，CRC 已按本协议计算。

### 单路 GPIO：打开 PC0 / 1D

```text
TX 12B: 55 AA 01 01 10 00 02 00 00 01 53 EF
RX 12B: 55 AA 01 03 82 02 02 00 00 01 15 5D
RX 11B: 55 AA 01 02 10 00 01 00 00 9A 21
```

第一帧 RX 是 GPIO 状态事件，第二帧 RX 是命令响应。

### 一次打开全部 GPIO

```text
TX 11B: 55 AA 01 01 11 01 01 00 01 67 EE
RX 11B: 55 AA 01 02 11 01 01 00 00 A6 1D
```

### 读取 UID

```text
TX 10B: 55 AA 01 01 20 02 00 00 96 0A
RX 27B: 55 AA 01 02 20 02 11 00 00 50 32 00 42 33 37 30 33 F7 03 17 00 03 01 44 2D C9 88
```

### 读取电池和 NTC ADC

```text
TX 10B: 55 AA 01 01 21 03 00 00 C6 36
RX 13B: 55 AA 01 02 21 03 03 00 00 D8 0E 28 4C

TX 10B: 55 AA 01 01 22 04 00 00 77 B3
RX 13B: 55 AA 01 02 22 04 03 00 00 72 06 65 9D
```

上例中电池值为 `0x0ED8 = 3800 mV`，NTC 值为 `0x0672 = 1650 mV`。

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