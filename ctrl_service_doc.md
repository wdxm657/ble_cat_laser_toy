## 通用控制 Service 协议说明（给 APP 开发）

#### 设备名称 SincereSPNC

### 1. GATT 结构概览

- **Service：Custom Control Service**
- UUID（16 字节）：`00 A0 0D 0C 0B 0A 09 08 07 06 05 04 03 02 01 00`()
- **Characteristic 1：Ctrl RX（APP → 设备）**
- UUID：`01 A0 0D 0C 0B 0A 09 08 07 06 05 04 03 02 01 00`
- 属性：Read | Write | Write Without Response
- User Description：`Ctrl RX`
- 用途：APP 向设备发送所有指令 / 配置信息。
- **Characteristic 2：Ctrl TX（设备 → APP）**
- UUID：`02 A0 0D 0C 0B 0A 09 08 07 06 05 04 03 02 01 00`
- 属性：Read | Notify
- User Description：`Ctrl TX`
- 需先向其 CCC 写 `0x0001` 以开启 Notify。
- 用途：设备向 APP 返回响应、状态、事件。

ATT Value 有效负载最大长度由 MTU 决定：`maxLen = MTU - 3`。当前固件出于 MCU RAM 考量，仅实现默认 **MTU=23** 的场景，即：**单帧最大 20 字节**。若 MTU 被协商为更大，仍建议单帧控制在 20 字节以内。

---

### 2. 通用帧格式（Ctrl RX / Ctrl TX 共用）

所有命令、响应、事件统一使用如下帧格式：

```
byte0 : version      协议版本，目前固定 0x01
byte1 : msgType      消息类型
byte2 : cmdId        命令 ID
byte3 : seq          序号（请求-响应对应）
byte4 : payloadLen L 负载长度低字节
byte5 : payloadLen H 负载长度高字节
byte6.. : payload    具体负载，长度 = payloadLen
```

- **msgType**
- `0x01`：命令（APP → 设备）
- `0x02`：响应（设备 → APP）
- **payloadLen**
- 小端：`len = byte4 + (byte5 << 8)`；
- **单帧长度上限**（当前实现）：`6 + len ≤ 20` 字节。超出 20 字节的“业务数据”需要在应用层拆分为多帧。

APP 写 Ctrl RX、设备通过 Ctrl TX Notify 返回的内容均从 `byte0` 开始。

---

### 3. 错误码（响应 payload[0]）

设备响应帧的 `payload[0]` 用作状态码：

- `0x00`：OK（成功）
- `0x01`：LEN_ERROR（长度错误）
- `0x02`：UNSUPPORTED_CMD（不支持的命令）
- `0x03`：PARAM_ERROR（参数错误）
- `0x04`：INTERNAL_ERROR（内部错误）

建议统一响应负载结构：

```
payload[0] = status      // 上述错误码
payload[1] = errDetail   // 目前为 0，预留
... 其余字段依具体命令而定
```

---

### 4. 已实现命令列表及完整帧结构

以下每个命令/响应都给出**完整帧字段含义**（从 `byte0` 开始），APP 可直接按此封包 / 解包。

#### 4.1 LED 控制（CMD = 0x10）

**请求帧（APP → 设备）**

```
byte0 : 0x01           // version
byte1 : 0x01           // msgType = CMD
byte2 : 0x10           // cmdId = LED_CTRL
byte3 : seq            // 0~255, 由 APP 决定
byte4 : 0x02           // payloadLen L = 2
byte5 : 0x00           // payloadLen H = 0
byte6 : ledId          // 0:全部, 1:蓝, 2:绿, 3:白, 4:红
byte7 : state          // 0:关, 1:开

01 01 10 01 02 00 00 01
```

**响应帧（设备 → APP）**

```
byte0 : 0x01           // version
byte1 : 0x02           // msgType = RSP
byte2 : 0x10           // cmdId = LED_CTRL
byte3 : seq            // 与请求一致
byte4 : 0x02           // payloadLen L = 2
byte5 : 0x00           // payloadLen H = 0
byte6 : status         // 0x00 成功，其它为错误码
byte7 : 0x00           // errDetail，当前为 0
```

#### 4.2 二轴电机控制（MOTOR_CTRL，CMD = 0x20）

该命令用于控制 Pan/Tilt 两轴步进电机，支持：查询当前角度坐标信息。

- 轴定义：`0 = Pan`，`1 = Tilt`

**请求帧（APP → 设备）**

```
byte0 : 0x01           // version
byte1 : 0x01           // msgType = CMD
byte2 : 0x20           // cmdId = MOTOR_CTRL
byte3 : seq
byte4 : payloadLen L
byte5 : payloadLen H
01 01 20 01 06 00
byte6 : op
byte7.. : parameters by op
```

`op` 定义：

1. `op = 0x02`：查询当前角度坐标信息

**示例**：

```
01 01 20 01 01 00 02
```

**响应帧（设备 → APP）**

- `op = 0x02` 查询响应：

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x20           // cmdId = MOTOR_CTRL
byte3 : seq
byte4 : 0x06           // payloadLen = 6
byte5 : 0x00
byte6 : status
byte7 : 0x02           // 回显 op
byte8 : x_L            // 当前坐标 x (mm)
byte9 : x_H
byte10: y_L            // 当前坐标 y (mm)
byte11: y_H
```

#### 4.3 四方向电机控制（MOTOR_DIR_CTRL，CMD = 0x22）

用于 APP 端通过 **上/下/左/右** 按键控制电机运动到机械限位（阈值由固件限位决定），可选速度档位。

- 方向定义：
- `0x00`：上（TILT+）
- `0x01`：下（TILT-）
- `0x02`：左（PAN-）
- `0x03`：右（PAN+）
- 速度档位：
- `0`：默认速度（与原 MOTOR_CTRL 的默认一致）
- `1`：慢
- `2`：中
- `3`：快

**请求帧（APP → 设备）**

```
byte0 : 0x01           // version
byte1 : 0x01           // msgType = CMD
byte2 : 0x22           // cmdId = MOTOR_DIR_CTRL
byte3 : seq
byte4 : 0x03           // payloadLen = 3
byte5 : 0x00
byte6 : op             // 0x00=stop, 0x01=move
byte7 : direction      // 上下左右
byte8 : speedLevel     // 0/1/2/3
```

**示例**：

```
（停止）
01 01 22 01 03 00 00 01 02
（下转，中速）
01 01 22 01 03 00 01 01 02
（右转，中速）
01 01 22 01 03 00 01 03 02
01 01 20 01 01 00 02
```

**响应帧（设备 → APP）**（立即响应）

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x22           // cmdId = MOTOR_DIR_CTRL
byte3 : seq
byte4 : 0x03           // payloadLen = 3
byte5 : 0x00
byte6 : status
byte6 : direction
byte7 : op             // 回显 0x00/0x01
```

**事件帧（设备 → APP）**

1. 到达机械限位时：

```
byte0 : 0x01
byte1 : 0x03           // msgType = EVENT
byte2 : 0x22           // cmdId = MOTOR_DIR_CTRL
byte3 : seq            // 事件序号
byte4 : 0x02           // payloadLen = 2
byte5 : 0x00
byte6 : 0x01           // evtId=到达限位
byte7 : direction
```

1. 边界点设置模式下触发“越界保护”时：

```
byte0 : 0x01
byte1 : 0x03           // msgType = EVENT
byte2 : 0x22           // cmdId = MOTOR_DIR_CTRL
byte3 : seq            // 事件序号
byte4 : 0x04           // payloadLen = 4
byte5 : 0x00
byte6 : 0x02           // evtId=边界越界保护触发
byte7 : direction      // 0x00=上,0x01=下,0x02=左,0x03=右
byte8 : pointIndex     // 当前正在设置的点
byte9 : limitPointIndex// 约束参考点
```

1. **仅边界标定模式**下，俯仰“上”达到当前安装高度对应的地面投射最大距离（固件按 6 m 水平距离推算的俯仰上限）时：

```
byte0 : 0x01
byte1 : 0x03           // msgType = EVENT
byte2 : 0x22           // cmdId = MOTOR_DIR_CTRL
byte3 : seq            // 事件序号
byte4 : 0x04           // payloadLen = 4
byte5 : 0x00
byte6 : 0x03           // evtId=安装高度俯仰上限（6 m 投射几何上限）
byte7 : direction      // 固定为 0x00（上）
byte8 : tilt_deg10_L   // 当前俯仰角 deg×10，小端 s16
byte9 : tilt_deg10_H
```

说明：在边界点设置模式中，设备会在 PAN/TILT 两方向做约束，即将越界时自动停止电机并上报 evtId=0x02。另：evtId=0x03 仅在边界标定流程中、且已设置安装高度时可能上报（俯仰“上”超过按 6 m 地面投射推算的上限时）；设备停止俯仰并锁角，普通四向控制（非标定）不触发 0x03。

约束规则：

- 右移（0x03）：LU(0) 受 RU(1) 约束；LD(3) 受 RD(2) 约束。
- 左移（0x02）：RU(1) 受 LU(0) 约束；RD(2) 受 LD(3) 约束。
- 上移（0x00）：LD(3) 受 LU(0) 约束；RD(2) 受 RU(1) 约束。
- 下移（0x01）：LU(0) 受 LD(3) 约束；RU(1) 受 RD(2) 约束。

#### 4.4 设置电机原点（MOTOR_SET_ZERO，CMD = 0x21）

用于把“当前角度”设为逻辑 0°，便于现场校准。

**请求帧（APP → 设备）**

```
byte0 : 0x01           // version
byte1 : 0x01           // msgType = CMD
byte2 : 0x21           // cmdId = MOTOR_SET_ZERO
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : axis           // 0:Pan, 1:Tilt, 0xFF:两轴同时置零
01 01 21 01 01 00 ff
```

说明：当电机先运动到某个角度后，发送该命令即可把该位置作为新原点。之后查询角度和目标角度控制都基于该新原点。

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x21           // cmdId = MOTOR_SET_ZERO
byte3 : seq
byte4 : 0x04           // payloadLen = 4
byte5 : 0x00
byte6 : status
byte7 : axis           // 回显请求 axis
byte8 : 0x00           // 预留
byte9 : 0x00           // 预留
```

#### 4.5 电源开关（POWER_CTRL，CMD = 0x12）

用途：APP 控制自动逗宠开关。`1` 开启，`0` 关闭。

限制/冷却规则：

- 当设备处于“开机后”状态时，在首次开机后的 `30s` 冷却期内禁止发送 `on=0` 关机请求。
- 当设备处于“关机后”状态时，在最近一次关机后的 `30s` 冷却期内禁止重复发送 `on=0` 关机请求。
- 若请求被拒绝：
  - 冷却限制：`status` 为非 `0`（当前实现使用 `CTRL_STATUS_INTERNAL_ERROR`），并在 `reason` 中区分：
    - `reason=0x02`：开机失败（开机请求时处于离上次关机不足 30s 的冷却中）
    - `reason=0x03`：关机失败（关机请求时处于离上次开机不足 30s 的冷却中）
    - `byte7 (on_effective)` 中回显设备当前真实电源状态
  - 低电量禁止开机：当 `on=1` 且 `电量<15% 且未充电` 时，`status=CTRL_STATUS_REJECT_ERROR (0x05)`，`reason=0x01`（LOW_BATTERY），并回显 `on_effective=0`。
  - 电池温度过高禁止开机：当 `on=1` 且 NTC 有效且 `温度>70°C` 时，`status=CTRL_STATUS_REJECT_ERROR (0x05)`，`reason=0x04`（BATTERY_TEMP_HIGH），并回显 `on_effective=0`。此时固件已关闭充电开关（`CHARGE_SWITCH`）。

**示例**：

```
（开启）
01 01 12 01 01 00 01
（关闭）
01 01 12 01 01 00 00
```

**请求帧（APP → 设备）**

```
byte0 : 0x01           // version
byte1 : 0x01           // msgType = CMD
byte2 : 0x12           // cmdId = POWER_CTRL
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : on             // 0x00=关, 0x01=开
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x12           // cmdId = POWER_CTRL
byte3 : seq
byte4 : 0x03           // payloadLen = 3
byte5 : 0x00
byte6 : status
byte7 : on_effective  // 回显 0x00/0x01（是否实际生效）
byte8 : reason         // 0x00=NONE, 0x01=LOW_BATTERY, 0x02=POWER_ON_COOLDOWN_30S, 0x03=POWER_OFF_COOLDOWN_30S, 0x04=BATTERY_TEMP_HIGH
```

**设备主动上报（EVENT，温度过高强制关机）**

当设备处于开机状态且 NTC 有效、`温度>70°C` 时，固件关闭充电开关并强制关机；若 APP 已对 Ctrl TX 写入 CCC `0x0001` 开启 Notify，设备发送：

```
byte0 : 0x01
byte1 : 0x03           // msgType = EVENT
byte2 : 0x12           // cmdId = POWER_CTRL
byte3 : seq            // 设备自增
byte4 : 0x03           // payloadLen = 3
byte5 : 0x00
byte6 : 0x05           // status = REJECT_ERROR
byte7 : 0x00           // on_effective = 关
byte8 : 0x04           // reason = BATTERY_TEMP_HIGH
```

#### 4.6 设备状态查询（STATUS_GET，CMD = 0x13）

用途：获取设备开关机、逗宠区域是否设置、安装高度、充电状态以及狩猎模式状态（设置中/狩猎中/待机/休眠）。

**请求帧（APP → 设备）**

```
byte0 : 0x01           // version
byte1 : 0x01           // msgType = CMD
byte2 : 0x13           // cmdId = STATUS_GET
byte3 : seq
byte4 : 0x00           // payloadLen = 0
byte5 : 0x00
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x13           // cmdId = STATUS_GET
byte3 : seq
byte4 : 0x0A           // payloadLen = 10
byte5 : 0x00
byte6 : status
byte7 : power_on       // 0x00=关, 0x01=开
byte8 : play_zone_set  // 0x00=未设置, 0x01=已设置
byte9 : height_lo      // 高度 低位
byte10: height_hi      // 高度 高位
byte11: charging       // 充电状态
byte12: setting_mode   // 设置中：0x00/0x01
byte13: hunting_mode   // 狩猎中：0x00/0x01
byte14: standby_mode   // 待机：0x00/0x01
byte15: sleeping_mode  // 休眠：0x00/0x01
```

四种状态互斥：`setting_mode` > `hunting_mode` > `standby_mode` > `sleeping_mode`，任意时刻最多只有一个为 `1`。

- 设备开机后处于逗宠等待状态，检测到目标自动开始狩猎（`hunting_mode=1`）。
- 15 秒无目标进入待机（`standby_mode=1`），激光和电机关闭，雷达保持检测。
- 15 秒目标停在猎物点附近进入 30 秒休眠（`sleeping_mode=1`），雷达+激光+电机关闭。
- 达到狩猎完成次数或累计活跃时长上限后进入长期休眠（`sleeping_mode=1`）。
- 狩猎设置模式下 `setting_mode=1`。

**事件帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x03           // msgType = EVENT
byte2 : 0x13           // cmdId = STATUS_GET
byte3 : seq
byte4 : 0x0A           // payloadLen = 10
byte5 : 0x00
byte6 : status
byte7 : power_on       // 0x00=关, 0x01=开
byte8 : play_zone_set  // 0x00=未设置, 0x01=已设置
byte9 : height_lo      // 高度 低位
byte10: height_hi      // 高度 高位
byte11: charging       // 充电状态
byte12: setting_mode   // 设置中：0x00/0x01
byte13: hunting_mode   // 狩猎中：0x00/0x01
byte14: standby_mode   // 待机：0x00/0x01
byte15: sleeping_mode  // 休眠：0x00/0x01
```

#### 4.7 设置设备时间（TIME_SET，CMD = 0x32）

用途：APP 发送 Unix 时间戳（秒）和时区，设备侧更新当前时间。

**示例**：

```
01 01 32 01 05 00 80 D3 27 66 00
```

**请求帧（APP → 设备）**

```
byte0 : 0x01           // version
byte1 : 0x01           // msgType = CMD
byte2 : 0x32           // cmdId = TIME_SET
byte3 : seq
byte4 : 0x05           // payloadLen = 5
byte5 : 0x00
byte6 : epochSec_L0    // Unix 时间戳（秒）低字节
byte7 : epochSec_L1
byte8 : epochSec_L2
byte9 : epochSec_L3    // Unix 时间戳（秒）高字节
byte10: tz
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x32           // cmdId = TIME_SET
byte3 : seq
byte4 : 0x02           // payloadLen = 2
byte5 : 0x00
byte6 : status
byte7 : 0x00
```

#### 4.8 逗宠记录主动上报与 ACK（PLAY_RECORD_GET，CMD = 0x33）

用途：设备主动上报"完整逗宠记录"（即狩猎结果记录），APP 成功接收后发送 ACK 通知设备清理记录。  
说明：完整记录指同时有 `start_sec` 和 `end_sec`；仅有开始时间（`end_sec = 0xFFFFFFFF`）的进行中记录不会上报。
每条记录包含狩猎结果：`0=未完成`、`1=完成`、`2=捕猎成功`。
为满足 BLE 单帧 20 字节（`CTRL_TX_MAX_LEN`）限制，`end_sec` 压缩为 `duration_sec = end_sec − start_sec`（u16 LE），APP 侧恢复：`end_sec = start_sec + duration_sec`。

**触发时机（设备 → APP）**

- 每次新增一条完整记录（会话结束）时，若 BLE 已连接，主动上报未 ACK 的完整记录。
- BLE 首次连接成功或重连成功后，若存在未 ACK 的完整记录，主动补发。

**长度约束（与实现对齐）**

- 控制面单帧总长须满足 `6 + payloadLen ≤ CTRL_TX_MAX_LEN`（当前 **20** 字节，见 §1 / `app_ctrl.h`）。
- 由于 20 字节限制，本事件的 `end_sec` 字段压缩为 `duration_sec = end_sec - start_sec`（u16 LE），APP 端恢复：`end_sec = start_sec + duration_sec`。
- 本事件 **payload 固定 13 字节**（`payloadLen = 0x000D`），总长 **19 字节**。

**主动上报帧（设备 → APP，EVENT）**

帧头（6 字节）同 §1，以下为 **payload 内偏移**（从首字节算起，**小端 LE**）：

| payload 偏移 | 长度 | 类型   | 说明                                                                                                                                                                                     |
| ------------ | ---- | ------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0            | 1    | u8     | `status`                                                                                                                                                                                 |
| 1            | 1    | u8     | `total`：本批次完整记录条数（与 ACK 前设备内待上报条数一致）                                                                                                                             |
| 2            | 1    | u8     | `index`：当前条在批次中的序号 `0 .. total-1`                                                                                                                                             |
| 3            | 4    | u32 LE | `start_sec`：逗宠段开始 Unix 秒                                                                                                                                                          |
| 7            | 2    | u16 LE | `duration_sec`：**逗宠持续秒数**。`duration_sec = end_sec − start_sec`（u16，饱和 65535）。APP 恢复 `end_sec = start_sec + duration_sec`。替代原 4 字节 `end_sec` 以符合 20 字节单帧限制 |
| 9            | 2    | u16 LE | `motion_sec`：**累计运动时长**（秒）。由段内毫秒累计 **四舍五入**（`(ΣΔt_ms+500)/1000`）得到；毫秒累计规则见下 **「运动统计」**；**u16 上报饱和 65535**                                  |
| 11           | 1    | u8     | `avg_speed_cm_s`：**平均速度**（cm/s）。**时间加权**：`round( Σ(v×Δt_ms) / Σ(Δt_ms) )`，其中 `v` 为相邻轨迹点弦速（见 **「运动统计」**）；**u8 上报饱和 255**                            |
| 12           | 1    | u8     | `result`：**狩猎结果**。`0=未完成(HUNT_RESULT_INCOMPLETE)`、`1=完成(HUNT_RESULT_COMPLETE)`、`2=捕猎成功(HUNT_RESULT_SUCCESS)`                                                            |

**运动统计（与固件 `RadarMotionCachePush` / `radar_play_on_cache_displacement_ms` 对齐）**

- 仅当设备处于**工作模式**且该段逗宠**进行中**时计入本段。
- 雷达轨迹缓存每次 **Push** 新点前，若新点 `(x,y)` 与缓存内 **newest** 点任一坐标不同：取两点 `tick` 差，`Δt_us = (tick_now − tick_prev) >> 4`（与全文件 `clock_time` 用法一致），`Δt_ms = Δt_us / 1000`，**最小 1ms**；**`Δt_ms` 超过 `RADAR_MOTION_STEP_DT_MS_MAX`（固件常量，当前 120000）则截断**，将本次 **`Δt_ms` 累加到段内运动毫秒**。
- 弦速（cm/s）：`v = √(Δx²+Δy²)_mm × 100 / Δt_ms`（`Δx/Δy` 为相对 newest 的毫米位移）；位移小于 0.5mm 时 `v` 按 0，仍累加 **`Δt_ms`**。
- 段结束时写入 flash / 上报：`motion_sec` 为秒，`avg_speed_cm_s` 为上述加权平均（u16 存 flash，经 BLE 再截断为 u8）。

**整帧 19 字节示例（6 字节头 + 13 字节 payload；`payloadLen` 小端为 `0x0D 0x00`）**

```
byte0 : 0x01
byte1 : 0x03           // msgType = EVENT
byte2 : 0x33           // cmdId = PLAY_RECORD_GET
byte3 : seq
byte4 : 0x0D           // payloadLen L0 = 13
byte5 : 0x00           // payloadLen L1 = 0
byte6 : status         // payload[0]
byte7 : total          // payload[1]
byte8 : index          // payload[2]
byte9 : start_sec_L0   // payload[3..6]
byte10: start_sec_L1
byte11: start_sec_L2
byte12: start_sec_L3
byte13: duration_L0    // payload[7..8]  duration_sec = end_sec - start_sec (u16 LE)
byte14: duration_L1
byte15: motion_sec_L0  // payload[9..10]
byte16: motion_sec_L1
byte17: avg_speed_cm_s // payload[11]
byte18: result         // payload[12] 狩猎结果: 0=未完成 1=完成 2=捕猎成功
```

**ACK 请求帧（APP → 设备，CMD）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x33           // cmdId = PLAY_RECORD_GET
byte3 : seq
byte4 : 0x00           // payloadLen = 0（推荐）
byte5 : 0x00
```

兼容：固件也接受 `payloadLen = 1` 的 ACK 帧。

**ACK 响应帧（设备 → APP，RSP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x33           // cmdId = PLAY_RECORD_GET
byte3 : seq
byte4 : 0x02           // payloadLen = 2
byte5 : 0x00
byte6 : status         // 0x00=成功，其它为错误码
byte7 : remain_complete// 0:无剩余完整记录, 1:仍有完整记录
```

**清理策略**

- 设备仅在收到有效 ACK 后清理完整记录，并持久化到 flash。
- 进行中记录（仅开始时间）始终保留，不会因 ACK 被清理。

#### 4.9 读取 SN（UID_GET，CMD = 0x34）

用途：APP 读取设备 Flash UID（16 字节）。只发一次命令，设备连续返回 2 次响应，每次 8 字节。

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x34           // cmdId = UID_GET
byte3 : seq
byte4 : 0x00           // payloadLen = 0
byte5 : 0x00
```

**示例**：

```
01 01 34 01 00 00
```

**响应帧（设备 → APP）**（共返回 2 次）

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x34           // cmdId = UID_GET
byte3 : seq
byte4 : 0x0A           // payloadLen = 10
byte5 : 0x00
byte6 : status         // 0x00 成功，其它为错误码
byte7 : part           // 0: UID[0..7], 1: UID[8..15]
byte8..byte15 : uid8   // 8 字节 UID 分片
```

#### 4.10 设置安装高度（RADAR_CONFIG_SET_HEIGHT，CMD = 0x50）

用途：APP 设置雷达安装高度，设备立即应用。

**请求帧（APP → 设备）**

```
byte0 : 0x01           // version
byte1 : 0x01           // msgType = CMD
byte2 : 0x50           // cmdId = RADAR_CONFIG_SET_HEIGHT
byte3 : seq
byte4 : 0x02           // payloadLen = 2
byte5 : 0x00
byte6 : height_L       // s16, mm (小端)
byte7 : height_H
```

**示例**：

```
（设置高度 2500mm）
01 01 50 01 02 00 C4 09
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x50           // cmdId = RADAR_CONFIG_SET_HEIGHT
byte3 : seq
byte4 : 0x02           // payloadLen = 2
byte5 : 0x00
byte6 : status         // 0x00=成功，其它为错误码
byte7 : 0x00           // reserved
```

设备行为：
1. 将高度限制在 800~2500mm 范围内
2. 立即应用该高度到雷达配置

#### 4.11 文本分片传输（TEXT_CHUNK，CMD = 0x40）

用于发送长文本，设备端最多缓存 `CTRL_TEXT_MAX_TOTAL_LEN = 100` 字节文本，每帧最多携带 `CTRL_TEXT_CHUNK_DATA_MAX = 10` 字节纯文本。

**请求帧（APP → 设备）**

```
byte0 : 0x01            // version
byte1 : 0x01            // msgType = CMD
byte2 : 0x40            // cmdId = TEXT_CHUNK
byte3 : seq
byte4 : payloadLen L    // = 4 + dataLen  (dataLen <= 10)
byte5 : payloadLen H
byte6 : transferId      // 传输ID, 0~255，用于标识一次完整文本传输
byte7 : chunkIndex      // 当前分片序号，从 0 开始递增
byte8 : chunkTotal      // 本次传输的总分片数
byte9 : dataLen         // 本分片文本字节数，<= 10
byte10..(9+dataLen) : text data bytes (不要求每片结尾有 '\0')
```

约束：

- `6 + payloadLen ≤ 20`；
- `payloadLen = 4 + dataLen`，因此 `dataLen ≤ 10`；
- 所有分片的 `transferId` 和 `chunkTotal` 在一次传输过程中必须一致。

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02            // msgType = RSP
byte2 : 0x40            // cmdId = TEXT_CHUNK
byte3 : seq
byte4 : 0x03            // payloadLen = 3
byte5 : 0x00
byte6 : status          // 0x00 成功，其他错误码
byte7 : transferId      // 回显
byte8 : chunkIndex      // 回显
```

设备端行为说明：

- 收到 `chunkIndex == 0` 的首片时，会重置内部文本缓冲区，并记录本次的 `transferId` 和 `chunkTotal`；
- 每收到一片：
- 若 `4 + dataLen > payloadLen` 或 `dataLen > 10`，返回 `LEN_ERROR`；
- 若累计长度超过 100 字节，返回 `LEN_ERROR` 并丢弃本次传输；
- 否则将本片数据追加到内部缓冲区，并返回 `status = OK`；
- 当累计接收分片数达到 `chunkTotal` 时，认为文本传输完成：
- 会在本地补一个结尾 `'\0'` 方便调试日志；
- 当前实现通过日志输出完整文本：`[CTRL][TEXT] id=.. len=.. text="..."`；
- 然后清空缓冲区，等待下一次传输。

APP 侧发送完整长文本的推荐流程：

1. 假设要发送的字符串为 UTF-8 字节流 `text[]`，长度为 `N`；
2. 选择一个 `transferId`（如本地计数器取模 256）；
3. 将 `N` 按 `dataLen ≤ 10` 切分为若干块，得到 `chunkTotal`；
4. 对于每个 `chunkIndex`（0..chunkTotal-1）：

- 

- 构造请求帧，填入对应片段的数据；
- 写入 Ctrl RX；
- 等待 Ctrl TX 返回的响应，确认该片 `status == 0x00`；

1. 当所有分片均成功，应答 `status=0x00` 后，本次文本在设备端即可视为“已完整接收并处理”。

#### 4.12 解绑复位

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x56           // cmdId = CTRL_CMD_RADAR_RESET_FLASH_CONFIG
byte3 : seq
byte4 : 0x00           // payloadLen = 0
byte5 : 0x00
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x56           // cmdId = CTRL_CMD_RADAR_RESET_FLASH_CONFIG
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : status
```

设备行为：清除FLASH中的高度和坐标信息和逗宠记录

#### 4.13 设备软复位（DEVICE_REBOOT，CMD = 0x5A）

用途：APP 请求设备执行 MCU 软复位（重启）。  
注意：设备会尽量先回复一帧 RSP，但随后会很快复位，因此 **APP 不应依赖一定能收到响应**；链路会断开，设备会重新广播/可被重新连接。

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x5A           // cmdId = DEVICE_REBOOT
byte3 : seq
byte4 : 0x00           // payloadLen = 0
byte5 : 0x00
```

**响应帧（设备 → APP）**（best-effort）

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x5A           // cmdId = DEVICE_REBOOT
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : status         // 0x00=OK，其它为错误码
```

设备行为：回复 RSP 后在 `app_ctrl_task()` 中延时约 120ms 触发 `start_reboot()`。

#### 4.14 电池电量使用电池服务特帧读取

Battery Service
UUID:0000180F-0000-1000-8000-00805F9B34FB
Battery Level
UUID:00002A19-0000-1000-8000-00805F9B34FB

#### 4.15 狩猎游戏设置（CMD = 0x60 ~ 0x67）

用途：APP 配置狩猎游戏的各项参数，包括猎物点、狩猎时长、狩猎次数、休眠时长等。

---

##### 4.15.1 进入狩猎设置模式（HUNT_SETTINGS_ENTER，CMD = 0x60）

进入后光斑移动到当前猎物点，设备进入设置模式（`setting_mode=1`），停止自动狩猎。

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x60           // cmdId = HUNT_SETTINGS_ENTER
byte3 : seq
byte4 : 0x00           // payloadLen = 0
byte5 : 0x00
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x60           // cmdId = HUNT_SETTINGS_ENTER
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : status
```

---

##### 4.15.2 退出狩猎设置模式（HUNT_SETTINGS_EXIT，CMD = 0x61）

退出时可以选择应用或丢弃设置。

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x61           // cmdId = HUNT_SETTINGS_EXIT
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : apply          // 0x00=丢弃，0x01=应用全部设置
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x61           // cmdId = HUNT_SETTINGS_EXIT
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : status
```

---

##### 4.15.3 猎物点随机移动（HUNT_PREY_RANDOM，CMD = 0x62）

在设置模式下，控制猎物点在水平±60°、俯仰15°~30°范围内随机移动。停止随机移动时光斑停在当前位置，可用 `HUNT_PREY_SET` 设为猎物点。

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x62           // cmdId = HUNT_PREY_RANDOM
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : start          // 0x00=停止, 0x01=开始随机移动
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x62           // cmdId = HUNT_PREY_RANDOM
byte3 : seq
byte4 : 0x02           // payloadLen = 2
byte5 : 0x00
byte6 : status
byte7 : start          // 回显 0x00/0x01
```

---

##### 4.15.4 设置当前云台位置为猎物点（HUNT_PREY_SET，CMD = 0x63）

将当前云台角度设为猎物点（覆盖原值）。需先在设置模式下通过方向键或随机移动将光斑移动到目标位置。

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x63           // cmdId = HUNT_PREY_SET
byte3 : seq
byte4 : 0x00           // payloadLen = 0
byte5 : 0x00
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x63           // cmdId = HUNT_PREY_SET
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : status
```

---

##### 4.15.5 设置单次狩猎时长（HUNT_SET_DURATION，CMD = 0x64）

设置单次狩猎的时长（秒），范围 10~600 秒，默认 60 秒。

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x64           // cmdId = HUNT_SET_DURATION
byte3 : seq
byte4 : 0x02           // payloadLen = 2
byte5 : 0x00
byte6 : duration_L     // u16 LE 时长(秒)
byte7 : duration_H
```

**示例**：

```
（设置 90 秒）
01 01 64 01 02 00 5A 00
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x64           // cmdId = HUNT_SET_DURATION
byte3 : seq
byte4 : 0x03           // payloadLen = 3
byte5 : 0x00
byte6 : status
byte7 : applied_L      // 实际生效值低位
byte8 : applied_H      // 实际生效值高位
```

---

##### 4.15.6 设置狩猎次数（HUNT_SET_COUNT，CMD = 0x65）

设置达成多少次狩猎后进入休眠。最大值 = `ceil(600 / 单次狩猎时长)`。默认 3 次。

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x65           // cmdId = HUNT_SET_COUNT
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : count          // 狩猎次数（u8）
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x65           // cmdId = HUNT_SET_COUNT
byte3 : seq
byte4 : 0x02           // payloadLen = 2
byte5 : 0x00
byte6 : status
byte7 : applied        // 实际生效值（u8）
```

---

##### 4.15.7 设置休眠时长（HUNT_SET_SLEEP_DURATION，CMD = 0x66）

设置达成狩猎次数后休眠的时长（分钟），范围 1~20 分钟，默认 3 分钟。

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x66           // cmdId = HUNT_SET_SLEEP_DURATION
byte3 : seq
byte4 : 0x01           // payloadLen = 1
byte5 : 0x00
byte6 : minutes        // 休眠时长（分钟，u8）
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x66           // cmdId = HUNT_SET_SLEEP_DURATION
byte3 : seq
byte4 : 0x02           // payloadLen = 2
byte5 : 0x00
byte6 : status
byte7 : applied        // 实际生效值（u8）
```

---

##### 4.15.8 获取当前狩猎设置（HUNT_SETTINGS_GET，CMD = 0x67）

获取当前配置的单次狩猎时长、狩猎次数和休眠时长。无需 payload。

**请求帧（APP → 设备）**

```
byte0 : 0x01
byte1 : 0x01           // msgType = CMD
byte2 : 0x67           // cmdId = HUNT_SETTINGS_GET
byte3 : seq
byte4 : 0x00           // payloadLen = 0
byte5 : 0x00
```

**示例**：

```
01 01 67 01 00 00
```

**响应帧（设备 → APP）**

```
byte0 : 0x01
byte1 : 0x02           // msgType = RSP
byte2 : 0x67           // cmdId = HUNT_SETTINGS_GET
byte3 : seq
byte4 : 0x05           // payloadLen = 5
byte5 : 0x00
byte6 : status
byte7 : duration_L     // 单次狩猎时长(秒) u16 LE
byte8 : duration_H
byte9 : count          // 狩猎次数 u8
byte10: sleep_min      // 休眠时长(分钟) u8
```

---

##### 狩猎设置流程示例

1. APP 发送 `HUNT_SETTINGS_ENTER(0x60)` → 光斑移动到当前猎物点
2. APP 发送 `HUNT_PREY_RANDOM(0x62) start=1` → 光斑开始随机移动
3. 用户观察光斑位置合适时，发送 `HUNT_PREY_RANDOM(0x62) start=0` → 停止移动
4. APP 发送 `HUNT_PREY_SET(0x63)` → 当前云台位置设为猎物点
5. APP 发送 `HUNT_SET_DURATION(0x64)` 设置时长为 90s
6. APP 发送 `HUNT_SET_COUNT(0x65)` 设置次数为 3
7. APP 发送 `HUNT_SET_SLEEP_DURATION(0x66)` 设置休眠 5 分钟
8. APP 发送 `HUNT_SETTINGS_EXIT(0x61) apply=1` → 应用全部设置，返回自动狩猎

---

### 5. 长数据与 APP 侧处理建议

1. **单帧最大长度**

- 

- 当前实现按默认 MTU=23 设计：`6 + payloadLen ≤ 20`；
- 如果 APP 写入或设备试图发送超过 20 字节的帧，设备侧会返回长度错误（不会静默截断）。

1. **需要发送超过 20 字节的业务数据时**

- 

- 必须在应用层做**分片协议设计**，例如：
- 自定义一个新的命令（例如 `DATA_CHUNK`，cmdId 由双方约定），payload 中增加：
- `chunkIndex`（当前分片序号）、`chunkTotal`（总分片数）、`offset`、`chunkLen` 等字段；
- 每一帧的总长度仍然遵守 `6 + payloadLen ≤ 20`；
- 设备端在对应 handler 中按 `chunkIndex/offset` 进行重组或流式处理。
- 本协议文件只定义了**单帧格式**，具体的分片/重组规则由业务双方根据需求单独约定并在固件中实现对应命令 handler。

---

### 6. APP 侧典型调用流程

1. 扫描并连接设备；
2. 发现包含上述 UUID 的 **Custom Control Service**；
3. 找到 `Ctrl TX` 特征，向其 CCC 写入 `0x0001` 以启用 Notify；
4. 每次发送命令时：

- 

- 维护一个本地 `seq`（0~255 循环）；
- 按「帧格式 + 命令定义」组装数据后，写入 `Ctrl RX`；

1. 在 `Ctrl TX` 的 Notify 回调中：

- 

- 解析 `version / msgType / cmdId / seq / payloadLen`；
- 校验 `version == 0x01`，`msgType == 0x02`；
- 根据 `cmdId + seq` 找到对应的请求，读取 `payload[0]` 判断是否成功，然后按各命令的 payload 协议解析剩余内容。

如需新增业务，只需约定新的 `cmdId` 和对应的 `payload` 格式，固件会按相同头部格式进行收发。