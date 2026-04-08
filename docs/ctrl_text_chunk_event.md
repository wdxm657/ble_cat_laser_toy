## Ctrl 通用文本/字节流上报（EVENT: `CTRL_CMD_TEXT_CHUNK` = 0x40）

### 背景

由于 BLE 默认 MTU=23 时，单次 NOTIFY 的 ATT Value 有效负载只有 20 字节，而本工程 `CTRL_TX_MAX_LEN` 固定为 20，
因此 `CTRL` 协议单帧（含 6 字节头）最多只能携带 14 字节 payload。为了把 **任意长度的文本/字节流**上报到上位机，
采用 `CTRL_CMD_TEXT_CHUNK (0x40)` 进行**分片**传输。

该机制可用于：

- 将固件 `tl_printf/LOG_*` 输出镜像到上位机显示
- 上报调试用的任意 bytes（上位机可按 UTF-8 解码或按 hex 显示）

### 传输层

- **链路**：自定义 BLE Service，Ctrl TX characteristic（NOTIFY）
- **帧头**（与现有 `app_ctrl_send()` 一致）：
  - `version` (u8) = `CTRL_PROTO_VERSION` (0x01)
  - `msgType` (u8) = `CTRL_MSG_TYPE_EVENT` (0x03)
  - `cmdId` (u8) = `CTRL_CMD_TEXT_CHUNK` (0x40)
  - `seq` (u8) = 0..255（用于事件序号，不参与分片重组）
  - `payloadLen` (u16 LE)

### payload 格式（设备 → 上位机）

payload 字节布局：

| 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | `transferId` | u8 | 本次长文本/字节流的传输编号（0..255），新传输应使用新 id |
| 1 | `chunkIndex` | u8 | 分片序号，从 0 开始递增 |
| 2 | `chunkTotal` | u8 | 分片总数（>=1） |
| 3 | `dataLen` | u8 | 本片数据长度（0..`CTRL_TEXT_CHUNK_DATA_MAX`） |
| 4.. | `data` | bytes | 本片数据内容 |

约束：

- 必须满足：`6(CTRL头) + 4(分片字段) + dataLen <= CTRL_TX_MAX_LEN(20)`
- 在本工程里：`CTRL_TEXT_CHUNK_DATA_MAX = 10`

### 重组规则（上位机）

- 当收到 `chunkIndex==0` 的分片时，认为开始一条新传输，清空该 `transferId` 的缓存
- 依次拼接各分片 `data`
- 当收到 `chunkIndex == chunkTotal-1` 且已累计接收 `chunkTotal` 片时，认为重组完成
- 重组完成后的 bytes：
  - **尝试按 UTF-8 解码**显示（失败则以 hex 显示）
  - 建议按行（`\n`）切分追加到 UI 的 log window

### 兼容性说明

- 现有雷达调试 `CTRL_CMD_RADAR_PRED_DEBUG (0x57)` 不变
- 上位机旧版本忽略 `cmdId != 0x57` 的 EVENT；升级后即可显示 `0x40` 文本

