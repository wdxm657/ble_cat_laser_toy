OTA Protocol
Master端通过L2CAP层的Write Command向slave发命令和数据。

OTA_CMD组成：

OTA的CMD的PDU如下：

OTA Command Payload	
Opcode (2 octet)	invalid data
Opcode：

Opcode	Name
0xFF00	CMD_OTA_VERSION
0xFF01	CMD_OTA_START
0xFF02	CMD_OTA_END
(1) CMD_OTA_VERSION

该命令为获得slave当前firmware版本号的命令，user可以选择使用。在使用该命令时，可通过slave端预留的回调函数来完成firmware版本号的传递。

void blc_ota_registerOtaFirmwareVersionReqCb(ota_versionCb_t cb);
server端在收到CMD_OTA_VERSION命令时会触发该回调函数。

(2) CMD_OTA_START

该命令为OTA升级开始命令，master发这个命令给slave，用来正式启动OTA更新。

(3) CMD_OTA_END

该命令为结束命令，当master确定所有的OTA数据都被slave正确接收后，发送OTA end命令。为了让slave再次确定已经完全收到了master所有数据（double check，加一层保险），OTA end命令后面带4个有效的bytes，后面详细介绍。

-	CMD_data	-
Adr_index_max (2 octets)	Adr_index_max_xor (2 octets)	Reserved
Adr_index_max: 最大的adr_index值

Adr_index_max_xor: Adr_index_max的异或值，供校验使用

Reserved: 保留供以后功能扩展使用

OTA_Data介绍：

-	OTA PDU	-
Adr_Index (2 octets)	Data(16 octets)	CRC (2 octets)
注意：

OTA PDU长度固定大小为16octets

OTA_PDU Format：

前两个byte的范围在firmware_size_k之内时，表示一个OTA数据。由于firmware size不超过128K（0x20000），OTA data packet中每次传送16 byte的firmware数据，使用的adr_index为实际firmware地址除以16的值。adr_index=0，表示OTA数据是firmware地址0x0 ~ 0xF的值；adr_index=1，表示OTA数据是firmware地址0x10 ~ 0x1F的值。最后两个byte是将前面的Adr_Index和Data进行一个CRC_16计算得到第一个CRC的值，slave收到OTA data后，会进行同样的CRC计算，只有两者计算的CRC吻合时，才认为这是一个有效数据。

RF Transfer处理方法
基于BLE link layer RF数据自动ack确保所有数据包不丢的前提，OTA的数据 transform不检查每一个OTA数据是否被ack，即master通过write command发一个ota 数据后，不在软件上检查对方是否有ack信息回复，只要master端硬件TX buffer缓存的待发送数据未达到一定数量，直接将下一笔数据丢进TX buffer。

下面将对OTA具体实现流程进行介绍,阐述整个RF Transform中Salve和Master的交互过程。

OTA具体实现：

master端OTA相关的操作为：

(1) 检测查询是否有触发进入OTA模式的行为，一旦检测到该行为，进入OTA模式。

(2) master向slave传送OTA命令和数据，需要知道slave端当前OTA数据的Attribute的Attribute Handle值。

若user采用事先约定好的方式，直接定义该值；

若没有事先约定好，采用Read By Type Request的方式获得这个Attribute Handle值。

Telink所有BLE SDK 的OTA data的UUID都是16bytes，且永远都是下面这个值：

#define TELINK_SPP_DATA_OTA     {0x12,0x2B,0x0d,0x0c,0x0b,0x0a,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00}
在master的Read By Type Request中将Type设置为这16个bytes的UUID，slave端回复的Read By Type Rsp中可以查到OTA UUID所在的这个Attribute Handle

(3) 获取slave当前firmware版本号，决定是否要继续做OTA更新（若版本已经最新，不需要更新）。这一步为user自己选择是否要做。该BLE SDK不提供具体的版本号获取办法，user可以自行发挥。目前的B80 BLE SDK中并没有实现版本号的传送。user可以使用write cmd的形式通过OTA version cmd向slave传送一个获取OTA version的请求，但是slave那端在收到OTA version请求的时候只提供一个回调函数，user自己在回调函数里想办法将slave端的版本号传送给master（如手动送一个NOTIFY/INDICATE的数据）。

(4) 启动OTA开始的一个计时，后面要不断检测该计时是否超过30秒（这只是个默认参考时间，实际根据user测试的正常OTA需要多少时间后再做评估修改）。

如果超过30秒认为OTA超时失败，因为slave端收到OTA数据后会校验CRC，一旦CRC错误或者出现其他错误（如烧写flash错误），就会认为OTA失败，直接程序重启，此时link layer无法ack master，master端的数据一直发不出去导致超时。

(5) 读取Master flash 0x20018~0x2001b四个字节，确定firmware的size。

这个size是由我们的编译器实现的，假设firmware的size为20k = 0x5000，那么firmware的0x18 ~ 0x1b的值为0x00005000，所以在0x20018 ~ 0x2001b可以读到firmware的大小。
如下表所示的bin文件，0x18 ~ 0x1b内容为0x0000A164，所以大小为0xa164 = 41316Bytes，从0x0000 到 0xa164。
| 偏移地址 | 00 | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 0A | 0B | 0C | 0D | 0E | 0F | ASCII 译文 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **00000000** | 58 | 80 | 00 | 00 | 00 | 00 | 5D | 02 | 4B | 4E | 4C | 54 | 10 | 02 | 88 | 00 | `X.....].KNLT...` |
| **00000010** | E6 | 80 | 00 | 00 | 00 | 00 | 00 | 00 | **64** | **A1** | **00** | **00** | **00** | **00** | **00** | **00** | `........d......` |
| **00000020** | 0C | 64 | 81 | A2 | 22 | 0B | 1A | 40 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | `.d.."..@......` |
| **00000030** | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | `................` |
| **00000040** | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | `................` |
| **00000050** | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | `................` |
| **00000060** | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | `................` |
| **00000070** | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | C0 | 06 | `................` |
......
| 偏移地址 | 00 | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 0A | 0B | 0C | 0D | 0E | 0F | ASCII 译文 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **00000A0D0** | 1E | 00 | 00 | 00 | 02 | 09 | 05 | 00 | 04 | 00 | 01 | 00 | 00 | 00 | 0A | 00 | `................` |
| **00000A0E0** | 00 | 07 | 01 | 00 | 40 | 42 | 0F | 00 | 33 | 21 | 12 | 34 | 29 | 78 | 64 | 54 | `....@B..3!.4)xdT` |
| **00000A0F0** | 56 | 07 | 82 | 58 | 09 | 79 | 86 | 19 | 97 | 74 | 24 | 67 | 62 | 42 | 81 | 14 | `V..X.y...t$gbB..` |
| **00000A100** | 57 | 20 | 42 | 53 | 32 | 37 | 32 | 74 | 02 | 04 | 01 | 03 | 00 | 00 | 00 | 00 | `W BS272t........` |
| **00000A110** | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | `................` |
| **00000A120** | 02 | 15 | 11 | 00 | 06 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | `................` |
| **00000A130** | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 11 | 00 | 00 | 00 | `................` |
| **00000A140** | 00 | 40 | 07 | 00 | FF | FF | FF | FF | 80 | C3 | C9 | 01 | A1 | 22 | 00 | 00 | `.@..........."..` |
| **00000A150** | 06 | 00 | 00 | 00 | FF | FF | FF | FF | FF | FF | FF | FF | FF | FF | FF | FF | `................` |
| **00000A160** | AD | CB | 7A | DE | | | | | | | | | | | | | `..z.` |


(6) 向slave发一个OTA start命令，通知slave进入OTA模式，等待master端的OTA数据。


(7) 从Master flash 0x20000区域开始每次读16个byte的firmware，填入OTA data packet，设置对应的adr_index，并计算CRC值，将packet push到TX fifo，一直到firmware size最后一个16 byte为止，将firmware所有的数据全部发送给slave。

数据发送方法如前面介绍，使用OTA data的格式，有效数据为20 bytes，前两个bytes放adr_index，紧跟16个有效的firmware数据，最后两个是前18个数据的CRC计算值。

注意，如果firmware最后一笔数据不是16字节对齐，需要将剩余的部分按0xff补对齐，计算CRC的时候需要将补充的数据计算进去。

结合上表所示的bin文件来详细介绍OTA数据如何拼装。

第一笔数据：adr_index为0x00 00，16个数据为0x0000 ~ 0x000f地址的值，然后这18个数据计算CRC，假设CRC结果为 0xXYZW，那么20bytes排列为:

0x00 0x00 0x58 0x80 ....省略12个bytes..... 0x88 0x00 0xZW 0xXY

第二笔数据：

0x01 0x00 0xE6 0x80 ....省略12个bytes..... 0x00 0x00 0xJK 0xHI

第三笔数据：

0x02 0x00 0x0C 0x64 ....省略12个bytes..... 0xC0 0x06 0xNO 0xLM

........

倒数第二笔数据：

0x15 0x0a 0x06 0x00 ....省略12个bytes..... 0xff 0xff 0xST 0xPQ

最后一笔数据：

0x16 0x0a 0xad 0xcb 0x7a 0xde 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xWX 0xUV

12个0xff为补齐的数据。

0xad 0xcb 0x7a 0xde为第3个~第6个，它是整个firmware bin的CRC_32校验结果。slave在OTA升级过程中会同步计算接收到的整个bin的CRC_32校验值，在收到最后一包数据包时会将该CRC_32校验值与0xad 0xcb 0x7a 0xde进行比较。

0x16 ~0xff 共18个bytes的CRC计算结果为 0xUVWX。

(8) firmware数据发送完毕后，检查BLE link layer的数据是否已经完全发送出去（因为只有当link layer的数据被slave ack了，才会认为该数据发送成功）。若完全发送出去，master发送一个ota_end命令，通知slave所有数据已发送完毕。

OTA end的packet有效字节设为6个，前两个为0xff02，中间的两个bytes为新的firmware最大的adr_index值（这个是为了让slave端再次确认没有丢掉最后一条或几条OTA数据），最后两个bytes为中间最大的adr_index值的取反，相当于一个简单的校验。OTA end不需要CRC校验。

以上图所示的bin为例，最大的adr_index为0x0a16，其取反值为0xf5e9，最终的OTA end包如上图所示。

(9) 检查master端link layer TX fifo是否为空。若为空，说明之前所有的数据和命令都已成功发送出去，即master端的OTA任务已经全部完成。

CRC_16计算函数见本文档后面的“附录1：crc16算法”。

按照前面所述，Slave端在OTA Attribute中直接调用otaWrite和otaRead即可，master端发送过来的write command命令，BLE协议栈会自动解析并最终调用到otaWrite函数进行处理。

在otaWrite函数里对packet 20 byte的数据进行解析，首先判断是OTA CMD还是OTA data，对OTA cmd进行相应的响应，对OTA数据进行CRC校验并烧写到flash对应位置。

附录1：crc16 算法
unsigned short crc16 (unsigned char *pD, int len)
{
static unsigned short poly[2]={0, 0xa001};
unsigned short crc = 0xffff;
unsigned char ds;
int i,j;

for(j=len; j>0; j--)
    {
unsigned char ds = *pD++;
for(i=0; i<8; i++)
        {
            crc = (crc >> 1) ^ poly[(crc ^ ds ) & 1];
            ds = ds >> 1;
        }
    }

return crc;
}


## 附录2：上位机(APP) OTA 流程总结

### 1. BLE 服务发现

OTA 特性 UUID（Telink 标准）：
```
#define TELINK_SPP_DATA_OTA  {0x12,0x2B,0x0d,0x0c,0x0b,0x0a,0x09,0x08, \
                              0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00}
```
上层 APP 通过该 UUID 找到 OTA Characteristic，使用 **Write Command**（无应答写）发送固件数据。

### 2. 前置条件

- 固件 `.bin` 文件已通过 `tl_check_fw2.exe` 后处理工具（会追加 CRC32 尾部）
- 从 `.bin` 的偏移 `0x18` 处读取 `fw_size`（小端 4 字节，含尾部 4 字节 CRC32）
- `code_size = fw_size - 4`（即实际的固件代码长度）
- `total_packets = ceil(fw_size / 16)`（OTA 数据包总数）
- 固件 CRC32 = 从 `.bin` 尾部 `fw_size-4` 处直接读取 4 字节，
  由编译后工具 `tl_check_fw2.exe` 计算并追加到 .bin 文件末尾，
  **上位机不要自己计算**，直接从文件读取

### 3. 完整流程

```
步骤1: 发送 CMD_OTA_START (0xFF01)
       PDU: [0x01, 0xFF]
       通知 Slave 进入 OTA 模式
       → 等待约 2s 让 Slave 完成 Flash 擦除

步骤2: 发送所有 OTA 数据包 (共 total_packets 包)
       每包 PDU 共 20 字节:
         [Adr_Index(2)] [Data(16)] [CRC16(2)]

       Adr_Index = 包序号（从 0 开始），对应固件偏移 = Adr_Index × 16
       Data      = 从 .bin 文件读取的 16 字节固件数据
       CRC16     = 前 18 字节（Adr_Index + Data）的 CRC-16 校验

       最后一包特殊处理：
       Data[0:4] = 固件 CRC32（来自 .bin 尾部）
       Data[4:16] = 0xFF（补齐字节）
       
       关键参数：
       · 使用 Write Command（response=False），不等待 ACK
       · 每包间隔约 2ms
       · 总时长 ≈ total_packets × 2ms

步骤3: 等待 TX 缓冲区排空（约 1s）

步骤4: 发送 CMD_OTA_END (0xFF02)
       PDU: [0x02, 0xFF]
            [max_adr_index(2)]   ← 小端，total_packets - 1
            [~max_adr_index(2)]  ← 取反校验
       示例: max_adr_index=0x1F1F → xor=0xE0E0
       → 等待约 0.5s 后设备重启
```

### 4. 数据包构造示例

| 包类型 | Adr_Index | Data(16) | CRC16 |
|--------|-----------|----------|-------|
| 第 1 包 | 0x0000 | bin[0x00:0x10] | CRC16(adr_index + data) |
| 第 2 包 | 0x0001 | bin[0x10:0x20] | CRC16(adr_index + data) |
| ... | ... | ... | ... |
| 最后一包 | `total_packets-1` | [CRC32(4)] + [0xFF × 12] | CRC16(adr_index + data) |

### 5. CRC16 算法（每包校验）

```python
def crc16_ota(data: bytes) -> int:
    """匹配附录1的 C 实现：poly 0xA001, init 0xFFFF"""
    crc = 0xFFFF
    for b in data:
        ds = b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if (crc ^ ds) & 1 else 0)
            ds >>= 1
    return crc & 0xFFFF
```
