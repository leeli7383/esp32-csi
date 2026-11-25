# 原生 USB / UART0 原始 CSI 与 FreeRTOS 调度

更新：2026-10-19。适用于 esp32_send 和 esp32_recv，ESP-IDF 5.5.3。

## 接口与采集

当前两块板使用不同的电脑连接接口，配置如下：

| 工程 | 当前接口 | build_flags | 端口配置 |
| --- | --- | --- | --- |
| esp32_recv | CP210x USB 转串口 / UART0 | `-DCSI_STREAM_USE_USB=0` | upload_port / monitor_port 均为 COM9 |
| esp32_send | 原生 USB Serial/JTAG | `-DCSI_STREAM_USE_USB=1` | 按该板实际枚举端口选择 |

接收端改为 CP210x 后必须重新编译、烧录，固件才会把数据发送到 UART0。关闭 XCOM 后上传，完成后重新打开 COM9：921600、8N1、无流控、RTS/DTR 关闭。应看到 `#CSI_STATUS transport=UART0`；发送 `TEXT` 加换行查看十进制 CSI。只打开/修改 VS Code 工程不会改变板上已烧录固件。更换 USB 接口或电脑后，如果 COM 号改变，需要同步更新端口配置。

原生 USB 模式使用 **ESP32-C6 原生 USB Serial/JTAG**，设备 VID:PID 为 `303A:1001`，Windows 通常显示“USB 串行设备”。此前检查到的原生 USB 端口为 COM6；端口号会随电脑/接口变化。原生 USB 使用 GPIO12(D-)/GPIO13(D+)，与 UART0 是不同的硬件通道。之前固件只向 UART0 写入，所以在原生 USB 上只能看到 ROM 启动信息。

`platformio.ini` 的 `build_flags = -DCSI_STREAM_USE_USB=1` 选择原生 USB。只有实际使用 CH340/CP210x 等 USB 转串口桥时才改为 `0` 并重新编译、烧录；此时输出 UART0，TX=GPIO16、RX=GPIO17，921600 baud、8N1、无流控。两个接口不会同时输出。GPIO10/11 仍用于 LED。

原生 USB 不通过 UART 波特率分频传输，XCOM 继续设置921600即可；UART0 模式则必须匹配921600。两个节点要各自烧录对应工程并同时供电。只接一块板到电脑、另一块独立供电也可采集。

### 在 XCOM 查看

1. 关闭 XCOM 的串口，再分别烧录修改后的两个工程；打开源代码不会更新已烧录的固件。
2. 重新打开正确端口，921600、8N1、无流控，关闭 RTS/DTR 自动控制。
3. 默认是 BINARY 模式，但即使没有 CSI，也应每秒看到一行 `#CSI_STATUS ...`，其中 `local` 用于确认节点身份。
4. 要直接查看数值，用文本发送 **`TEXT` 并追加换行**。如果 XCOM 没有追加换行选项，可在十六进制发送模式下发送 `54 45 58 54 0A`。
5. 切换成功后看到 `mode=TEXT`，收到对端 CSI 时会出现：

```text
CSI_DATA,seq=...,t_us=...,peer=...,rssi=...,channel=11,len=...,first_word_invalid=0,data=[-12,3,...]
```

这里的省略号仅用于示例，实际行会包含全部原始 int8 数值。TEXT 模式只是十进制编码，不做算法处理；主要用于目视调试，无 CRC 且开销大。正式采集使用 BINARY。

发送 `BINARY` + 换行恢复原 `CSI1` 二进制帧；发送 `STATUS` + 换行立即打印状态。命令区分大小写。模式只保存在 RAM 中，复位后回到 BINARY。不要把字符串 `\n` 的两个字符当作换行发送。

### 状态信息如何判断

`#CSI_STATUS` 每秒一行，只由传输任务在 CSI 帧之间写出，不会插入 CSI 帧内部。

| 字段/现象 | 含义与检查方向 |
| --- | --- |
| `stage=running error=0x0` | NVS、LED、Wi-Fi、ESP-NOW、CSI 及任务初始化已完成 |
| `error` 非0 | 初始化/运行 API 失败，`stage` 指出失败表达式；首个错误锁存，状态任务继续输出 |
| `tx_ok` 持续增加 | ESP-NOW 发送完成且确认成功 |
| `tx_fail` 增加，`rx_peer=0` | 检查对端供电、两端是否错烧同一固件、信道和射频链路 |
| `tx_error` 增加 | esp_now_send 提交失败 |
| `rx_peer` 增加 | 已收到配置对端的 ESP-NOW 数据 |
| `csi_seen` 增加 | CSI 回调有数据，包含之后被 MAC 筛掉的帧 |
| `csi_filtered` 与 `csi_seen` 同步增加 | CSI 来源不匹配对端 MAC |
| `rx_peer` 增加但 `csi_seen=0` | 数据包已收到，需要检查 HT20 CSI 配置/发包速率 |
| `usb_drops` 增加 | 传输队列满或数据帧写入超时/失败；计数不包括状态行发送失败 |
| 一直只有 ROM 日志，没有任何状态行 | 检查是否已烧录新版、固件接口选项是否匹配实际端口；还不能据此断定运行成功 |

`rst:...USB_UART_HPSYS` 是复位原因，不等于 CSI 或 Wi-Fi 报错。ROM 的 `entry ...` 也不能证明应用初始化成功；以新版状态行为准。

每块板分别通过自己的 COM 口输出“本机接收到的对端 CSI”。保持原有固定 MAC、信道11、HT20、MCS0 和目标100Hz发包；USB 不传输运动评分。CSI 来源依然只限配置的对端 MAC，不采集所有周边设备。

两块板分别烧录对应项目。关闭占用 COM 口的串口监视器，再运行：

```powershell
python -m pip install pyserial
python tools/capture_csi.py --port COM9 --output recv.jsonl --raw recv.bin --seconds 60
```

另开终端用另一块板实际的 COM 口和不同输出文件，例如：

```powershell
python tools/capture_csi.py --port COM17 --output send.jsonl --raw send.bin --seconds 60
```

`--seconds 0` 持续采集，Ctrl+C 正常保存退出。`--raw` 保存原封不动的串口字节流；JSONL 每行一帧，`csi` 是 -128～127 的原始 int8 数组。离线解析不需要 pyserial：

```powershell
python tools/capture_csi.py --input recv.bin --output decoded.jsonl
python -m unittest discover -s test -p test_*.py -v
```

输出文件会覆盖同名文件，两个采集进程应使用不同名称。BINARY 模式在文本串口监视器中会出现二进制乱码，这是正常的；使用 TEXT 命令查看数值。采集脚本打开串口后会自动发送 BINARY/STATUS 命令。打开串口时脚本不主动拉起 DTR/RTS，但不同桥接驱动仍可能产生复位脉冲；解码器能够跳过 ROM 启动输出，并从下一个完整帧恢复。

## “原始”的具体含义

导出 Wi-Fi 驱动交给回调的 `info->buf[0:info->len]`，逐字节复制。不做幅值、相位、滤波、归一化、子载波筛选或删字节。检测算法处理独立的队列副本，不修改导出数据。`first_word_invalid` 为真时，前4字节仍保留，由上位机决定是否使用。这里的原始数据是芯片生成的 CSI，不是射频 ADC/IQ 采样。

每帧最大1024字节，足够当前 HT20 配置。超过上限整帧丢弃并累计 `oversize_drops`，不截断伪装成完整帧。序号在长度检查前分配，丢帧可通过后续序号缺口发现。

## 任务与资源所有权

ESP32-C6 在单个应用 CPU 上由 FreeRTOS 调度，tick 已设置为1000Hz。数值越大优先级越高，应用任务低于 Wi-Fi 系统任务。

| 执行上下文 | 优先级 / 栈字节 | 职责 / 阻塞策略 |
| --- | --- | --- |
| Wi-Fi CSI 回调 | SDK 管理 | MAC 过滤、复制数据，向两个队列零等待投递；不计算、不打印、不访问 USB/UART |
| espnow_send | 5 / 3072 | 每10ms调度；上一帧回调归还许可后才提交下一帧，未完成则跳过周期 |
| motion | 4 / 4096 | 独占 motion_core 和 GPIO11；最多等待 CSI 队列20ms，每帧处理后让出1 tick |
| usb_csi | 3 / 6144 | 独占所选接口；封包/CRC/文本编码/命令/状态。原生 USB 入队限时5ms；UART0 分段填充 FIFO，未写完让出1 tick，每帧限时30ms |
| breathing_led | 2 / 2048 | 独占 GPIO10 的 LEDC 通道，每20ms更新亮度 |

检测队列8帧、传输队列16帧，两个队列各自持有完整副本。队列满时丢弃该队列的新帧，另一个队列继续接收。回调不动态分配内存；队列和任务在启用 CSI 前创建，并检查创建结果。

GPIO11 保持时间仍为1.5秒，可由有效 PASS 续期。现在由检测任务统一管理截止时间，采用采集时间戳，避免积压的旧帧延长亮灯。无 CSI 时也会检查超时、熄灯并清除算法 PASS；检查粒度约20ms再加实际调度延迟。移除了原定时器与 CSI 回调对灯态的并发修改。

应用控制台与次级控制台均关闭，避免其他任务插入输出。状态行由传输任务显式输出。传输任务先于 NVS/Wi-Fi 初始化；初始化或 LED API 失败时保留错误并结束出错任务，状态任务继续运行。若传输驱动或状态任务自身无法创建，则仍可能没有状态行。`sdkconfig.defaults` 和现有 `sdkconfig.esp32-c6-devkitm-1` 都已修改；只改 defaults 不会覆盖已有配置。

## 二进制 CSI1 协议

下面为 BINARY 模式，协议与旧版完全一致。字节流现在还包含帧间 ASCII `#CSI_STATUS` 行；解码器搜索魔数并校验 CRC，跳过状态行及 ROM 输出。`.bin` 保存整个混合流，`.jsonl` 只保存校验通过的 CSI 帧。

所有多字节整数均为小端。固定48字节头 + `length` 字节原始 CSI + 4字节 CRC。不要按 C 结构体内存布局解包。

| 偏移 | 字节数 | 内容 |
| --- | --- | --- |
| 0 | 4 | ASCII `CSI1` |
| 4 | 1 | 版本1 |
| 5 | 1 | flags，bit0 = first_word_invalid，其余为0 |
| 6 | 2 | 头长度48 |
| 8 | 2 | 原始 CSI 字节数，1～1024 |
| 10 | 1 | RSSI，int8 |
| 11 | 1 | Wi-Fi 主信道 |
| 12 | 4 | 本机 CSI 回调序号，uint32，重启归零、溢出回绕 |
| 16 | 8 | 回调采集时 esp_timer_get_time()，启动以来微秒数 |
| 24 | 6 | 本机固定 MAC |
| 30 | 6 | 对端源 MAC |
| 36 | 4 | usb_drops：传输队列满 + UART 写入失败/超时 |
| 40 | 4 | motion_drops：检测队列满 |
| 44 | 4 | oversize_drops：超过长度上限 |
| 48 | length | 原始 CSI 字节，不删无效首字 |
| 48+length | 4 | CRC-32/ISO-HDLC，覆盖头和 CSI，与 Python zlib.crc32 一致 |

计数器是封包时的累计快照，uint32，重启归零。两个节点时间戳来自各自时钟，不能直接当作同步的绝对时间。PC 解码器处理分包、粘包、启动噪声、坏 CRC 和半帧恢复。

## 吞吐与实机验收

**仅 UART0 桥接模式：**921600 baud、8N1 的理论上限是92160字节/秒。每帧有52字节协议开销，例如256字节 CSI、100帧/秒约30800字节/秒；实际可用吞吐还取决于桥接芯片、USB驱动与任务负载。1024字节帧连续100Hz将超过该串口上限，不能承诺无损。

原生 USB 的写入超时会计入丢帧，主机未读时也不会阻塞无线或检测任务。TEXT 模式的十进制字符开销更大，连续采集建议 BINARY。

USB 转串口模式无硬件流控，也没有电脑接收确认。即使电脑不读取，UART 也可能继续发送，桥接芯片/主机侧丢失无法计入固件 `usb_drops`。因此需同时检查帧序号缺口、CRC 及主机端记录；不能把 `usb_drops=0` 解释为端到端零丢帧。系统资源不足时优先保持任务可调度，允许记录并丢弃数据，不无限阻塞。

实机验收步骤：

1. 两板静止上电，确认呼吸灯持续工作、CSI 每帧长度合理、源/目的 MAC 对应正确。
2. 连续采集至少60秒，检查帧率、序号缺口、CRC、三个丢帧计数器。
3. 移动触发检测灯，停止活动后检查保持时间；给对端断电，确认本机仍呼吸且检测灯超时熄灭。
4. 停止/重启采集程序，确认固件仍正常检测且解码能够恢复。拔掉作为唯一供电的 USB 会使整块板断电，不能用于验证“传输断开但固件继续运行”。
5. 长时间运行，观察复位、看门狗、堆余量和任务栈高水位，再根据测量调整队列/栈/优先级。

已知旧配置的 Flash 2MB/4MB 不一致仍需按实际硬件确认；本次未擅自改动 Flash 容量。软件编译和协议单测不能替代上述实机验收。

实现依据：[ESP-IDF CSI 回调应投递队列并由低优先级任务处理](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32c6/api-guides/wifi.html#wi-fi-channel-state-information-configure)、[UART 驱动 API](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32c6/api-reference/peripherals/uart.html)。

原生 USB 接口依据：[ESP-IDF ESP32-C6 USB Serial/JTAG 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32c6/api-guides/usb-serial-jtag-console.html)。
