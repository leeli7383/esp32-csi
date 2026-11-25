# ESP32-C6 双节点 CSI 运动检测

两块 ESP32-C6 通过 ESP-NOW 互发报文，各自采集对端 Wi-Fi CSI、运行运动检测，并可向电脑输出原始 CSI。运行检测无需路由器或电脑。

本仓库将原来的 `E:\esp32_recv\esp32_recv` 和 `E:\esp32_send` 合并管理，保留两个独立 PlatformIO 工程。`recv` / `send` 是历史名称，**两个节点都收发和检测**；但是两块板必须烧录各自固件，因为存在MAC筛选机制。

| 工程 | 本机 MAC | 对端 MAC | 默认 CSI 输出 | 初始发送延时 |
| --- | --- | --- | --- | --- |
| `esp32_recv/` | `02:00:00:00:00:02` | `02:00:00:00:00:01` | CP210x / UART0，原配置 COM9 | 5 ms |
| `esp32_send/` | `02:00:00:00:00:01` | `02:00:00:00:00:02` | 原生 USB Serial/JTAG | 1 ms |

公共参数：信道 11、HT20、MCS0 LGI、每端目标发包 100 Hz；GPIO10 为 PWM 呼吸灯，GPIO11 为高电平有效检测灯。接收并处理 80 帧完成预热后开始判定；有效 PASS 让检测灯保持约 1.5 秒，持续触发可续期。

## 目录

```text
esp32_recv/              接收节点固件、采集工具、协议测试和历史文档
esp32_send/              发送节点固件、采集工具、协议测试和历史文档
docs/ARCHITECTURE.md     当前实现、数据流和合并决策
docs/source-manifest.json 原始导入文件的 SHA256，供溯源
requirements.txt        电脑串口采集依赖
PROGRESS.md             验证结果、问题和下一步
```

## 编译和烧录

安装 PlatformIO Core（或 VS Code PlatformIO 扩展）。平台已锁定 `espressif32@6.13.0`，其配套 ESP-IDF 为 5.5.3。以下命令均在仓库根目录执行，串口号请按实际设备替换：

Windows 下请将仓库检出到纯英文路径（例如 `D:\projects\esp32-csi`）。本次验证发现包含中文用户名的路径会触发 ESP-IDF Kconfig 路径编码错误；同一提交在纯英文目录可继续编译。

```powershell
pio run -d esp32_recv -e esp32-c6-devkitm-1
pio run -d esp32_send -e esp32-c6-devkitm-1
pio run -d esp32_recv -e esp32-c6-devkitm-1 -t upload --upload-port COM9
pio run -d esp32_send -e esp32-c6-devkitm-1 -t upload --upload-port COM17
```

烧录前关闭占用串口的监视器。更换接口时修改对应工程 `platformio.ini` 的 `CSI_STREAM_USE_USB` 并重新编译烧录：`0` 为 UART0（GPIO16 TX / GPIO17 RX），`1` 为原生 USB（GPIO12 D- / GPIO13 D+）。两种接口不会同时输出。GPIO10/11 的 LED 应串联限流电阻后接至本板 GND。

**Flash 配置待实物核对：**保留了原工程有效 sdkconfig 中的 2MB 配置；defaults 和开发板定义为 4MB。构建容量警告属于配置差异，不能证明实体容量。合并未改变 Flash、分区或检测算法参数。确认板卡 Flash ID 后再统一配置。

## 采集和查看

```powershell
python -m pip install -r requirements.txt
New-Item -ItemType Directory -Force captures
python esp32_recv/tools/capture_csi.py --port COM9 --output captures/recv.jsonl --raw captures/recv.bin --seconds 60
# 在另一终端，使用另一块板的实际端口
python esp32_send/tools/capture_csi.py --port COM17 --output captures/send.jsonl --raw captures/send.bin --seconds 60
# 离线解码无需第三方依赖
python esp32_recv/tools/capture_csi.py --input captures/recv.bin --output captures/decoded.jsonl
```

两份采集脚本和测试相同，任一脚本都可用于任一节点。采集文件会覆盖同名文件，请使用不同文件名。

XCOM 设置 921600、8N1、无流控、关闭 RTS/DTR。发送 `TEXT` 加真实换行查看原始 int8 数组，`BINARY` 加换行恢复 CRC 校验的 CSI1 二进制协议，`STATUS` 加换行立即查看状态。默认每秒输出 `#CSI_STATUS`；`stage=running error=0x0` 表示初始化完成。采集脚本会自动请求 BINARY 模式。

协议详见 [RAW_CSI_USB.md](esp32_recv/docs/RAW_CSI_USB.md)。两节点时间戳来自各自启动时钟，未经同步，不能直接按绝对时间对齐。

## 软件检查

```powershell
python -m unittest discover -s esp32_recv/test -p 'test_*.py' -v
python -m unittest discover -s esp32_send/test -p 'test_*.py' -v
```

测试覆盖原始字节保留、分包粘包、CRC 损坏恢复、截断恢复、无效头、最大长度、序号回绕、启动噪声和文本/二进制切换。实际执行结果见 [PROGRESS.md](PROGRESS.md)。

## 使用边界

检测的是无线链路变化，不保证静止人员存在检测、人数统计或身份识别；两端独立检测，结果没有融合。ESP-NOW peer 使用 `encrypt=false`，适用于当前实验配置。目标 100 Hz 不等于实测有效帧率，`usb_drops=0` 也不代表端到端零丢帧。尚需双板实测帧率、漏报/误报、断链熄灯及长期稳定性。

原始 E 盘项目保留不变。本次合并排除了构建缓存、日志、编辑器本地配置和采集数据；保留第三方源文件原有版权及 SPDX 标记，未为整个仓库推定新的许可证。
