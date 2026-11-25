# 实现阅读与合并说明

日期：2026-11-25。依据：两个原工程当前源码、配置和文档；历史构建日志只作为环境线索，不作为本次验证结果。

## 当前数据流

```text
espnow_send 任务 → 固定对端 ESP-NOW → 对端 Wi-Fi 产生 CSI
本机 Wi-Fi CSI 回调 → MAC 过滤 → csi_stream_publish
                                  ├─ 检测队列 → motion_task → motion_core → GPIO11
                                  └─ 传输队列 → usb_csi_task → UART0 或 USB → PC Decoder
breathing_led_task → GPIO10 LEDC
```

两块板分别运行以上完整流程。无线负载仅为 uint32 递增序号，并不包含 CSI 或评分；CSI 是接收芯片根据无线信号生成的结果。

`app_main` 先创建独立传输任务，再初始化 NVS、LED、Wi-Fi、ESP-NOW、CSI 和发送任务。`APP_CHECK` 将首个 API 错误锁存到状态输出后结束当前任务。传输驱动自身初始化失败时不保证还能输出状态。

## 文件与调度

| 文件 | 职责 |
| --- | --- |
| `src/main.c` | 节点 MAC、射频参数、任务建立、回调与 LED 控制 |
| `src/csi_stream.c/.h` | 双队列、原始字节复制、CSI1 封包、CRC、命令与诊断 |
| `src/motion_core.c/.h` | 幅值、子载波筛选、背景与阈值更新、PASS/CLEAR 状态 |
| `tools/capture_csi.py` | 串口采集、增量解码、JSONL / 原始流保存 |
| `test/test_capture_csi.py` | 现有协议解码测试 |

单核 FreeRTOS 使用 1000 Hz tick。发送任务优先级 5、检测 4、传输 3、呼吸灯 2。CSI 回调仅筛选、复制和零等待投递；检测队列深度 8，传输队列深度 16。任一队列满只丢弃该队列新帧。超过 1024 字节的 CSI 整帧丢弃并计数。

motion_core 将相邻 int8 对转换为幅值，最多处理 306 个子载波；质量窗口 80 帧、方差窗口 6 帧、评分历史 300 帧。评分为 `0.70 × baseline_diff + 0.05 × frame_diff + 0.25 × variance_diff`；阈值基于历史中位数与 MAD，连续两次超触发阈值进入 PASS，连续四次低于恢复阈值进入 CLEAR。

GPIO11 由 motion_task 独占，以 CSI 采集时间戳续期 1.5 秒。无帧时仍约每 20ms 检查到期，熄灯并强制算法 CLEAR。灯的显示保持状态与算法即时状态可能不同。导出数据独立复制，不做幅值、滤波或无效首字删除；`first_word_invalid` 交给上位机解释。

CSI1 帧采用 48 字节头、原始数据和 4 字节 CRC-32，小端编码。PC 解码器搜索魔数并校验 CRC，跳过帧间状态文本和 ROM 噪声。两端各自计时和编号，未实现时钟同步。

## 合并决策与原文件差异

- 保留 `esp32_recv/` 与 `esp32_send/` 两套可独立构建的工程，避免改动现有固件身份、接口与部署方式。
- 两份 `main.c` 的功能差异为本机/对端 MAC、标签、首次发送延时（5ms / 1ms）、peer 地址和启动日志。传输代码、采集工具、测试相同；motion_core 只有空白差异。
- 仅将平台从浮动 `espressif32` 锁定到历史成功构建使用的 `6.13.0`；保留现有完整 sdkconfig，以免仅复制 defaults 导致配置变化。
- 删除两端未使用的示例 `esp_now_set_pmk` 调用；所有 peer 原本已设置 `encrypt=false`，无需发布固定示例密钥。MAC、信道、CSI 协议和检测计算均保持原值。
- 在旧交接文档顶部标注历史性质；部分关于回调内计算、定时器和关闭输出的内容已过时，以当前源码及 RAW_CSI_USB.md 为准。
- 清理入口文件行尾空白和文件尾多余空行，统一两端 motion_core 的空白格式，不改变算法代码。
- 不导入 `.pio`、`.vscode`、缓存或构建日志。`source-manifest.json` 记录导入前源文件 SHA256；它不是合并后文件的校验清单。

公共实现继续保留两份，后续修改需要同步检查。本次没有将两个固件重构为单个参数化工程；后续若需要统一源码，应验证 MAC 配对、两种传输路径、启动行为及构建产物身份。

## 待解决

1. 根据实体 Flash ID 统一有效 sdkconfig 的 2MB 与 defaults/board 的 4MB 差异。
2. 双板采集至少 60 秒，记录有效帧率、序号缺口、CRC、丢帧计数与复位情况。
3. 验证静态预热、运动续期、对端断电后熄灯、长期运行与不同环境检测效果。
4. 算法未使用 `first_word_invalid` 跳过前四字节；原始导出保留该标记和字节。需实测影响后再决定是否调整算法。
