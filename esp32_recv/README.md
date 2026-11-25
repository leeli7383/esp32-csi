# esp32_recv：ESP32-C6 双向 CSI 检测节点

本节点同时发送 ESP-NOW 报文、采集对端 CSI，并在本机通过 motion_core 计算运动评分。节点身份固定，不随启动顺序变化。

- 本机 MAC：`02:00:00:00:00:02`
- 对端 MAC：`02:00:00:00:00:01`
- GPIO10：PWM 呼吸灯。
- GPIO11：检测灯；有效 PASS 判定可续期1.5秒保持时间。
- 两端均使用信道11、HT20、MCS0，目标发包频率100Hz。
- 接收端通过 CP210x USB 转串口输出原始 CSI：UART0、921600 baud，当前上传/监视端口 COM9（CSI_STREAM_USE_USB=0）。
- XCOM 发送 `TEXT` 加换行查看原始数值；`BINARY` 加换行恢复二进制，每秒提供 `#CSI_STATUS` 诊断。
- FreeRTOS 分别调度无线发送、运动检测、CSI 传输和呼吸灯，队列隔离各功能。
- 采集命令、二进制协议和验收步骤见 [原始 CSI USB 使用说明](docs/RAW_CSI_USB.md)。

完整的功能、接线、代码讲解、编译烧录、算法参数、已知问题和验收清单见：

[项目交接说明](docs/PROJECT_HANDOVER.md)

在本工程目录的 PlatformIO 终端执行：

```powershell
pio run -e esp32-c6-devkitm-1
pio run -e esp32-c6-devkitm-1 -t upload --upload-port COM16
```

COM16为示例，请换成此节点实际端口。另一块板应使用其对应工程的固件，不能将本工程固件烧录到两块板上。

交接前重点核对：Flash配置存在2MB/4MB冲突；内部算法状态与检测灯保持状态不完全相同。详见交接说明第4、8、11节。
