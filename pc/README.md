# PC 上位机：Motion Score 最小界面

`csi_upper_pc_min.py` 为用户提供的运动评分单图版：读取 CSI，独立完成 PC 端预处理与运动检测，显示 motion_score、触发/恢复阈值、PASS 区域、状态与日志。

**它不是此前视频中的六图版本。** 本文件没有单帧幅值图、相位图、IQ 散点图、热力图、功率/RSSI 趋势图，也不显示固件事件。

## 与当前固件的兼容性

| 数据来源 | 本程序能否直接读取 | 建议 |
| --- | --- | --- |
| 旧版 13 列 `CSI_DATA` CSV 文本行 | 可以 | 使用下方串口或文本回放命令 |
| 当前固件默认 CSI1 二进制流 | 不可以 | 用仓库现有 `capture_csi.py` 采集/离线解码 |
| 当前固件 `TEXT` 命令输出的键值格式 | 不可以 | 与本程序要求的 CSV 字段顺序不同，不能仅切到 TEXT 就直连 |
| `capture_csi.py` 输出的 JSONL | 不可以 | JSONL 尚未接入此 GUI |

本次按用户提供程序收录，仅规范行尾和空白，不改变输入协议、算法或界面。没有为兼容旧上位机而修改两端固件。后续可另行为 GUI 增加 CSI1/JSONL 输入；在适配前，当前固件采集请继续使用 [仓库采集说明](../README.md#采集和查看)。

## 安装与快速回放

推荐 Python 3.11，实测环境为 Python 3.11.9、Qt 5.15.2；依赖版本已写入 `pc/requirements.txt`。以下命令在仓库根目录运行：

```powershell
python -m venv D:\CodexTools\csi-upper-pc\venv
& D:\CodexTools\csi-upper-pc\venv\Scripts\python.exe -m pip install --cache-dir D:\CodexTools\csi-upper-pc\cache -r pc/requirements.txt
& D:\CodexTools\csi-upper-pc\venv\Scripts\python.exe pc/csi_upper_pc_min.py --replay pc/examples/synthetic_csi.txt
```

示例文件包含 **256 帧人工生成数据**，不是实测 CSI，也不能用于评价检测性能。每帧 16 个 I/Q 对，前段恒定幅值，中间插入短暂幅值阶跃，后段恢复；用于检查解析、80 帧预热、评分曲线和状态切换。

默认每行间隔 0.02 秒，约 5 秒完成回放；完成后窗口会保留，关闭窗口退出。回放按固定间隔读取，不按输入时间戳同步。已有环境也可直接：

```powershell
python -m pip install -r pc/requirements.txt
python pc/csi_upper_pc_min.py --help
python pc/csi_upper_pc_min.py --replay captures/legacy_csi.txt --replay-interval 0.02
```

若 Qt 初始化失败或报告 platform plugin 错误，先检查 Python/Qt 安装路径。此机的中文用户名导致 Qt 内置插件路径包含 `??`；验证时显式设置实际插件目录后通过。建议使用上面纯英文路径的独立环境，不向业务代码硬编码本机路径。无显示环境只能做冒烟检查，真实桌面显示效果仍需本机验证。

也可以在当前 PowerShell 会话显式指定当前 Python 环境的 Qt 插件位置（本机已验证此方法，不写入全局配置）：

```powershell
$env:PYTHONIOENCODING = 'utf-8'
$env:QT_QPA_PLATFORM_PLUGIN_PATH = python -c "from pathlib import Path; import PyQt5; print(Path(PyQt5.__path__[0]) / 'Qt5' / 'plugins')"
python pc/csi_upper_pc_min.py --replay pc/examples/synthetic_csi.txt
```

## 串口连接：仅限输出旧 CSV 的固件

```powershell
python pc/csi_upper_pc_min.py --port COM9 --baud 921600
```

COM9 为示例，Linux 可使用 `/dev/ttyUSB0`。先关闭 XCOM、PlatformIO monitor 和其他采集进程，避免端口占用。默认 921600、8N1，脚本不会自动发送 TEXT/BINARY 命令。它使用 pyserial 默认 DTR/RTS 行为，部分板卡在打开串口时可能复位；不能将其当作仓库 `capture_csi.py` 的等价替代。

## 输入协议

字段顺序固定，必须恰好为 13 列：

```text
type,seq,mac,rssi,rate,noise_floor,channel,local_timestamp,sig_len,rx_format,len,first_word,data
```

有效示例（4 字节/2 个 I/Q 对，仅演示 CSV 语法）：

```text
CSI_DATA,1,02:00:00:00:00:01,-40,0,-96,11,10000,0,0,4,0,"[0,20,0,20]"
```

- `data` 必须是被 CSV 双引号包裹的 JSON 数组；遗漏引号会把数组内部逗号解析为额外列。
- `len` 必须与数组长度一致，为 2～612 的偶数。最大 306 个 I/Q 对；它与当前固件传输层的 1024 字节上限不同。
- `seq`、RSSI、rate、noise_floor、channel、时间戳等数值列必须能转换为整数。日志前缀允许存在，解析器从首个 `CSI_DATA` 开始读取。
- 非 CSI 行被跳过，字段/长度/JSON 错误会计为 `[BAD CSI]`，后台继续读取。
- 默认 `--iq-order esp-idf` 使用 `real=raw[2k+1], imag=raw[2k]`；`normal` 使用相反配对。两种顺序交换对当前幅值评分的影响有限，不应解释为做过相位标定。
- `first_word` 被记录但不用于删除原始字节；本程序也不严格校验每个输入数值是否处于 int8 范围，输入应来自可信的正确采集记录。

## 命令行参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `-p / --port` | 无 | 串口；指定 `--replay` 时不使用串口 |
| `-b / --baud` | 921600 | 波特率 |
| `--replay` | 无 | 旧 CSV 文本日志路径 |
| `--replay-interval` | 0.02 | 每行回放间隔秒数；0 为尽快回放 |
| `--history` | 300 | 界面曲线的历史帧数 |
| `--warmup-frames` | 80 | 预热帧数 |
| `--iq-order` | esp-idf | I/Q 重组顺序，可选 normal |

`--history` 调整图形缓存，不会修改 `CsiDetector` 当前固定的 100 帧阈值历史。请使用合理的正数参数；原程序没有完整参数边界校验。不要将 `--replay-interval 0` 用于很大的日志，否则后台可能比界面处理更快。

## PC 判定与板端判定

| 项目 | 此 PC 上位机 | 当前板端 motion_core |
| --- | --- | --- |
| 评分权重：基线差 / 帧间差 / 方差 | 0.50 / 0.25 / 0.25 | 0.70 / 0.05 / 0.25 |
| CLEAR 阈值历史 | 100 帧 | 300 帧 |
| 普通基线更新 alpha | 0.08 | 0.08 / 3 |
| 预热 | 80 帧（可通过 CLI 改变） | 80 帧 |
| PASS / CLEAR 连续判定 | 2 帧 / 4 帧 | 2 帧 / 4 帧 |
| 无数据时超时清除 | 没有独立清除，保留末次显示 | 检测任务检查 1.5 秒保持时间 |

两者还存在子载波筛选刷新频率、阈值统计和状态保持等差异，不能期待逐帧评分或 PASS 状态完全一致。PC 使用中位数/MAD 动态阈值与滞回状态机；界面显示的是 PC 计算结果，不是板端 GPIO11 的实时回传。

## 来源与验证

原始文件：用户于 2026-11-25 提供的 `csi_upper_pc_min.py`。导入前 SHA256：

```text
17c28e89e1fe53d9f8d070ec91ba8f08806389ddf81123257eb948f966809525
```

原附件未修改；仓库副本仅清理行尾空白、空行和统一 LF，Python AST 对比一致。验证覆盖 CLI 帮助、256 帧合成 CSV 解析、Qt 无显示环境下曲线/状态更新、预热与 PASS/CLEAR 切换，以及 QThread 完整回放和退出。没有连接串口或实体板；验证不证明实际检测率、桌面显示效果或长期稳定性。完整交接见 [PROGRESS.md](../PROGRESS.md)。
