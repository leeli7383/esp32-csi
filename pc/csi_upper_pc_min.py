#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-C6 CSI motion detection - score-only upper-computer version.

保留内容：
1. 串口 / 离线日志读取 CSI_DATA
2. CSI 行解析：raw I/Q -> complex CSI -> amplitude
3. CSI 预处理：基线维护 + 稳定子载波筛选
4. 运动检测：baseline_diff（基线差异） + frame_diff（帧间差异） + variance_diff（波动差异） -> motion_score
5. 仅显示 Motion Score + Trigger/Clear Threshold 曲线，并打印 PASS / CLEAR
6. 滞回阈值动态调整，自动适应环境变化
7. 前期预热机制，避免启动时误触发

删除内容：
- 读取固件端事件（如人体接近/远离）并显示在界面上
- 单帧幅值图
- 单帧相位图
- IQ 散点图
- 平均功率 / RSSI 趋势图
- 事件窗口保存
- 固件事件显示面板
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from collections import deque
from dataclasses import dataclass
from io import StringIO
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

try:
    import serial
except ImportError:
    serial = None

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg


CSI_COLUMNS_C6 = [
    "type",              #type-数据类型标识
    "seq",               #seq-帧序号
    "mac",               #mac-发送设备 MAC 地址
    "rssi",              #rssi-接收信号强度指示
    "rate",              #rate-数据传输速率
    "noise_floor",       #noise_floor-噪声底
    "channel",           #channel-信道
    "local_timestamp",   #local_timestamp-本地时间戳
    "sig_len",           #sig_len-信号长度
    "rx_format",         #rx_format-接收格式
    "len",               #len-长度
    "first_word",        #first_word-首字
    "data",              #data-数据
]

MAX_RAW_LEN = 612       #  ESP32-C6 CSI_DATA 最多 306 个 I/Q 对，即 612 个原始数值。
DEFAULT_BAUD = 921600   #  默认使用 921600 波特率输出 CSI_DATA
DEFAULT_HISTORY = 300   #  曲线显示和自适应阈值计算的历史帧数
DEFAULT_WARMUP_FRAMES = 80  # 启动预热帧数；预热期只建立 baseline，不触发 PASS。


@dataclass
class CsiFrame:
    """解析后的单帧 CSI 数据。"""

    seq: int
    mac: str
    rssi: int
    rate: int
    noise_floor: int
    channel: int
    local_timestamp: int
    sig_len: int
    rx_format: int
    raw_len: int
    first_word: int
    raw: np.ndarray
    complex_csi: np.ndarray
    amplitude: np.ndarray
    avg_power: float


@dataclass
class DetectorResult:
    """PC 端运动检测结果。"""

    baseline_diff: float
    frame_diff: float
    variance_diff: float
    motion_score: float
    trigger_threshold: float
    clear_threshold: float
    state: str
    state_changed: bool
    selected_count: int
    in_warmup: bool


def clean_serial_line(raw_line) -> str:
    """统一清理串口 bytes 和回放文本行。"""

    if isinstance(raw_line, bytes):
        return raw_line.decode("utf-8", errors="ignore").strip()
    return str(raw_line).strip()


def parse_csi_line(line: str, iq_order: str = "esp-idf") -> Optional[CsiFrame]:
    """从包含 CSI_DATA 的一行日志中解析出 CsiFrame。"""

    idx = line.find("CSI_DATA")
    if idx < 0:
        return None

    line = line[idx:]

    try:
        row = next(csv.reader(StringIO(line)))
    except csv.Error:
        return None

    if len(row) != len(CSI_COLUMNS_C6):
        raise ValueError(
            f"CSI field count mismatch: got {len(row)}, expected {len(CSI_COLUMNS_C6)}; line={line[:120]}"
        )

    try:
        raw_len = int(row[10])
        raw_list = json.loads(row[12])
    except Exception as exc:
        raise ValueError(f"CSI JSON/length parse failed: {exc}; line={line[:120]}") from exc

    if raw_len != len(raw_list):
        raise ValueError(f"CSI length mismatch: len column={raw_len}, actual={len(raw_list)}")
    if raw_len < 2 or raw_len % 2 != 0:
        raise ValueError(f"CSI raw length should be positive even number, got {raw_len}")
    if raw_len > MAX_RAW_LEN:
        raise ValueError(f"CSI raw length {raw_len} exceeds MAX_RAW_LEN={MAX_RAW_LEN}")

    raw = np.asarray(raw_list, dtype=np.float32)

    # ESP-IDF 输出的 I/Q 顺序可能与常规顺序相反，保留参数便于现场校准。
    if iq_order == "normal":
        complex_csi = raw[0::2] + 1j * raw[1::2]
    else:
        complex_csi = raw[1::2] + 1j * raw[0::2]

    amplitude = np.abs(complex_csi)
    avg_power = float(np.mean(raw[0::2] ** 2 + raw[1::2] ** 2))

    return CsiFrame(
        seq=int(row[1]),
        mac=row[2],
        rssi=int(row[3]),
        rate=int(row[4]),
        noise_floor=int(row[5]),
        channel=int(row[6]),
        local_timestamp=int(row[7]),
        sig_len=int(row[8]),
        rx_format=int(row[9]),
        raw_len=raw_len,
        first_word=int(row[11]),
        raw=raw,
        complex_csi=complex_csi,
        amplitude=amplitude,
        avg_power=avg_power,
    )


class CsiPreprocessor:
    """维护幅值基线，并筛选稳定、可靠的子载波。"""

    def __init__(
        self,
        quality_window: int = 80,   #  用于评估子载波质量的静态样本窗口大小，单位为帧数。
        baseline_alpha: float = 0.08,   # 基线更新的平滑系数，越大响应越快但越不稳定。
        pass_baseline_alpha: float = 0.015,   # 用于更新 PASS 状态下基线的平滑系数。
        edge_ratio: float = 0.08,   # 评估子载波质量时排除边缘子载波的比例，默认排除两侧各 8%。
        spike_z_threshold: float = 6.0, # 判断子载波是否有异常尖峰的 z-score 阈值。
        cv_threshold: float = 0.18,   # 判断子载波是否稳定的变异系数（CV）阈值。
        min_mean_quantile: float = 0.2, # 判断子载波是否可靠的平均幅值下限分位数，只保留高平均幅值的子载波。
    ):
        self.quality_window = quality_window
        self.baseline_alpha = baseline_alpha
        self.pass_baseline_alpha = pass_baseline_alpha
        self.edge_ratio = edge_ratio
        self.spike_z_threshold = spike_z_threshold
        self.cv_threshold = cv_threshold
        self.min_mean_quantile = min_mean_quantile
        self.static_buffer = deque(maxlen=quality_window)
        self.baseline: Optional[np.ndarray] = None  # 当前的幅值基线，形状与 CSI 子载波数相同。
        self.selected_mask: Optional[np.ndarray] = None # 稳定可靠子载波的布尔掩码，筛选子载波。

    def _build_quality_mask(self, samples: np.ndarray) -> np.ndarray:
        """根据一段静态样本评估每个子载波质量。"""

        n = samples.shape[1]
        edge = max(1, int(round(n * self.edge_ratio)))
        inner_mask = np.ones(n, dtype=bool)
        inner_mask[:edge] = False
        inner_mask[-edge:] = False

        inner_samples = samples[:, inner_mask]
        mean_amp = np.mean(inner_samples, axis=0)
        std_amp = np.std(inner_samples, axis=0)
        cv_i = std_amp / np.maximum(mean_amp, 1e-6)

        median_amp = np.median(inner_samples, axis=0)
        mad_amp = np.median(np.abs(inner_samples - median_amp), axis=0)
        robust_sigma = np.maximum(mad_amp * 1.4826, 1e-3)
        spike_score = np.max(np.abs(inner_samples - median_amp) / robust_sigma, axis=0)

        mean_floor = float(np.quantile(mean_amp, self.min_mean_quantile))
        quality_inner = (
            (mean_amp >= mean_floor)
            & (cv_i <= self.cv_threshold)
            & (spike_score <= self.spike_z_threshold)
        )

        selected_mask = np.zeros(n, dtype=bool)
        selected_mask[inner_mask] = quality_inner

        if not np.any(selected_mask):
            selected_mask = inner_mask.copy()

        return selected_mask

    def update(
        self,
        amplitude: np.ndarray,
        allow_baseline_update: bool = True,
        slow_baseline_recovery: bool = False,   # 在 PASS 状态下更新基线时使用更小的 alpha，以避免基线被运动干扰过快拉升。
    ) -> dict:
        """更新基线和稳定子载波掩码。"""

        amplitude = np.asarray(amplitude, dtype=np.float32)

        if self.baseline is None or self.baseline.shape != amplitude.shape:
            self.baseline = amplitude.copy()
            self.selected_mask = np.ones_like(amplitude, dtype=bool)
            self.static_buffer.clear()
        # 只有在基线已经建立的情况下才允许更新静态样本，否则可能会把运动状态的样本误加入静态样本，导致基线被拉升。
        if allow_baseline_update:
            self.static_buffer.append(amplitude.copy())

        if len(self.static_buffer) >= max(12, self.quality_window // 3):
            samples = np.stack(self.static_buffer, axis=0)
            self.selected_mask = self._build_quality_mask(samples)
            baseline_candidate = np.mean(samples, axis=0).astype(np.float32)
            alpha = self.pass_baseline_alpha if slow_baseline_recovery else self.baseline_alpha
            self.baseline = ((1.0 - alpha) * self.baseline + alpha * baseline_candidate).astype(np.float32)

        assert self.selected_mask is not None
        assert self.baseline is not None

        return {
            "amplitude_selected": amplitude[self.selected_mask],
            "baseline_selected": self.baseline[self.selected_mask],
            "selected_mask": self.selected_mask.copy(),
            "baseline": self.baseline.copy(),
        }


class CsiDetector:
    """计算 motion score，并用自适应滞回阈值判断 CLEAR / PASS。"""

    def __init__(
        self,
        history_len: int = 100, # 用于估计阈值的历史 CLEAR 状态分数的最大帧数。
        warmup_frames: int = DEFAULT_WARMUP_FRAMES,     # 启动预热帧数；预热期只建立 baseline，不触发 PASS。
        min_trigger: float = 1.5,   # 触发阈值的下限，防止过于敏感。
        min_clear: float = 0.8, # 恢复阈值的下限，防止过于迟钝。
        trigger_k: float = 3.5, # 触发阈值的动态调整系数。
        clear_k: float = 1.8, # 恢复阈值的动态调整系数，应该小于 trigger_k 以形成合理的滞回。
        trigger_hold_frames: int = 2,   # 连续多少帧超过触发阈值才切换到 PASS。
        clear_hold_frames: int = 4,  # 连续多少帧低于恢复阈值才切换到 CLEAR。
        variance_window: int = 6,   # 计算 短时间波动指标-variance_diff 时使用的最近帧数。
        variance_weight: float = 0.25,  #   motion_score 中 variance_diff 的权重。
    ):
        self.score_history_clear = deque(maxlen=history_len)
        self.last_amp_selected: Optional[np.ndarray] = None
        self.recent_amp_selected = deque(maxlen=variance_window)
        self.state = "CLEAR"
        self.warmup_frames = warmup_frames
        self.processed_frames = 0
        self.min_trigger = min_trigger
        self.min_clear = min_clear
        self.trigger_k = trigger_k
        self.clear_k = clear_k
        self.trigger_hold_frames = trigger_hold_frames
        self.clear_hold_frames = clear_hold_frames
        self.variance_weight = variance_weight
        self.trigger_counter = 0
        self.clear_counter = 0

    def _thresholds(self) -> tuple[float, float]:
        """用 CLEAR 状态历史分数估计触发阈值和恢复阈值。"""

        if len(self.score_history_clear) < 8:
            return self.min_trigger, self.min_clear
        # 使用中位数和 MAD（中位数绝对偏差）来估计 CLEAR 状态分数的分布，具有更强的鲁棒性。
        scores = np.asarray(self.score_history_clear, dtype=np.float32)
        median = float(np.median(scores))
        mad = float(np.median(np.abs(scores - median)))
        robust_sigma = max(1e-3, 1.4826 * mad)
        # 触发阈值和恢复阈值都基于 CLEAR 状态分数的分布动态调整，同时设置下限以防止过度调整。
        trigger = max(self.min_trigger, median + self.trigger_k * robust_sigma)
        clear = max(self.min_clear, median + self.clear_k * robust_sigma)
        clear = min(clear, trigger * 0.8)
        return trigger, clear

    def update(self, amp_selected: np.ndarray, baseline_selected: np.ndarray) -> DetectorResult:
        """输入当前稳定子载波幅值，输出运动识别结果。"""
        #   更新历史状态和计算分数
        self.processed_frames += 1  # 处理帧数计数器，用于判断是否处于预热阶段。
        in_warmup = self.processed_frames <= self.warmup_frames
        if amp_selected.size == 0 or baseline_selected.size == 0:
            return DetectorResult(0.0, 0.0, 0.0, 0.0, self.min_trigger, self.min_clear, self.state, False, 0, in_warmup)
        # 维护最近一次选定的稳定子载波幅值，以计算与上一帧的差异。
        if self.last_amp_selected is None or self.last_amp_selected.shape != amp_selected.shape:
            self.last_amp_selected = amp_selected.copy()
            self.recent_amp_selected.clear()

        self.recent_amp_selected.append(amp_selected.copy())
        # 计算三个指标：与基线的差异、与上一帧的差异、以及最近波动的方差，综合成一个 motion_score。
        baseline_diff = float(np.mean(np.abs(amp_selected - baseline_selected)))
        frame_diff = float(np.mean(np.abs(amp_selected - self.last_amp_selected)))

        if len(self.recent_amp_selected) >= 3:
            recent_stack = np.stack(self.recent_amp_selected, axis=0)
            variance_diff = float(np.mean(np.var(recent_stack, axis=0)))
        else:
            variance_diff = 0.0

        motion_score = 0.5 * baseline_diff + 0.25 * frame_diff + self.variance_weight * variance_diff
        trigger_threshold, clear_threshold = self._thresholds()

        previous_state = self.state

        if in_warmup:
            self.state = "CLEAR"
            self.trigger_counter = 0
            self.clear_counter = 0
            self.score_history_clear.append(motion_score)
        elif self.state == "CLEAR":
            self.score_history_clear.append(motion_score)
            if motion_score >= trigger_threshold:
                self.trigger_counter += 1
                if self.trigger_counter >= self.trigger_hold_frames:
                    self.state = "PASS"
                    self.trigger_counter = 0
                    self.clear_counter = 0
            else:
                self.trigger_counter = 0
        else:
            if motion_score <= clear_threshold:
                self.clear_counter += 1
                if self.clear_counter >= self.clear_hold_frames:
                    self.state = "CLEAR"
                    self.clear_counter = 0
                    self.trigger_counter = 0
                    self.score_history_clear.append(motion_score)
            else:
                self.clear_counter = 0

        self.last_amp_selected = amp_selected.copy()

        return DetectorResult(
            baseline_diff=baseline_diff,
            frame_diff=frame_diff,
            variance_diff=variance_diff,
            motion_score=motion_score,
            trigger_threshold=trigger_threshold,
            clear_threshold=clear_threshold,
            state=self.state,
            state_changed=self.state != previous_state,
            selected_count=int(amp_selected.size),
            in_warmup=in_warmup,
        )


class ReaderThread(QtCore.QThread):
    """后台读取串口或回放文件，并把解析后的 CsiFrame 发给界面线程。"""

    frame_ready = QtCore.pyqtSignal(object)
    log_ready = QtCore.pyqtSignal(str)

    def __init__(
        self,
        port: Optional[str],
        baud: int,
        replay_path: Optional[Path] = None,
        replay_interval: float = 0.02,
        iq_order: str = "esp-idf",
    ):
        super().__init__()
        self.port = port
        self.baud = baud
        self.replay_path = replay_path
        self.replay_interval = replay_interval
        self.iq_order = iq_order
        self._running = True

    def stop(self):
        self._running = False

    def _line_source(self) -> Iterable[str]:
        if self.replay_path:
            with self.replay_path.open("r", encoding="utf-8", errors="ignore") as fd:
                for line in fd:
                    if not self._running:
                        break
                    yield line.rstrip("\n")
                    if self.replay_interval > 0:
                        time.sleep(self.replay_interval)
            return

        if serial is None:
            raise RuntimeError("pyserial is not installed. Install it with: pip install pyserial")
        if not self.port:
            raise RuntimeError("Serial port is required unless --replay is used")

        with serial.Serial(self.port, self.baud, bytesize=8, parity="N", stopbits=1, timeout=1) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self.log_ready.emit(f"serial opened: {self.port} @ {self.baud}")
            while self._running:
                line = ser.readline()
                if line:
                    yield clean_serial_line(line)

    def run(self):
        frame_count = 0
        bad_count = 0
        try:
            for line in self._line_source():
                if not line or "CSI_DATA" not in line:
                    continue

                try:
                    frame = parse_csi_line(line, iq_order=self.iq_order)
                except Exception as exc:
                    bad_count += 1
                    self.log_ready.emit(f"[BAD CSI] {exc}")
                    continue

                if frame is None:
                    continue

                frame_count += 1
                self.frame_ready.emit(frame)

                if frame_count % 50 == 0:
                    self.log_ready.emit(
                        f"frames={frame_count}, bad={bad_count}, len={frame.raw_len}, "
                        f"rssi={frame.rssi}, avg_power={frame.avg_power:.2f}"
                    )
        except Exception as exc:
            self.log_ready.emit(f"reader stopped: {exc}")


class ScoreOnlyWindow(QtWidgets.QMainWindow):
    """显示 Motion Score + Threshold 的最小窗口。"""

    def __init__(self, history_len: int = DEFAULT_HISTORY, warmup_frames: int = DEFAULT_WARMUP_FRAMES):
        super().__init__()
        self.setWindowTitle("ESP32-C6 CSI Motion Score")
        self.resize(1000, 650)

        self.frame_index = 0
        self.history_len = history_len

        self.motion_history = deque(maxlen=history_len)
        self.trigger_history = deque(maxlen=history_len)
        self.clear_history = deque(maxlen=history_len)
        self.state_history = deque(maxlen=history_len)
        self.pass_region_items = []

        self.preprocessor = CsiPreprocessor()
        self.detector = CsiDetector(warmup_frames=warmup_frames)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        self.status_label = QtWidgets.QLabel("waiting for CSI data...")
        self.status_label.setMinimumHeight(36)
        layout.addWidget(self.status_label)

        self.metrics_label = QtWidgets.QLabel(
            "motion_score=-  trigger_threshold=-  clear_threshold=-  state=-  "
            "baseline_diff=-  frame_diff=-  selected=-  warmup=-"
        )
        layout.addWidget(self.metrics_label)

        pg.setConfigOptions(antialias=True)
        self.score_plot = pg.PlotWidget(title="Motion Score + Threshold")
        self.score_plot.setLabel("left", "Score")
        self.score_plot.setLabel("bottom", "Frame")
        curve_pen = lambda color, style=QtCore.Qt.SolidLine: pg.mkPen(color, width=2, style=style)
        self.motion_curve = self.score_plot.plot(name="motion_score", pen=curve_pen("#f97316"))
        self.trigger_curve = self.score_plot.plot(name="trigger_threshold", pen=curve_pen("#eab308", QtCore.Qt.DashLine))
        self.clear_curve = self.score_plot.plot(name="clear_threshold", pen=curve_pen("#2563eb", QtCore.Qt.DotLine))
        layout.addWidget(self.score_plot, stretch=1)

        self.log_box = QtWidgets.QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumHeight(150)
        layout.addWidget(self.log_box)

    @QtCore.pyqtSlot(str)
    def append_log(self, text: str):
        self.log_box.append(text)
        cursor = self.log_box.textCursor()
        cursor.movePosition(cursor.End)
        self.log_box.setTextCursor(cursor)

    def _set_status(self, state: str, text: str):
        if state == "PASS":
            style = "background:#7f1d1d;color:white;font-weight:bold;padding:8px;"
        elif state == "WARMUP":
            style = "background:#78350f;color:white;font-weight:bold;padding:8px;"
        else:
            style = "background:#14532d;color:white;font-weight:bold;padding:8px;"
        self.status_label.setStyleSheet(style)
        self.status_label.setText(text)

    def _refresh_score_plot(self):
        x = np.arange(len(self.motion_history))
        motion_scores = np.asarray(self.motion_history, dtype=np.float32)
        self.motion_curve.setData(x, motion_scores)
        self.trigger_curve.setData(x, np.asarray(self.trigger_history, dtype=np.float32))
        self.clear_curve.setData(x, np.asarray(self.clear_history, dtype=np.float32))
        self._refresh_pass_regions(motion_scores)

    def _refresh_pass_regions(self, motion_scores: np.ndarray):
        for item in self.pass_region_items:
            self.score_plot.removeItem(item)
        self.pass_region_items.clear()

        if not self.state_history:
            return

        states = list(self.state_history)
        y_values = list(motion_scores) + list(self.trigger_history) + list(self.clear_history)
        y_top = max(y_values) if y_values else 1.0

        start = None
        for index, state in enumerate(states + ["CLEAR"]):
            if state == "PASS" and start is None:
                start = index
            elif state != "PASS" and start is not None:
                end = index - 1
                region = pg.LinearRegionItem(
                    values=(start - 0.5, end + 0.5),
                    orientation="vertical",
                    brush=pg.mkBrush(248, 113, 113, 55),
                    pen=pg.mkPen(185, 28, 28, 100),
                    movable=False,
                )
                region.setZValue(-10)
                label = pg.TextItem(
                    "PASS",
                    color="#7f1d1d",
                    anchor=(0.5, 1.0),
                    fill=pg.mkBrush(254, 226, 226, 220),
                )
                label.setPos((start + end) / 2.0, y_top)
                label.setZValue(10)
                self.score_plot.addItem(region)
                self.score_plot.addItem(label)
                self.pass_region_items.extend([region, label])
                start = None

    @QtCore.pyqtSlot(object)
    def update_frame(self, frame: CsiFrame):
        """整体识别流程：CSI 幅值 -> 预处理 -> 检测 -> 更新分数曲线。"""

        self.frame_index += 1

        pre_result = self.preprocessor.update(
            frame.amplitude,
            allow_baseline_update=True,
            slow_baseline_recovery=(self.detector.state == "PASS"),
        )
        detector_result = self.detector.update(
            pre_result["amplitude_selected"],
            pre_result["baseline_selected"],
        )

        self.motion_history.append(detector_result.motion_score)
        self.trigger_history.append(detector_result.trigger_threshold)
        self.clear_history.append(detector_result.clear_threshold)
        self.state_history.append(detector_result.state)

        visual_state = "WARMUP" if detector_result.in_warmup else detector_result.state

        self.metrics_label.setText(
            f"frame={self.frame_index}  seq={frame.seq}  rssi={frame.rssi}  "   # 显示帧序号和 RSSI 以便现场调试时参考信号强度。
            f"motion_score={detector_result.motion_score:.3f}  "    # 显示综合运动分数，便于评估检测灵敏度和调整参数。
            f"trigger_threshold={detector_result.trigger_threshold:.3f}  "
            f"clear_threshold={detector_result.clear_threshold:.3f}  "
            f"state={detector_result.state}  "
            f"baseline_diff={detector_result.baseline_diff:.3f}  "
            f"frame_diff={detector_result.frame_diff:.3f}  "
            f"variance_diff={detector_result.variance_diff:.3f}  "
            f"selected={detector_result.selected_count}/{len(frame.amplitude)}  "
            f"warmup={'YES' if detector_result.in_warmup else 'NO'}"
        )

        if detector_result.state_changed and detector_result.state == "PASS":
            msg = (
                f"[PC PASS] seq={frame.seq} motion_score={detector_result.motion_score:.3f} "
                f"trigger={detector_result.trigger_threshold:.3f}"
            )
            self._set_status(visual_state, msg)
            self.append_log(msg)
        elif detector_result.state_changed and detector_result.state == "CLEAR":
            msg = (
                f"[PC CLEAR] seq={frame.seq} motion_score={detector_result.motion_score:.3f} "
                f"clear={detector_result.clear_threshold:.3f}"
            )
            self._set_status(visual_state, msg)
            self.append_log(msg)
        elif detector_result.in_warmup:
            remain = max(0, self.detector.warmup_frames - self.detector.processed_frames)
            self._set_status(
                "WARMUP",
                f"[WARMUP] building baseline... remain={remain} "
                f"motion_score={detector_result.motion_score:.3f}",
            )
        else:
            self._set_status(detector_result.state, detector_result.state)

        self._refresh_score_plot()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ESP32-C6 CSI motion score-only upper-computer")
    parser.add_argument("-p", "--port", default=None, help="串口号，例如 COM5 或 /dev/ttyUSB0。使用 --replay 时可省略。")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help="串口波特率，默认 921600。")
    parser.add_argument("--replay", default=None, help="从 txt 日志回放，便于无硬件调试。")
    parser.add_argument("--replay-interval", type=float, default=0.02, help="回放模式每行间隔秒数，默认 0.02。")
    parser.add_argument("--history", type=int, default=DEFAULT_HISTORY, help="曲线缓存帧数。")
    parser.add_argument("--warmup-frames", type=int, default=DEFAULT_WARMUP_FRAMES, help="启动预热帧数；预热期只建立 baseline，不触发 PASS。")
    parser.add_argument(
        "--iq-order",
        choices=["esp-idf", "normal"],
        default="esp-idf",
        help="复数重组顺序：esp-idf 表示 real=raw[2k+1], imag=raw[2k]；normal 表示 real=raw[2k], imag=raw[2k+1]。",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    app = QtWidgets.QApplication(sys.argv)
    win = ScoreOnlyWindow(history_len=args.history, warmup_frames=args.warmup_frames)

    thread = ReaderThread(
        port=args.port,
        baud=args.baud,
        replay_path=Path(args.replay) if args.replay else None,
        replay_interval=args.replay_interval,
        iq_order=args.iq_order,
    )
    thread.frame_ready.connect(win.update_frame)
    thread.log_ready.connect(win.append_log)

    def shutdown():
        thread.stop()
        thread.wait(1500)

    app.aboutToQuit.connect(shutdown)
    thread.start()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
