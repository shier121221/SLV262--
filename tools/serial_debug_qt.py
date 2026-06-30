#!/usr/bin/env python3
"""
PyQt serial debug host for patrol car protocol.

Protocol:
    0x55 + msg_id + len + payload + checksum
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore
except ImportError:
    serial = None


HEAD = 0x55
MSG_CAR_STATUS = 0x01
MSG_ENV_BATTERY = 0x02
MSG_TRACK_STATUS = 0x03
MSG_THERMAL_SUMMARY = 0x04
MSG_THERMAL_ROW = 0x05
MSG_FAULT_EVENT = 0x12

MSG_NAMES = {
    MSG_CAR_STATUS: "CAR",
    MSG_ENV_BATTERY: "ENV",
    MSG_TRACK_STATUS: "TRACK",
    MSG_THERMAL_SUMMARY: "THERMAL_SUM",
    MSG_THERMAL_ROW: "THERMAL_ROW",
    MSG_FAULT_EVENT: "FAULT",
}


@dataclass
class ThermalFrame:
    frame_id: int
    tick_ms: int
    rows: Dict[int, list[float]] = field(default_factory=dict)

    def add_row(self, row: int, temps: list[float]) -> bool:
        self.rows[row] = temps
        return len(self.rows) >= 24


class ProtocolParser:
    WAIT_HEAD = 0
    READ_MSG = 1
    READ_LEN = 2
    READ_PAYLOAD = 3
    READ_CHECKSUM = 4

    def __init__(self, max_payload: int = 80):
        self.max_payload = max_payload
        self.state = self.WAIT_HEAD
        self.msg_id = 0
        self.length = 0
        self.payload = bytearray()
        self.checksum = 0
        self.frames = 0
        self.bad_sum = 0
        self.bad_len = 0

    def feed(self, data: bytes):
        for b in data:
            frame = self._feed_byte(b)
            if frame is not None:
                yield frame

    def _feed_byte(self, b: int) -> Optional[tuple[int, bytes]]:
        if self.state == self.WAIT_HEAD:
            if b == HEAD:
                self.state = self.READ_MSG
            return None
        if self.state == self.READ_MSG:
            self.msg_id = b
            self.checksum = b
            self.state = self.READ_LEN
            return None
        if self.state == self.READ_LEN:
            self.length = b
            self.checksum = (self.checksum + b) & 0xFF
            self.payload.clear()
            if self.length > self.max_payload:
                self.bad_len += 1
                self.state = self.WAIT_HEAD
            else:
                self.state = self.READ_CHECKSUM if self.length == 0 else self.READ_PAYLOAD
            return None
        if self.state == self.READ_PAYLOAD:
            self.payload.append(b)
            self.checksum = (self.checksum + b) & 0xFF
            if len(self.payload) >= self.length:
                self.state = self.READ_CHECKSUM
            return None
        if self.state == self.READ_CHECKSUM:
            self.state = self.WAIT_HEAD
            if b != self.checksum:
                self.bad_sum += 1
                return None
            self.frames += 1
            return self.msg_id, bytes(self.payload)
        self.state = self.WAIT_HEAD
        return None


class MainWindow(QMainWindow):
    def __init__(self, default_port: str = "", default_baud: int = 921600):
        super().__init__()
        self.setWindowTitle("SLV262 机房巡检小车上位机")
        self.resize(1280, 780)

        self.ser = None
        self.parser = ProtocolParser()
        self.thermal_frames: Dict[int, ThermalFrame] = {}
        self.last_stat_time = time.time()

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.baud_edit = QLineEdit(str(default_baud))
        self.connect_btn = QPushButton("连接")
        self.refresh_btn = QPushButton("刷新")
        self.raw_check = QCheckBox("显示原始帧")

        self.conn_label = QLabel("未连接")
        self.state_label = QLabel("--")
        self.fault_label = QLabel("--")
        self.lap_label = QLabel("--")
        self.batt_label = QLabel("--")
        self.speed_label = QLabel("--")
        self.yaw_label = QLabel("--")
        self.temp_label = QLabel("--")
        self.humi_label = QLabel("--")
        self.voltage_label = QLabel("--")
        self.power_label = QLabel("--")
        self.track_error_label = QLabel("--")
        self.track_ready_label = QLabel("--")
        self.track_mode_label = QLabel("--")
        self.track_values_label = QLabel("--")
        self.thermal_label = QLabel("--")
        self.stat_label = QLabel("总帧:0  校验错:0  长度错:0")
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(800)

        self.table = QTableWidget(24, 32)
        self.table.setAlternatingRowColors(False)
        self.table.verticalHeader().setVisible(True)
        self.table.horizontalHeader().setVisible(True)
        for r in range(24):
            self.table.setVerticalHeaderItem(r, QTableWidgetItem(str(r)))
        for c in range(32):
            self.table.setHorizontalHeaderItem(c, QTableWidgetItem(str(c)))
        for r in range(24):
            for c in range(32):
                item = QTableWidgetItem("")
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(r, c, item)
        for c in range(32):
            self.table.setColumnWidth(c, 42)
        for r in range(24):
            self.table.setRowHeight(r, 24)

        self._build_ui()
        self._apply_style()
        self.refresh_ports(default_port)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll_serial)
        self.timer.start(20)

    def _build_ui(self):
        top = QHBoxLayout()
        top.addWidget(QLabel("串口"))
        top.addWidget(self.port_combo, 2)
        top.addWidget(QLabel("波特率"))
        top.addWidget(self.baud_edit)
        top.addWidget(self.refresh_btn)
        top.addWidget(self.connect_btn)
        top.addWidget(self.raw_check)
        top.addWidget(self.conn_label)

        self.refresh_btn.clicked.connect(lambda: self.refresh_ports(self.port_combo.currentText()))
        self.connect_btn.clicked.connect(self.toggle_serial)

        run_box = self._make_card(
            "运行状态",
            [
                ("状态", self.state_label),
                ("故障", self.fault_label),
                ("圈数", self.lap_label),
                ("电量", self.batt_label),
                ("速度", self.speed_label),
                ("角度", self.yaw_label),
            ],
        )
        env_box = self._make_card(
            "环境与电池",
            [
                ("温度", self.temp_label),
                ("湿度", self.humi_label),
                ("电压", self.voltage_label),
                ("SHT31状态", self.power_label),
            ],
        )
        track_box = self._make_card(
            "八路巡线",
            [
                ("偏差", self.track_error_label),
                ("有效", self.track_ready_label),
                ("模式", self.track_mode_label),
                ("模拟量", self.track_values_label),
            ],
        )
        comm_box = self._make_card(
            "通信",
            [
                ("解析", self.stat_label),
                ("热成像", self.thermal_label),
            ],
        )

        left = QVBoxLayout()
        left.addLayout(top)
        left.addWidget(run_box)
        left.addWidget(env_box)
        left.addWidget(track_box)
        left.addWidget(comm_box)
        left.addWidget(QLabel("日志"))
        left.addWidget(self.log, 1)

        root = QHBoxLayout()
        left_widget = QWidget()
        left_widget.setLayout(left)
        root.addWidget(left_widget, 2)
        thermal_wrap = QVBoxLayout()
        thermal_title = QLabel("热成像 32 x 24")
        thermal_title.setObjectName("sectionTitle")
        thermal_wrap.addWidget(thermal_title)
        thermal_wrap.addWidget(self.table, 1)
        thermal_widget = QWidget()
        thermal_widget.setLayout(thermal_wrap)
        root.addWidget(thermal_widget, 3)

        central = QWidget()
        central.setLayout(root)
        self.setCentralWidget(central)

    def _make_card(self, title: str, rows: list[tuple[str, QLabel]]) -> QGroupBox:
        box = QGroupBox(title)
        layout = QGridLayout()
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(8)
        for row, (name, value) in enumerate(rows):
            name_label = QLabel(name)
            name_label.setObjectName("fieldName")
            value.setObjectName("fieldValue")
            value.setWordWrap(True)
            layout.addWidget(name_label, row, 0)
            layout.addWidget(value, row, 1)
        layout.setColumnStretch(1, 1)
        box.setLayout(layout)
        return box

    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow {
                background: #f4f6f8;
                font-family: "Microsoft YaHei", "Segoe UI";
                font-size: 13px;
            }
            QGroupBox {
                background: #ffffff;
                border: 1px solid #d9e0e7;
                border-radius: 8px;
                margin-top: 12px;
                padding: 12px;
                font-weight: 600;
                color: #1f2937;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
            }
            QLabel#fieldName {
                color: #667085;
                font-weight: 400;
                min-width: 64px;
            }
            QLabel#fieldValue {
                color: #111827;
                font-weight: 600;
            }
            QLabel#sectionTitle {
                color: #111827;
                font-size: 18px;
                font-weight: 700;
                padding: 4px 0;
            }
            QPushButton {
                background: #2563eb;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 7px 14px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #1d4ed8;
            }
            QComboBox, QLineEdit {
                background: white;
                border: 1px solid #cfd8e3;
                border-radius: 6px;
                padding: 6px;
                min-height: 20px;
            }
            QPlainTextEdit {
                background: #101828;
                color: #d1e7ff;
                border: 1px solid #26344a;
                border-radius: 8px;
                padding: 8px;
                font-family: Consolas, "Microsoft YaHei";
            }
            QTableWidget {
                background: #ffffff;
                gridline-color: #e5e7eb;
                border: 1px solid #d9e0e7;
                border-radius: 8px;
                font-size: 11px;
            }
            QHeaderView::section {
                background: #eef2f7;
                color: #475467;
                border: none;
                padding: 3px;
            }
            """
        )

    def refresh_ports(self, keep: str = ""):
        self.port_combo.clear()
        ports = []
        if serial is not None:
            ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        if keep:
            index = self.port_combo.findText(keep)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
            else:
                self.port_combo.setEditText(keep)

    def toggle_serial(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None
            self.connect_btn.setText("连接")
            self.conn_label.setText("未连接")
            self.append_log("串口已关闭")
            return

        if serial is None:
            QMessageBox.critical(self, "缺少依赖", "需要安装 pyserial：pip install pyserial")
            return

        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "串口", "请选择或输入串口号")
            return

        try:
            baud = int(self.baud_edit.text().strip())
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0)
        except Exception as exc:
            QMessageBox.critical(self, "打开失败", str(exc))
            self.ser = None
            return

        self.connect_btn.setText("断开")
        self.conn_label.setText(f"已连接 {port}")
        self.append_log(f"打开串口 {port} @ {baud}")

    def append_log(self, text: str):
        now = time.strftime("%H:%M:%S")
        self.log.appendPlainText(f"[{now}] {text}")

    def poll_serial(self):
        if self.ser is None:
            return
        try:
            data = self.ser.read(1024)
        except Exception as exc:
            self.append_log(f"串口错误: {exc}")
            self.toggle_serial()
            return

        try:
            for msg_id, payload in self.parser.feed(data):
                if self.raw_check.isChecked():
                    self.append_log(f"原始帧 id=0x{msg_id:02X} len={len(payload)} {payload.hex(' ')}")
                self.handle_frame(msg_id, payload)
        except Exception as exc:
            self.append_log(f"解析错误: {exc}")

        now = time.time()
        if now - self.last_stat_time >= 0.5:
            self.last_stat_time = now
            self.stat_label.setText(
                f"总帧:{self.parser.frames}  校验错:{self.parser.bad_sum}  长度错:{self.parser.bad_len}"
            )

    def handle_frame(self, msg_id: int, payload: bytes):
        if msg_id == MSG_CAR_STATUS:
            self.handle_car(payload)
        elif msg_id == MSG_ENV_BATTERY:
            self.handle_env(payload)
        elif msg_id == MSG_TRACK_STATUS:
            self.handle_track(payload)
        elif msg_id == MSG_THERMAL_SUMMARY:
            self.handle_thermal_summary(payload)
        elif msg_id == MSG_THERMAL_ROW:
            self.handle_thermal_row(payload)
        elif msg_id == MSG_FAULT_EVENT:
            self.handle_fault(payload)
        else:
            name = MSG_NAMES.get(msg_id, f"0x{msg_id:02X}")
            self.append_log(f"未知帧 {name} len={len(payload)}")

    def handle_car(self, payload: bytes):
        if len(payload) < 14:
            self.append_log(f"小车状态长度错误 len={len(payload)}")
            return
        tick, state, fault, lap, batt, spd_l, spd_r, yaw = struct.unpack_from("<IBBBBhhh", payload, 0)
        self.state_label.setText(f"{state}  ({tick} ms)")
        self.fault_label.setText("无" if fault == 0 else f"故障码 {fault}")
        self.lap_label.setText(str(lap))
        self.batt_label.setText(f"{batt}%")
        self.speed_label.setText(f"左 {spd_l} cm/s    右 {spd_r} cm/s")
        self.yaw_label.setText(f"{yaw / 100:.2f} deg")

    def handle_env(self, payload: bytes):
        if len(payload) < 12:
            self.append_log(f"环境帧长度错误 len={len(payload)}")
            return
        tick, temp, humi, batt_mv, batt_pct, pwr = struct.unpack_from("<IhhHBB", payload, 0)
        self.temp_label.setText(f"{temp / 100:.2f} °C")
        self.humi_label.setText(f"{humi / 100:.2f} %RH")
        self.voltage_label.setText(f"{batt_mv} mV / {batt_pct}%")
        self.power_label.setText(f"{self.sht31_status_text(pwr)}  ({tick} ms)")

    def sht31_status_text(self, code: int) -> str:
        names = {
            0: "正常",
            1: "参数错误",
            2: "发送测量命令失败",
            3: "读取数据失败",
            4: "温度CRC错误",
            5: "湿度CRC错误",
            6: "未找到传感器",
        }
        if code >= 20:
            stage = code // 10
            detail = code % 10
            stage_name = names.get(stage, f"阶段{stage}")
            detail_names = {
                1: "总线忙",
                2: "START失败",
                3: "地址写ACK失败",
                4: "地址读ACK失败",
                5: "寄存器字节失败",
                6: "数据1失败",
                7: "数据2失败",
                8: "RESTART失败",
                9: "接收字节失败",
            }
            return f"{stage_name}: {detail_names.get(detail, f'I2C错误{detail}')} ({code})"
        return names.get(code, f"错误码 {code}")

    def handle_track(self, payload: bytes):
        if len(payload) != 24:
            self.append_log(f"巡线帧长度错误 len={len(payload)}")
            return
        tick, err, ready, use_analog = struct.unpack_from("<IhBB", payload, 0)
        analog = struct.unpack_from("<8H", payload, 8)
        self.track_error_label.setText(str(err))
        self.track_ready_label.setText("有效" if ready else "无效")
        self.track_mode_label.setText("模拟量" if use_analog else "数字量")
        self.track_values_label.setText(" ".join(str(v) for v in analog))

    def handle_thermal_summary(self, payload: bytes):
        if len(payload) < 14:
            self.append_log(f"热成像摘要长度错误 len={len(payload)}")
            return
        tick, frame_id, max_t, min_t, avg_t, max_x, max_y = struct.unpack_from("<IHhhhBB", payload, 0)
        self.thermal_label.setText(
            f"帧 {frame_id}  最大 {max_t / 100:.2f} °C @({max_x},{max_y})  "
            f"最小 {min_t / 100:.2f} °C  平均 {avg_t / 100:.2f} °C"
        )

    def handle_thermal_row(self, payload: bytes):
        if len(payload) != 72:
            self.append_log(f"热成像行长度错误 len={len(payload)}")
            return
        tick, frame_id, row, width = struct.unpack_from("<IHBB", payload, 0)
        raw = struct.unpack_from("<32H", payload, 8)
        temps = [v * 0.02 for v in raw]

        frame = self.thermal_frames.get(frame_id)
        if frame is None:
            frame = ThermalFrame(frame_id=frame_id, tick_ms=tick)
            self.thermal_frames[frame_id] = frame

        complete = frame.add_row(row, temps)
        self.update_table_row(row, temps)
        self.thermal_label.setText(f"帧 {frame_id}  行 {row}/23  已收 {len(frame.rows)} 行")

        if complete:
            max_temp = max(max(r) for r in frame.rows.values())
            min_temp = min(min(r) for r in frame.rows.values())
            self.append_log(f"热成像帧 {frame_id} 完整  min={min_temp:.2f}°C max={max_temp:.2f}°C")
            self.thermal_frames.pop(frame_id, None)

        if len(self.thermal_frames) > 4:
            oldest = sorted(self.thermal_frames.keys())[0]
            self.thermal_frames.pop(oldest, None)

    def update_table_row(self, row: int, temps: list[float]):
        if row >= 24:
            return
        low = min(temps)
        high = max(temps)
        span = max(high - low, 0.01)
        for col, temp in enumerate(temps[:32]):
            item = self.table.item(row, col)
            if item is None:
                item = QTableWidgetItem()
                self.table.setItem(row, col, item)
            item.setText(f"{temp:.1f}")
            ratio = (temp - low) / span
            red = int(40 + ratio * 215)
            blue = int(255 - ratio * 180)
            item.setBackground(QColor(red, 60, blue))

    def handle_fault(self, payload: bytes):
        if len(payload) < 7:
            self.append_log(f"故障帧长度错误 len={len(payload)}")
            return
        tick, code, detail = struct.unpack_from("<IBH", payload, 0)
        self.append_log(f"故障事件 t={tick}ms code={code} detail={detail}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="")
    ap.add_argument("--baud", type=int, default=921600)
    args = ap.parse_args()

    app = QApplication(sys.argv)
    win = MainWindow(default_port=args.port, default_baud=args.baud)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
