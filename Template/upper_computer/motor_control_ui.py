"""
电机控制上位机 - 基于 PySide6
通讯协议: 自定义串口协议 (SOF+LENGTH+DATA+CRC+EOF)
"""

import sys
import struct
import time
from datetime import datetime
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QLabel, QComboBox, QSpinBox, QTextEdit,
    QGridLayout, QFrame, QSizePolicy, QMessageBox, QDoubleSpinBox
)
from PySide6.QtCore import Qt, QTimer, Signal, QThread
from PySide6.QtGui import QFont, QColor, QTextCursor, QPalette

import serial
import serial.tools.list_ports


# ============================================================
# 协议常量
# ============================================================
SOF = 0xA5
EOF = 0x5A


class FuncCode(IntEnum):
    FC_READ = 0x01
    FC_WRITE = 0x02
    FC_CONTROL = 0x03
    FC_RESPONSE = 0x81
    FC_EVENT = 0x82


class CmdCode(IntEnum):
    CMD_FORWARD = 0x0001
    CMD_REVERSE = 0x0002
    CMD_STOP = 0x0003
    CMD_BRAKE = 0x0004


class MotorState(IntEnum):
    STOP = 0
    FORWARD = 1
    REVERSE = 2
    BRAKE = 3


# 寄存器偏移
REG_MOTOR_STATE = 0x0002
REG_COUNT = 6  # 一次读取 6 个寄存器


# ============================================================
# 协议编解码
# ============================================================
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_frame(addr: int, func: int, data: bytes) -> bytes:
    length = len(data)
    payload = struct.pack(">BB", func, length) + data
    crc = crc16(payload)
    frame = struct.pack(">BB", SOF, addr) + payload + struct.pack("<H", crc) + struct.pack(">B", EOF)
    return frame


def parse_frame(frame: bytes) -> Optional[tuple]:
    if len(frame) < 7:
        return None
    if frame[0] != SOF or frame[-1] != EOF:
        return None
    addr = frame[1]
    func = frame[2]
    length = frame[3]
    if len(frame) != 7 + length:
        return None
    data = frame[4:4 + length]
    crc_recv = struct.unpack("<H", frame[4 + length:6 + length])[0]
    payload = frame[2:4 + length]
    if crc16(payload) != crc_recv:
        return None
    return (addr, func, data)


# ============================================================
# 数据模型
# ============================================================
@dataclass
class MotorData:
    state: int = 0
    current: int = 0
    speed: int = 0
    stroke: int = 0
    position_detected: int = 0

    STATE_NAMES = {0: "停止", 1: "正转", 2: "反转", 3: "刹车"}

    @property
    def state_name(self) -> str:
        return self.STATE_NAMES.get(self.state, "未知")


# ============================================================
# 串口工作线程
# ============================================================
class SerialWorker(QThread):
    data_updated = Signal(MotorData)
    error_occurred = Signal(str)
    log_message = Signal(str)
    connected = Signal(bool)

    def __init__(self):
        super().__init__()
        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._port = ""
        self._baudrate = 115200
        self._slave_addr = 0x01
        self._rx_buffer = bytearray()

    def set_config(self, port: str, baudrate: int, slave_addr: int):
        self._port = port
        self._baudrate = baudrate
        self._slave_addr = slave_addr

    def connect(self):
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02
            )
            self._running = True
            self._rx_buffer.clear()
            self.connected.emit(True)
            self.log_message.emit(f"串口 {self._port} 已连接, 波特率 {self._baudrate}")
        except Exception as e:
            self.error_occurred.emit(f"连接失败: {str(e)}")

    def disconnect(self):
        self._running = False
        if self._serial and self._serial.is_open:
            self._serial.close()
        self.connected.emit(False)
        self.log_message.emit("串口已断开")

    def send_frame(self, func: int, data: bytes = b""):
        if not self._serial or not self._serial.is_open:
            self.error_occurred.emit("串口未连接")
            return
        frame = build_frame(self._slave_addr, func, data)
        self._serial.write(frame)
        self.log_message.emit(f"发送: {frame.hex(' ').upper()}")

    def send_control(self, cmd: int, speed: int = 0):
        data = struct.pack(">HH", cmd, speed)
        self.send_frame(FuncCode.FC_CONTROL, data)

    def send_read_status(self):
        data = struct.pack(">HH", REG_MOTOR_STATE, REG_COUNT)
        self.send_frame(FuncCode.FC_READ, data)

    def _parse_response(self, data: bytes) -> Optional[MotorData]:
        if len(data) < 12:
            print(f"[DBG] parse_resp: data too short len={len(data)}")
            return None
        try:
            m = MotorData()
            m.state = struct.unpack(">H", data[0:2])[0]
            m.current = struct.unpack(">H", data[2:4])[0]
            m.speed = struct.unpack(">H", data[4:6])[0]
            m.stroke = struct.unpack(">H", data[6:8])[0]
            m.position_detected = struct.unpack(">H", data[10:12])[0]
            print(f"[DBG] parsed: raw_data={data.hex(' ')} state={m.state} cur={m.current} spd={m.speed}")
            return m
        except struct.error:
            print(f"[DBG] parse_resp: struct.error")
            return None

    def run(self):
        while self._running:
            if self._serial and self._serial.is_open:
                try:
                    n = self._serial.in_waiting
                    if n > 0:
                        chunk = self._serial.read(n)
                        self._rx_buffer.extend(chunk)
                        self._process_buffer()
                except serial.SerialException as e:
                    self.error_occurred.emit(f"串口错误: {str(e)}")
                    self.disconnect()
                    break
            self.msleep(10)

    def _process_buffer(self):
        while True:
            sof_idx = self._rx_buffer.find(bytes([SOF]))
            if sof_idx < 0:
                break
            if sof_idx > 0:
                self._rx_buffer = self._rx_buffer[sof_idx:]

            if len(self._rx_buffer) < 7:
                break

            length = self._rx_buffer[3]
            frame_len = 7 + length
            if len(self._rx_buffer) < frame_len:
                break

            frame = bytes(self._rx_buffer[:frame_len])
            self._rx_buffer = self._rx_buffer[frame_len:]

            self.log_message.emit(f"RAW: {frame.hex(' ').upper()}")

            self.log_message.emit(f"RAW: {frame.hex(' ').upper()}")

            result = parse_frame(frame)
            if result is None:
                self.log_message.emit(f"无效帧: {frame.hex(' ').upper()}")
                continue

            addr, func, data = result
            if addr != self._slave_addr and func != FuncCode.FC_EVENT:
                continue

            self.log_message.emit(f"收到: {frame.hex(' ').upper()}")

            if func == FuncCode.FC_RESPONSE:
                md = self._parse_response(data)
                if md:
                    self.data_updated.emit(md)
            elif func == FuncCode.FC_EVENT:
                if len(data) >= 4:
                    event = struct.unpack(">H", data[0:2])[0]
                    current = struct.unpack(">H", data[2:4])[0]
                    if event == 0x01:
                        self.log_message.emit(f"⚠ 堵转告警! 电流: {current}mA")


# ============================================================
# 状态指示圆点
# ============================================================
class StatusDot(QFrame):
    def __init__(self):
        super().__init__()
        self.setFixedSize(16, 16)
        self.setStyleSheet("background: gray; border-radius: 8px; border: 1px solid #888;")

    def set_state(self, color: str):
        self.setStyleSheet(f"background: {color}; border-radius: 8px; border: 1px solid #888;")


# ============================================================
# 主界面
# ============================================================
class MotorControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("电机控制系统 V1.0")
        self.setMinimumSize(800, 650)

        self._motor_data = MotorData()
        self._worker = SerialWorker()

        self._worker.data_updated.connect(self._on_data_updated)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.log_message.connect(self._on_log)
        self._worker.connected.connect(self._on_connected)

        self._setup_ui()
        self._refresh_ports()

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(10)

        # ---- 连接设置 ----
        conn_group = QGroupBox("连接设置")
        conn_layout = QHBoxLayout(conn_group)

        conn_layout.addWidget(QLabel("串口:"))
        self._combo_port = QComboBox()
        self._combo_port.setMinimumWidth(100)
        conn_layout.addWidget(self._combo_port)

        conn_layout.addWidget(QLabel("波特率:"))
        self._combo_baud = QComboBox()
        self._combo_baud.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "460800"])
        self._combo_baud.setCurrentText("115200")
        conn_layout.addWidget(self._combo_baud)

        conn_layout.addWidget(QLabel("地址:"))
        self._spin_addr = QSpinBox()
        self._spin_addr.setRange(1, 247)
        self._spin_addr.setValue(1)
        self._spin_addr.setFixedWidth(60)
        conn_layout.addWidget(self._spin_addr)

        self._btn_refresh = QPushButton("刷新")
        self._btn_refresh.clicked.connect(self._refresh_ports)
        conn_layout.addWidget(self._btn_refresh)

        self._btn_connect = QPushButton("连接")
        self._btn_connect.clicked.connect(self._toggle_connect)
        self._btn_connect.setFixedWidth(80)
        conn_layout.addWidget(self._btn_connect)

        self._dot_status = StatusDot()
        conn_layout.addWidget(self._dot_status)

        conn_layout.addStretch()
        layout.addWidget(conn_group)

        # ---- 控制面板 ----
        ctrl_group = QGroupBox("电机控制")
        ctrl_layout = QHBoxLayout(ctrl_group)
        ctrl_layout.setSpacing(15)

        ctrl_layout.addWidget(QLabel("方向:"))
        self._combo_direction = QComboBox()
        self._combo_direction.addItems(["正转", "反转"])
        self._combo_direction.setCurrentIndex(0)
        self._combo_direction.setFixedWidth(80)
        ctrl_layout.addWidget(self._combo_direction)

        ctrl_layout.addWidget(QLabel("转速(RPM):"))
        self._spin_speed = QSpinBox()
        self._spin_speed.setRange(0, 6000)
        self._spin_speed.setSingleStep(100)
        self._spin_speed.setValue(500)
        self._spin_speed.setFixedWidth(80)
        ctrl_layout.addWidget(self._spin_speed)

        btn_small = "QPushButton { font-size: 14px; padding: 8px 16px; border-radius: 6px; }"

        self._btn_set_speed = QPushButton("设置转速")
        self._btn_set_speed.setStyleSheet(btn_small + "QPushButton { background: #2196F3; color: white; } QPushButton:hover { background: #42A5F5; }")
        self._btn_set_speed.clicked.connect(self._set_speed)
        ctrl_layout.addWidget(self._btn_set_speed)

        ctrl_layout.addSpacing(25)

        self._btn_stop = QPushButton("■ 停止")
        self._btn_stop.setStyleSheet(btn_small + "QPushButton { background: #FF9800; color: white; } QPushButton:hover { background: #FFA726; }")
        self._btn_stop.clicked.connect(self._stop_motor)
        ctrl_layout.addWidget(self._btn_stop)

        ctrl_layout.addStretch()

        layout.addWidget(ctrl_group)

        # ---- 实时数据 ----
        data_group = QGroupBox("实时数据")
        data_grid = QGridLayout(data_group)
        data_grid.setSpacing(10)

        font_label = QFont("Microsoft YaHei", 10)
        font_value = QFont("Consolas", 14, QFont.Bold)

        self._labels = {}

        items = [
            ("电机状态:", 0, 0), ("_state", 0, 1),
            ("实时电流:", 0, 2), ("_current", 0, 3),
            ("实时转速:", 1, 0), ("_speed", 1, 1),
            ("行程计数:", 1, 2), ("_stroke", 1, 3),
            ("到位检测:", 2, 0), ("_position", 2, 1),
        ]

        for label_text, row, col in zip(items[::2], *[iter(range(5))] * 2):
            pass

        row_data = [
            ("电机状态:", "停止"),
            ("实时电流:", "0 mA"),
            ("实时转速:", "0 RPM"),
            ("行程计数:", "0"),
            ("到位检测:", "未到位"),
        ]

        for i, (label, default) in enumerate(row_data):
            r, c = i // 2 * 2, (i % 2) * 2
            lbl = QLabel(label)
            lbl.setFont(font_label)
            data_grid.addWidget(lbl, r, c)

            val = QLabel(default)
            val.setFont(font_value)
            val.setStyleSheet("color: white;")
            data_grid.addWidget(val, r, c + 1)
            self._labels[label] = val

        layout.addWidget(data_group)

        # ---- 日志 ----
        log_group = QGroupBox("通讯日志")
        log_layout = QVBoxLayout(log_group)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFont(QFont("Consolas", 9))
        #self._log.setMaximumBlockCount(500)
        self._log.document().setMaximumBlockCount(500)
        log_layout.addWidget(self._log)

        btn_clear = QPushButton("清空日志")
        btn_clear.clicked.connect(self._log.clear)
        log_layout.addWidget(btn_clear)

        layout.addWidget(log_group)

    # ---- 事件处理 ----
    def _refresh_ports(self):
        self._combo_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self._combo_port.addItem(f"{p.device} - {p.description}")
        if len(ports) == 0:
            self._combo_port.addItem("无可用串口")

    def _toggle_connect(self):
        if self._worker.isRunning():
            self._worker.disconnect()
            self._worker.wait(1000)
            self._btn_connect.setText("连接")
            self._dot_status.set_state("gray")
        else:
            port_text = self._combo_port.currentText()
            port = port_text.split(" - ")[0] if " - " in port_text else port_text
            self._worker.set_config(port, int(self._combo_baud.currentText()), self._spin_addr.value())
            self._worker.connect()
            self._worker.start()
            self._btn_connect.setText("断开")
            self._dot_status.set_state("red")

    _DIRECTION_CMD = {
        0: CmdCode.CMD_FORWARD,
        1: CmdCode.CMD_REVERSE,
    }

    def _stop_motor(self):
        self._worker.send_control(CmdCode.CMD_STOP, 0)

    def _set_speed(self):
        direction = self._combo_direction.currentIndex()
        cmd = self._DIRECTION_CMD.get(direction, CmdCode.CMD_FORWARD)
        speed = self._spin_speed.value()
        self._worker.send_control(cmd, speed)

    def _on_data_updated(self, data: MotorData):
        self._motor_data = data
        print(f"[UI] received: state={data.state} cur={data.current} spd={data.speed} strk={data.stroke} pos={data.position_detected}")
        self._labels["电机状态:"].setText(data.state_name)
        self._labels["实时电流:"].setText(f"{data.current} mA")
        self._labels["实时转速:"].setText(f"{data.speed} RPM")
        self._labels["行程计数:"].setText(str(data.stroke))
        self._labels["到位检测:"].setText("已到位" if data.position_detected else "未到位")

        # 堵转告警
        if data.current > 3000:
            self._labels["实时电流:"].setStyleSheet("color: red;")
        else:
            self._labels["实时电流:"].setStyleSheet("color: white;")

    def _on_error(self, msg: str):
        QMessageBox.warning(self, "错误", msg)
        self._add_log(msg, "red")

    def _on_log(self, msg: str):
        self._add_log(msg)

    def _on_connected(self, connected: bool):
        if connected:
            self._dot_status.set_state("green")
        else:
            self._dot_status.set_state("gray")

    def _add_log(self, msg: str, color: str = None):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        text = f"[{timestamp}] {msg}"
        if color:
            self._log.setTextColor(QColor(color))
        self._log.append(text)
        self._log.setTextColor(QColor("black"))
        self._log.moveCursor(QTextCursor.End)

    def closeEvent(self, event):
        if self._worker.isRunning():
            self._worker.disconnect()
            self._worker.wait(2000)
        event.accept()


# ============================================================
# 入口
# ============================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # 暗色主题
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(35, 35, 35))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)

    window = MotorControlUI()
    window.show()
    sys.exit(app.exec())