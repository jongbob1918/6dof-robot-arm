import sys
import serial
from PyQt6.QtWidgets import (
    QApplication, QWidget, QSlider, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QCheckBox, QListWidget, QMessageBox
)
from PyQt6.QtCore import Qt

class RobotArmController(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Robot Arm Controller")
        self.commands = []

        # 시리얼 포트 설정
        try:
            self.ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=2)
        except Exception as e:
            QMessageBox.critical(self, "Serial Error", str(e))
            sys.exit()

        # 슬라이더 구성
        self.sliders = []
        self.slider_labels = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2"]
        self.slider_ranges = [
            (0, 180),  # Base
            (0, 180),  # Shoulder
            (0, 180),  # Elbow
            (0, 180),  # Wrist1
            (0, 180),  # Wrist2
        ]

        for label_text, (min_val, max_val) in zip(self.slider_labels, self.slider_ranges):
            label = QLabel(f"{label_text}: 0")
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(min_val, max_val)
            slider.setValue(0)
            slider.valueChanged.connect(lambda val, l=label, t=label_text: l.setText(f"{t}: {val}"))
            self.sliders.append((label, slider))

        # 버튼 및 리스트
        self.command_list = QListWidget()
        self.add_btn = QPushButton("Add Command")
        self.remove_btn = QPushButton("Remove Command")
        self.send_all_btn = QPushButton("Send All")
        self.live_checkbox = QCheckBox("Send Live")
        self.home_btn = QPushButton("Set Home")

        # 레이아웃 설정
        layout = QVBoxLayout()
        for lbl, sld in self.sliders:
            layout.addWidget(lbl)
            layout.addWidget(sld)

        layout.addWidget(self.live_checkbox)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self.add_btn)
        btn_row.addWidget(self.remove_btn)
        layout.addLayout(btn_row)

        layout.addWidget(self.command_list)

        btn_row2 = QHBoxLayout()
        btn_row2.addWidget(self.send_all_btn)
        btn_row2.addWidget(self.home_btn)
        layout.addLayout(btn_row2)

        self.setLayout(layout)

        # 이벤트 연결
        self.add_btn.clicked.connect(self.add_command)
        self.remove_btn.clicked.connect(self.remove_command)
        self.send_all_btn.clicked.connect(self.send_all)
        self.home_btn.clicked.connect(self.set_home)

        for _, sld in self.sliders:
            sld.valueChanged.connect(self.send_live_if_needed)

    def make_command(self):
        values = [slider.value() for _, slider in self.sliders]
        return f"mov,arms,{','.join(str(v) for v in values)}\n"

    def add_command(self):
        cmd = self.make_command()
        self.commands.append(cmd)
        self.command_list.addItem(cmd.strip())

    def remove_command(self):
        row = self.command_list.currentRow()
        if row >= 0:
            self.commands.pop(row)
            self.command_list.takeItem(row)

    def send_all(self):
        for cmd in self.commands:
            self.ser.write(cmd.encode('utf-8'))
            print("Sent:", cmd.strip())
            if self.ser.in_waiting:
                print("Recv:", self.ser.readline().decode('utf-8'))

    def send_live_if_needed(self):
        if self.live_checkbox.isChecked():
            cmd = self.make_command()
            self.ser.write(cmd.encode('utf-8'))

    def set_home(self):
        self.ser.write(b"mov,default\n")
        for _, sld in self.sliders:
            sld.setValue(0)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = RobotArmController()
    win.show()
    sys.exit(app.exec())
