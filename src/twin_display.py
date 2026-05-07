#!/usr/bin/env python3
"""
数字孪生实时显示: PyQt6 双面板连杆机构渲染。

通过 UDP 接收 Unit5 固件发来的 4 路电机角度, 左右腿并排显示。

用法:
  .venv/bin/python -m src.twin_display --port 9876

数据格式 (UDP 每帧一行):
  t_ms,left_θa,left_θb,right_θa,right_θb
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from PyQt6.QtCore import QTimer
from PyQt6.QtNetwork import QUdpSocket, QHostAddress
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout,
    QVBoxLayout, QLabel, QStatusBar,
)
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont
from PyQt6.QtCore import Qt, QRectF

# 复用现有运动学模型
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mechanism import MechanismParams, default_params
from src.kinematics import solve_linkage


# ---- 颜色方案 (与 visualization.py 一致) ----
BAR_COLORS = {
    'bar_a':   QColor('#2196F3'),  # blue
    'bar_b':   QColor('#4CAF50'),  # green
    'bar_c':   QColor('#FF9800'),  # orange
    'bar_d':   QColor('#9C27B0'),  # purple
    'bar_e':   QColor('#F44336'),  # red
    'bar_f':   QColor('#00BCD4'),  # cyan
}
COLOR_JOINT  = QColor('#333333')
COLOR_MOTOR  = QColor('#E91E63')
COLOR_WHEEL  = QColor('#FF5722')
COLOR_GROUND = QColor('#795548')
COLOR_BG     = QColor('#FAFAFA')
COLOR_GRID   = QColor('#E0E0E0')

VIEW_MARGIN = 30
WHEEL_RADIUS = 30.0  # mm

# 杆绘制列表: (起点键, 终点键, 颜色键, 线宽, 虚线)
BAR_SPECS = [
    ('O',  'P1', 'bar_a', 3.0, False),
    ('O',  'P2', 'bar_a', 3.0, False),
    ('P1', 'P2', 'bar_a', 1.5, True),
    ('O',  'P3', 'bar_b', 3.0, False),
    ('P3', 'P4', 'bar_c', 2.5, False),
    ('P1', 'P4', 'bar_d', 3.0, False),
    ('P1', 'P5', 'bar_d', 1.5, True),
    ('P4', 'P5', 'bar_d', 1.5, True),
    ('P5', 'P6', 'bar_e', 2.5, False),
    ('P2', 'P6', 'bar_f', 3.0, False),
    ('P2', 'P7', 'bar_f', 3.0, False),
    ('P6', 'P7', 'bar_f', 1.5, True),
]


def rotate_view(v: np.ndarray) -> np.ndarray:
    """机构坐标系 → 显示坐标系 (旋转 -90°, 腿向下)"""
    return np.array([v[1], -v[0]])


class LegPanel(QWidget):
    """单腿连杆机构绘制面板"""

    def __init__(self, title: str, params: MechanismParams):
        super().__init__()
        self._title = title
        self._params = params
        self._result = None       # solve_linkage 返回的 dict
        self._theta_a = 0.0
        self._theta_b = 0.0
        self._frame_count = 0
        self.setMinimumSize(350, 400)
        self.setMouseTracking(True)

    def update_angles(self, theta_a: float, theta_b: float):
        self._theta_a = theta_a
        self._theta_b = theta_b
        self._result = solve_linkage(theta_a, theta_b, self._params)
        self._frame_count += 1
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2

        # 背景
        painter.fillRect(0, 0, w, h, COLOR_BG)

        # 标题
        font_title = QFont('Sans', 12, QFont.Weight.Bold)
        painter.setFont(font_title)
        painter.setPen(QColor('#333333'))
        painter.drawText(QRectF(10, 6, w - 20, 22), Qt.AlignmentFlag.AlignCenter, self._title)

        # 网格
        painter.setPen(QPen(COLOR_GRID, 0.5))
        grid_step = 30
        for x in range(0, w, grid_step):
            painter.drawLine(x, 0, x, h)
        for y in range(0, h, grid_step):
            painter.drawLine(0, y, w, y)

        # 原点十字
        painter.setPen(QPen(QColor('#CCCCCC'), 1))
        painter.drawLine(cx - 20, cy, cx + 20, cy)
        painter.drawLine(cx, cy - 20, cx, cy + 20)

        if self._result is None:
            painter.setPen(QColor('#999999'))
            font = QFont('Sans', 11)
            painter.setFont(font)
            painter.drawText(QRectF(0, cy - 15, w, 30),
                             Qt.AlignmentFlag.AlignCenter, "等待数据...")
            painter.end()
            return

        res = self._result
        scale = self._compute_scale()

        def to_px(v):
            return np.array([cx + v[0] * scale, cy - v[1] * scale])

        # 绘制杆
        for ka, kb, color_key, lw, dashed in BAR_SPECS:
            a = to_px(rotate_view(res[ka]))
            b = to_px(rotate_view(res[kb]))
            pen = QPen(BAR_COLORS[color_key], lw)
            if dashed:
                pen.setStyle(Qt.PenStyle.DashLine)
            painter.setPen(pen)
            painter.drawLine(int(a[0]), int(a[1]), int(b[0]), int(b[1]))

        # 绘制关节
        joint_radius = 4
        for key in ['P1', 'P2', 'P3', 'P4', 'P5', 'P6']:
            p = to_px(rotate_view(res[key]))
            painter.setBrush(QBrush(COLOR_JOINT))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(int(p[0] - joint_radius), int(p[1] - joint_radius),
                                joint_radius * 2, joint_radius * 2)

        # 电机 O
        p_o = to_px(rotate_view(res['O']))
        painter.setBrush(QBrush(COLOR_MOTOR))
        painter.drawEllipse(int(p_o[0] - 7), int(p_o[1] - 7), 14, 14)

        # 末端 P7 + 轮子
        p7 = to_px(rotate_view(res['P7']))
        wheel_px = WHEEL_RADIUS * scale
        painter.setPen(QPen(QColor('#15191f'), 1.5))
        painter.setBrush(QBrush(QColor('#3f4650')))
        painter.drawEllipse(int(p7[0] - wheel_px), int(p7[1] - wheel_px),
                            int(wheel_px * 2), int(wheel_px * 2))
        # 轮毂
        hub_px = wheel_px * 0.18
        painter.setBrush(QBrush(QColor('#d7dde7')))
        painter.drawEllipse(int(p7[0] - hub_px), int(p7[1] - hub_px),
                            int(hub_px * 2), int(hub_px * 2))

        # 角度信息
        font_info = QFont('Monospace', 9)
        painter.setFont(font_info)
        painter.setPen(QColor('#555555'))
        deg_a = np.rad2deg(self._theta_a)
        deg_b = np.rad2deg(self._theta_b)
        y_base = h - 38
        painter.drawText(8, y_base, f"θa={deg_a:+7.1f}°")
        painter.drawText(8, y_base + 15, f"θb={deg_b:+7.1f}°")

        p7_pos = res['P7']
        painter.drawText(8, y_base + 30,
                         f"P7=({p7_pos[0]:.0f}, {p7_pos[1]:.0f}) mm")

        painter.end()

    def _compute_scale(self) -> float:
        """根据 P7 可达范围计算像素缩放"""
        r_max = self._params.L_OP2 + self._params.L_P2P7  # ~235.4 mm
        view_size = r_max + VIEW_MARGIN
        min_dim = min(self.width(), self.height()) / 2
        return min_dim / view_size


class TwinWindow(QMainWindow):
    """双面板数字孪生主窗口"""

    def __init__(self, port: int):
        super().__init__()
        self.setWindowTitle("Unit5 数字孪生 — 连杆机构实时显示")
        self.resize(900, 500)

        self._params = default_params()

        # 两个面板
        self._left_panel = LegPanel("左腿 (M1/M2 — CAN1)", self._params)
        self._right_panel = LegPanel("右腿 (M3/M4 — CAN2)", self._params)

        central = QWidget()
        layout = QHBoxLayout(central)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        layout.addWidget(self._left_panel)
        layout.addWidget(self._right_panel)
        self.setCentralWidget(central)

        # 状态栏
        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._status_label = QLabel("等待 UDP 数据...")
        self._status.addWidget(self._status_label)
        self._fps_label = QLabel("")
        self._status.addPermanentWidget(self._fps_label)

        # UDP socket
        self._sock = QUdpSocket(self)
        self._sock.bind(QHostAddress('127.0.0.1'), port)
        self._sock.readyRead.connect(self._on_datagram)

        self._frame_count = 0
        self._last_fps_update = 0

        # 定时刷新 FPS 显示
        self._fps_timer = QTimer(self)
        self._fps_timer.timeout.connect(self._update_fps)
        self._fps_timer.start(1000)

    def _on_datagram(self):
        while self._sock.hasPendingDatagrams():
            data, host, port = self._sock.readDatagram(
                self._sock.pendingDatagramSize()
            )
            try:
                text = data.data().decode('utf-8', errors='replace').strip()
            except UnicodeDecodeError:
                continue

            parts = text.split(',')
            if len(parts) != 5:
                continue

            try:
                t_ms = int(parts[0])
                left_a  = float(parts[1])
                left_b  = float(parts[2])
                right_a = float(parts[3])
                right_b = float(parts[4])
            except ValueError:
                continue

            self._left_panel.update_angles(left_a, left_b)
            self._right_panel.update_angles(right_a, right_b)

            self._frame_count += 1
            self._status_label.setText(
                f"t={t_ms} ms  |  "
                f"L: θa={np.rad2deg(left_a):+.1f}°  θb={np.rad2deg(left_b):+.1f}°  |  "
                f"R: θa={np.rad2deg(right_a):+.1f}°  θb={np.rad2deg(right_b):+.1f}°"
            )

    def _update_fps(self):
        self._fps_label.setText(f"{self._frame_count} FPS")
        self._frame_count = 0


def parse_args():
    p = argparse.ArgumentParser(description="Unit5 数字孪生实时显示")
    p.add_argument("--port", type=int, default=9876,
                   help="UDP 监听端口 (默认 9876)")
    return p.parse_args()


def main():
    args = parse_args()
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = TwinWindow(args.port)
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
