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


# mount_offset: encoder → mechanism-frame angle
OFFSET_A = np.deg2rad(-162.4)   # M1/M4 (θa)
OFFSET_B = np.deg2rad(-10.0)    # M2/M3 (θb)

# 实车实测编码器限位 (rad), 2026-05-08 bring-up
#   Row1 (后限位): 1.67, 1.61, -1.62, -1.54
#   Row2 (前限位): -3.40, -3.30, 3.40, 3.20
#   Row3 (b杆后限): 1.65, 3.14, -1.66, -3.14
ENC_LIMIT_LEFT  = {'a': (-3.40, 1.67), 'b': (-3.30, 3.14)}
ENC_LIMIT_RIGHT = {'a': (-1.66, 3.40), 'b': (-3.14, 3.20)}
LIMIT_MARGIN = np.deg2rad(5.0)  # 5° 预警余量


class LegPanel(QWidget):
    """单腿连杆机构绘制面板"""

    def __init__(self, title: str, params: MechanismParams,
                 offset_a: float = OFFSET_A, offset_b: float = OFFSET_B,
                 dir_a: float = 1.0, dir_b: float = 1.0,
                 limit_a: tuple = None, limit_b: tuple = None):
        super().__init__()
        self._title = title
        self._params = params
        self._offset_a = offset_a
        self._offset_b = offset_b
        self._dir_a = dir_a
        self._dir_b = dir_b
        self._limit_a = limit_a or (-99, 99)
        self._limit_b = limit_b or (-99, 99)
        self._result = None
        self._theta_m = 0.0   # mechanism-frame angle
        self._theta_b = 0.0
        self._enc_a = 0.0     # raw encoder
        self._enc_b = 0.0
        self._frame_count = 0
        self.setMinimumSize(350, 420)
        self.setMouseTracking(True)

    def update_encoders(self, enc_a: float, enc_b: float):
        """输入原始编码器值(rad), 内部加方向+offset 转机构帧"""
        self._enc_a = enc_a
        self._enc_b = enc_b
        self._theta_m = self._dir_a * enc_a + self._offset_a
        self._theta_b = self._dir_b * enc_b + self._offset_b
        self._result = solve_linkage(self._theta_m, self._theta_b, self._params)
        self._frame_count += 1
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2

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
            painter.drawText(QRectF(0, cy - 30, w, 30),
                             Qt.AlignmentFlag.AlignCenter, "等待数据...\n(若姿态全伸展,检查OFFSET)")
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

        # 信息区域: 编码器 → 机构帧 → h/φ
        font_info = QFont('Monospace', 8)
        painter.setFont(font_info)
        painter.setPen(QColor('#555555'))

        y_base = h - 82
        deg_ea = np.rad2deg(self._enc_a)
        deg_eb = np.rad2deg(self._enc_b)
        deg_ma = np.rad2deg(self._theta_m)
        deg_mb = np.rad2deg(self._theta_b)

        p7 = res['P7']
        hp = np.hypot(p7[0], p7[1])
        phi = -np.arctan2(p7[1], p7[0]) - np.pi/2

        lines = [
            f"enc:  M1={deg_ea:+7.1f}  M2={deg_eb:+7.1f} deg",
            f"mech: θa={deg_ma:+7.1f}  θb={deg_mb:+7.1f} deg",
            f"h={hp:.0f} mm  φ={np.rad2deg(phi):+.1f} deg",
        ]
        for i, line in enumerate(lines):
            painter.drawText(8, y_base + i * 14, line)

        # 编码器限位条 (a/b 各一条)
        bar_w, bar_h = 120, 5
        bar_x, bar_y_a, bar_y_b = w - bar_w - 10, y_base - 4, y_base + 10
        for enc_val, (lo, hi), bar_y, label in [
            (self._enc_a, self._limit_a, bar_y_a, 'enc_a'),
            (self._enc_b, self._limit_b, bar_y_b, 'enc_b'),
        ]:
            rng = hi - lo
            if rng <= 0:
                continue
            frac = np.clip((enc_val - lo) / rng, 0, 1)
            # 底色 (灰)
            painter.fillRect(bar_x, bar_y, bar_w, bar_h, QColor('#E0E0E0'))
            # 限位范围 (浅蓝)
            painter.fillRect(bar_x, bar_y, int(bar_w * frac), bar_h,
                             BAR_COLORS['bar_a'])
            # 边界标记
            painter.setPen(QPen(QColor('#888888'), 1))
            painter.drawRect(bar_x, bar_y, bar_w, bar_h)
            # 标签
            painter.setPen(QColor('#555555'))
            font_tiny = QFont('Monospace', 6)
            painter.setFont(font_tiny)
            painter.drawText(bar_x + bar_w + 4, bar_y + 5,
                             f"{np.rad2deg(lo):+.0f}…{np.rad2deg(hi):+.0f}°")
            # 越界告警
            near = LIMIT_MARGIN
            if enc_val < lo + near or enc_val > hi - near:
                painter.setPen(QColor('#E91E63'))
                font_warn = QFont('Sans', 9, QFont.Weight.Bold)
                painter.setFont(font_warn)
                painter.drawText(bar_x, bar_y - 10, "LIMIT!")

        # h 限位指示
        h_limits = [
            (hp < 45, "h<45!"),
            (hp > 235, "h>235!"),
        ]
        for i, (cond, txt) in enumerate(h_limits):
            if cond:
                painter.setPen(QColor('#E91E63'))
                font_warn = QFont('Sans', 10, QFont.Weight.Bold)
                painter.setFont(font_warn)
                painter.drawText(8, y_base + 52 + i * 14, txt)

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

        # 两个面板: 左 M1→θa(-1),M2→θb(-1) | 右 M3→θa(+1),M4→θb(+1)
        self._left_panel = LegPanel("左腿 (M1=θa, M2=θb)", self._params,
                                    dir_a=-1.0, dir_b=+1.0,
                                    limit_a=ENC_LIMIT_LEFT['a'],
                                    limit_b=ENC_LIMIT_LEFT['b'])
        self._right_panel = LegPanel("右腿 (M4=θa, M3=θb)", self._params,
                                     dir_a=-1.0, dir_b=+1.0,
                                     limit_a=ENC_LIMIT_RIGHT['a'],
                                     limit_b=ENC_LIMIT_RIGHT['b'])

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
                text = data.decode('utf-8', errors='replace').strip()
            except UnicodeDecodeError:
                continue

            parts = text.split(',')
            if len(parts) != 5:
                print(f"[twin] 字段数异常: {len(parts)} -> {text!r}", flush=True)
                continue

            try:
                t_ms = int(parts[0])
                left_a  = float(parts[1])
                left_b  = float(parts[2])
                right_a = float(parts[3])
                right_b = float(parts[4])
            except ValueError:
                continue

            self._left_panel.update_encoders(left_a, left_b)
            self._right_panel.update_encoders(right_b, right_a)  # CSV: right_b=M4→θa, right_a=M3→θb

            if self._frame_count == 0:
                print(f"[twin] 首帧: t={t_ms} L=({left_a:.4f},{left_b:.4f}) R=({right_a:.4f},{right_b:.4f})", flush=True)

            self._frame_count += 1
            self._status_label.setText(
                f"t={t_ms} ms  |  "
                f"L: M1/2 enc({np.rad2deg(left_a):+.1f},{np.rad2deg(left_b):+.1f}) → "
                f"mech({np.rad2deg(-left_a + OFFSET_A):+.1f},{np.rad2deg(left_b + OFFSET_B):+.1f})  |  "
                f"R: M4/3 enc({np.rad2deg(right_a):+.1f},{np.rad2deg(right_b):+.1f}) → "
                f"mech({np.rad2deg(-right_a + OFFSET_A):+.1f},{np.rad2deg(right_b + OFFSET_B):+.1f})"
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
