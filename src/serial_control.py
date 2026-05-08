"""
串口命令接口：将 PC 侧安全目标发送到 Unit5 MCU。

用法:
    from src.serial_control import SerialController
    ctrl = SerialController("/dev/ttyACM0")
    ctrl.connect()
    ctrl.send_hp_target(100.0, -10.0)  # h=100mm, phi=-10deg
    ctrl.send_stop()
    ctrl.close()
"""

import time


class SerialController:
    """Unit5 串口命令发送器 (PC → MCU)"""

    def __init__(self, port: str, baud: int = 115200):
        self._port = port
        self._baud = baud
        self._ser = None

    # ---------- 连接管理 ----------

    def connect(self) -> bool:
        """打开串口，返回是否成功"""
        try:
            import serial
        except ImportError:
            raise RuntimeError("请安装 pyserial: pip install pyserial")

        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.1)
        except serial.SerialException as e:
            raise RuntimeError(f"无法打开串口 {self._port}: {e}")

        time.sleep(0.5)
        self._ser.reset_input_buffer()
        return True

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
            self._ser = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ---------- 命令发送 ----------

    def send_raw(self, cmd: str):
        """发送原始字符串 (自动追加 \\r\\n)"""
        if not self.is_connected:
            raise RuntimeError("串口未连接")
        self._ser.write((cmd + "\r\n").encode("utf-8"))
        self._ser.flush()

    # ---- 对齐 Unit5 Shell 接口 ----
    # 当前可用命令: robot move <h> <phi>, robot jog h <delta>, robot jog phi <delta>
    # h: 腿伸展量 [mm], phi: 腿摆角 [deg], 0=竖直向下

    def send_move(self, h: float, phi: float):
        """绝对位置指令: robot move <h> <phi>"""
        self.send_raw(f"robot move {h:.1f} {phi:.1f}")

    def send_jog_h(self, delta: float):
        """增量: robot jog h <delta> [mm]"""
        self.send_raw(f"robot jog h {delta:.1f}")

    def send_jog_phi(self, delta: float):
        """增量: robot jog phi <delta> [deg]"""
        self.send_raw(f"robot jog phi {delta:.1f}")

    # ---- 紧急停止 ----

    def send_stop(self):
        """紧急停止 (最高优先级)"""
        self.send_raw("robot stop")
