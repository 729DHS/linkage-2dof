"""
PC 侧安全预检查：工作空间、奇异点、编码器限位、分支连续性。

所有检查在发送目标到 MCU 之前执行 (第一层安全)。
MCU 固件负责第二层硬保护。
"""

import numpy as np
from typing import Tuple, List

# 保守安全工作区 (对齐 Unit5 Shell 限位, bringup workspace)
# 运动学理论范围 20.6~235.4, Shell 当前收窄到 45~100
# 后续可随 bringup 进展放宽至全范围
H_SAFE_MIN = 45.0   # mm, 避开折叠奇异 (理论 20.6)
H_SAFE_MAX = 100.0  # mm, bringup 保守上限

# 奇异点: det(J) / (L1*L2) = |sin(θb-θa)| 下限
SING_SIN_MIN = np.sin(np.deg2rad(10))  # ~0.174, 10° 余量

# 分支连续性: 单帧最大角度跳变
MAX_BRANCH_JUMP_RAD = np.deg2rad(45)  # 拖拽场景允许较大单帧跳变, 以捕获真正的分支翻转 (>60°)

# 编码器限位余量
ENC_MARGIN_DEG = 2.0


def check_h(h: float) -> Tuple[bool, str]:
    """h 范围检查 (腿伸展量)"""
    if h < H_SAFE_MIN:
        return False, f"h={h:.1f} < {H_SAFE_MIN} mm (过回缩, 近折叠奇异)"
    if h > H_SAFE_MAX:
        return False, f"h={h:.1f} > {H_SAFE_MAX} mm (过伸展, 近完全伸展奇异)"
    return True, "OK"


def check_singularity(theta_a: float, theta_b: float) -> Tuple[bool, str]:
    """奇异点检查: det(J) = L1*L2*sin(θb-θa)"""
    abs_sin = abs(np.sin(theta_b - theta_a))
    if abs_sin < SING_SIN_MIN:
        d_norm = abs((theta_b - theta_a + np.pi) % (2 * np.pi) - np.pi)
        return False, f"近奇异: |θb-θa|={np.rad2deg(d_norm):.1f}° (min {np.rad2deg(np.arcsin(SING_SIN_MIN)):.0f}°)"
    return True, "OK"


def check_branch_continuity(
    theta_a: float, theta_b: float,
    prev_theta_a: float, prev_theta_b: float,
    max_jump: float = MAX_BRANCH_JUMP_RAD,
) -> Tuple[bool, str]:
    """拒绝与上一帧角度差距过大的解 (防止分支跳变)"""
    if prev_theta_a is None or prev_theta_b is None:
        return True, "OK (first frame)"

    def angle_dist(a, b):
        return abs(((a - b + np.pi) % (2 * np.pi)) - np.pi)

    da = angle_dist(theta_a, prev_theta_a)
    db = angle_dist(theta_b, prev_theta_b)
    if da > max_jump or db > max_jump:
        return False, (
            f"分支跳变: Δθa={np.rad2deg(da):.1f}° "
            f"Δθb={np.rad2deg(db):.1f}° (max {np.rad2deg(max_jump):.0f}°)"
        )
    return True, "OK"


def check_encoder_limits(
    theta_a: float, theta_b: float,
    offset_a: float, offset_b: float,
    enc_lim_a: Tuple[float, float],
    enc_lim_b: Tuple[float, float],
    dir_a: float = -1.0,
    dir_b: float = -1.0,
    margin_deg: float = ENC_MARGIN_DEG,
) -> Tuple[bool, str]:
    """编码器限位检查: 机构帧角度 → 编码器值 → 与限位比较"""
    margin = np.deg2rad(margin_deg)
    ea = dir_a * (theta_a - offset_a)
    eb = dir_b * (theta_b - offset_b)

    lo_a, hi_a = enc_lim_a
    lo_b, hi_b = enc_lim_b

    in_a = (lo_a + margin) <= ea <= (hi_a - margin)
    in_b = (lo_b + margin) <= eb <= (hi_b - margin)

    msgs = []
    if not in_a:
        msgs.append(f"enc_a={np.rad2deg(ea):+.1f}° 超限 [{np.rad2deg(lo_a):.0f}..{np.rad2deg(hi_a):.0f}]")
    if not in_b:
        msgs.append(f"enc_b={np.rad2deg(eb):+.1f}° 超限 [{np.rad2deg(lo_b):.0f}..{np.rad2deg(hi_b):.0f}]")
    if msgs:
        return False, "; ".join(msgs)
    return True, "OK"


def compute_h_phi(theta_a: float, theta_b: float,
                  L1: float = 107.4, L2: float = 128.0) -> Tuple[float, float]:
    """等效 2R 臂: θa/θb → h (腿伸展量) / φ (腿摆角, 0=竖直向下)"""
    x = L1 * np.cos(theta_a) + L2 * np.cos(theta_b)
    y = L1 * np.sin(theta_a) + L2 * np.sin(theta_b)
    h = np.hypot(x, y)
    phi = -np.arctan2(y, x) - np.pi / 2
    return h, phi


FULL_CHECK_NAMES = ["h", "singularity", "branch", "encoder"]


def full_check(
    theta_a: float, theta_b: float,
    prev_theta_a: float, prev_theta_b: float,
    offset_a: float, offset_b: float,
    enc_lim_a: Tuple[float, float], enc_lim_b: Tuple[float, float],
    dir_a: float = -1.0, dir_b: float = -1.0,
    L1: float = 107.4, L2: float = 128.0,
) -> List[str]:
    """全部安全预检查，返回错误信息列表 (空=全部通过)"""
    h, phi = compute_h_phi(theta_a, theta_b, L1, L2)
    errors = []

    ok, msg = check_h(h)
    if not ok:
        errors.append(f"[h] {msg}")

    ok, msg = check_singularity(theta_a, theta_b)
    if not ok:
        errors.append(f"[SING] {msg}")

    ok, msg = check_branch_continuity(theta_a, theta_b, prev_theta_a, prev_theta_b)
    if not ok:
        errors.append(f"[BRANCH] {msg}")

    ok, msg = check_encoder_limits(theta_a, theta_b, offset_a, offset_b,
                                    enc_lim_a, enc_lim_b, dir_a, dir_b)
    if not ok:
        errors.append(f"[ENC] {msg}")

    return errors
