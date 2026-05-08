"""
PC 侧上位机拖拽控制器: 拖拽 P7 → IK → 安全检查 → 串口指令 → MCU 执行。

用法:
    # 仅可视化 (dry-run, 不连硬件)
    .venv/bin/python -m src.main live_control

    # 连接实机
    .venv/bin/python -m src.main live_control --port /dev/ttyACM0
"""

import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt

from .mechanism import MechanismParams, default_params
from .kinematics import solve_linkage, solve_inverse
from .safety import (
    full_check, compute_h_phi, H_SAFE_MIN, H_SAFE_MAX,
)
from .serial_control import SerialController

# 复用 visualization.py 的颜色和渲染
from .visualization import (
    COLOR_BAR_A, COLOR_BAR_B, COLOR_BAR_C, COLOR_BAR_D, COLOR_BAR_E, COLOR_BAR_F,
    COLOR_JOINT, COLOR_MOTOR, COLOR_END,
)

# ---- 左腿编码器限位 & offset (与 Unit5 对齐) ----
OFFSET_A = np.deg2rad(-162.4)
OFFSET_B = np.deg2rad(-10.0)
ENC_LIM_A = (-3.40, 1.67)   # M1 (θa) 编码器限位 [rad]
ENC_LIM_B = (-3.30, 3.14)   # M2 (θb) 编码器限位 [rad]
DIR_A = -1.0  # enc = -1*(θa - offset)
DIR_B = +1.0  # enc = +1*(θb - offset) (M2 dir=+1, Unit5 2026-05-09)

# 右腿 (M4=θa, M3=θb): 编码器限位
ENC_LIM_RA = (-1.66, 3.40)
ENC_LIM_RB = (-3.14, 3.20)


def rotate_view(v):
    """机构帧 → 显示帧 (-90°)"""
    return np.array([v[1], -v[0]])


def unrotate_view(v):
    """显示帧 → 机构帧"""
    return np.array([-v[1], v[0]])


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def angle_distance(sol, prev):
    da = wrap_angle(sol['theta_a'] - prev[0])
    db = wrap_angle(sol['theta_b'] - prev[1])
    return da * da + db * db


def build_mechanism_artists(ax):
    """构建机构渲染 artist (复用 visualization.py 的杆/关节定义)"""
    bar_specs = [
        ('O', 'P1', COLOR_BAR_A, 3.0, '-'),
        ('O', 'P2', COLOR_BAR_A, 3.0, '-'),
        ('P1', 'P2', COLOR_BAR_A, 1.5, '--'),
        ('O', 'P3', COLOR_BAR_B, 3.0, '-'),
        ('P3', 'P4', COLOR_BAR_C, 2.5, '-'),
        ('P1', 'P4', COLOR_BAR_D, 3.0, '-'),
        ('P1', 'P5', COLOR_BAR_D, 1.5, '--'),
        ('P4', 'P5', COLOR_BAR_D, 1.5, '--'),
        ('P5', 'P6', COLOR_BAR_E, 2.5, '-'),
        ('P2', 'P6', COLOR_BAR_F, 3.0, '-'),
        ('P2', 'P7', COLOR_BAR_F, 3.0, '-'),
        ('P6', 'P7', COLOR_BAR_F, 1.5, '--'),
    ]
    bar_artists = [
        (ka, kb, ax.plot([], [], color=color, lw=lw, ls=ls, zorder=2)[0])
        for ka, kb, color, lw, ls in bar_specs
    ]

    joint_specs = {
        'O': (COLOR_MOTOR, 80),
        'P1': (COLOR_JOINT, 40),
        'P2': (COLOR_JOINT, 40),
        'P3': (COLOR_JOINT, 40),
        'P4': (COLOR_JOINT, 40),
        'P5': (COLOR_JOINT, 40),
        'P6': (COLOR_JOINT, 40),
        'P7': (COLOR_END, 80),
    }
    joint_artists = {
        key: ax.scatter([], [], c=color, s=size, zorder=3, edgecolors='none')
        for key, (color, size) in joint_specs.items()
    }
    return bar_artists, joint_artists


def draw_mechanism_state(ax, bar_artists, joint_artists, res, theta_a, theta_b,
                         status_text, info_text, target_artist=None, target_view=None, safe=True):
    """更新机构渲染和状态文本"""
    for key in ['O', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7']:
        res[key + '_view'] = rotate_view(res[key])

    for ka, kb, artist in bar_artists:
        a, b = res[ka + '_view'], res[kb + '_view']
        artist.set_data([a[0], b[0]], [a[1], b[1]])
        artist.set_visible(True)

    for key, artist in joint_artists.items():
        artist.set_offsets([res[key + '_view']])
        artist.set_visible(True)

    if target_artist is not None and target_view is not None:
        target_artist.set_offsets([target_view])

    # 安全状态色
    status_color = '#2E7D32' if safe else '#C62828'
    status_text.set_text("SAFE" if safe else "BLOCKED")
    status_text.set_color(status_color)

    info_text.set_text(info_text.get_text())  # 外部更新


def run_live_control(args):
    """上位机拖拽控制器主循环"""
    params = default_params()
    ser_ctrl = None

    # 决定输出模式: stdout (转发给 Unit6) / serial (直连 Unit5) / dry-run (仅可视化)
    if args.stdout:
        print("[live] STDOUT 转发模式: 安全目标打印到终端, 供 Unit6 转发")
    elif args.port:
        try:
            ser_ctrl = SerialController(args.port, args.baud)
            ser_ctrl.connect()
            print(f"[live] 串口直连: {args.port}")
        except RuntimeError as e:
            print(f"[live] 串口连接失败: {e}, 回退到 dry-run")
            args.dry_run = True
    else:
        args.dry_run = True

    if args.dry_run and not args.stdout:
        print("[live] DRY-RUN 模式: 不输出指令, 仅显示安全检查结果")

    # ---- UI 初始化 ----
    view_xlim = (-280, 280)
    view_ylim = (-260, 260)
    target_radius = 9.0
    ta0 = np.deg2rad(-162.4)
    tb0 = np.deg2rad(-10.0)

    fig = plt.figure(figsize=(11, 9))
    fig.canvas.manager.set_window_title("Linkage Live Control — Drag P7 to Move Robot")

    ax = fig.add_axes([0.06, 0.22, 0.70, 0.74])
    ax.set_aspect('equal')
    ax.set_xlim(*view_xlim)
    ax.set_ylim(*view_ylim)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('display X [mm]')
    ax.set_ylabel('display Y [mm]')

    # 安全工作区叠加 (保守区域)
    from matplotlib.patches import Circle
    safe_inner = Circle((0, 0), H_SAFE_MIN, fill=True, alpha=0.08,
                        color='red', zorder=0, label=f'h<{H_SAFE_MIN} 禁止')
    safe_outer = Circle((0, 0), H_SAFE_MAX, fill=False, alpha=0.5,
                        color='orange', lw=1.5, ls='--', zorder=0,
                        label=f'h={H_SAFE_MAX} 边界')
    ax.add_patch(safe_inner)
    ax.add_patch(safe_outer)
    ax.legend(loc='upper right', fontsize=8)

    bar_artists, joint_artists = build_mechanism_artists(ax)

    target_artist = ax.scatter([], [], c='none', edgecolors='black',
                               s=180, linewidths=1.8, zorder=5)

    # 安全状态指示器
    status_text = ax.text(0.98, 0.98, '', transform=ax.transAxes,
                          ha='right', va='top', fontsize=22, fontweight='bold',
                          bbox=dict(boxstyle='round,pad=0.3',
                                    facecolor='white', alpha=0.9,
                                    edgecolor='#999999'))

    # 信息面板 (右侧)
    info_ax = fig.add_axes([0.78, 0.22, 0.20, 0.74])
    info_ax.axis('off')
    info_text = info_ax.text(0.02, 0.98, '', transform=info_ax.transAxes,
                             ha='left', va='top', fontsize=9,
                             fontfamily='monospace',
                             bbox=dict(boxstyle='round,pad=0.5',
                                       facecolor='#FAFAFA', alpha=0.95,
                                       edgecolor='#CCCCCC'))

    # 发送日志区域
    log_ax = fig.add_axes([0.06, 0.04, 0.92, 0.15])
    log_ax.axis('off')
    log_text = log_ax.text(0.01, 0.98, '', transform=log_ax.transAxes,
                           ha='left', va='top', fontsize=8,
                           fontfamily='monospace',
                           bbox=dict(boxstyle='round,pad=0.4',
                                     facecolor='#FFFDE7', alpha=0.9,
                                     edgecolor='#CCCCCC'))
    log_lines = []

    def add_log(msg):
        nonlocal log_lines
        log_lines.append(msg)
        if len(log_lines) > 12:
            log_lines = log_lines[-12:]
        log_text.set_text('\n'.join(log_lines))

    # 状态
    state = {
        'dragging': False,
        'theta': np.array([ta0, tb0]),
        'target_base': None,
        'safe': True,
        'errors': [],
        'send_count': 0,
        'block_count': 0,
    }

    def update_target_from_view(x_view, y_view):
        """核心: 拖拽 → IK → 安全检查 → 串口输出"""
        target_base = unrotate_view(np.array([x_view, y_view]))
        sols = solve_inverse(target_base, params, elbow=0)
        if not sols:
            info_text.set_text("UNREACHABLE")
            status_text.set_text("UNREACHABLE")
            status_text.set_color('#E65100')
            draw_mechanism_state(ax, bar_artists, joint_artists,
                                 solve_linkage(state['theta'][0], state['theta'][1], params),
                                 state['theta'][0], state['theta'][1],
                                 status_text, info_text,
                                 target_artist=target_artist,
                                 target_view=rotate_view(target_base),
                                 safe=False)
            fig.canvas.draw_idle()
            return

        # 选距上一帧最近的 IK 解
        sol = min(sols, key=lambda s: angle_distance(s, state['theta']))
        ta, tb = sol['theta_a'], sol['theta_b']

        # 运行全部安全检查
        errors = full_check(
            ta, tb,
            state['theta'][0], state['theta'][1],
            OFFSET_A, OFFSET_B,
            ENC_LIM_A, ENC_LIM_B,
            DIR_A, DIR_B,
        )

        safe = len(errors) == 0

        # 安全 → 发送 (同时更新跟踪位置)
        if safe:
            # 更新跟踪位置 (仅为安全目标更新, 确保从危险区拖回时比较的是上一个安全位置)
            state['theta'][:] = [ta, tb]
            h, phi = compute_h_phi(ta, tb)
            phi_deg = np.rad2deg(phi)

            if args.stdout:
                # 转发模式: 打印 TARGET 行到 stdout 供 Unit6 解析
                print(f"TARGET {h:.0f} {phi_deg:.1f}", flush=True)
                state['send_count'] += 1
                add_log(f"→ TARGET {h:.0f} {phi_deg:.1f}")
            elif ser_ctrl and ser_ctrl.is_connected:
                try:
                    ser_ctrl.send_move(h, phi_deg)
                    state['send_count'] += 1
                    add_log(f"→ robot move {h:.0f} {phi_deg:.1f}")
                except RuntimeError as e:
                    add_log(f"SEND ERR: {e}")
            else:
                state['send_count'] += 1
                add_log(f"[DRY] robot move h={h:.0f} φ={phi_deg:.1f}")
        else:
            state['block_count'] += 1
            for err in errors:
                add_log(f"BLOCK: {err}")

        # 渲染: 机构用最后安全位置, 信息面板显示拖拽目标参数
        res = solve_linkage(state['theta'][0], state['theta'][1], params)
        if res is None:
            return

        h_cur, phi_cur = compute_h_phi(ta, tb)  # 拖拽目标的 h/φ (无论是否安全)
        ea = wrap_angle(DIR_A * (ta - OFFSET_A))
        eb = wrap_angle(DIR_B * (tb - OFFSET_B))

        safe_flag = "SAFE" if safe else "BLOCKED"
        info_text.set_text(
            f"机构帧:\n"
            f"  θa = {np.rad2deg(state['theta'][0]):7.2f}°\n"
            f"  θb = {np.rad2deg(state['theta'][1]):7.2f}°\n"
            f"\n编码器 (左):\n"
            f"  enc_a = {np.rad2deg(ea):+7.1f}°\n"
            f"  enc_b = {np.rad2deg(eb):+7.1f}°\n"
            f"\n目标 (拖拽):\n"
            f"  h = {h_cur:6.1f} mm\n"
            f"  φ = {np.rad2deg(phi_cur):+6.1f}°\n"
            f"\n目标 (机构):\n"
            f"  x = {target_base[0]:6.1f} mm\n"
            f"  y = {target_base[1]:6.1f} mm\n"
            f"\n限位 (enc):\n"
            f"  a: [{np.rad2deg(ENC_LIM_A[0]):+.0f}..{np.rad2deg(ENC_LIM_A[1]):+.0f}]°\n"
            f"  b: [{np.rad2deg(ENC_LIM_B[0]):+.0f}..{np.rad2deg(ENC_LIM_B[1]):+.0f}]°\n"
            f"\n状态: {safe_flag}\n"
            f"发送: {state['send_count']}  拦截: {state['block_count']}"
        )

        draw_mechanism_state(ax, bar_artists, joint_artists, res,
                             state['theta'][0], state['theta'][1],
                             status_text, info_text,
                             target_artist=target_artist,
                             target_view=rotate_view(target_base),
                             safe=safe)
        fig.canvas.draw_idle()

    # ---- 鼠标事件 ----
    def on_press(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        cur_target = rotate_view(state['target_base']) if state['target_base'] is not None else np.array([0, 0])
        d = np.hypot(event.xdata - cur_target[0], event.ydata - cur_target[1])
        if d <= target_radius * 2.5:
            state['dragging'] = True
            update_target_from_view(event.xdata, event.ydata)

    def on_motion(event):
        if not state['dragging']:
            return
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        update_target_from_view(event.xdata, event.ydata)

    def on_release(event):
        state['dragging'] = False

    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('button_release_event', on_release)

    # 初始画面
    res0 = solve_linkage(ta0, tb0, params)
    if res0 is not None:
        state['target_base'] = res0['P7'].copy()
        h0, phi0 = compute_h_phi(ta0, tb0)
        info_text.set_text(f"初始化: θa=-162.4° θb=-10.0°\n"
                           f"cali: h={h0:.0f}mm φ={np.rad2deg(phi0):+.1f}°\n\n"
                           f"拖拽黑色圆环移动末端 P7\n"
                           f"安全工作区: h ∈ [{H_SAFE_MIN:.0f}, {H_SAFE_MAX:.0f}] mm\n"
                           f"(对齐 Unit5 Shell 入口限位)\n"
                           f"红色圆 = h<{H_SAFE_MIN:.0f} 禁区\n"
                           f"橙色虚线 = h={H_SAFE_MAX:.0f} 边界")
        draw_mechanism_state(ax, bar_artists, joint_artists, res0,
                             ta0, tb0, status_text, info_text,
                             target_artist=target_artist,
                             target_view=rotate_view(state['target_base']),
                             safe=True)
        add_log("就绪 — 拖拽 P7 开始控制")
    fig.canvas.draw_idle()

    plt.show()

    # 清理
    if ser_ctrl:
        ser_ctrl.send_stop()
        ser_ctrl.close()
        print("[live] 串口已关闭, 已发送 stop")


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Linkage 上位机拖拽控制器 — 拖拽 P7 实时控制轮腿机器人")
    p.add_argument("--port", default=None,
                   help="串口设备 (如 /dev/ttyACM0), 不指定则自动进入 dry-run")
    p.add_argument("--baud", type=int, default=115200,
                   help="串口波特率 (默认 115200)")
    p.add_argument("--dry-run", action="store_true",
                   help="仅可视化, 不输出任何指令")
    p.add_argument("--stdout", action="store_true",
                   help="转发模式: 安全目标以 'TARGET <h> <phi>' 格式打印到 stdout (供 Unit6 解析)")
    return p.parse_args(argv)


def main():
    args = parse_args()
    run_live_control(args)


if __name__ == '__main__':
    main()
