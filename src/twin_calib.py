#!/usr/bin/env python3
"""
数字孪生实时显示 — zero_calib 风格双面板 + 角度数值。

UDP 接收 Unit5 的 4 路电机角度，左右腿并排显示机构帧。

用法:
  .venv/bin/python -m src.twin_calib --port 9876
"""

import argparse
import socket
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('QtAgg')  # PyQt6 已安装
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mechanism import default_params
from src.kinematics import solve_linkage

BAR_COLORS = ['#e74c3c', '#e74c3c', '#3498db', '#2ecc71', '#f39c12',
              '#f39c12', '#9b59b6', '#1abc9c', '#1abc9c']
BAR_PAIRS = [
    ('O', 'P1'), ('P1', 'P2'), ('O', 'P3'), ('P3', 'P4'),
    ('P4', 'P1'), ('P1', 'P5'), ('P5', 'P6'), ('P6', 'P2'), ('P2', 'P7'),
]
VIEW_MARGIN = 50


def draw_mechanism(ax, res, theta_a, theta_b, motor_names, title):
    ax.clear()

    for (ka, kb), clr in zip(BAR_PAIRS, BAR_COLORS):
        a, b = res[ka], res[kb]
        ax.plot([a[0], b[0]], [a[1], b[1]], '-', color=clr, lw=2.5)

    ax.scatter(0, 0, c='red', s=100, zorder=5)
    ax.scatter(*res['P7'], c='green', s=120, zorder=5)

    p3 = res['P3']
    d3 = p3 / np.linalg.norm(p3) * 55
    ax.arrow(0, 0, d3[0], d3[1], head_width=2.5, head_length=4,
             fc='#3498db', ec='#3498db', alpha=0.4, lw=2)

    deg_a = np.rad2deg(theta_a)
    deg_b = np.rad2deg(theta_b)
    ax.set_title(f'{title}\n'
                 f'$\\theta_a={deg_a:+.1f}\\degree$  '
                 f'$\\theta_b={deg_b:+.1f}\\degree$', fontsize=11)

    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    r = max(np.linalg.norm(res['P7']) + VIEW_MARGIN, 200)
    ax.set_xlim(-r, r)
    ax.set_ylim(-r, r)


def parse_udp_data(data: bytes):
    try:
        text = data.decode('utf-8', errors='replace').strip()
    except UnicodeDecodeError:
        return None
    parts = text.split(',')
    if len(parts) != 5:
        return None
    try:
        return (
            int(parts[0]),
            float(parts[1]), float(parts[2]),
            float(parts[3]), float(parts[4]),
        )
    except ValueError:
        return None


def main():
    parser = argparse.ArgumentParser(description="Unit5 Digital Twin")
    parser.add_argument("--port", type=int, default=9876)
    args = parser.parse_args()

    params = default_params()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', args.port))
    sock.setblocking(False)

    left_a = left_b = right_a = right_b = 0.0
    t_ms = 0
    frame_count = 0

    plt.ion()
    fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(14, 7))
    fig.canvas.manager.set_window_title('Unit5 Digital Twin')

    print(f"UDP listening on 127.0.0.1:{args.port} ...", flush=True)
    print(f"  Terminal: angle readout every 1s", flush=True)
    print(f"  Window:   mechanism visualization (don't close it!)", flush=True)

    last_print = 0
    try:
        while plt.fignum_exists(fig.number):
            # 非阻塞收包
            try:
                data, _ = sock.recvfrom(4096)
                result = parse_udp_data(data)
                if result is not None:
                    t_ms, left_a, left_b, right_a, right_b = result
                    frame_count += 1
            except BlockingIOError:
                pass

            # 每秒打印角度到终端
            now = t_ms
            if now - last_print >= 1000:
                last_print = now
                print(f"\r  L: {np.rad2deg(left_a):+7.2f}  {np.rad2deg(left_b):+7.2f} deg  |  "
                      f"R: {np.rad2deg(right_a):+7.2f}  {np.rad2deg(right_b):+7.2f} deg  |  "
                      f"{frame_count} fr",
                      end="", flush=True)

            res_l = solve_linkage(left_a, left_b, params)
            res_r = solve_linkage(right_a, right_b, params)

            if res_l is not None:
                draw_mechanism(ax_l, res_l, left_a, left_b,
                               'M1/M2', 'L Leg (CAN1)')
            else:
                ax_l.set_title('L Leg: no solution')

            if res_r is not None:
                draw_mechanism(ax_r, res_r, right_a, right_b,
                               'M3/M4', 'R Leg (CAN2)')
            else:
                ax_r.set_title('R Leg: no solution')

            fig.suptitle(
                f'Unit5 Digital Twin  |  t={t_ms}ms  |  '
                f'L {np.rad2deg(left_a):+.1f}/{np.rad2deg(left_b):+.1f}  '
                f'R {np.rad2deg(right_a):+.1f}/{np.rad2deg(right_b):+.1f} deg',
                fontsize=10, fontweight='bold')

            plt.pause(0.03)

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        plt.ioff()
        plt.close('all')
        print(f"\nDone. {frame_count} frames received.")


if __name__ == '__main__':
    main()
