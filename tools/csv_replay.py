#!/usr/bin/env python3
"""
CSV 角度文件 → UDP 回放，供 twin_display.py / twin_calib.py 数字孪生消费。

用法:
  # 回放到 twin_display.py (PyQt6)
  .venv/bin/python tools/csv_replay.py mapping.csv &
  .venv/bin/python -m src.twin_display --port 9876

  # 回放到 twin_calib.py (matplotlib)
  .venv/bin/python tools/csv_replay.py mapping.csv &
  .venv/bin/python -m src.twin_calib --port 9876

CSV 格式 (Unit5 12 列):
  t_ms,m1,m2,m3,m4,t1,t2,t3,t4,pitch,pitch_rate,dt_us
  前 5 列 = t_ms, θaL, θbL, θaR, θbR (编码器值 rad)

回放: 按时序逐行发送前 5 列到 UDP 目标端口。
"""

import argparse
import socket
import sys
import time


def parse_args():
    p = argparse.ArgumentParser(description="CSV 角度回放到 UDP 数字孪生")
    p.add_argument("csv_file", help="Unit5 CSV 角度文件")
    p.add_argument("--target", default="127.0.0.1:9876",
                   help="UDP 目标地址:端口 (默认 127.0.0.1:9876)")
    p.add_argument("--speed", type=float, default=1.0,
                   help="回放倍速 (默认 1.0 实时)")
    p.add_argument("--loop", action="store_true",
                   help="循环回放")
    return p.parse_args()


def main():
    args = parse_args()

    host, port_str = args.target.rsplit(":", 1)
    port = int(port_str)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 读取 CSV, 提取前 5 列 (t_ms, m1, m2, m3, m4)
    frames = []
    with open(args.csv_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) < 5:
                continue
            try:
                t_ms = int(parts[0])
                floats = [float(p) for p in parts[1:5]]
                frames.append((t_ms, floats))
            except (ValueError, IndexError):
                continue

    if not frames:
        sys.exit(f"未在 {args.csv_file} 中找到有效帧")

    print(f"加载 {len(frames)} 帧, 时间范围 {frames[0][0]}..{frames[-1][0]} ms")
    print(f"回放目标: {host}:{port}, 倍速: {args.speed}x")
    print("Ctrl+C 退出")

    first_ts = frames[0][0]
    start_wall = time.time()

    try:
        while True:
            for t_ms, values in frames:
                rel_time = (t_ms - first_ts) / 1000.0 / args.speed
                elapsed = time.time() - start_wall
                if rel_time > elapsed:
                    time.sleep(rel_time - elapsed)

                payload = f"{t_ms},{values[0]:.6f},{values[1]:.6f},{values[2]:.6f},{values[3]:.6f}"
                sock.sendto(payload.encode('utf-8'), (host, port))

            if not args.loop:
                break
            start_wall = time.time()
            print(f"\n[replay] 循环回放 ({len(frames)} 帧)")

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print(f"\n回放完成: {len(frames)} 帧")


if __name__ == '__main__':
    main()
