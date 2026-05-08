#!/usr/bin/env python3
"""
FK 验证工具: 从 Unit5 CSV 角度数据批量计算 FK, 输出 h/φ/P7 轨迹。

用于验证: 真实 P7 轨迹是否与 FK 预测一致、左右腿是否镜像正确、
          φ 方向是否一致、是否存在 branch 异常。

用法:
  .venv/bin/python tools/fk_validate.py mapping.csv              # 终端摘要
  .venv/bin/python tools/fk_validate.py mapping.csv --full       # 逐帧输出
  .venv/bin/python tools/fk_validate.py mapping.csv --out traj.csv  # 导出轨迹 CSV

输出 CSV 列:
  t_ms, hL, phiL, P7xL, P7yL, hR, phiR, P7xR, P7yR,
  θaL, θbL, θaR, θbR, enc_aL, enc_bL, enc_aR, enc_bR, validL, validR
"""

import argparse
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mechanism import default_params
from src.kinematics import solve_linkage

# 编码器→机构帧 (对齐 Unit5 2026-05-09)
OFFSET_A = np.deg2rad(-162.4)
OFFSET_B = np.deg2rad(-10.0)

# 左腿: M1 dir=-1, M2 dir=+1
# 右腿: M4 dir=-1 (θa), M3 dir=+1 (θb)
def enc_to_mech_left(enc_a, enc_b):
    return -enc_a + OFFSET_A, enc_b + OFFSET_B

def enc_to_mech_right(m3, m4):
    # CSV: m3=M3 enc (→θbR, dir=+1), m4=M4 enc (→θaR, dir=-1)
    return -m4 + OFFSET_A, m3 + OFFSET_B


def compute_h_phi(P7):
    """机构帧 P7 → h (伸展量) / φ (摆角, 0=竖直向下)"""
    h = np.hypot(P7[0], P7[1])
    phi = -np.arctan2(P7[1], P7[0]) - np.pi / 2
    return h, phi


def load_csv(path):
    """加载 Unit5 12 列 CSV → [(t_ms, m1,m2,m3,m4), ...]"""
    frames = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) < 5:
                continue
            try:
                t_ms = int(parts[0])
                vals = [float(p) for p in parts[1:5]]
                frames.append((t_ms, vals[0], vals[1], vals[2], vals[3]))
            except (ValueError, IndexError):
                continue
    return frames


def validate_csv(path, full=False, out_path=None):
    params = default_params()
    frames = load_csv(path)
    if not frames:
        sys.exit(f"未在 {path} 中找到有效帧")

    print(f"加载 {len(frames)} 帧, 范围 {frames[0][0]}..{frames[-1][0]} ms")

    results = []
    n_invalid_l = 0
    n_invalid_r = 0
    h_min_l, h_max_l = float('inf'), float('-inf')
    h_min_r, h_max_r = float('inf'), float('-inf')
    branch_flips_l = 0
    branch_flips_r = 0
    prev_bd_l, prev_bf_l = None, None
    prev_bd_r, prev_bf_r = None, None

    out_f = open(out_path, 'w') if out_path else None
    if out_f:
        out_f.write("t_ms,hL,phiL,P7xL,P7yL,hR,phiR,P7xR,P7yR,"
                    "θaL,θbL,θaR,θbR,enc_aL,enc_bL,enc_aR,enc_bR,validL,validR\n")

    # 首帧追踪
    prev_d_l = prev_f_l = prev_d_r = prev_f_r = None

    for i, (t_ms, m1, m2, m3, m4) in enumerate(frames):
        # 编码器 → 机构帧
        ta_l, tb_l = enc_to_mech_left(m1, m2)
        ta_r, tb_r = enc_to_mech_right(m3, m4)

        # FK 求解 (带连续性追踪)
        res_l = solve_linkage(ta_l, tb_l, params, prev_d_l, prev_f_l)
        res_r = solve_linkage(ta_r, tb_r, params, prev_d_r, prev_f_r)

        row = {
            't_ms': t_ms,
            'ta_l': ta_l, 'tb_l': tb_l,
            'ta_r': ta_r, 'tb_r': tb_r,
            'enc_a_l': m1, 'enc_b_l': m2,
            'enc_a_r': m3, 'enc_b_r': m4,
        }

        # 左腿
        if res_l is not None:
            h_l, phi_l = compute_h_phi(res_l['P7'])
            row['h_l'] = h_l
            row['phi_l'] = phi_l
            row['P7x_l'] = res_l['P7'][0]
            row['P7y_l'] = res_l['P7'][1]
            row['valid_l'] = 1
            h_min_l = min(h_min_l, h_l)
            h_max_l = max(h_max_l, h_l)
            # 分支翻转检测
            bd, bf = res_l.get('branch_d'), res_l.get('branch_f')
            if prev_bd_l is not None and (bd != prev_bd_l or bf != prev_bf_l):
                branch_flips_l += 1
            prev_bd_l, prev_bf_l = bd, bf
            prev_d_l, prev_f_l = res_l['theta_d'], res_l['theta_f']
        else:
            row.update({'h_l': np.nan, 'phi_l': np.nan, 'P7x_l': np.nan, 'P7y_l': np.nan, 'valid_l': 0})
            n_invalid_l += 1

        # 右腿
        if res_r is not None:
            h_r, phi_r = compute_h_phi(res_r['P7'])
            row['h_r'] = h_r
            row['phi_r'] = phi_r
            row['P7x_r'] = res_r['P7'][0]
            row['P7y_r'] = res_r['P7'][1]
            row['valid_r'] = 1
            h_min_r = min(h_min_r, h_r)
            h_max_r = max(h_max_r, h_r)
            bd, bf = res_r.get('branch_d'), res_r.get('branch_f')
            if prev_bd_r is not None and (bd != prev_bd_r or bf != prev_bf_r):
                branch_flips_r += 1
            prev_bd_r, prev_bf_r = bd, bf
            prev_d_r, prev_f_r = res_r['theta_d'], res_r['theta_f']
        else:
            row.update({'h_r': np.nan, 'phi_r': np.nan, 'P7x_r': np.nan, 'P7y_r': np.nan, 'valid_r': 0})
            n_invalid_r += 1

        results.append(row)

        if out_f:
            out_f.write(f"{t_ms},"
                        f"{row.get('h_l', '')},{np.rad2deg(row.get('phi_l', 0)) if row.get('h_l') is not None and not np.isnan(row.get('h_l', 0)) else ''},"
                        f"{row.get('P7x_l', '')},{row.get('P7y_l', '')},"
                        f"{row.get('h_r', '')},{np.rad2deg(row.get('phi_r', 0)) if row.get('h_r') is not None and not np.isnan(row.get('h_r', 0)) else ''},"
                        f"{row.get('P7x_r', '')},{row.get('P7y_r', '')},"
                        f"{np.rad2deg(ta_l)},{np.rad2deg(tb_l)},{np.rad2deg(ta_r)},{np.rad2deg(tb_r)},"
                        f"{np.rad2deg(m1)},{np.rad2deg(m2)},{np.rad2deg(m3)},{np.rad2deg(m4)},"
                        f"{row.get('valid_l', 0)},{row.get('valid_r', 0)}\n")

    if out_f:
        out_f.close()
        print(f"轨迹已导出: {out_path}")

    # ---- 摘要 ----
    print(f"\n{'='*60}")
    print(f"FK 验证摘要: {Path(path).name}")
    print(f"{'='*60}")
    print(f"总帧数: {len(frames)}")
    print(f"左腿无解: {n_invalid_l}/{len(frames)} ({100*n_invalid_l/len(frames):.1f}%)")
    print(f"右腿无解: {n_invalid_r}/{len(frames)} ({100*n_invalid_r/len(frames):.1f}%)")
    print(f"左腿分支翻转: {branch_flips_l}")
    print(f"右腿分支翻转: {branch_flips_r}")
    print(f"左腿 h 范围: [{h_min_l:.0f}, {h_max_l:.0f}] mm")
    print(f"右腿 h 范围: [{h_min_r:.0f}, {h_max_r:.0f}] mm")

    # 左右对称性检查
    if n_invalid_l == 0 and n_invalid_r == 0:
        h_diffs = []
        for r in results:
            h_diffs.append(abs(r['h_l'] - r['h_r']))
        h_diffs = np.array(h_diffs)
        print(f"左右 h 差异: max={h_diffs.max():.1f} mean={h_diffs.mean():.1f} mm")

        phi_diffs = []
        for r in results:
            d = abs(np.rad2deg(r['phi_l'] - r['phi_r']))
            phi_diffs.append(d)
        phi_diffs = np.array(phi_diffs)
        print(f"左右 φ 差异: max={phi_diffs.max():.1f} mean={phi_diffs.mean():.1f}°")

    # 完整输出
    if full:
        print(f"\n{'='*60}")
        print("逐帧 FK 输出 (前 30 帧):")
        print(f"{'t_ms':>8} {'hL':>6} {'φL':>7} {'hR':>6} {'φR':>7} {'vL vR'}")
        print("-" * 50)
        for r in results[:30]:
            hl = f"{r['h_l']:.0f}" if not np.isnan(r.get('h_l', np.nan)) else 'N/A'
            pl = f"{np.rad2deg(r['phi_l']):+.1f}" if not np.isnan(r.get('phi_l', np.nan)) else 'N/A'
            hr = f"{r['h_r']:.0f}" if not np.isnan(r.get('h_r', np.nan)) else 'N/A'
            pr = f"{np.rad2deg(r['phi_r']):+.1f}" if not np.isnan(r.get('phi_r', np.nan)) else 'N/A'
            print(f"{r['t_ms']:>8} {hl:>6} {pl:>7} {hr:>6} {pr:>7} "
                  f"{r.get('valid_l',0)}  {r.get('valid_r',0)}")

    # 关键判定
    print(f"\n--- 验证结论 ---")
    issues = []
    if n_invalid_l > 0.1 * len(frames):
        issues.append(f"⚠ 左腿 {n_invalid_l} 帧无解 (>10%)")
    if n_invalid_r > 0.1 * len(frames):
        issues.append(f"⚠ 右腿 {n_invalid_r} 帧无解 (>10%)")
    if branch_flips_l > 0:
        issues.append(f"⚠ 左腿 {branch_flips_l} 次分支翻转")
    if branch_flips_r > 0:
        issues.append(f"⚠ 右腿 {branch_flips_r} 次分支翻转")
    if h_min_l < 20 or h_max_l > 240:
        issues.append(f"⚠ 左腿 h 越界 [{h_min_l:.0f}, {h_max_l:.0f}]")
    if h_min_r < 20 or h_max_r > 240:
        issues.append(f"⚠ 右腿 h 越界 [{h_min_r:.0f}, {h_max_r:.0f}]")

    if issues:
        for issue in issues:
            print(issue)
    else:
        print("✓ FK 验证通过: 无异常求解, 无分支翻转, h 在可达范围内")

    return results


def main():
    p = argparse.ArgumentParser(description="FK 验证: CSV 角度 → h/φ/P7 轨迹")
    p.add_argument("csv_file", help="Unit5 12 列 CSV 角度文件")
    p.add_argument("--full", action="store_true", help="逐帧输出 (前 30 帧)")
    p.add_argument("--out", default=None, help="导出轨迹 CSV 路径")
    args = p.parse_args()
    validate_csv(args.csv_file, full=args.full, out_path=args.out)


if __name__ == '__main__':
    main()
