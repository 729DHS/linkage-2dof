#!/usr/bin/env python3
"""
2-DOF Wheel-Legged Robot Linkage Mechanism Simulation.

Usage:
    python main.py              # Single configuration demo
    python main.py anim         # Animation
    python main.py workspace    # Workspace analysis
    python main.py trajectory   # Trajectory following
    python main.py branches     # Show all 4 assembly modes
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

from src.mechanism import MechanismParams, default_params
from src.kinematics import solve_linkage, solve_all_branches, solve_trajectory, solve_workspace
from src.visualization import (
    plot_mechanism, plot_workspace, plot_trajectory,
    plot_all_branches, animate_mechanism,
)


def demo_single(params: MechanismParams):
    """Plot a single mechanism configuration (convex parallelogram)."""
    theta_a = np.deg2rad(0)
    theta_b = np.deg2rad(90)

    result = solve_linkage(theta_a, theta_b, params)

    if result is None:
        print(f"No valid configuration for "
              f"theta_a={np.rad2deg(theta_a):.0f}°, "
              f"theta_b={np.rad2deg(theta_b):.0f}°")
        return

    fig, ax = plt.subplots(figsize=(10, 8))
    plot_mechanism(result, params, ax=ax,
                   title=(rf"Linkage Configuration: "
                          rf"$\theta_a={np.rad2deg(theta_a):.0f}°$, "
                          rf"$\theta_b={np.rad2deg(theta_b):.0f}°$ "
                          rf"(branch_d={result['branch_d']:+d}, "
                          rf"branch_f={result['branch_f']:+d})"))

    print(f"End-effector (P7) position: ({result['P7'][0]:.1f}, "
          f"{result['P7'][1]:.1f}) mm")
    print(f"theta_d = {np.rad2deg(result['theta_d']):.2f}°")
    print(f"theta_f = {np.rad2deg(result['theta_f']):.2f}°")
    print(params)

    plt.tight_layout()
    plt.show()


def demo_branches(params: MechanismParams):
    """Show all valid assembly modes for given motor angles."""
    theta_a = np.deg2rad(0)
    theta_b = np.deg2rad(90)

    print(f"Solving all branches for "
          f"theta_a={np.rad2deg(theta_a):.0f}°, "
          f"theta_b={np.rad2deg(theta_b):.0f}° ...")

    all_results = solve_all_branches(theta_a, theta_b, params)

    n_valid = sum(1 for r in all_results.values() if r is not None)
    print(f"Valid assembly modes: {n_valid}/4")

    for (bd, bf), res in sorted(all_results.items()):
        if res is not None:
            P1 = res['P1']; P3 = res['P3']; P4 = res['P4']
            p4_vec = P1 + P3  # expected for convex parallelogram
            is_convex = np.linalg.norm(P4 - p4_vec) < 30
            para_type = "凸 (convex)" if is_convex else "交叉 (crossed)"
            print(f"  branch_d={bd:+d}, branch_f={bf:+d}: "
                  f"P7=({res['P7'][0]:6.1f}, {res['P7'][1]:6.1f}) mm  "
                  f"O-P1-P4-P3: {para_type}")
        else:
            print(f"  branch_d={bd:+d}, branch_f={bf:+d}: INVALID")

    fig = plot_all_branches(all_results, theta_a, theta_b, params)
    plt.show()


def demo_animation(params: MechanismParams):
    """Animate mechanism motion."""
    t = np.linspace(0, 2 * np.pi, 200)
    theta_a_seq = np.deg2rad(30) * np.sin(t) + np.deg2rad(10)
    theta_b_seq = np.deg2rad(60) * np.sin(2 * t + 1) + np.deg2rad(90)

    print("Solving trajectory...")
    results = solve_trajectory(theta_a_seq, theta_b_seq, params)

    n_valid = sum(1 for r in results if r is not None)
    print(f"Valid frames: {n_valid}/{len(results)}")

    if n_valid == 0:
        print("No valid frames to animate.")
        return

    print("Creating animation...")
    anim = animate_mechanism(
        results, params, theta_a_seq, theta_b_seq,
        interval=50, title="Linkage Mechanism Animation",
        save_path="animation.gif",
    )
    plt.show()


def demo_workspace(params: MechanismParams):
    """Compute and plot workspace of the end-effector."""
    print("Sampling workspace (this may take a moment)...")
    ws = solve_workspace(60, params,
                         theta_a_range=(-np.pi, np.pi),
                         theta_b_range=(-np.pi, np.pi))

    print(f"Valid configurations: {ws['n_valid']}/{ws['n_total']} "
          f"({100*ws['n_valid']/ws['n_total']:.1f}%)")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

    plot_workspace(ws, ax=ax1,
                   title="Valid Configurations in Motor-Angle Space")

    ax2.scatter(ws['P7_x'][ws['valid']], ws['P7_y'][ws['valid']],
                c='steelblue', s=1, alpha=0.6)
    ax2.scatter(0, 0, c='red', s=100, marker='*', label='Motor axis O')
    ax2.set_aspect('equal')
    ax2.set_xlabel('X [mm]')
    ax2.set_ylabel('Y [mm]')
    ax2.set_title("End-Effector Reachable Positions")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    plt.tight_layout()
    plt.show()


def demo_trajectory(params: MechanismParams):
    """Demonstrate trajectory following."""
    t = np.linspace(0, 2 * np.pi, 300)
    theta_a_seq = np.deg2rad(20) * np.sin(t)
    theta_b_seq = np.deg2rad(45) * np.sin(2 * t) + np.deg2rad(90)

    print("Solving trajectory...")
    results = solve_trajectory(theta_a_seq, theta_b_seq, params)

    n_valid = sum(1 for r in results if r is not None)
    print(f"Valid frames: {n_valid}/{len(results)}")

    if n_valid == 0:
        print("No valid trajectory.")
        return

    fig, ax = plt.subplots(figsize=(10, 8))
    plot_trajectory(
        results, ax=ax,
        title="End-Effector Trajectory with Mechanism Overlay",
        show_mechanism_at=[0, len(results)//3, 2*len(results)//3, len(results)-1],
        params=params,
    )
    plt.tight_layout()
    plt.show()


def main():
    params = default_params()

    cmds = {
        'anim': demo_animation,
        'workspace': demo_workspace,
        'trajectory': demo_trajectory,
        'branches': demo_branches,
    }

    if len(sys.argv) > 1:
        cmd = sys.argv[1]
        if cmd in cmds:
            cmds[cmd](params)
        else:
            print(f"Unknown command: {cmd}")
            print(f"Available: {list(cmds.keys())}")
    else:
        demo_single(params)


if __name__ == '__main__':
    main()
