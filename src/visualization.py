"""
Visualization tools for the linkage mechanism.

Provides functions for:
  - Static mechanism pose plotting.
  - Workspace / trajectory plotting.
  - Animation of mechanism motion.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, FancyArrowPatch
from typing import Dict, List, Optional
from .mechanism import MechanismParams
from .kinematics import solve_trajectory, solve_linkage


# Color palette
COLOR_BAR_A = '#2196F3'  # blue
COLOR_BAR_B = '#4CAF50'  # green
COLOR_BAR_C = '#FF9800'  # orange
COLOR_BAR_D = '#9C27B0'  # purple
COLOR_BAR_E = '#F44336'  # red
COLOR_BAR_F = '#00BCD4'  # cyan
COLOR_JOINT = '#333333'
COLOR_MOTOR = '#E91E63'  # pink
COLOR_END = '#FF5722'    # deep orange
COLOR_TRACE = '#607D8B'  # gray-blue
COLOR_GROUND = '#795548' # brown


def plot_mechanism(
    result: Dict,
    params: Optional[MechanismParams] = None,
    ax: Optional[plt.Axes] = None,
    title: str = "Linkage Mechanism",
    show_labels: bool = True,
    trace_points: Optional[np.ndarray] = None,
) -> plt.Axes:
    """
    Plot the mechanism in a single configuration.

    Parameters
    ----------
    result : dict
        Output from solve_linkage() with keys O, P1..P7.
    params : MechanismParams, optional
        Used to annotate bar lengths.
    ax : matplotlib Axes, optional
    title : str
    show_labels : bool
        Show point labels.
    trace_points : (N,2) array, optional
        End-effector trace to plot.

    Returns
    -------
    ax
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(10, 8))

    O = result['O']
    P1, P2, P3 = result['P1'], result['P2'], result['P3']
    P4, P5, P6, P7 = result['P4'], result['P5'], result['P6'], result['P7']

    # ---- Draw bars as line segments ----
    def draw_bar(p1, p2, color, lw=2.5, ls='-'):
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, lw=lw, ls=ls,
                zorder=2)

    draw_bar(O, P1, COLOR_BAR_A, lw=3)
    draw_bar(O, P2, COLOR_BAR_A, lw=3)
    draw_bar(P1, P2, COLOR_BAR_A, lw=1.5, ls='--')

    draw_bar(O, P3, COLOR_BAR_B, lw=3)
    draw_bar(P3, P4, COLOR_BAR_C, lw=2.5)

    draw_bar(P1, P4, COLOR_BAR_D, lw=3)
    draw_bar(P1, P5, COLOR_BAR_D, lw=1.5, ls='--')
    draw_bar(P4, P5, COLOR_BAR_D, lw=1.5, ls='--')

    draw_bar(P5, P6, COLOR_BAR_E, lw=2.5)

    draw_bar(P2, P6, COLOR_BAR_F, lw=3)
    draw_bar(P2, P7, COLOR_BAR_F, lw=3)
    draw_bar(P6, P7, COLOR_BAR_F, lw=1.5, ls='--')

    # ---- Draw joints ----
    def draw_joint(p, color=COLOR_JOINT, size=8, zorder=3):
        ax.scatter(p[0], p[1], c=color, s=size, zorder=zorder, edgecolors='none')

    draw_joint(O, COLOR_MOTOR, size=80)          # motor axis
    draw_joint(P1, size=40)
    draw_joint(P2, size=40)
    draw_joint(P3, size=40)
    draw_joint(P4, size=40)
    draw_joint(P5, size=40)
    draw_joint(P6, size=40)
    draw_joint(P7, COLOR_END, size=80)           # wheel hub

    # ---- Labels ----
    if show_labels:
        offset = 0.02
        ax.annotate('O (Motor)', (O[0] + offset, O[1] + offset), fontsize=9,
                    color=COLOR_MOTOR, fontweight='bold')
        ax.annotate('P1 (AD)', (P1[0] + offset, P1[1] + offset), fontsize=8)
        ax.annotate('P2 (AF)', (P2[0] + offset, P2[1] + offset), fontsize=8)
        ax.annotate('P3', (P3[0] + offset, P3[1] + offset), fontsize=8)
        ax.annotate('P4 (CD)', (P4[0] + offset, P4[1] + offset), fontsize=8)
        ax.annotate('P5', (P5[0] + offset, P5[1] + offset), fontsize=8)
        ax.annotate('P6 (EF)', (P6[0] + offset, P6[1] + offset), fontsize=8)
        ax.annotate('P7 (Wheel)', (P7[0] + offset, P7[1] + offset), fontsize=9,
                    color=COLOR_END, fontweight='bold')

    # ---- Trace ----
    if trace_points is not None:
        ax.plot(trace_points[:, 0], trace_points[:, 1],
                color=COLOR_TRACE, lw=1, alpha=0.5, zorder=1)

    # ---- Styling ----
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='gray', lw=0.5, alpha=0.5)
    ax.axvline(x=0, color='gray', lw=0.5, alpha=0.5)

    return ax


def plot_workspace(
    ws_result: Dict,
    ax: Optional[plt.Axes] = None,
    title: str = "End-Effector Workspace",
) -> plt.Axes:
    """
    Plot the workspace (end-effector reachable positions).

    Parameters
    ----------
    ws_result : dict
        Output from solve_workspace().
    ax : matplotlib Axes, optional
    title : str

    Returns
    -------
    ax
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 8))

    TA = ws_result['theta_a']
    TB = ws_result['theta_b']

    # Motor-angle space: mark valid/invalid regions
    ax.pcolormesh(np.rad2deg(TA), np.rad2deg(TB),
                  ws_result['valid'].astype(float),
                  cmap='RdYlGn', shading='auto', alpha=0.6)

    ax.set_xlabel(r'$\theta_a$ [deg]')
    ax.set_ylabel(r'$\theta_b$ [deg]')
    ax.set_title(title)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    return ax


def plot_trajectory(
    results: List[Optional[Dict]],
    ax: Optional[plt.Axes] = None,
    title: str = "End-Effector Trajectory",
    show_mechanism_at: Optional[List[int]] = None,
    params: Optional[MechanismParams] = None,
) -> plt.Axes:
    """
    Plot end-effector trajectory from a sequence of solutions.

    Parameters
    ----------
    results : list of dict
        Output from solve_trajectory().
    ax : matplotlib Axes, optional
    title : str
    show_mechanism_at : list of int, optional
        Frame indices at which to overlay the mechanism.
    params : MechanismParams, optional

    Returns
    -------
    ax
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(10, 8))

    # Collect end-effector positions
    valid_mask = np.array([r is not None for r in results])
    valid_results = [r for r in results if r is not None]
    if not valid_results:
        ax.set_title("No valid configurations")
        return ax

    P7_all = np.array([r['P7'] for r in valid_results])
    ax.plot(P7_all[:, 0], P7_all[:, 1], '-', color=COLOR_END, lw=2,
            label='End-effector path', zorder=3)
    ax.scatter(P7_all[0, 0], P7_all[0, 1], c='green', s=80, marker='o',
               label='Start', zorder=4)
    ax.scatter(P7_all[-1, 0], P7_all[-1, 1], c='red', s=80, marker='x',
               label='End', zorder=4)

    # Overlay mechanism at specified frames
    if show_mechanism_at is not None and params is not None:
        for idx in show_mechanism_at:
            if idx < len(results) and results[idx] is not None:
                plot_mechanism(results[idx], params, ax=ax, show_labels=False,
                               title="")
                ax.annotate(f'Frame {idx}', results[idx]['P7'],
                            fontsize=7, color=COLOR_END)

    ax.set_aspect('equal')
    ax.set_title(title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True, alpha=0.3)
    ax.legend()

    return ax


def plot_all_branches(
    branch_results: dict,
    theta_a: float,
    theta_b: float,
    params: Optional[MechanismParams] = None,
) -> plt.Figure:
    """
    Plot all valid assembly modes side by side.

    Parameters
    ----------
    branch_results : dict
        Output from solve_all_branches(), mapping (branch_d, branch_f) -> result.
    theta_a, theta_b : float
        Motor angles [rad].
    params : MechanismParams, optional

    Returns
    -------
    matplotlib Figure.
    """
    valid_branches = [
        (bd, bf, res)
        for (bd, bf), res in branch_results.items()
        if res is not None
    ]
    n_valid = len(valid_branches)

    if n_valid == 0:
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.text(0.5, 0.5, "No valid assembly modes",
                ha='center', va='center', fontsize=14)
        ax.set_title(f"No solutions for "
                     rf"$\theta_a={np.rad2deg(theta_a):.0f}°$, "
                     rf"$\theta_b={np.rad2deg(theta_b):.0f}°$")
        return fig

    cols = min(2, n_valid)
    rows = (n_valid + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(9 * cols, 7 * rows))
    if n_valid == 1:
        axes = np.array([axes])
    axes = np.atleast_1d(axes).flatten()

    for idx, (bd, bf, res) in enumerate(valid_branches):
        ax = axes[idx]
        plot_mechanism(res, params, ax=ax, show_labels=True,
                       title="")

        # Determine parallelogram type
        # Check convexity of O-P1-P4-P3 by cross product sign
        # Vector O->P1 and O->P3, check if P4 is on same side
        cross_OP1_OP3 = np.cross(res['P1'], res['P3'])
        cross_OP1_P1P4 = np.cross(res['P1'], res['P4'] - res['P1'])

        # Check if P4 is roughly P1 + P3 (convex) or not (crossed)
        p4_expected_convex = res['P1'] + res['P3']
        dist_convex = np.linalg.norm(res['P4'] - p4_expected_convex)
        p4_expected_crossed = res['P1'] - res['P3']  # rough check
        dist_crossed = np.linalg.norm(res['P4'] - p4_expected_crossed)

        if dist_convex < dist_crossed:
            para1_type = "convex"
        else:
            para1_type = "crossed"

        # Similarly for parallelogram P1-P2-P6-P5
        p6_expected_convex = res['P2'] + res['P5'] - res['P1']
        dist_c2 = np.linalg.norm(res['P6'] - (res['P5'] + res['P2'] - res['P1']))

        ax.set_title(
            rf"branch_d={bd:+d}, branch_f={bf:+d}  |  "
            rf"$\theta_a={np.rad2deg(theta_a):.0f}°$, "
            rf"$\theta_b={np.rad2deg(theta_b):.0f}°$\n"
            rf"O-P1-P4-P3: {para1_type}  |  "
            rf"P7=({res['P7'][0]:.0f}, {res['P7'][1]:.0f}) mm",
            fontsize=10,
        )

    # Hide unused axes
    for idx in range(n_valid, len(axes)):
        axes[idx].set_visible(False)

    fig.suptitle("All Assembly Modes", fontsize=14, fontweight='bold', y=1.01)
    fig.tight_layout()
    return fig


def animate_mechanism(
    results: List[Optional[Dict]],
    params: MechanismParams,
    theta_a_seq: np.ndarray,
    theta_b_seq: np.ndarray,
    interval: int = 50,
    title: str = "Linkage Animation",
    save_path: Optional[str] = None,
) -> FuncAnimation:
    """
    Create animation of mechanism motion.

    Parameters
    ----------
    results : list of dict
        Output from solve_trajectory().
    params : MechanismParams
    theta_a_seq, theta_b_seq : array
        Motor angles at each frame.
    interval : int
        Milliseconds between frames.
    title : str
    save_path : str, optional
        If provided, save animation to this path (e.g., 'animation.gif').

    Returns
    -------
    matplotlib FuncAnimation object.
    """
    # Filter to valid frames
    valid_frames = [(i, r) for i, r in enumerate(results) if r is not None]
    if not valid_frames:
        raise ValueError("No valid frames to animate.")

    fig, ax = plt.subplots(figsize=(10, 8))

    # Compute bounds
    all_P7 = np.array([r['P7'] for _, r in valid_frames])
    margin = 0.15
    x_min, x_max = all_P7[:, 0].min() - margin, all_P7[:, 0].max() + margin
    y_min, y_max = all_P7[:, 1].min() - margin, all_P7[:, 1].max() + margin
    # Include motor at origin
    x_min, x_max = min(x_min, -margin), max(x_max, margin)
    y_min, y_max = min(y_min, -margin), max(y_max, margin)

    # Trace storage
    trace_x, trace_y = [], []

    def init():
        ax.clear()
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='gray', lw=0.5, alpha=0.5)
        ax.axvline(x=0, color='gray', lw=0.5, alpha=0.5)
        return []

    def update(frame_data):
        i, result = frame_data
        ax.clear()
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='gray', lw=0.5, alpha=0.5)
        ax.axvline(x=0, color='gray', lw=0.5, alpha=0.5)

        # Update trace
        trace_x.append(result['P7'][0])
        trace_y.append(result['P7'][1])

        # Plot trace
        ax.plot(trace_x, trace_y, color=COLOR_TRACE, lw=1, alpha=0.5, zorder=1)

        # Plot mechanism
        plot_mechanism(result, params, ax=ax, show_labels=False, title="")

        ax.set_title(
            f"{title}\n"
            rf"$\theta_a={np.rad2deg(theta_a_seq[i]):.1f}°$  "
            rf"$\theta_b={np.rad2deg(theta_b_seq[i]):.1f}°$  "
            f"Frame {i}"
        )
        ax.set_xlabel('X [mm]')
        ax.set_ylabel('Y [mm]')
        return []

    anim = FuncAnimation(
        fig, update, frames=valid_frames, init_func=init,
        interval=interval, blit=False
    )

    if save_path is not None:
        anim.save(save_path, writer='pillow', fps=1000 // interval)
        print(f"Animation saved to {save_path}")

    return anim


def interactive_sliders(params: MechanismParams):
    """
    Interactive exploration with sliders for theta_a and theta_b.

    Drag sliders to change motor angles. The view follows the default
    physical branch only, which keeps matplotlib responsive for live tuning.
    """
    from matplotlib.widgets import Slider
    from .kinematics import solve_linkage

    # View rotation: -90 deg clockwise so leg points downward
    # (x, y) -> (y, -x)
    def rotate_view(v):
        return np.array([v[1], -v[0]])

    # Fixed view bounds (mm), motor O at (0, 0) center
    VIEW_XLIM = (-250, 250)
    VIEW_YLIM = (-300, 100)

    # Initial angles
    ta0 = 0.0
    tb0 = np.deg2rad(90)

    fig = plt.figure(figsize=(10, 9))

    # Main mechanism plot
    ax_mech = fig.add_axes([0.08, 0.24, 0.86, 0.72])
    ax_mech.set_aspect('equal')
    ax_mech.set_xlim(*VIEW_XLIM)
    ax_mech.set_ylim(*VIEW_YLIM)
    ax_mech.grid(True, alpha=0.3)
    # Ground marker
    ax_mech.axhline(y=VIEW_YLIM[0] + 20, color='brown', lw=4, alpha=0.5)

    # Slider axes
    ax_slider_a = fig.add_axes([0.16, 0.12, 0.72, 0.035])
    ax_slider_b = fig.add_axes([0.16, 0.06, 0.72, 0.035])

    slider_a = Slider(ax_slider_a, r'$\theta_a$ [deg]', -180, 180,
                      valinit=0, valstep=1)
    slider_b = Slider(ax_slider_b, r'$\theta_b$ [deg]', -180, 180,
                      valinit=90, valstep=1)

    def update(val=None):
        ta = np.deg2rad(slider_a.val)
        tb = np.deg2rad(slider_b.val)

        res = solve_linkage(ta, tb, params)
        ax_mech.clear()
        ax_mech.set_aspect('equal')
        ax_mech.set_xlim(*VIEW_XLIM)
        ax_mech.set_ylim(*VIEW_YLIM)
        ax_mech.grid(True, alpha=0.3)
        ax_mech.axhline(y=VIEW_YLIM[0] + 20, color='brown', lw=4, alpha=0.5)

        if res is not None:
            # Rotate all points by -90 deg for leg-down view
            res_rotated = {}
            for key in ['O', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7']:
                res_rotated[key] = rotate_view(res[key])
            for key in ['theta_d', 'theta_f', 'valid', 'branch_d', 'branch_f']:
                if key in res:
                    res_rotated[key] = res[key]

            plot_mechanism(res_rotated, params, ax=ax_mech,
                          show_labels=False, title="")

            dist_to_convex = np.linalg.norm(
                res['P4'] - (res['P1'] + res['P3']))
            para_type = "convex" if dist_to_convex < 30 else "crossed"

            ax_mech.set_title(
                rf"$\theta_a={slider_a.val:.1f}°$  "
                rf"$\theta_b={slider_b.val:.1f}°$  |  "
                rf"branch_d={res['branch_d']:+d}, "
                rf"branch_f={res['branch_f']:+d}  |  "
                rf"O-P1-P4-P3: {para_type}  |  "
                rf"P7=({res['P7'][0]:.0f}, {res['P7'][1]:.0f}) mm",
                fontsize=11,
            )
        else:
            ax_mech.text(0, -100, "No solution for this branch",
                         ha='center', va='center', fontsize=14, color='red')
            ax_mech.set_title(f"No valid config for "
                              rf"$\theta_a={slider_a.val:.1f}°$, "
                              rf"$\theta_b={slider_b.val:.1f}°$")

        fig.canvas.draw_idle()

    slider_a.on_changed(update)
    slider_b.on_changed(update)

    # Initial draw
    update()

    plt.show()
