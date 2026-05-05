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

    # Fixed view bounds (mm), motor O at (0, 0) center.
    VIEW_XLIM = (-280, 280)
    VIEW_YLIM = (-260, 260)

    # Initial angles match zero_calib.py.
    ta0_deg = -162.4
    tb0_deg = -10.0

    fig = plt.figure(figsize=(10, 9))

    # Main mechanism plot
    ax_mech = fig.add_axes([0.08, 0.24, 0.86, 0.72])
    ax_mech.set_aspect('equal')
    ax_mech.set_xlim(*VIEW_XLIM)
    ax_mech.set_ylim(*VIEW_YLIM)
    ax_mech.grid(True, alpha=0.3)

    # Slider axes
    ax_slider_a = fig.add_axes([0.16, 0.12, 0.72, 0.035])
    ax_slider_b = fig.add_axes([0.16, 0.06, 0.72, 0.035])

    slider_a = Slider(ax_slider_a, r'$\theta_a$ [deg]', -180, 180,
                      valinit=ta0_deg, valstep=0.2, valfmt='%.1f')
    slider_b = Slider(ax_slider_b, r'$\theta_b$ [deg]', -180, 180,
                      valinit=tb0_deg, valstep=0.2, valfmt='%.1f')

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
        (ka, kb, ax_mech.plot([], [], color=color, lw=lw, ls=ls,
                              zorder=2)[0])
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
        key: ax_mech.scatter([], [], c=color, s=size, zorder=3,
                             edgecolors='none')
        for key, (color, size) in joint_specs.items()
    }
    invalid_text = ax_mech.text(0, -100, "No solution for this branch",
                                ha='center', va='center', fontsize=14,
                                color='red', visible=False)

    def update(val=None):
        ta = np.deg2rad(slider_a.val)
        tb = np.deg2rad(slider_b.val)

        res = solve_linkage(ta, tb, params)

        if res is not None:
            # Rotate all points by -90 deg for leg-down view
            res_rotated = {}
            for key in ['O', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7']:
                res_rotated[key] = rotate_view(res[key])

            for ka, kb, artist in bar_artists:
                a, b = res_rotated[ka], res_rotated[kb]
                artist.set_data([a[0], b[0]], [a[1], b[1]])
                artist.set_visible(True)

            for key, artist in joint_artists.items():
                artist.set_offsets([res_rotated[key]])
                artist.set_visible(True)

            invalid_text.set_visible(False)

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
            for _, _, artist in bar_artists:
                artist.set_visible(False)
            for artist in joint_artists.values():
                artist.set_visible(False)
            invalid_text.set_visible(True)
            ax_mech.set_title(f"No valid config for "
                              rf"$\theta_a={slider_a.val:.1f}°$, "
                              rf"$\theta_b={slider_b.val:.1f}°$")

        fig.canvas.draw_idle()

    slider_a.on_changed(update)
    slider_b.on_changed(update)

    # Initial draw
    update()

    plt.show()


def interactive_inverse(params: MechanismParams):
    """
    Drag the end-effector target and show the inverse-kinematics angles.

    The initial target matches zero_calib.py.  The displayed view is rotated
    by -90 degrees, same as interactive_sliders().
    """
    from .kinematics import solve_inverse

    def rotate_view(v):
        return np.array([v[1], -v[0]])

    def unrotate_view(v):
        return np.array([-v[1], v[0]])

    def wrap_angle(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def angle_distance(sol, prev):
        da = wrap_angle(sol['theta_a'] - prev[0])
        db = wrap_angle(sol['theta_b'] - prev[1])
        return da * da + db * db

    view_xlim = (-280, 280)
    view_ylim = (-260, 260)
    target_radius = 9.0
    ta0 = np.deg2rad(-162.4)
    tb0 = np.deg2rad(-10.0)

    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_axes([0.08, 0.18, 0.86, 0.78])
    ax.set_aspect('equal')
    ax.set_xlim(*view_xlim)
    ax.set_ylim(*view_ylim)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('display X [mm]')
    ax.set_ylabel('display Y [mm]')

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
        (ka, kb, ax.plot([], [], color=color, lw=lw, ls=ls,
                         zorder=2)[0])
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
        key: ax.scatter([], [], c=color, s=size, zorder=3,
                        edgecolors='none')
        for key, (color, size) in joint_specs.items()
    }
    target_artist = ax.scatter([], [], c='none', edgecolors='black',
                               s=180, linewidths=1.8, zorder=5)
    info_text = ax.text(0.02, 0.02, '', transform=ax.transAxes,
                        ha='left', va='bottom', fontsize=11,
                        bbox=dict(boxstyle='round,pad=0.35',
                                  facecolor='white', alpha=0.85,
                                  edgecolor='#999999'))

    state = {
        'dragging': False,
        'theta': np.array([ta0, tb0]),
        'target_base': None,
    }

    def draw_from_angles(theta_a, theta_b, target_base=None, status=''):
        res = solve_linkage(theta_a, theta_b, params)
        if res is None:
            return

        for key in ['O', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7']:
            res[key + '_view'] = rotate_view(res[key])

        for ka, kb, artist in bar_artists:
            a, b = res[ka + '_view'], res[kb + '_view']
            artist.set_data([a[0], b[0]], [a[1], b[1]])
            artist.set_visible(True)

        for key, artist in joint_artists.items():
            artist.set_offsets([res[key + '_view']])
            artist.set_visible(True)

        if target_base is None:
            target_base = res['P7']
        target_view = rotate_view(target_base)
        target_artist.set_offsets([target_view])
        state['target_base'] = target_base.copy()

        info_text.set_text(
            f"theta_a = {np.rad2deg(theta_a):7.2f} deg\n"
            f"theta_b = {np.rad2deg(theta_b):7.2f} deg\n"
            f"P7_base = ({target_base[0]:7.1f}, {target_base[1]:7.1f}) mm\n"
            f"{status}"
        )
        ax.set_title(
            "Inverse IK Drag Target  |  drag P7 ring to update theta_a/theta_b",
            fontsize=11,
        )
        fig.canvas.draw_idle()

    def update_target_from_view(x_view, y_view):
        target_base = unrotate_view(np.array([x_view, y_view]))
        sols = solve_inverse(target_base, params, elbow=0)
        if not sols:
            draw_from_angles(state['theta'][0], state['theta'][1],
                             target_base=target_base, status='UNREACHABLE')
            return
        sol = min(sols, key=lambda s: angle_distance(s, state['theta']))
        state['theta'][:] = [sol['theta_a'], sol['theta_b']]
        draw_from_angles(sol['theta_a'], sol['theta_b'],
                         target_base=target_base, status='reachable')

    def on_press(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        target_view = rotate_view(state['target_base'])
        d = np.hypot(event.xdata - target_view[0], event.ydata - target_view[1])
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

    draw_from_angles(ta0, tb0)
    plt.show()
