"""
Forward kinematics solver for the 2-DOF linkage mechanism.

Solution method: sequential circle-intersection.

Given motor angles (theta_a, theta_b):
  1. Compute P1, P2 from bar_a orientation (driven by theta_a).
  2. Compute P3 from bar_b orientation (driven by theta_b).
  3. Find P4 by intersecting:
       circle(P1, |P1-P4|) ∩ circle(P3, L_c)
     This yields 0, 1, or 2 solutions. Select by branch_d.
  4. Orientation theta_d = atan2(P4 - P1). Compute P5 from bar_d triangle.
  5. Find P6 by intersecting:
       circle(P2, |P2-P6|) ∩ circle(P5, L_e)
     Select by branch_f.
  6. Orientation theta_f = atan2(P6 - P2). Compute P7 (end-effector) from bar_f triangle.

All formulas are closed-form (analytical), involving only trigonometric
functions and square roots via the circle-intersection formula.
"""

import numpy as np
from typing import Optional, List, Dict
from .mechanism import MechanismParams
from .geometry import rot, rotate_vec, circle_intersection


def solve_linkage(
    theta_a: float,
    theta_b: float,
    params: MechanismParams,
    prev_theta_d: Optional[float] = None,
    prev_theta_f: Optional[float] = None,
) -> Optional[Dict]:
    """
    Solve forward kinematics for given motor angles.

    Parameters
    ----------
    theta_a : float
        Motor shaft A angle [rad]. Defines bar_a orientation.
    theta_b : float
        Motor shaft B angle [rad]. Defines bar_b orientation.
    params : MechanismParams
        Linkage geometry.
    prev_theta_d : float, optional
        Previous bar_d orientation for assembly continuity.
    prev_theta_f : float, optional
        Previous bar_f orientation for assembly continuity.

    Returns
    -------
    dict or None
        {'O', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7',
         'theta_d', 'theta_f', 'valid'}
        Returns None if no valid configuration exists.
    """
    O = np.zeros(2)

    # ---- Step 1: bar_a (ternary) ----
    # Local frame: O at origin, P1 on +x axis at distance L_OP1.
    # P2 in local coords pre-computed from triangle.
    # Global: rotate local frame by theta_a.
    P1 = rotate_vec(np.array([params.L_OP1, 0.0]), theta_a)
    P2 = rotate_vec(params._a2_local, theta_a)

    # ---- Step 2: bar_b (binary) ----
    P3 = params.L_b * np.array([np.cos(theta_b), np.sin(theta_b)])

    # ---- Step 3: find P4 (bar_d point D_CD) ----
    # |P4 - P1| = L_P1P4   (on bar_d, from reference P1 to P4)
    # |P4 - P3| = L_c      (bar_c connects P3 to P4)
    p4_left, p4_right = circle_intersection(P1, params.L_P1P4, P3, params.L_c)
    if p4_left is None:
        return None

    if np.allclose(p4_left, p4_right):
        P4 = p4_left
    elif prev_theta_d is not None:
        # Choose closer to previous configuration
        P4_prev = P1 + rotate_vec(
            np.array([params.L_P1P4, 0.0]), prev_theta_d
        )
        P4 = p4_left if (
            np.linalg.norm(p4_left - P4_prev)
            <= np.linalg.norm(p4_right - P4_prev)
        ) else p4_right
    else:
        P4 = p4_left if params.branch_d == -1 else p4_right

    # ---- Step 4: bar_d orientation and P5 ----
    v_d = P4 - P1
    theta_d = np.arctan2(v_d[1], v_d[0])
    P5 = P1 + rotate_vec(params._d5_local, theta_d)

    # ---- Step 5: find P6 (bar_f point F_EF) ----
    # |P6 - P2| = L_P2P6   (on bar_f, from reference P2 to P6)
    # |P6 - P5| = L_e      (bar_e connects P5 to P6)
    p6_left, p6_right = circle_intersection(P2, params.L_P2P6, P5, params.L_e)
    if p6_left is None:
        return None

    if np.allclose(p6_left, p6_right):
        P6 = p6_left
    elif prev_theta_f is not None:
        P6_prev = P2 + rotate_vec(
            np.array([params.L_P2P6, 0.0]), prev_theta_f
        )
        P6 = p6_left if (
            np.linalg.norm(p6_left - P6_prev)
            <= np.linalg.norm(p6_right - P6_prev)
        ) else p6_right
    else:
        P6 = p6_left if params.branch_f == -1 else p6_right

    # ---- Step 6: bar_f orientation and P7 (end-effector) ----
    v_f = P6 - P2
    theta_f = np.arctan2(v_f[1], v_f[0])
    P7 = P2 + rotate_vec(params._f7_local, theta_f)

    return {
        'O': O,
        'P1': P1, 'P2': P2, 'P3': P3,
        'P4': P4, 'P5': P5, 'P6': P6, 'P7': P7,
        'theta_d': theta_d, 'theta_f': theta_f,
        'valid': True,
    }


def solve_trajectory(
    theta_a_seq: np.ndarray,
    theta_b_seq: np.ndarray,
    params: MechanismParams,
) -> List[Optional[Dict]]:
    """
    Solve linkage over a sequence of motor angle pairs.

    Tracks assembly continuity by using previous solution as initial guess.

    Parameters
    ----------
    theta_a_seq : array-like
        Sequence of motor A angles [rad].
    theta_b_seq : array-like
        Sequence of motor B angles [rad].
    params : MechanismParams
        Linkage geometry.

    Returns
    -------
    list of dict or None
        One result per input pair. None for invalid configurations.
    """
    results = []
    prev_d = None
    prev_f = None

    for ta, tb in zip(theta_a_seq, theta_b_seq):
        res = solve_linkage(ta, tb, params, prev_d, prev_f)
        results.append(res)
        if res is not None:
            prev_d = res['theta_d']
            prev_f = res['theta_f']

    return results


def solve_workspace(
    n_theta: int,
    params: MechanismParams,
    theta_a_range=(-np.pi, np.pi),
    theta_b_range=(-np.pi, np.pi),
) -> Dict:
    """
    Sample the mechanism workspace over a grid of motor angles.

    Returns
    -------
    dict with keys:
        theta_a_grid, theta_b_grid: 2D arrays of motor angles.
        P7_x, P7_y: 2D arrays of end-effector positions.
        valid: boolean 2D array indicating valid configurations.
    """
    ta = np.linspace(theta_a_range[0], theta_a_range[1], n_theta)
    tb = np.linspace(theta_b_range[0], theta_b_range[1], n_theta)
    TA, TB = np.meshgrid(ta, tb)

    P7x = np.full_like(TA, np.nan)
    P7y = np.full_like(TA, np.nan)
    valid = np.zeros_like(TA, dtype=bool)

    prev_d, prev_f = None, None
    n_valid = 0
    n_total = TA.size

    for i in range(TA.shape[0]):
        for j in range(TA.shape[1]):
            res = solve_linkage(TA[i, j], TB[i, j], params, prev_d, prev_f)
            if res is not None:
                P7x[i, j] = res['P7'][0]
                P7y[i, j] = res['P7'][1]
                valid[i, j] = True
                prev_d = res['theta_d']
                prev_f = res['theta_f']
                n_valid += 1

    return {
        'theta_a': TA,
        'theta_b': TB,
        'P7_x': P7x,
        'P7_y': P7y,
        'valid': valid,
        'n_valid': n_valid,
        'n_total': n_total,
    }
