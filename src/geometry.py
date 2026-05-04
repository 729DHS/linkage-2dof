"""
2D geometry utilities: rotation, circle intersection, triangle solving.
"""

import numpy as np
from typing import Optional, Tuple


def rot(th: float) -> np.ndarray:
    """2D rotation matrix."""
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s], [s, c]])


def rotate_vec(v: np.ndarray, th: float) -> np.ndarray:
    """Rotate 2D vector by angle th."""
    c, s = np.cos(th), np.sin(th)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def circle_intersection(
    c1: np.ndarray, r1: float, c2: np.ndarray, r2: float
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Compute intersection points of two circles.

    Circle 1: center c1, radius r1.
    Circle 2: center c2, radius r2.

    Returns (p_left, p_right) where:
      - p_left  is the point to the left of vector c1 -> c2.
      - p_right is the point to the right.
      - Returns (None, None) if no intersection.
    """
    v = c2 - c1
    d = np.linalg.norm(v)

    if d < 1e-12:
        return None, None  # concentric

    if d > r1 + r2 + 1e-10 or d < abs(r1 - r2) - 1e-10:
        return None, None  # no intersection

    a = (r1 ** 2 - r2 ** 2 + d ** 2) / (2.0 * d)
    h_sq = max(r1 ** 2 - a ** 2, 0.0)
    h = np.sqrt(h_sq)

    p_mid = c1 + (a / d) * v

    if h < 1e-10:
        return p_mid.copy(), p_mid.copy()  # tangent

    perp = np.array([-v[1], v[0]]) / d
    p_left = p_mid + h * perp
    p_right = p_mid - h * perp
    return p_left, p_right


def triangle_solve_local(a: float, b: float, c: float) -> np.ndarray:
    """
    Solve triangle vertex position in local frame.

    Given triangle with side lengths:
      - a = distance v0 -> v1
      - b = distance v1 -> v2
      - c = distance v0 -> v2

    Place v0 at (0,0), v1 at (a, 0) on x-axis.
    Return v2 position (x, y) with y > 0 by convention.

    Raises ValueError if triangle inequality is violated (with tolerance).
    """
    if a <= 0 or b <= 0 or c <= 0:
        raise ValueError(f"Triangle sides must be positive: a={a}, b={b}, c={c}")
    if a + b < c - 1e-10 or a + c < b - 1e-10 or b + c < a - 1e-10:
        raise ValueError(
            f"Triangle inequality violated: a={a:.4f}, b={b:.4f}, c={c:.4f}"
        )

    # Law of cosines at v0: angle between side a (v0->v1) and side c (v0->v2)
    cos_alpha = (a ** 2 + c ** 2 - b ** 2) / (2.0 * a * c)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    alpha = np.arccos(cos_alpha)
    return np.array([c * np.cos(alpha), c * np.sin(alpha)])


def norm_angle(th: float) -> float:
    """Normalize angle to [-pi, pi)."""
    return ((th + np.pi) % (2 * np.pi)) - np.pi
