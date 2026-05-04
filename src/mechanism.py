"""
Linkage mechanism parameter definition.

The mechanism is a 2-DOF planar linkage for a wheel-legged robot leg.
Two coaxial motor shafts at origin O drive bars A and B respectively.

Topology (point labels):
  Motor shaft A drives bar_a, motor shaft B drives bar_b.
  Both motor shafts are coaxial at origin O.

  bar_a (ternary, 3 joints):
    O  -- motor axis (joint 1, fixed)
    P1 -- middle connection (joint 2, connects to bar_d at D_AD)
    P2 -- end connection   (joint 3, connects to bar_f at F_AF)

  bar_b (binary):  O -> P3

  bar_c (binary):  P3 -> P4

  bar_d (ternary, 3 joints):
    P1 -- AD connection (from bar_a)
    P4 -- CD connection (from bar_c)
    P5 -- end point (connects to bar_e)

  bar_e (binary):  P5 -> P6

  bar_f (ternary, 3 joints):
    P2 -- AF connection (from bar_a)
    P6 -- EF connection (from bar_e)
    P7 -- end-effector (wheel hub motor)

All ternary links are straight (collinear) and defined by three side lengths.
Binary links are defined by a single length.
"""

from dataclasses import dataclass, field
import numpy as np
from .geometry import triangle_solve_local


@dataclass
class MechanismParams:
    """
    Linkage geometric parameters. Unit: mm.

    Ternary links are defined by three side lengths (triangle inequality
    handles both triangular and straight/collinear cases).

    Topology: two parallelograms in an 8-shape:
      Parallelogram 1: O-P1-P4-P3  (motor → bar_a → bar_d → bar_c → bar_b)
      Parallelogram 2: P1-P2-P6-P5  (bar_a → bar_f → bar_e → bar_d)

    Assembly branch selection:
      branch_d: +1 or -1, circle-intersection branch for bar_d (P4).
      branch_f: +1 or -1, circle-intersection branch for bar_f (P6).
    """

    # --- bar_a (ternary): O -- P1 -- P2 ---
    L_OP1: float   # |O   - P1|
    L_OP2: float   # |O   - P2|
    L_P1P2: float  # |P1  - P2|

    # --- bar_b (binary): O -> P3 ---
    L_b: float

    # --- bar_c (binary): P3 -> P4 ---
    L_c: float

    # --- bar_d (ternary): P1 -- P4 -- P5 ---
    L_P1P4: float  # |P1  - P4|
    L_P1P5: float  # |P1  - P5|
    L_P4P5: float  # |P4  - P5|

    # --- bar_e (binary): P5 -> P6 ---
    L_e: float

    # --- bar_f (ternary): P2 -- P6 -- P7 ---
    L_P2P6: float  # |P2  - P6|
    L_P2P7: float  # |P2  - P7|
    L_P6P7: float  # |P6  - P7|

    # --- assembly mode ---
    branch_d: int = -1
    branch_f: int = -1

    # Pre-computed local coordinates (set in __post_init__)
    _a2_local: np.ndarray = field(init=False, repr=False)
    _d5_local: np.ndarray = field(init=False, repr=False)
    _f7_local: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        """Validate and pre-compute local coordinates for ternary links."""
        # bar_a: local frame with O at origin, P1 on +x axis
        # P2 position from triangle O-P1-P2
        self._a2_local = triangle_solve_local(
            self.L_OP1, self.L_P1P2, self.L_OP2
        )

        # bar_d: local frame with P1 at origin, P4 on +x axis
        # P5 position from triangle P1-P4-P5
        self._d5_local = triangle_solve_local(
            self.L_P1P4, self.L_P4P5, self.L_P1P5
        )

        # bar_f: local frame with P2 at origin, P6 on +x axis
        # P7 position from triangle P2-P6-P7
        self._f7_local = triangle_solve_local(
            self.L_P2P6, self.L_P6P7, self.L_P2P7
        )

    @property
    def bar_a_angle_offset(self) -> float:
        """Angle from O->P1 to O->P2 in bar_a local frame (radians)."""
        return np.arctan2(self._a2_local[1], self._a2_local[0])

    def __repr__(self) -> str:
        lines = ["MechanismParams (mm):"]
        lines.append(f"  bar_a: O-P1={self.L_OP1}, O-P2={self.L_OP2}, "
                     f"P1-P2={self.L_P1P2}  (straight)")
        lines.append(f"  bar_b: O-P3={self.L_b}")
        lines.append(f"  bar_c: P3-P4={self.L_c}")
        lines.append(f"  bar_d: P1-P4={self.L_P1P4}, P1-P5={self.L_P1P5}, "
                     f"P4-P5={self.L_P4P5}  (straight)")
        lines.append(f"  bar_e: P5-P6={self.L_e}")
        lines.append(f"  bar_f: P2-P6={self.L_P2P6}, P2-P7={self.L_P2P7}, "
                     f"P6-P7={self.L_P6P7}  (straight)")
        lines.append(f"  branch_d={self.branch_d}, branch_f={self.branch_f}")
        return "\n".join(lines)


def default_params() -> MechanismParams:
    """Return the actual mechanism parameters (unit: mm).

    Based on the physical leg mechanism with two parallelograms in 8-shape.
    All ternary bars are straight (collinear).

    Parallelogram 1: O-P1-P4-P3  (48.4 × 57.3)
    Parallelogram 2: P1-P2-P6-P5 (59 × 32.4)
    """
    return MechanismParams(
        # bar_a (straight): O -- P1(AD) -- P2(AF)
        L_OP1=48.4,       # O to AD connection
        L_OP2=48.4 + 59,  # O to AF connection (straight: 107.4)
        L_P1P2=59,         # AD to AF

        # bar_b: O -> P3
        L_b=57.3,

        # bar_c: P3 -> P4
        L_c=48.4,

        # bar_d (straight): P4(CD) -- P1(AD) -- P5(D-end)
        L_P1P4=57.3,            # CD to AD
        L_P1P5=32.4,            # AD to D-end
        L_P4P5=57.3 + 32.4,     # CD to D-end (straight: 89.7)

        # bar_e: P5 -> P6
        L_e=59,

        # bar_f (straight): P6(EF) -- P2(AF) -- P7(wheel)
        L_P2P6=32.4,              # EF to AF
        L_P2P7=128,               # AF to wheel
        L_P6P7=32.4 + 128,        # EF to wheel (straight: 160.4)

        # assembly
        branch_d=+1,
        branch_f=-1,
    )
