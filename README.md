# 2-DOF Wheel-Legged Robot Linkage Simulation

轮腿机器人腿部连杆机构运动学仿真。

Two coaxial motor shafts drive an 8-shaped double-parallelogram linkage.
Closed-form analytical forward kinematics with interactive visualization.

## Mechanism

```
Motor A, B (coaxial at O)
       O
      / \
  bar_a   bar_b
  (straight) (binary)
  /    \      |
 P1    P2     P3
(AD)  (AF)     |
 |      |     bar_c
 |      |      |
bar_d  bar_f  P4
(straight) (straight)
 |  \    /  \
 P4 P5  P6  P7 (wheel)
```

- **Parallelogram 1**: O-P1-P4-P3 (48.4 × 57.3 mm)
- **Parallelogram 2**: P1-P2-P6-P5 (59 × 32.4 mm)

## Solution Method

Sequential circle-intersection (analytical). Each step is a closed-form
expression involving only trig functions and square roots.

## Quick Start

```bash
uv sync                      # Install dependencies
python main.py               # Single config (convex parallelogram, default branch)
python main.py interactive  # Interactive sliders (drag to explore angles)
python main.py branches     # All 4 assembly modes side-by-side
python main.py anim         # Animation
python main.py workspace    # Workspace analysis
python main.py trajectory   # Trajectory tracking
python main.py ik           # Inverse kinematics demo
```

**Default branch**: `branch_d=+1, branch_f=-1` (convex parallelogram).
This is the physically correct assembly, corresponding to the
analytical formula: P7 = L1·[cos θa, sin θa] + L2·[cos θb, sin θb].

详见 / See [docs/USAGE.md](docs/USAGE.md) for full bilingual manual.

## Dependencies

- Python >= 3.12
- numpy, matplotlib, scipy, PyQt6

## Project Structure

```
src/
  geometry.py      # 2D vector math, circle intersection, triangle solver
  mechanism.py     # Mechanism parameters (bar lengths, triangle geometry)
  kinematics.py    # Forward/inverse kinematics + all-branch solver
  visualization.py # Plots, animation, interactive sliders
docs/
  USAGE.md                 # Bilingual user manual (中/EN)
  mechanism.md             # Mechanism topology & kinematics theory
  analytical_solution.md   # Analytical FK/IK derivation
  devlog.md                # Development log
```

## License

MIT
