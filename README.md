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
# Install dependencies
uv sync

# Run
python main.py              # Single configuration (convex parallelogram)
python main.py interactive  # Interactive sliders (drag to explore)
python main.py branches     # All 4 assembly modes side-by-side
python main.py anim         # Animation
python main.py workspace    # Workspace analysis
python main.py trajectory   # Trajectory tracking
```

## Dependencies

- Python >= 3.12
- numpy, matplotlib, scipy
- PyQt6 (for interactive GUI)

## Project Structure

```
src/
  geometry.py      # 2D vector math, circle intersection, triangle solver
  mechanism.py     # Mechanism parameters (lengths, triangle geometry)
  kinematics.py    # Forward kinematics (solve_linkage, solve_all_branches)
  visualization.py # Plots, animation, interactive sliders
docs/
  mechanism.md     # Theory & derivation
  devlog.md        # Development log
```

## License

MIT
