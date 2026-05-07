# 2-DOF Wheel-Legged Robot Linkage Simulation<br>轮腿机器人二自由度连杆机构仿真

[English](#english) | [中文](#中文)

---

<a name="english"></a>
## English

### Overview

Forward and inverse kinematics simulation of a 2-DOF planar linkage
mechanism for a wheel-legged robot. Two coaxial motor shafts drive an
8-shaped double-parallelogram linkage. The mechanism simplifies to an
equivalent 2R planar arm with closed-form analytical solutions.

### Mechanism

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

### Analytical Solution

**Forward kinematics** (convex branch):

```
P7_x = 107.4·cos(θa) + 128·cos(θb)
P7_y = 107.4·sin(θa) + 128·sin(θb)
```

**Inverse kinematics** (given P7 = (x, y), let r = √(x²+y²)):

```
cos(α) = (r² + 107.4² - 128²) / (2·107.4·r)
θa = atan2(y, x) ± acos(cos(α))
θb = atan2(y - 107.4·sin(θa), x - 107.4·cos(θa))
```

Workspace: r ∈ [20.6, 235.4] mm. Up to 2 inverse solutions (elbow up/down).

### Quick Start

```bash
git clone https://github.com/729DHS/linkage-2dof.git
cd linkage-2dof/v1
uv sync
```

Run commands from the `v1` directory. Debian may not provide a `python`
command, so examples use `.venv/bin/python`.

If you are at `/home/huiming/桌面/sim/linkage`, run:

```bash
cd v1
.venv/bin/python -m src.main ik_interactive
```

| Command | Description |
|---------|-------------|
| `.venv/bin/python -m src.main` | Single configuration (convex, default branch) |
| `.venv/bin/python -m src.main interactive` | Interactive sliders (drag θa, θb) |
| `.venv/bin/python -m src.main branches` | All 4 assembly modes side-by-side |
| `.venv/bin/python -m src.main anim` | Animation → `pic/animation.gif` |
| `.venv/bin/python -m src.main workspace` | Workspace analysis |
| `.venv/bin/python -m src.main trajectory` | Trajectory tracking |
| `.venv/bin/python -m src.main ik_interactive` | Drag P7 target and solve IK |
| `.venv/bin/python -m src.main ik` | Inverse kinematics demo |
| `.venv/bin/python -m src.twin_calib` | Digital twin: real-time display (zero_calib style) |
| `.venv/bin/python tools/serial_bridge.py --port /dev/ttyACM0` | Serial CSV → UDP bridge |

**Default branch**: `branch_d=+1, branch_f=-1` (convex parallelogram).

Full manual: [docs/USAGE.md](docs/USAGE.md)

### Dependencies

Python ≥ 3.12, numpy, matplotlib, scipy, PyQt6

### Project Structure

```
src/
  main.py          # CLI/demo entry: .venv/bin/python -m src.main
  twin_calib.py  # Digital twin: real-time display (zero_calib style)
  zero_calib.py    # Zero calibration plot: .venv/bin/python -m src.zero_calib
  geometry.py      # 2D vector math, circle intersection
  mechanism.py     # Bar length parameters
  kinematics.py    # FK / IK / all-branch solver
  visualization.py # Plots, animation, interactive sliders
tools/
  serial_bridge.py # Serial CSV → UDP bridge for digital twin
pic/
  .gitkeep         # Generated images/animations are written here and ignored
docs/
  USAGE.md                 # Bilingual manual
  mechanism.md             # Mechanism topology
  analytical_solution.md   # FK/IK derivation
  devlog.md                # Development log
```

---

<a name="中文"></a>
## 中文

### 概述

轮腿机器人腿部连杆机构的 2-DOF 正逆运动学仿真。两个同轴电机驱动
8 字形双平行四边形机构。机构等价于 2R 平面机械臂，具有闭式解析解。

### 机构拓扑

- **平行四边形 1**: O-P1-P4-P3 (48.4 × 57.3 mm)
- **平行四边形 2**: P1-P2-P6-P5 (59 × 32.4 mm)

### 解析解

**正运动学** (凸四边形分支):

```
P7_x = 107.4·cos(θa) + 128·cos(θb)
P7_y = 107.4·sin(θa) + 128·sin(θb)
```

**逆运动学** (给定 P7 = (x, y), 令 r = √(x²+y²)):

```
cos(α) = (r² + 107.4² - 128²) / (2·107.4·r)
θa = atan2(y, x) ± acos(cos(α))
θb = atan2(y - 107.4·sin(θa), x - 107.4·cos(θa))
```

工作空间: r ∈ [20.6, 235.4] mm。最多 2 个逆解 (肘向上/肘向下)。

### 快速开始

```bash
git clone https://github.com/729DHS/linkage-2dof.git
cd linkage-2dof/v1
uv sync
```

下面命令需要在 `v1` 目录执行。Debian 默认可能没有 `python` 命令，所以统一使用 `.venv/bin/python`。

如果你当前在 `/home/huiming/桌面/sim/linkage`，直接运行:

```bash
cd v1
.venv/bin/python -m src.main ik_interactive
```

| 命令 | 说明 |
|------|------|
| `.venv/bin/python -m src.main` | 单帧演示 (凸四边形, 默认分支) |
| `.venv/bin/python -m src.main interactive` | 交互滑块 (拖动 θa, θb) |
| `.venv/bin/python -m src.main branches` | 并排显示全部 4 种装配模式 |
| `.venv/bin/python -m src.main anim` | 动画 → `pic/animation.gif` |
| `.venv/bin/python -m src.main workspace` | 工作空间分析 |
| `.venv/bin/python -m src.main trajectory` | 轨迹跟踪 |
| `.venv/bin/python -m src.main ik_interactive` | 拖动 P7 目标点并实时逆解 |
| `.venv/bin/python -m src.main ik` | 逆运动学演示 |
| `.venv/bin/python -m src.twin_calib` | 数字孪生: 实时显示 (zero_calib 风格) |
| `.venv/bin/python tools/serial_bridge.py --port /dev/ttyACM0` | 串口 CSV → UDP 桥接 |

**默认分支**: `branch_d=+1, branch_f=-1` (凸四边形).

完整手册: [docs/USAGE.md](docs/USAGE.md)

### 依赖

Python ≥ 3.12, numpy, matplotlib, scipy, PyQt6

### 项目结构

```
src/
  main.py          # 命令/演示入口: .venv/bin/python -m src.main
  twin_calib.py    # 数字孪生: 实时显示 (zero_calib 风格)
  zero_calib.py    # 零点校准图入口: .venv/bin/python -m src.zero_calib
  geometry.py      # 2D 向量运算, 两圆相交
  mechanism.py     # 杆长参数定义
  kinematics.py    # 正逆运动学 + 全分支求解
  visualization.py # 绘图, 动画, 交互滑块
tools/
  serial_bridge.py # 串口 CSV → UDP 桥接 (数字孪生数据源)
pic/
  .gitkeep         # 生成图片/动画统一输出到此目录并被忽略
docs/
  USAGE.md                 # 中英双语使用手册
  mechanism.md             # 机构拓扑说明
  analytical_solution.md   # 正逆解推导
  devlog.md                # 开发日志
```
