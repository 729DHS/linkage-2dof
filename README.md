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

| Command | Description |
|---------|-------------|
| `python -m src.main` | Single configuration (convex, default branch) |
| `python -m src.main interactive` | Interactive sliders (drag θa, θb) |
| `python -m src.main branches` | All 4 assembly modes side-by-side |
| `python -m src.main anim` | Animation → `pic/animation.gif` |
| `python -m src.main workspace` | Workspace analysis |
| `python -m src.main trajectory` | Trajectory tracking |
| `python -m src.main ik` | Inverse kinematics demo |

**Default branch**: `branch_d=+1, branch_f=-1` (convex parallelogram).

Full manual: [docs/USAGE.md](docs/USAGE.md)

### Dependencies

Python ≥ 3.12, numpy, matplotlib, scipy, PyQt6

### Project Structure

```
src/
  main.py          # CLI/demo entry: python -m src.main
  zero_calib.py    # Zero calibration plot: python -m src.zero_calib
  geometry.py      # 2D vector math, circle intersection
  mechanism.py     # Bar length parameters
  kinematics.py    # FK / IK / all-branch solver
  visualization.py # Plots, animation, interactive sliders
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

| 命令 | 说明 |
|------|------|
| `python -m src.main` | 单帧演示 (凸四边形, 默认分支) |
| `python -m src.main interactive` | 交互滑块 (拖动 θa, θb) |
| `python -m src.main branches` | 并排显示全部 4 种装配模式 |
| `python -m src.main anim` | 动画 → `pic/animation.gif` |
| `python -m src.main workspace` | 工作空间分析 |
| `python -m src.main trajectory` | 轨迹跟踪 |
| `python -m src.main ik` | 逆运动学演示 |

**默认分支**: `branch_d=+1, branch_f=-1` (凸四边形).

完整手册: [docs/USAGE.md](docs/USAGE.md)

### 依赖

Python ≥ 3.12, numpy, matplotlib, scipy, PyQt6

### 项目结构

```
src/
  main.py          # 命令/演示入口: python -m src.main
  zero_calib.py    # 零点校准图入口: python -m src.zero_calib
  geometry.py      # 2D 向量运算, 两圆相交
  mechanism.py     # 杆长参数定义
  kinematics.py    # 正逆运动学 + 全分支求解
  visualization.py # 绘图, 动画, 交互滑块
pic/
  .gitkeep         # 生成图片/动画统一输出到此目录并被忽略
docs/
  USAGE.md                 # 中英双语使用手册
  mechanism.md             # 机构拓扑说明
  analytical_solution.md   # 正逆解推导
  devlog.md                # 开发日志
```
