# 使用手册 / User Manual

轮腿机器人 2-DOF 连杆机构仿真 — 使用说明
Wheel-legged robot 2-DOF linkage simulation — usage guide.

---

## 目录 / Table of Contents

1. [环境安装 / Installation](#1-环境安装--installation)
2. [快速开始 / Quick Start](#2-快速开始--quick-start)
3. [运行模式 / Run Modes](#3-运行模式--run-modes)
4. [交互模式详解 / Interactive Mode](#4-交互模式详解--interactive-mode)
5. [参数修改 / Parameter Tuning](#5-参数修改--parameter-tuning)
6. [API 使用 / API Reference](#6-api-使用--api-reference)

---

## 1. 环境安装 / Installation

```bash
# 克隆仓库 / Clone repo
git clone https://github.com/729DHS/linkage-2dof.git
cd linkage-2dof/v1

# 安装依赖 / Install dependencies
uv sync
```

依赖项 / Dependencies: `numpy`, `matplotlib`, `scipy`, `PyQt6`

---

## 2. 快速开始 / Quick Start

```bash
python -m src.main              # 单帧演示 / Single config demo
python -m src.main interactive  # 交互滑块 / Interactive sliders
python -m src.main branches     # 全部装配模式 / All 4 assembly modes
python -m src.main anim         # 动画 / Animation
python -m src.main workspace    # 工作空间 / Workspace
python -m src.main trajectory   # 轨迹跟踪 / Trajectory
python -m src.main ik           # 逆解演示 / Inverse kinematics
python -m src.zero_calib        # 零点校准图 / Zero calibration plot
```

---

## 3. 运行模式 / Run Modes

### `python -m src.main` — 单帧 / Single Config

显示机构在默认角度下的单帧静态图。

Shows mechanism at default angles (θa=0°, θb=90°, **凸四边形 / convex parallelogram**).

输出 / Output:
- P7 (轮毂电机/Wheel) 坐标
- θd, θf (中间转角/Intermediate angles)
- 机构参数列表 / Parameter list

---

### `python -m src.main interactive` — 交互滑块 / Interactive Sliders

![交互界面说明 / Interactive UI]

打开交互窗口，可拖动滑块实时探索机构运动。

Opens an interactive window with dragable sliders.

#### 界面说明 / UI Elements

| 元素 / Element | 位置 / Position | 功能 / Function |
|---|---|---|
| θa 滑块 / Slider | 左下 / Bottom-left | 拖动控制电机 A 转角 (-180° ~ 180°) / Drag to set motor A angle |
| θb 滑块 / Slider | 左下 / Bottom-left | 拖动控制电机 B 转角 (-180° ~ 180°) / Drag to set motor B angle |
| 分支选择 / Radio | 右下 / Bottom-right | 切换装配模式 (d=±1, f=±1) / Switch assembly branch |
| 机构图 / Plot | 上方 / Top area | 实时机构图形 / Real-time mechanism view |

#### 画面说明 / View Layout

- **电机轴 O 固定在画面中心** / Motor O fixed at view center
- **视角不随角度变化** / View bounds locked (x: -250~250, y: -300~100 mm)
- **整体旋转 -90°** (机构向下伸展，像腿一样) / Rotated -90° (leg points downward)
- **底部棕色线** = 地面参考 / Brown line = ground reference
- **标题栏** 显示: 角度、装配模式、平行四边形凹凸性、P7 坐标
- **Title bar** shows: angles, branch, parallelogram type, P7 position

#### 默认装配模式 / Default Assembly

默认选中 **凸四边形 (convex parallelogram)** 分支 (branch_d=+1, branch_f=-1)。

The default branch corresponds to the convex parallelogram O-P1-P4-P3.

---

### `python -m src.main branches` — 全部装配模式 / All Branches

并排显示 4 种装配模式 (最多 4 = 2×2 分支组合)。

Shows all valid assembly modes (up to 4) side by side.

每种模式标注:
- branch_d, branch_f 值
- 平行四边形凹凸性 (convex / crossed)
- P7 位置 / P7 position

---

### `python -m src.main anim` — 动画 / Animation

生成机构运动动画，保存为 `pic/animation.gif`。

Generates mechanism motion animation, saved as `pic/animation.gif`.

---

### `python -m src.main workspace` — 工作空间 / Workspace

采样电机角度空间，绘制末端可达区域。

Samples motor angle space, plots reachable workspace.

- 左图: 电机角度空间有效区域 / Left: valid regions in motor-angle space
- 右图: 末端 P7 可达位置 / Right: end-effector reachable positions
- 工作范围: r ∈ [20.6, 235.4] mm / Workspace radius

---

### `python -m src.main trajectory` — 轨迹跟踪 / Trajectory

沿一组正弦变化的电机角度轨迹求解，绘制末端路径。

Solves mechanism along sinusoidal motor angle trajectory, plots path.

在路径上叠加 4 帧机构图 (起点、1/3、2/3、终点)。

Overlays mechanism snapshots at 4 points along the trajectory.

---

### `python -m src.main ik` — 逆解演示 / Inverse Kinematics

给定 4 个目标 P7 位置，求解对应的电机转角。

Given 4 target P7 positions, finds motor angles.

每个目标显示 2 个解 (肘向上 / 肘向下)。

Each target shows 2 solutions (elbow-up / elbow-down).

---

### `python -m src.zero_calib` — 零点校准图 / Zero Calibration Plot

根据当前零点角度生成 `pic/zero_calib.png`，用于检查轮腿关节初始零点和车体坐标系关系。

Generates `pic/zero_calib.png` from the current zero-position angles to check the
initial joint zero offsets and the cart-frame relationship.

当前参数 / Current constants:
- θa = -72.4°
- θb = 80.0°
- φ = 13.9°
- 轮子半径 / wheel radius = 30 mm

右图约定 / Right-plot convention:
- 车体为水平长方形 / cart body is a horizontal rectangle
- P7 为轮心 / P7 is the wheel hub
- 轮子在车体下方并接触地面 / wheel is under the cart body and touches ground
- 绘图姿态使用 `φ + 180°`，匹配小车实际方向 / drawing pose uses `φ + 180°`

---

## 4. 交互模式详解 / Interactive Mode Details

### 默认选中哪个分支? / Which branch is default?

**默认选中凸四边形分支** (branch_d=+1, branch_f=-1)。

This is the "correct" physical configuration where both parallelograms
are convex (not crossed). The analytical forward kinematics formula
(2R arm model) corresponds to this branch.

### 为什么不是随便调的? / Why is this not arbitrary?

正运动学解析公式 / Analytical FK formula:

```
P7_x = 107.4·cos(θa) + 128·cos(θb)
P7_y = 107.4·sin(θa) + 128·sin(θb)
```

这个公式**只对凸四边形分支成立**。其他 3 个分支对应交叉四边形等
非物理装配模式，P7 位置不同。

This formula holds **only for the convex parallelogram branch**.
The other 3 branches correspond to crossed/non-physical assemblies.

### 默认参数值 / Default Parameters

| 参数 / Param | 值 / Value | 说明 / Description |
|---|---|---|
| L_OP1 | 48.4 mm | O 到 AD 连接点 / O to AD joint |
| L_OP2 | 107.4 mm | O 到 AF 连接点 (a杆总长) / O to AF joint |
| L_P1P2 | 59 mm | AD 到 AF / AD to AF |
| L_b | 57.3 mm | b杆 / bar b |
| L_c | 48.4 mm | c杆 / bar c |
| L_P1P4 | 57.3 mm | bar_d CD到AD / bar_d CD to AD |
| L_P1P5 | 32.4 mm | bar_d AD到D端 / bar_d AD to D-end |
| L_P4P5 | 89.7 mm | bar_d总长 / bar_d total |
| L_e | 59 mm | e杆 / bar e |
| L_P2P6 | 32.4 mm | bar_f EF到AF / bar_f EF to AF |
| L_P2P7 | 128 mm | bar_f AF到轮 / bar_f AF to wheel |
| L_P6P7 | 160.4 mm | bar_f总长 / bar_f total |
| branch_d | **+1** | **默认凸四边形 / convex default** |
| branch_f | **-1** | **默认凸四边形 / convex default** |

---

## 5. 参数修改 / Parameter Tuning

编辑 `src/mechanism.py` 中的 `default_params()` 函数:

Edit `default_params()` in `src/mechanism.py`:

```python
def default_params() -> MechanismParams:
    return MechanismParams(
        L_OP1=48.4,       # 修改为你需要的值 / Change to your values
        L_OP2=107.4,
        ...
    )
```

所有三元杆为直杆 (第三边 = 第一边 + 第二边)。
All ternary bars are straight (3rd side = 1st side + 2nd side).

---

## 6. API 使用 / API Reference

```python
from src.mechanism import MechanismParams, default_params
from src.kinematics import solve_linkage, solve_all_branches, solve_inverse, solve_trajectory

params = default_params()

# ---- 正运动学 / Forward Kinematics ----
import numpy as np
result = solve_linkage(np.deg2rad(0), np.deg2rad(90), params)
print(result['P7'])  # 末端位置 / End-effector position

# 解析公式 / Analytical formula:
L1, L2 = params.L_OP2, params.L_P2P7
P7 = np.array([L1*np.cos(θa) + L2*np.cos(θb),
               L1*np.sin(θa) + L2*np.sin(θb)])

# ---- 全部装配模式 / All Assembly Modes ----
all_modes = solve_all_branches(np.deg2rad(0), np.deg2rad(90), params)
for (bd, bf), res in all_modes.items():
    if res:
        print(f"branch_d={bd:+d}, branch_f={bf:+d}: P7={res['P7']}")

# ---- 逆运动学 / Inverse Kinematics ----
solutions = solve_inverse(np.array([150, 100]), params, elbow=0)  # 0=both
for sol in solutions:
    print(f"θa={np.rad2deg(sol['theta_a']):.1f}°, θb={np.rad2deg(sol['theta_b']):.1f}°")

# ---- 轨迹 / Trajectory ----
theta_a_seq = np.deg2rad(20) * np.sin(np.linspace(0, 2*np.pi, 100))
theta_b_seq = np.deg2rad(30) * np.sin(np.linspace(0, 2*np.pi, 100) + 1) + np.deg2rad(90)
results = solve_trajectory(theta_a_seq, theta_b_seq, params)
```
