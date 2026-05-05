# 开发环境备忘

## Python 环境

```bash
# venv 位置
cd /home/huiming/桌面/sim/linkage/v1
.venv/bin/python3  # 可执行文件

# 入口脚本统一作为 src 包模块运行:
cd /home/huiming/桌面/sim/linkage/v1
.venv/bin/python -m src.main
.venv/bin/python -m src.main workspace
.venv/bin/python -m src.zero_calib
```

## 目录结构和维护要求

- `src/`: Python 源码和可单独运行的入口模块。
- `src/main.py`: 常规仿真入口，使用 `python -m src.main [mode]` 运行。
- `src/zero_calib.py`: 零点校准图入口，使用 `python -m src.zero_calib` 运行。
- `docs/`: 使用说明、开发日志、机制说明、开发备忘。
- `pic/`: 所有脚本生成的图片和动画输出目录。
- 根目录只放项目级文件，如 `README.md`、`pyproject.toml`、`.gitignore`。
- 新增脚本时优先放入 `src/`，并确认是否应作为 `python -m src.xxx` 的入口模块。
- 新增输出文件时统一写到 `pic/`，不要散落到 `v1` 根目录。
- 修改运行命令、输出路径、目录结构时，同步维护 `README.md`、`docs/USAGE.md`、`docs/devlog.md` 和本文件。
- 提交前检查 `git status --short --branch`，避免遗漏未跟踪源码或文档。

## matplotlib 中文显示

```python
# 可用中文字体: AR PL UKai CN, AR PL UMing CN
# 如需中文标签，在脚本头部加:
import matplotlib
matplotlib.rcParams['font.family'] = ['AR PL UKai CN', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False
```

## 图片显示

- `plt.show()` 在无 GUI 环境会阻塞，生成图片用 `matplotlib.use('Agg')` 并去掉 `plt.show()`
- 查看图片: `eog /path/to/image.png &`

## 零点校准图

```bash
cd /home/huiming/桌面/sim/linkage/v1
.venv/bin/python -m src.zero_calib
```

- 输出文件: `/home/huiming/桌面/sim/linkage/v1/pic/zero_calib.png`
- 当前轮子半径: 30 mm
- 右图车体坐标: 车体水平，P7 为轮心，轮子在车体下方并与地面相切
- 右图姿态: 使用 `phi + 180°` 修正小车实际方向
- `*.png` 被 `.gitignore` 忽略，生成图片不进入版本库

### 本次问题复盘

原问题:
- 图 2 的机构姿态相对小车实际方向反转 180°。
- 右图的 P7 机构末端和绿色轮子标记不是同一个坐标点。
- 小车车体用一条横线表示，不符合“车体平行地面，用长方形代替”的需求。
- 轮子位置没有明确在车体下方，也没有按真实半径显示。
- 脚本位于 `v1/zero_calib.py`，根目录脚本和输出图片过多，结构不清晰。

解决方式:
- 右图坐标变换使用 `cart_angle = phi + pi`，修正整体 180° 方向错误。
- 以 `P7` 作为轮心，统一机构末端、轮子圆心和标注坐标。
- 车体使用 `Rectangle` 绘制水平长方形，轮子使用 `Circle` 绘制。
- 轮子半径设为 30 mm，轮心固定在 `Y=30`，地面为 `Y=0`。
- 脚本移动到 `src/zero_calib.py`，以 `python -m src.zero_calib` 运行。
- 生成图片输出到 `pic/zero_calib.png`，避免污染 `v1` 根目录。

## Unit4 固件对接备忘

### C 端接口现状

Unit4 固件侧已实现 `linkage_kinematics.h`:
- 正解: `lk_forward()`
- 逆解: `lk_inverse()`
- 接口与 Python 等效 2R 模型一致

固化参数:

```c
#define LK_L1_MM 107.4f  // O -> P2
#define LK_L2_MM 128.0f  // P2 -> P7
```

### 4 电机顺序

| 索引 | 变量 | CAN | 角色 |
|---:|---|---|---|
| 0 | LEFT_THETA_A | CAN1 | 左腿 theta_a |
| 1 | LEFT_THETA_B | CAN1 | 左腿 theta_b |
| 2 | RIGHT_THETA_A | CAN2 | 右腿 theta_a |
| 3 | RIGHT_THETA_B | CAN2 | 右腿 theta_b |

每台 DM4310 直接对应等效 2R 模型的一个关节角。

### CAD 零位角

当前仿真只有单腿局部基座坐标系；左右腿没有在仿真里单独建两个 CAD 坐标系。

单腿局部模型中，硬件推到限位时的 CAD 绝对角为:

```c
theta_a_zero = -1.26364f;  // -72.4 deg
theta_b_zero =  1.39626f;  //  80.0 deg
```

如果 Unit4 左右腿都按“各自独立、同向定义的局部基座坐标系”调用 FK/IK，则 4 电机零位偏移可用:

```c
static const float cad_angle_at_zero[4] = {
    -1.26364f,  // LEFT_THETA_A
     1.39626f,  // LEFT_THETA_B
    -1.26364f,  // RIGHT_THETA_A
     1.39626f,  // RIGHT_THETA_B
};
```

这组值的前提:
- 左右腿 CAD 零位角使用同一套局部腿坐标定义。
- 左右镜像不通过改 CAD 零位角实现。
- Unit4 中 `MIRROR_SIGN = -1.0` 和右腿 FK 后 `right_pose.x_mm = -right_pose.x_mm` 负责车体层面的镜像。

如果 CAD 角度是在共用车体坐标系下读取，而不是在左右腿各自局部基座下读取，则右腿零位可能不能直接复用左腿值，需要按 CAD 镜像关系重新换算。

### 车体坐标变换

当前 Python 零点图只确认了单腿基座局部坐标到车体显示方向的旋转:

```text
phi = 13.9 deg
cart drawing angle = phi + 180 deg = 193.9 deg
```

零位姿态下，单腿局部 P7 转到车体显示方向后:

```text
P7_cart_rel_O = (-47.4, -36.1) mm
```

轮半径 30 mm，地面为 `Y=0` 时:

```text
P7_wheel_hub = (-47, 30) mm
O_motor      = (0, 66) mm
```

这些值只说明“单腿图里的 O 和 P7 相对关系”，不能替代整车坐标系安装位姿。

Unit4 若要输出末端车体坐标，还缺:
- 车体参考原点定义。
- 左腿基座 `O_left_body = (x_mm, y_mm)`。
- 右腿基座 `O_right_body = (x_mm, y_mm)`。
- 车体坐标轴定义: +X 向前/向右，+Y 向上/向前，需要与 C 端保持一致。

### 车体坐标接口定义

需要从 Unit4 侧明确的三项参数如下:

| 参数 | 含义 | 例子 |
|---|---|---|
| 车体参考原点 | 整车坐标系原点定义在哪 | 两电机重合轴，`(0, 55)` |
| `O_left_body` | 左腿机构基座 O 点在车体坐标系下的安装位置 (x, y), mm | `(-80, 55)` |
| `O_right_body` | 右腿机构基座 O 点在车体坐标系下的安装位置 (x, y), mm | `(+80, 55)` |

已确认的车身外形:

- 侧视图车身尺寸: `220 x 100 mm`
- 车宽: `150 mm`
- 车身侧面高度: `100 mm`
- 原点: 两电机重合轴，位于侧面居中高度 `55 mm`

这意味着车体坐标系下的基准不是车身几何中心，而是电机同轴中心点。

建议输出方式:

```c
P7_body_left  = O_left_body  + P7_base_left;
P7_body_right = O_right_body + P7_base_right;  // right base already mirrored in x
```

每腿基座坐标输出照旧打印，再补一行车体坐标系输出，方便调试和联调。

建议 C 端车体变换保持显式:

```c
left_body.x_mm = O_left_body.x_mm + left_local.x_mm;
left_body.y_mm = O_left_body.y_mm + left_local.y_mm;

right_body.x_mm = O_right_body.x_mm - right_local.x_mm;
right_body.y_mm = O_right_body.y_mm + right_local.y_mm;
```

这里的右腿 `x` 取负对应 Unit4 当前镜像约定；如果之后把旋转/平移统一成 2D rigid transform，则不要再重复做一次 `x` 镜像。

## 脚本模板

```python
import matplotlib
matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
# ...
pic_dir = Path(__file__).resolve().parents[1] / "pic"
pic_dir.mkdir(exist_ok=True)
plt.savefig(pic_dir / "output.png", dpi=150, bbox_inches='tight')
```
