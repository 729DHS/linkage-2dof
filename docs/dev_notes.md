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
