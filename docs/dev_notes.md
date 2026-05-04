# 开发环境备忘

## Python 环境

```bash
# venv 位置
cd /home/huiming/桌面/sim/linkage/v1
.venv/bin/python3  # 可执行文件

# 运行脚本务必使用绝对路径，否则 cwd 不同会导致:
#   1. 相对路径的 savefig 写到错误位置
#   2. ModuleNotFoundError: No module named 'src'
python3 /home/huiming/桌面/sim/linkage/v1/xxx.py      # 正确
cd v1 && .venv/bin/python3 xxx.py                      # 也行，但要先 cd
```

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
cd /home/huiming/桌面/sim/linkage
v1/.venv/bin/python v1/zero_calib.py
```

- 输出文件: `/home/huiming/桌面/sim/linkage/v1/zero_calib.png`
- 当前轮子半径: 30 mm
- 右图车体坐标: 车体水平，P7 为轮心，轮子在车体下方并与地面相切
- 右图姿态: 使用 `phi + 180°` 修正小车实际方向
- `*.png` 被 `.gitignore` 忽略，生成图片不进入版本库

## 脚本模板

```python
import matplotlib
matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt
# ...
plt.savefig('/absolute/path/to/output.png', dpi=150, bbox_inches='tight')
```
