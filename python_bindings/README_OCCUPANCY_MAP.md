# ORB-SLAM3 占用地图生成功能

## 项目概述

本项目为ORB-SLAM3添加了从轨迹数据生成2D占用地图（房间平面图）的功能。占用地图显示了机器人可以安全行走的区域，是导航规划和环境分析的重要工具。

## 🚀 快速开始

### 方法1：交互式界面（推荐新手）
```bash
python3 quick_start_occupancy.py
```

### 方法2：命令行直接使用
```bash
# 基本用法
python3 visualize_trajectory.py --tum your_trajectory.txt --occupancy

# 高质量地图生成
python3 visualize_trajectory.py --tum your_trajectory.txt --occupancy \
    --resolution 0.02 --robot-radius 0.3 --save-map my_room
```

### 方法3：使用示例数据测试
```bash
# 生成示例数据
python3 test_occupancy_map.py

# 使用示例数据测试
python3 visualize_trajectory.py --tum sample_trajectory.txt --occupancy
```

## 📁 项目文件

### 核心文件
- **`visualize_trajectory.py`** - 主要功能，轨迹可视化和占用地图生成
- **`quick_start_occupancy.py`** - 交互式快速开始工具
- **`test_occupancy_map.py`** - 测试脚本，生成示例数据

### 文档文件
- **`OCCUPANCY_MAP_USAGE.md`** - 详细使用指南
- **`README_OCCUPANCY_MAP.md`** - 本文件，项目总览

## 🎯 主要功能

### 1. 多格式轨迹支持
- **TUM格式**：`timestamp tx ty tz qx qy qz qw`
- **KITTI格式**：3x4变换矩阵的12个元素
- **NumPy格式**：`.npz`文件包含poses, timestamps, frame_ids

### 2. 占用地图生成
- 从3D轨迹数据提取2D平面信息
- 自动检测可行走区域（白色）
- 推断障碍物位置（黑色）
- 标记未探索区域（灰色）

### 3. 可视化功能
- **轨迹地图**：俯视图、侧视图、3D视图、高度编码图
- **占用地图**：2D房间平面图，显示可行走区域
- **统计信息**：面积计算、覆盖率分析

### 4. 多种输出格式
- **PGM格式**：ROS标准占用地图格式
- **YAML文件**：地图元数据（分辨率、原点等）
- **NumPy格式**：便于Python程序处理

## 🛠️ 安装要求

### 系统要求
- Python 3.6+
- macOS/Linux/Windows

### 依赖库
```bash
pip install numpy matplotlib opencv-python scipy scikit-image
```

### 验证安装
```bash
python3 quick_start_occupancy.py  # 自动检查依赖库
```

## 📊 使用示例

### 示例1：基本占用地图生成
```bash
python3 visualize_trajectory.py --tum trajectory.txt --occupancy
```
- 使用默认参数
- 分辨率：0.05米/像素
- 机器人半径：0.3米

### 示例2：高精度地图
```bash
python3 visualize_trajectory.py --tum trajectory.txt --occupancy \
    --resolution 0.02 \
    --robot-radius 0.25 \
    --map-extension 1.0 \
    --save-map high_res_map
```

### 示例3：大范围环境
```bash
python3 visualize_trajectory.py --kitti poses.txt --occupancy \
    --resolution 0.1 \
    --robot-radius 0.5 \
    --map-extension 5.0
```

## 📈 输出结果

### 控制台输出
```
轨迹统计:
  位姿数量: 360
  总距离: 102.93 米
  起点: (0.00, 0.50, 0.00)
  终点: (5.00, 4.00, 0.00)

生成2D占用地图...
  分辨率: 0.05 米/像素
  机器人半径: 0.3 米
  地图尺寸: 241 x 181 像素
  可行走区域: 36.93 平方米 (33.9%)
  障碍物区域: 20.10 平方米 (18.4%)
  未知区域: 52.03 平方米 (47.7%)
```

### 生成文件
```
occupancy_map_20250926_235327.pgm   # PGM格式地图
occupancy_map_20250926_235327.yaml  # ROS元数据
occupancy_map_20250926_235327.npz   # NumPy格式
```

### 可视化界面
- 轨迹可视化窗口（4个子图）
- 3D轨迹显示窗口
- 2D占用地图窗口

## 🎨 地图颜色编码

| 颜色 | 含义 | 数值 |
|------|------|------|
| 白色 | 可行走区域 | 1.0 |
| 黑色 | 障碍物/墙壁 | 0.0 |
| 灰色 | 未探索区域 | 0.5 |
| 红线 | 机器人轨迹 | - |

## ⚙️ 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--resolution` | 0.05 | 地图分辨率（米/像素）。值越小地图越精细 |
| `--robot-radius` | 0.3 | 机器人半径（米）。影响可行走区域大小 |
| `--map-extension` | 2.0 | 地图边界扩展距离（米） |
| `--save-map` | 自动生成 | 保存地图的文件名前缀 |

## 🔧 高级用法

### 批量处理
```bash
# 处理多个轨迹文件
for file in trajectory_*.txt; do
    python3 visualize_trajectory.py --tum "$file" --occupancy \
        --save-map "${file%.txt}_map"
done
```

### 与ROS集成
生成的PGM和YAML文件可直接用于ROS导航：
```bash
# 在ROS中加载地图
rosrun map_server map_server occupancy_map.yaml
```

### Python API使用
```python
from visualize_trajectory import generate_occupancy_map, load_tum_trajectory

# 加载轨迹
data = load_tum_trajectory("trajectory.txt")
positions = data[:, 1:4]

# 生成占用地图
occupancy_map, map_info = generate_occupancy_map(
    positions, 
    resolution=0.02,
    robot_radius=0.3
)
```

## 🧪 测试和验证

### 运行测试套件
```bash
python3 test_occupancy_map.py
```

### 验证输出
1. 检查生成的PGM文件是否为有效图像
2. 验证YAML文件格式是否正确
3. 确认可行走区域与轨迹一致

## 📚 相关资源

- [OCCUPANCY_MAP_USAGE.md](OCCUPANCY_MAP_USAGE.md) - 详细使用指南
- [ORB-SLAM3官方文档](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [ROS地图格式说明](http://wiki.ros.org/map_server)

## 🐛 故障排除

### 常见问题

1. **ImportError: No module named 'xxx'**
   ```bash
   pip install numpy matplotlib opencv-python scipy scikit-image
   ```

2. **地图生成失败**
   - 检查轨迹文件格式是否正确
   - 确认轨迹数据不为空
   - 尝试调整分辨率参数

3. **内存不足**
   - 增加分辨率值（如0.1）
   - 减少地图扩展范围
   - 分段处理长轨迹

4. **可视化不显示**
   - 确认matplotlib后端设置
   - 在远程服务器上使用`--save-map`参数

### 性能优化

- **分辨率选择**：
  - 室内小环境：0.01-0.02m
  - 办公室环境：0.05m
  - 大型环境：0.1m以上

- **内存使用**：地图大小 ∝ (范围/分辨率)²

## 🤝 贡献

欢迎提交问题报告和功能请求！

## 📄 许可证

遵循ORB-SLAM3的GPLv3许可证。

---

**快速开始命令总结：**
```bash
# 最简单的使用方式
python3 quick_start_occupancy.py

# 直接命令行使用
python3 visualize_trajectory.py --tum your_file.txt --occupancy

# 生成测试数据
python3 test_occupancy_map.py
```
