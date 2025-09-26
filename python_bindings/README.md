# ORB-SLAM3 Python Bindings

这个目录包含了ORB-SLAM3的Python接口，使用ctypes实现。

## 文件结构

```
python_bindings/
├── orb_slam3_wrapper.h          # C++ 包装器头文件
├── orb_slam3_wrapper.cpp        # C++ 包装器实现
├── orb_slam3_python.py          # Python ctypes 接口
├── CMakeLists.txt               # CMake 构建文件
├── build.sh                     # 构建脚本
├── example_rgbd.py              # RGB-D SLAM 示例
├── example_stereo.py            # 立体视觉 SLAM 示例
└── README.md                    # 这个文件
```

## 构建步骤

### 1. 确保ORB-SLAM3已构建

首先确保主ORB-SLAM3项目已经成功构建：

```bash
cd /Users/huyiyang/Workspace/ORB_SLAM3
./build.sh
```

### 2. 构建Python绑定

```bash
cd python_bindings
./build.sh
```

构建脚本会：
- 检查ORB-SLAM3库是否存在
- 使用CMake配置项目
- 编译C++包装器
- 生成共享库 `orb_slam3_python.so`

### 3. 安装Python依赖

```bash
pip install numpy opencv-python
```

## 使用方法

### RGB-D SLAM 示例

```bash
python3 example_rgbd.py --vocab /path/to/ORBvoc.txt --settings /path/to/RealSense_D435i.yaml
```

### 立体视觉 SLAM 示例

```bash
python3 example_stereo.py --vocab /path/to/ORBvoc.txt --settings /path/to/EuRoC.yaml
```

### 基本Python代码示例

```python
import numpy as np
import cv2
from orb_slam3_python import ORBSLAMSystem, SensorType

# 创建ORB-SLAM3系统
slam = ORBSLAMSystem(
    vocab_file="Vocabulary/ORBvoc.txt",
    settings_file="Examples/RGB-D/RealSense_D435i.yaml",
    sensor_type=SensorType.RGBD,
    use_viewer=True
)

# 读取RGB和深度图像
rgb_image = cv2.imread("rgb_image.png")
depth_image = cv2.imread("depth_image.png", cv2.IMREAD_ANYDEPTH).astype(np.float32)

# 转换颜色格式
rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

# 跟踪帧
timestamp = time.time()
pose = slam.track_rgbd(rgb_image, depth_image, timestamp)

if pose is not None:
    print("Camera pose:")
    print(pose)
    
    # 获取地图点
    map_points = slam.get_tracked_map_points()
    print(f"Tracked {len(map_points)} map points")
    
    # 获取关键点
    keypoints = slam.get_tracked_keypoints()
    print(f"Tracked {len(keypoints)} keypoints")

# 关闭系统
slam.shutdown()
```

## API 接口

### ORBSLAMSystem 类

主要的Python接口类，提供以下方法：

#### 构造函数
- `__init__(vocab_file, settings_file, sensor_type, use_viewer=True)`

#### 跟踪方法
- `track_rgbd(rgb_image, depth_image, timestamp)` - RGB-D跟踪
- `track_stereo(left_image, right_image, timestamp)` - 立体视觉跟踪
- `track_monocular(image, timestamp)` - 单目跟踪

#### 状态查询
- `get_tracking_state()` - 获取跟踪状态
- `is_lost()` - 检查是否跟踪丢失
- `map_changed()` - 检查地图是否变化

#### 数据获取
- `get_tracked_map_points()` - 获取跟踪到的地图点
- `get_tracked_keypoints()` - 获取跟踪到的关键点

#### 控制方法
- `activate_localization_mode()` - 激活定位模式（停止建图）
- `deactivate_localization_mode()` - 取消定位模式（恢复建图）
- `reset()` - 重置系统
- `shutdown()` - 关闭系统

#### 保存方法
- `save_trajectory_tum(filename)` - 保存TUM格式轨迹
- `save_trajectory_kitti(filename)` - 保存KITTI格式轨迹

### 数据类型

#### SensorType 枚举
- `MONOCULAR` - 单目相机
- `STEREO` - 立体相机
- `RGBD` - RGB-D相机
- `IMU_MONOCULAR` - 单目+IMU
- `IMU_STEREO` - 立体+IMU
- `IMU_RGBD` - RGB-D+IMU

#### TrackingState 枚举
- `SYSTEM_NOT_READY` - 系统未就绪
- `NO_IMAGES_YET` - 尚未接收图像
- `NOT_INITIALIZED` - 未初始化
- `OK` - 正常跟踪
- `LOST` - 跟踪丢失

## 注意事项

1. **图像格式**：
   - RGB图像：uint8类型，shape为(H, W, 3)
   - 深度图像：float32类型，shape为(H, W)
   - 立体图像：uint8类型，shape为(H, W)

2. **坐标系**：
   - 返回的位姿矩阵为4x4变换矩阵
   - 采用右手坐标系
   - 位姿表示从世界坐标系到相机坐标系的变换

3. **内存管理**：
   - 地图点和关键点数据会自动释放
   - 系统析构时会自动清理资源

4. **线程安全**：
   - 同一个ORBSLAMSystem实例不应在多线程中同时调用

## 故障排除

1. **构建错误**：
   - 确保ORB-SLAM3主项目已正确构建
   - 检查所有依赖库是否正确安装
   - 查看CMake配置是否正确

2. **运行时错误**：
   - 检查词汇文件和设置文件路径是否正确
   - 确保图像格式和尺寸符合要求
   - 检查相机标定参数是否正确

3. **性能问题**：
   - 可以关闭viewer来提高性能
   - 调整图像分辨率
   - 优化编译选项
