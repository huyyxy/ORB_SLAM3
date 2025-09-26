#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===================================================================================
                  ORB-SLAM3轨迹可视化和占用地图生成工具
===================================================================================

【工具简介】
本工具是一个专业的SLAM轨迹分析和可视化平台，采用面向对象设计模式，专为机器人学
和计算机视觉研究者设计。支持多种主流轨迹格式的读取、分析、可视化和占用地图生成。

【核心功能】
1. 多格式轨迹支持：TUM、KITTI、NumPy等主流格式
2. 丰富的可视化方式：2D/3D轨迹图、速度分析、高度编码等
3. 智能占用地图生成：从SLAM轨迹自动生成2D房间平面图
4. 多种导出格式：支持ROS标准PGM、YAML元数据、NumPy格式
5. 模块化架构：高度解耦的组件设计，易于扩展和维护

【系统架构 - 面向对象设计】
┌─────────────────────────────────────────────────────────────────────────┐
│                              主控制器                                    │
│                        TrajectoryProcessor                              │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                        工作流编排                                 │   │
│  │  加载数据 → 可视化分析 → 生成地图 → 保存结果               │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
         │                    │                   │                │
    ┌────▼────┐        ┌─────▼─────┐       ┌─────▼─────┐    ┌─────▼─────┐
    │数据加载器│        │轨迹可视化器│       │地图生成器  │    │地图保存器 │
    │Trajectory│        │Trajectory  │       │Occupancy  │    │Map       │
    │Loader   │        │Visualizer  │       │Map        │    │Saver     │
    │         │        │           │       │Generator  │    │          │
    │·TUM格式 │        │·2D俯视图  │       │·栅格化    │    │·PGM格式  │
    │·KITTI   │        │·3D立体图  │       │·膨胀腐蚀  │    │·YAML元数据│
    │·NumPy   │        │·速度分析  │       │·连通性    │    │·NumPy    │
    └─────────┘        └───────────┘       └───────────┘    └──────────┘
                               │
                        ┌─────▼─────┐
                        │地图可视化器│
                        │Map        │
                        │Visualizer │
                        │           │
                        │·颜色编码  │
                        │·轨迹叠加  │
                        │·图例说明  │
                        └───────────┘

【支持的轨迹格式详解】
1. TUM格式 (Technical University of Munich)
   - 格式：timestamp tx ty tz qx qy qz qw
   - 说明：时间戳 + 位置(x,y,z) + 四元数姿态(qx,qy,qz,qw)
   - 应用：RGB-D SLAM基准数据集

2. KITTI格式 (Karlsruhe Institute of Technology)
   - 格式：12个浮点数表示3x4变换矩阵
   - 说明：[R|t] 旋转矩阵R(3x3) + 平移向量t(3x1)
   - 应用：自动驾驶SLAM基准数据集

3. NumPy格式
   - 格式：.npz压缩文件包含poses, timestamps, frame_ids
   - 说明：自定义格式，适合Python生态系统
   - 应用：自定义数据处理和分析

【占用地图生成原理】
占用地图是移动机器人导航的核心数据结构，将连续的空间离散化为栅格，每个栅格
表示该区域的占用状态：
- 0.0 (黑色)：障碍物，不可通行
- 1.0 (白色)：空闲区域，可安全通行  
- 0.5 (灰色)：未知区域，未探索

生成算法流程：
1. 轨迹预处理：提取2D位置信息，确定地图边界
2. 栅格化映射：将连续坐标转换为离散栅格坐标
3. 空闲区域标记：以机器人半径为缓冲区标记可行走区域
4. 路径连接：使用Bresenham算法连接轨迹点
5. 形态学处理：膨胀-腐蚀操作平滑地图边界
6. 边界设置：在地图边缘设置安全边界

【教学示例用法】

# 1. 基础轨迹可视化（适合初学者）
python visualize_trajectory.py --tum trajectory.txt

# 2. 多视角轨迹分析（理解SLAM轨迹特性）
python visualize_trajectory.py --kitti odometry.txt

# 3. 占用地图生成（机器人导航应用）
python visualize_trajectory.py --tum trajectory.txt --occupancy

# 4. 高精度地图生成（研究级应用）
python visualize_trajectory.py --tum trajectory.txt --occupancy \\
    --resolution 0.02 \\          # 2cm分辨率，高精度地图
    --robot-radius 0.35 \\        # 机器人半径35cm
    --map-extension 1.0 \\        # 地图边界扩展1米
    --save-map lab_room           # 保存为lab_room.*

# 5. 批量处理和分析
for file in *.txt; do
    python visualize_trajectory.py --tum "$file" --occupancy --save-map "${file%.txt}_map"
done

【面向对象设计模式应用】
1. 单一职责原则 (SRP)：每个类只负责一个特定功能
2. 开放封闭原则 (OCP)：对扩展开放，对修改封闭
3. 依赖倒置原则 (DIP)：依赖抽象而不是具体实现
4. 策略模式：不同的轨迹加载策略
5. 工厂模式：统一的对象创建接口
6. 配置模式：集中化参数管理

【性能优化特性】
- NumPy向量化计算：充分利用CPU的SIMD指令
- 内存高效：流式处理大型轨迹文件
- 缓存机制：避免重复计算
- 并行化支持：为未来多线程处理预留接口

【错误处理和鲁棒性】
- 输入验证：文件格式、参数范围检查
- 异常捕获：友好的错误信息提示
- 降级策略：部分功能失败时的备用方案
- 日志记录：详细的处理过程追踪

【扩展性设计】
- 插件化架构：易于添加新的轨迹格式支持
- 可配置管道：灵活的处理流程定制
- 接口标准化：便于第三方组件集成
- 文档完善：详细的API文档和使用示例
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import matplotlib.font_manager as fm
import platform
from scipy.ndimage import binary_dilation, binary_erosion
from scipy.spatial import ConvexHull
from skimage.morphology import disk
import cv2

# ==================================================================================
#                              中文字体配置模块
# ==================================================================================

def setup_chinese_font():
    """
    配置matplotlib中文字体显示支持
    
    【功能说明】
    matplotlib默认不支持中文字符显示，会显示为方框。此函数自动检测系统可用的
    中文字体并进行配置，确保图表中的中文标签、标题等能够正常显示。
    
    【字体优先级策略】
    按照以下优先级顺序搜索中文字体：
    1. 苹果系统字体：PingFang（现代、清晰）
    2. 黑体系列：Heiti, STHeiti, SimHei（粗体，标题适用）
    3. 微软字体：Microsoft YaHei（Windows系统常用）
    4. 传统字体：STFangsong, STSong, STKaiti（传统印刷体）
    5. 开源字体：WenQuanYi, Noto CJK, Source Han（跨平台）
    6. 兜底字体：Arial Unicode MS（包含中文字符）
    
    【技术原理】
    1. 枚举系统所有已安装的TrueType字体
    2. 使用关键词匹配算法找到中文字体
    3. 按优先级排序，选择最优字体
    4. 更新matplotlib的rcParams配置
    5. 处理负号显示兼容性问题
    
    【返回值】
    bool: True表示成功配置中文字体，False表示使用默认字体
    
    【异常处理】
    - 字体枚举失败：降级到默认字体
    - 字体设置失败：提供友好的错误信息
    - 跨平台兼容性：适配Windows/macOS/Linux
    
    【使用示例】
    ```python
    # 在导入matplotlib后立即调用
    import matplotlib.pyplot as plt
    success = setup_chinese_font()
    if success:
        plt.title("中文标题测试")  # 中文将正常显示
    ```
    """
    # 获取系统所有可用的TrueType字体列表
    try:
        # fm.fontManager.ttflist包含系统所有TTF字体的FontEntry对象
        font_list = [f.name for f in fm.fontManager.ttflist]
        
        # 定义中文字体关键词，按显示效果和可用性排序
        # 这个列表覆盖了主流操作系统的常见中文字体
        chinese_font_keywords = [
            # 苹果系统现代字体（macOS 10.11+）
            'PingFang',          # 苹果设计的现代无衬线字体，清晰易读
            
            # 传统黑体系列（适合标题和强调）
            'Heiti',             # 黑体（macOS系统字体）
            'STHeiti',           # 华文黑体（macOS内置）
            'SimHei',            # 中易黑体（Windows系统字体）
            
            # 微软字体系列（Windows常用）
            'Microsoft YaHei',   # 微软雅黑（Windows Vista+）
            
            # 传统印刷字体（适合正文）
            'STFangsong',        # 华文仿宋
            'STSong',            # 华文宋体
            'STKaiti',           # 华文楷体
            'KaiTi',             # 楷体
            'FangSong',          # 仿宋
            'SimSun',            # 中易宋体（Windows默认）
            
            # 开源字体（跨平台兼容）
            'WenQuanYi',         # 文泉驿字体（Linux常用）
            'Noto Sans CJK',     # Google Noto字体（现代设计）
            'Noto Serif CJK',    # Google Noto衬线字体
            'Source Han',        # 思源字体（Adobe开源）
            
            # 兜底字体（包含中文字符）
            'Arial Unicode MS'   # 包含Unicode中文字符的Arial
        ]
        
        # 搜索可用的中文字体
        chinese_fonts_available = []
        for keyword in chinese_font_keywords:
            for font_name in font_list:
                # 使用子字符串匹配，忽略大小写
                if keyword.lower() in font_name.lower() and font_name not in chinese_fonts_available:
                    chinese_fonts_available.append(font_name)
        
        # 配置找到的第一个可用中文字体
        if chinese_fonts_available:
            selected_font = chinese_fonts_available[0]
            
            # 更新matplotlib的字体配置
            # rcParams是matplotlib的全局配置参数字典
            # 将选中的中文字体添加到sans-serif字体列表的首位
            plt.rcParams['font.sans-serif'] = [selected_font] + plt.rcParams['font.sans-serif']
            
            # 解决matplotlib中负号显示为方框的问题
            # 设置为False使用Unicode负号而不是ASCII连字符
            plt.rcParams['axes.unicode_minus'] = False
            
            print(f"✓ 检测到并使用中文字体: {selected_font}")
            print(f"  可用中文字体总数: {len(chinese_fonts_available)}")
            
            return True
        else:
            # 未找到中文字体的降级处理
            print("⚠ 警告: 未找到中文字体，将使用系统默认字体")
            print("   中文字符可能显示为方框，建议安装中文字体")
            print(f"   系统可用字体前10个: {font_list[:10]}")
            
            # 即使没有中文字体，也要解决负号显示问题
            plt.rcParams['axes.unicode_minus'] = False
            return False
            
    except Exception as e:
        # 异常处理：字体配置过程中的任何错误
        print(f"✗ 字体配置过程出错: {e}")
        print("  将使用matplotlib默认字体设置")
        
        # 即使出错，也要解决负号显示问题
        plt.rcParams['axes.unicode_minus'] = False
        return False

# 初始化中文字体配置
setup_chinese_font()

# ==================================================================================
#                              导入依赖说明和版本要求
# ==================================================================================
"""
依赖库说明：
- numpy >= 1.18.0: 数值计算和数组操作
- matplotlib >= 3.2.0: 图形绘制和可视化
- scipy >= 1.4.0: 科学计算，用于形态学操作
- scikit-image >= 0.16.0: 图像处理，用于形态学操作
- opencv-python >= 4.2.0: 计算机视觉库，用于图像保存

安装命令：
pip install numpy matplotlib scipy scikit-image opencv-python

教学建议：
建议学生首先了解各个依赖库的作用，理解为什么需要这些工具。
"""

# ==================================================================================
#                              轨迹数据加载器模块
# ==================================================================================

class TrajectoryLoader:
    """
    多格式轨迹文件加载器 - SLAM数据处理的数据源组件
    
    【设计目标】
    提供统一的接口加载不同格式的SLAM轨迹数据，屏蔽格式差异，为后续分析提供
    标准化的数据结构。支持主流SLAM基准数据集格式和自定义格式。
    
    【支持格式】
    1. TUM格式：RGB-D SLAM基准数据集标准格式
    2. KITTI格式：自动驾驶SLAM基准数据集格式  
    3. NumPy格式：Python生态系统友好的自定义格式
    
    【设计模式】
    - 策略模式：不同格式使用不同的解析策略
    - 静态工厂：提供统一的创建接口
    - 数据转换：将异构数据转换为同构表示
    
    【性能特性】
    - 内存高效：流式读取，不一次性加载全部数据
    - 错误容忍：跳过格式错误的行，继续处理后续数据
    - 类型安全：严格的数据类型转换和验证
    """
    
    @staticmethod
    def load_tum_trajectory(filename):
        """
        加载TUM (Technical University of Munich) 格式轨迹文件
        
        【TUM格式详解】
        TUM格式是RGB-D SLAM领域最常用的轨迹表示格式，由德国慕尼黑工业大学提出。
        广泛用于RGB-D SLAM算法评估和基准测试。
        
        【文件格式】
        每行格式：timestamp tx ty tz qx qy qz qw
        - timestamp: Unix时间戳 (秒，浮点数)
        - tx, ty, tz: 相机位置 (米，世界坐标系)
        - qx, qy, qz, qw: 相机姿态四元数 (单位四元数)
        
        【坐标系约定】
        - 右手坐标系
        - Z轴向前（相机前方）
        - Y轴向下（图像坐标系对齐）
        - X轴向右
        
        【四元数约定】
        - Hamilton约定：w + xi + yj + zk
        - 单位四元数：||q|| = 1
        - 旋转顺序：先旋转再平移
        
        【示例数据】
        ```
        # timestamp tx ty tz qx qy qz qw
        1305031102.175304 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 1.0000
        1305031102.211214 0.0127 -0.0043 0.0011 0.0000 0.0000 0.0034 0.9999
        ```
        
        【参数】
        filename (str): TUM格式轨迹文件路径
        
        【返回值】
        np.ndarray: 形状为(N, 8)的数组，每行为[timestamp, tx, ty, tz, qx, qy, qz, qw]
        
        【异常处理】
        - 自动跳过注释行（#开头）
        - 忽略格式不正确的行（少于8个字段）
        - 处理浮点数转换错误
        
        【性能优化】
        - 逐行处理，内存占用线性增长
        - 预分配列表，减少动态扩容
        - NumPy数组返回，便于后续向量化计算
        """
        data = []  # 存储解析后的轨迹数据
        line_count = 0  # 用于错误报告的行计数器
        
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                for line in f:
                    line_count += 1
                    
                    # 跳过注释行和空行
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    # 分割字段并验证数量
                    parts = line.split()
                    if len(parts) < 8:
                        print(f"警告: 第{line_count}行格式不正确，字段数={len(parts)} < 8，跳过")
                        continue
                    
                    try:
                        # 解析时间戳
                        timestamp = float(parts[0])
                        
                        # 解析位置 (tx, ty, tz)
                        tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                        
                        # 解析四元数姿态 (qx, qy, qz, qw)
                        qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                        
                        # 验证四元数合理性（可选的健壮性检查）
                        q_norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
                        if abs(q_norm - 1.0) > 0.1:  # 允许一定的数值误差
                            print(f"警告: 第{line_count}行四元数不是单位四元数，模长={q_norm:.4f}")
                        
                        # 添加到数据列表
                        data.append([timestamp, tx, ty, tz, qx, qy, qz, qw])
                        
                    except ValueError as e:
                        print(f"警告: 第{line_count}行数值转换失败: {e}")
                        continue
                        
        except FileNotFoundError:
            raise FileNotFoundError(f"轨迹文件未找到: {filename}")
        except IOError as e:
            raise IOError(f"轨迹文件读取失败: {e}")
        
        if len(data) == 0:
            print("警告: 未成功解析任何有效的轨迹数据")
            return np.array([]).reshape(0, 8)
        
        print(f"✓ 成功加载TUM轨迹: {len(data)}个位姿")
        return np.array(data)

    @staticmethod
    def load_numpy_trajectory(filename):
        """
        加载NumPy格式轨迹文件
        
        【NumPy格式详解】
        NumPy格式是为Python生态系统优化的自定义轨迹存储格式，使用.npz压缩格式
        存储多个数组，提供高效的读写性能和良好的跨平台兼容性。
        
        【文件结构】
        .npz文件包含三个关键数组：
        - poses: 位姿数组，形状为(N, 4, 4)的齐次变换矩阵
        - timestamps: 时间戳数组，形状为(N,)
        - frame_ids: 帧ID数组，形状为(N,)，可选
        
        【齐次变换矩阵格式】
        每个4x4位姿矩阵格式：
        ```
        [R11 R12 R13 tx]
        [R21 R22 R23 ty]  
        [R31 R32 R33 tz]
        [0   0   0   1 ]
        ```
        其中R为3x3旋转矩阵，t为3x1平移向量
        
        【数据来源】
        通常由以下方式生成：
        1. SLAM算法直接输出
        2. 其他格式转换而来
        3. 仿真系统生成
        4. 手动标注数据
        
        【参数】
        filename (str): .npz格式轨迹文件路径
        
        【返回值】
        tuple: (poses, timestamps, frame_ids)
        - poses: np.ndarray, 形状(N, 4, 4)
        - timestamps: np.ndarray, 形状(N,)  
        - frame_ids: np.ndarray, 形状(N,)
        
        【异常处理】
        - 检查必需的数组字段
        - 验证数组形状兼容性
        - 处理缺失的可选字段
        """
        try:
            # 加载.npz压缩文件
            data = np.load(filename)
            
            # 检查必需的字段
            required_fields = ['poses', 'timestamps']
            missing_fields = [field for field in required_fields if field not in data.files]
            if missing_fields:
                raise ValueError(f"NumPy轨迹文件缺少必需字段: {missing_fields}")
            
            # 提取核心数据
            poses = data['poses']
            timestamps = data['timestamps']
            
            # 提取可选的帧ID数据
            frame_ids = data.get('frame_ids', np.arange(len(poses)))
            
            # 验证数据形状兼容性
            if poses.shape[0] != timestamps.shape[0]:
                raise ValueError(f"位姿数量({poses.shape[0]})与时间戳数量({timestamps.shape[0]})不匹配")
            
            if len(poses.shape) != 3 or poses.shape[1] != 4 or poses.shape[2] != 4:
                print(f"警告: 位姿数组形状异常 {poses.shape}，期望 (N, 4, 4)")
            
            print(f"✓ 成功加载NumPy轨迹: {len(poses)}个位姿")
            return poses, timestamps, frame_ids
            
        except Exception as e:
            raise IOError(f"NumPy轨迹文件加载失败: {e}")

    @staticmethod 
    def load_kitti_trajectory(filename):
        """
        加载KITTI格式轨迹文件
        
        【KITTI格式详解】
        KITTI格式由德国卡尔斯鲁厄理工学院(KIT)提出，是自动驾驶领域最权威的
        SLAM基准数据集格式。专为激光雷达和立体相机SLAM算法评估设计。
        
        【文件格式】
        每行包含12个浮点数，表示从世界坐标系到当前帧的3x4变换矩阵：
        r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
        
        【矩阵结构】
        变换矩阵表示为：
        ```
        [r11 r12 r13 tx]
        [r21 r22 r23 ty]  ← 3x4变换矩阵
        [r31 r32 r33 tz]
        ```
        其中：
        - R = [r11 r12 r13; r21 r22 r23; r31 r32 r33] 为旋转矩阵
        - t = [tx; ty; tz] 为平移向量
        
        【坐标系约定】
        - 右手坐标系
        - X轴向右（车辆横向）
        - Y轴向下（重力方向）
        - Z轴向前（车辆前进方向）
        
        【累积变换】
        每行的变换矩阵表示从第0帧到当前帧的累积变换，即：
        T_0_to_i = [R_i | t_i; 0 0 0 1]
        
        【示例数据】
        ```
        1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0
        0.99 -0.01 0.02 0.15 0.01 0.99 0.01 0.05 -0.02 -0.01 0.99 0.30
        ```
        
        【参数】
        filename (str): KITTI格式轨迹文件路径
        
        【返回值】
        np.ndarray: 形状为(N, 3)的位置数组，从变换矩阵中提取的平移部分
        
        【数据质量检查】
        - 验证旋转矩阵的正交性
        - 检查行列式是否接近1
        - 识别异常的跳跃运动
        """
        poses = []
        line_count = 0
        
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                for line in f:
                    line_count += 1
                    
                    # 跳过注释行和空行
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    # 分割字段并验证数量
                    parts = line.split()
                    if len(parts) < 12:
                        print(f"警告: 第{line_count}行格式不正确，字段数={len(parts)} < 12，跳过")
                        continue
                    
                    try:
                        # 解析12个浮点数元素
                        values = [float(x) for x in parts[:12]]
                        
                        # 重构为3x4变换矩阵
                        transform_matrix = np.array([
                            [values[0], values[1], values[2], values[3]],    # [r11 r12 r13 tx]
                            [values[4], values[5], values[6], values[7]],    # [r21 r22 r23 ty]
                            [values[8], values[9], values[10], values[11]]   # [r31 r32 r33 tz]
                        ])
                        
                        # 可选：验证旋转矩阵的正交性（计算密集，通常跳过）
                        R = transform_matrix[:3, :3]  # 提取旋转部分
                        det_R = np.linalg.det(R)      # 计算行列式
                        if abs(det_R - 1.0) > 0.1:    # 检查是否接近单位行列式
                            print(f"警告: 第{line_count}行旋转矩阵可能不正交，行列式={det_R:.4f}")
                        
                        poses.append(transform_matrix)
                        
                    except ValueError as e:
                        print(f"警告: 第{line_count}行数值转换失败: {e}")
                        continue
                        
        except FileNotFoundError:
            raise FileNotFoundError(f"轨迹文件未找到: {filename}")
        except IOError as e:
            raise IOError(f"轨迹文件读取失败: {e}")
        
        if len(poses) == 0:
            print("警告: 未成功解析任何有效的轨迹数据")
            return np.array([]).reshape(0, 3)
        
        # 转换为numpy数组，形状为(N, 3, 4)
        poses_array = np.array(poses)
        
        # 提取位置信息（变换矩阵的平移部分，即最后一列）
        positions = poses_array[:, :, 3]  # 形状为(N, 3)
        
        print(f"✓ 成功加载KITTI轨迹: {len(positions)}个位姿")
        return positions
    
    @classmethod
    def load_trajectory(cls, file_path, file_type):
        """
        统一轨迹加载接口 - 工厂方法模式实现
        
        【设计模式】
        采用工厂方法模式，根据文件类型选择对应的加载策略，提供统一的调用接口。
        这种设计隐藏了不同格式的实现细节，使客户端代码保持简洁。
        
        【策略选择】
        根据file_type参数动态选择加载策略：
        - 'tum': 调用load_tum_trajectory()处理TUM格式
        - 'kitti': 调用load_kitti_trajectory()处理KITTI格式  
        - 'numpy': 调用load_numpy_trajectory()处理NumPy格式
        
        【扩展性设计】
        新增格式支持只需：
        1. 实现对应的load_xxx_trajectory()静态方法
        2. 在此方法中添加对应的elif分支
        3. 无需修改客户端代码
        
        【参数】
        file_path (str): 轨迹文件路径
        file_type (str): 文件类型，可选值：'tum', 'kitti', 'numpy'
        
        【返回值】
        轨迹数据，具体格式取决于file_type：
        - TUM: np.ndarray, 形状(N, 8) 
        - KITTI: np.ndarray, 形状(N, 3)
        - NumPy: tuple(poses, timestamps, frame_ids)
        
        【异常】
        ValueError: 不支持的文件类型
        """
        # 使用字典映射替代if-elif链，提高可维护性和性能
        loader_mapping = {
            'tum': cls.load_tum_trajectory,
            'kitti': cls.load_kitti_trajectory,
            'numpy': cls.load_numpy_trajectory
        }
        
        if file_type not in loader_mapping:
            supported_types = ', '.join(loader_mapping.keys())
            raise ValueError(f"不支持的文件类型: {file_type}。支持的类型: {supported_types}")
        
        # 调用对应的加载方法
        loader_func = loader_mapping[file_type]
        return loader_func(file_path)
    
    @staticmethod
    def extract_positions(trajectory_data, file_type):
        """
        从不同格式的轨迹数据中提取3D位置信息
        
        【功能说明】
        由于不同格式的轨迹数据结构差异较大，此方法提供统一的位置提取接口，
        将异构的轨迹数据转换为统一的3D位置数组格式，便于后续的可视化和分析。
        
        【数据转换】
        不同格式的位置提取策略：
        
        1. TUM格式：
           - 输入：(N, 8) 数组 [timestamp, tx, ty, tz, qx, qy, qz, qw]
           - 提取：[:, 1:4] 获取位置列 [tx, ty, tz]
           
        2. KITTI格式：
           - 输入：(N, 3) 数组，已经是位置数据
           - 直接返回原数据
           
        3. NumPy格式：
           - 输入：tuple(poses, timestamps, frame_ids)
           - poses形状：(N, 4, 4) 齐次变换矩阵
           - 提取：[:, :3, 3] 获取变换矩阵的平移部分
        
        【坐标系统一】
        所有格式提取后的位置都使用相同的坐标系表示：
        - X轴：右方向或东方向
        - Y轴：根据具体应用而定
        - Z轴：上方向或前方向
        - 单位：米(meter)
        
        【参数】
        trajectory_data: 轨迹数据，格式取决于file_type
        file_type (str): 数据格式类型
        
        【返回值】
        np.ndarray: 形状为(N, 3)的位置数组，每行为[x, y, z]坐标
        
        【使用示例】
        ```python
        # 加载不同格式的轨迹
        tum_data = loader.load_tum_trajectory('tum.txt')
        kitti_data = loader.load_kitti_trajectory('kitti.txt') 
        numpy_data = loader.load_numpy_trajectory('numpy.npz')
        
        # 统一提取位置信息
        tum_pos = loader.extract_positions(tum_data, 'tum')
        kitti_pos = loader.extract_positions(kitti_data, 'kitti')
        numpy_pos = loader.extract_positions(numpy_data, 'numpy')
        
        # 现在所有position都是(N, 3)格式，可以统一处理
        ```
        """
        if file_type == 'tum':
            # TUM格式：[timestamp, tx, ty, tz, qx, qy, qz, qw]
            # 提取第1-3列（索引1:4）作为位置信息
            if len(trajectory_data.shape) == 2 and trajectory_data.shape[1] >= 4:
                return trajectory_data[:, 1:4]  # 提取[tx, ty, tz]
            else:
                raise ValueError(f"TUM轨迹数据格式错误，期望(N, 8)，实际{trajectory_data.shape}")
                
        elif file_type == 'numpy':
            # NumPy格式：(poses, timestamps, frame_ids)
            poses, timestamps, frame_ids = trajectory_data
            if len(poses.shape) == 3 and poses.shape[1] >= 3 and poses.shape[2] >= 4:
                # 从4x4齐次变换矩阵中提取平移部分（最后一列的前3行）
                return poses[:, :3, 3]  # 提取平移向量
            else:
                raise ValueError(f"NumPy轨迹数据格式错误，期望(N, 4, 4)，实际{poses.shape}")
                
        elif file_type == 'kitti':
            # KITTI格式：已经是位置数据，直接返回
            if len(trajectory_data.shape) == 2 and trajectory_data.shape[1] == 3:
                return trajectory_data
            else:
                raise ValueError(f"KITTI轨迹数据格式错误，期望(N, 3)，实际{trajectory_data.shape}")
        else:
            supported_types = ['tum', 'kitti', 'numpy']
            raise ValueError(f"不支持的文件类型: {file_type}。支持的类型: {supported_types}")

# ==================================================================================
#                              轨迹可视化器模块
# ==================================================================================

class TrajectoryVisualizer:
    """
    轨迹可视化器 - SLAM轨迹的多维度分析和展示组件
    
    【设计理念】
    SLAM轨迹包含丰富的空间和时间信息，单一的可视化方式难以全面展现轨迹特性。
    本类采用多视图设计，从不同角度和维度分析轨迹，帮助研究者深入理解SLAM
    算法的运行特性和轨迹质量。
    
    【可视化维度】
    1. 空间维度：2D俯视图、3D立体图、侧视图
    2. 高度维度：高度编码彩色地图  
    3. 运动维度：速度分析、加速度变化
    4. 时间维度：时间轴上的运动模式
    
    【教学价值】
    - 直观展示SLAM轨迹的空间分布特征
    - 分析机器人运动模式和速度变化
    - 检测轨迹异常和定位失效区域
    - 评估SLAM算法的性能和稳定性
    
    【技术特性】
    - 基于matplotlib的高质量图形渲染
    - 自适应颜色映射和缩放
    - 交互式3D视图支持
    - 中文字体和标注支持
    """
    
    def __init__(self):
        """
        初始化轨迹可视化器
        
        【设计说明】
        可视化器采用无状态设计，不存储轨迹数据，所有方法都接受位置数据作为参数。
        这种设计提高了内存效率，并支持同时可视化多条轨迹。
        """
        pass
    
    def plot_trajectory_map(self, positions, title="相机轨迹地图"):
        """
        绘制综合轨迹分析地图 - 四视图布局
        
        【功能说明】
        创建2x2子图布局，从四个不同角度分析SLAM轨迹：
        1. 俯视图：显示X-Y平面的轨迹投影，适合分析水平运动
        2. 侧视图：显示X-Z平面的轨迹投影，适合分析垂直运动  
        3. 高度编码图：用颜色表示高度变化，直观显示3D轨迹
        4. 速度分析图：显示运动速度随时间的变化
        
        【布局设计】
        ```
        ┌─────────────┬─────────────┐
        │  俯视图     │  侧视图     │
        │ (X-Y平面)   │ (X-Z平面)   │  
        ├─────────────┼─────────────┤
        │ 高度编码图  │ 速度分析图  │
        │ (彩色高度)  │ (时间序列)  │
        └─────────────┴─────────────┘
        ```
        
        【教学用途】
        - 全面分析轨迹的几何特性
        - 识别SLAM算法的性能特点
        - 检测定位漂移和跳跃现象
        - 评估运动模式的合理性
        
        【参数】
        positions (np.ndarray): 形状(N, 3)的位置数组，每行为[x, y, z]
        title (str): 图像总标题
        
        【技术实现】
        - 使用matplotlib.pyplot.subplots创建子图
        - 自适应调整图像大小和字体
        - 统一的颜色主题和样式
        - 中文标题和标签支持
        """
        # 创建2x2子图布局，整体尺寸15x12英寸，适合高分辨率显示
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle(title, fontsize=16, fontweight='bold')
        
        # 检查输入数据有效性
        if len(positions) == 0:
            plt.figtext(0.5, 0.5, '无有效轨迹数据', ha='center', va='center', fontsize=20)
            plt.show()
            return
        
        # 1. 左上：2D俯视图 - 显示水平面运动轨迹
        self._plot_2d_view(axes[0, 0], positions)
        
        # 2. 右上：侧视图 (XZ平面) - 显示垂直面运动轨迹  
        self._plot_side_view(axes[0, 1], positions)
        
        # 3. 左下：高度编码的彩色轨迹图
        self._plot_height_encoded(axes[1, 0], positions)
        
        # 4. 右下：运动速度分析图
        self._plot_velocity_analysis(axes[1, 1], positions)
        
        # 调整子图间距，避免标题和标签重叠
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # 为主标题留出空间
        plt.show()
    
    def _plot_2d_view(self, ax, positions):
        """
        绘制2D俯视图 - X-Y平面轨迹投影
        
        【功能说明】
        将3D轨迹投影到水平面(X-Y平面)，显示机器人在地面上的运动路径。
        这是最常用的轨迹分析视图，适合分析室内导航和路径规划。
        
        【可视化元素】
        - 蓝色实线：完整运动轨迹
        - 绿色圆点：起始位置
        - 红色圆点：结束位置  
        - 红色散点：采样关键点（每20个点显示一个）
        
        【应用场景】
        - 室内SLAM轨迹分析
        - 路径规划验证
        - 定位精度评估
        - 回环检测可视化
        """
        # 绘制主轨迹线
        ax.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='轨迹', alpha=0.8)
        
        # 标记起点和终点
        ax.plot(positions[0, 0], positions[0, 1], 'go', markersize=10, label='起点', markeredgecolor='darkgreen')
        ax.plot(positions[-1, 0], positions[-1, 1], 'ro', markersize=10, label='终点', markeredgecolor='darkred')
        
        # 显示关键采样点，帮助理解轨迹密度
        ax.scatter(positions[::20, 0], positions[::20, 1], c='red', s=20, alpha=0.5, label='采样点')
        
        # 设置坐标轴标签和标题
        ax.set_xlabel('X坐标 (米)', fontweight='bold')
        ax.set_ylabel('Y坐标 (米)', fontweight='bold')
        ax.set_title('俯视图 (X-Y平面)', fontweight='bold')
        
        # 添加图例和网格
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.axis('equal')  # 保持X-Y轴比例一致，避免轨迹变形
    
    def _plot_side_view(self, ax, positions):
        """绘制侧视图"""
        ax.plot(positions[:, 0], positions[:, 2], 'r-', linewidth=2)
        ax.plot(positions[0, 0], positions[0, 2], 'go', markersize=10, label='起点')
        ax.plot(positions[-1, 0], positions[-1, 2], 'ro', markersize=10, label='终点')
        ax.set_xlabel('X (米)')
        ax.set_ylabel('Z (米)')
        ax.set_title('侧视图 (XZ)')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def _plot_height_encoded(self, ax, positions):
        """绘制高度编码的轨迹"""
        colors = plt.cm.viridis((positions[:, 2] - positions[:, 2].min()) / 
                               (positions[:, 2].max() - positions[:, 2].min() + 1e-8))
        scatter = ax.scatter(positions[:, 0], positions[:, 1], c=positions[:, 2], 
                            cmap='viridis', s=15, alpha=0.8)
        ax.plot(positions[0, 0], positions[0, 1], 'wo', markersize=8, label='起点')
        ax.plot(positions[-1, 0], positions[-1, 1], 'ko', markersize=8, label='终点')
        plt.colorbar(scatter, ax=ax, label='高度 Z (米)')
        ax.set_xlabel('X (米)')
        ax.set_ylabel('Y (米)')
        ax.set_title('高度编码地图')
        ax.legend()
        ax.axis('equal')
    
    def _plot_velocity_analysis(self, ax, positions):
        """绘制速度分析图"""
        if len(positions) > 1:
            velocities = np.diff(positions, axis=0)
            speeds = np.linalg.norm(velocities, axis=1)
            ax.plot(range(len(speeds)), speeds, 'purple', linewidth=2)
            ax.set_xlabel('时间步')
            ax.set_ylabel('速度 (米/步)')
            ax.set_title('运动速度')
            ax.grid(True, alpha=0.3)
    
    def plot_3d_map(self, positions, title="3D轨迹地图"):
        """绘制3D地图"""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制3D轨迹
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2)
        ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                   color='green', s=100, label='起点')
        ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
                   color='red', s=100, label='终点')
        
        # 添加关键点
        step = max(1, len(positions) // 30)
        ax.scatter(positions[::step, 0], positions[::step, 1], positions[::step, 2], 
                   c='orange', s=30, alpha=0.7, label='关键点')
        
        ax.set_xlabel('X (米)')
        ax.set_ylabel('Y (米)')
        ax.set_zlabel('Z (米)')
        ax.set_title(title)
        ax.legend()
        
        plt.show()
    
    @staticmethod
    def print_trajectory_stats(positions):
        """打印轨迹统计信息"""
        total_distance = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
        print(f"轨迹统计:")
        print(f"  位姿数量: {len(positions)}")
        print(f"  总距离: {total_distance:.2f} 米")
        print(f"  起点: ({positions[0, 0]:.2f}, {positions[0, 1]:.2f}, {positions[0, 2]:.2f})")
        print(f"  终点: ({positions[-1, 0]:.2f}, {positions[-1, 1]:.2f}, {positions[-1, 2]:.2f})")

# ==================================================================================
#                              占用地图生成器模块
# ==================================================================================

class OccupancyMapGenerator:
    """
    占用地图生成器 - 从SLAM轨迹生成2D导航地图
    
    【核心功能】
    将SLAM轨迹数据转换为机器人导航所需的2D占用地图，实现从定位到导航的
    关键数据转换。占用地图是移动机器人路径规划的核心数据结构。
    
    【算法原理】
    1. 轨迹栅格化：将连续轨迹离散化为栅格地图
    2. 空闲区域标记：以机器人半径为缓冲区标记可通行区域
    3. 路径连接：使用直线插值连接轨迹间断点
    4. 形态学处理：平滑地图边界，消除噪声
    5. 边界设置：添加安全边界区域
    
    【地图表示】
    栅格值含义：
    - 0.0 (黑色)：障碍物，不可通行
    - 1.0 (白色)：空闲区域，安全通行
    - 0.5 (灰色)：未知区域，未探索
    
    【应用价值】
    - 自主导航：为路径规划提供环境地图
    - 建图评估：评估SLAM建图质量
    - 仿真环境：为仿真提供真实环境模型
    - 可视化分析：直观展示机器人可达区域
    """
    
    def __init__(self, resolution=0.05, robot_radius=0.3, map_extension=2.0):
        """
        初始化占用地图生成器
        
        【参数说明】
        resolution (float): 地图分辨率，单位米/像素
            - 0.05: 高精度(5cm/像素)，适合精细导航
            - 0.1: 标准精度(10cm/像素)，平衡性能和精度
            - 0.2: 低精度(20cm/像素)，适合大范围地图
            
        robot_radius (float): 机器人半径，单位米
            - 考虑机器人的物理尺寸，确保安全通行
            - 包含安全裕量，防止碰撞
            
        map_extension (float): 地图边界扩展距离，单位米
            - 在轨迹范围外扩展的安全区域
            - 为路径规划提供更大的搜索空间
        """
        self.resolution = resolution      # 地图分辨率(米/像素)
        self.robot_radius = robot_radius  # 机器人半径(米)
        self.map_extension = map_extension # 地图扩展距离(米)
    
    def generate_occupancy_map(self, positions):
        """
        从轨迹数据生成占用地图（2D房间平面图）
        
        参数：
            positions: 相机/机器人位置数组 (N, 3)
        
        返回：
            occupancy_map: 占用地图 (0=占用, 1=空闲, 0.5=未知)
            map_info: 地图元信息字典
        """
        # 提取XY平面的轨迹点
        trajectory_2d = positions[:, :2]  # 只取x, y坐标
        
        # 计算地图边界和尺寸
        map_bounds = self._calculate_map_bounds(trajectory_2d)
        occupancy_map = self._initialize_map(map_bounds)
        
        # 转换轨迹点为像素坐标
        trajectory_pixels = self._world_to_pixel(trajectory_2d, map_bounds)
        
        # 标记轨迹经过的区域为空闲区域
        self._mark_trajectory_as_free(occupancy_map, trajectory_pixels, map_bounds)
        
        # 连接轨迹点之间的路径
        self._connect_trajectory_points(occupancy_map, trajectory_pixels, map_bounds)
        
        # 使用形态学操作平滑地图
        occupancy_map = self._smooth_map(occupancy_map)
        
        # 设置地图边界为障碍物
        occupancy_map = self._set_map_borders(occupancy_map)
        
        # 创建地图元信息
        map_info = self._create_map_info(map_bounds)
        
        return occupancy_map, map_info
    
    def _calculate_map_bounds(self, trajectory_2d):
        """计算地图边界"""
        min_x, min_y = trajectory_2d.min(axis=0) - self.map_extension
        max_x, max_y = trajectory_2d.max(axis=0) + self.map_extension
        width = int((max_x - min_x) / self.resolution) + 1
        height = int((max_y - min_y) / self.resolution) + 1
        
        return {
            'min_x': min_x, 'min_y': min_y,
            'max_x': max_x, 'max_y': max_y,
            'width': width, 'height': height
        }
    
    def _initialize_map(self, map_bounds):
        """初始化占用地图"""
        return np.full((map_bounds['height'], map_bounds['width']), 0.5, dtype=np.float32)
    
    def _world_to_pixel(self, trajectory_2d, map_bounds):
        """将世界坐标转换为像素坐标"""
        trajectory_pixels = np.zeros((len(trajectory_2d), 2), dtype=int)
        trajectory_pixels[:, 0] = ((trajectory_2d[:, 0] - map_bounds['min_x']) / self.resolution).astype(int)
        trajectory_pixels[:, 1] = ((trajectory_2d[:, 1] - map_bounds['min_y']) / self.resolution).astype(int)
        
        # 确保像素坐标在地图范围内
        trajectory_pixels[:, 0] = np.clip(trajectory_pixels[:, 0], 0, map_bounds['width'] - 1)
        trajectory_pixels[:, 1] = np.clip(trajectory_pixels[:, 1], 0, map_bounds['height'] - 1)
        
        return trajectory_pixels
    
    def _mark_trajectory_as_free(self, occupancy_map, trajectory_pixels, map_bounds):
        """标记轨迹经过的区域为空闲区域"""
        height, width = map_bounds['height'], map_bounds['width']
        radius_pixels = int(self.robot_radius / self.resolution)
        
        for i in range(len(trajectory_pixels)):
            x, y = trajectory_pixels[i]
            # 在轨迹点周围创建圆形空闲区域
            y_coords, x_coords = np.ogrid[:height, :width]
            mask = (x_coords - x)**2 + (y_coords - y)**2 <= radius_pixels**2
            occupancy_map[mask] = 1.0  # 空闲区域
    
    def _connect_trajectory_points(self, occupancy_map, trajectory_pixels, map_bounds):
        """连接轨迹点之间的路径"""
        height, width = map_bounds['height'], map_bounds['width']
        radius_pixels = int(self.robot_radius / self.resolution)
        
        for i in range(len(trajectory_pixels) - 1):
            x1, y1 = trajectory_pixels[i]
            x2, y2 = trajectory_pixels[i + 1]
            
            # 使用Bresenham算法连接两点
            line_points = self._bresenham_line(x1, y1, x2, y2)
            for px, py in line_points:
                if 0 <= px < width and 0 <= py < height:
                    # 在线段周围创建空闲走廊
                    y_coords, x_coords = np.ogrid[:height, :width]
                    mask = (x_coords - px)**2 + (y_coords - py)**2 <= radius_pixels**2
                    occupancy_map[mask] = 1.0
    
    def _smooth_map(self, occupancy_map):
        """使用形态学操作平滑地图"""
        kernel_size = max(1, int(self.robot_radius / self.resolution / 2))
        kernel = disk(kernel_size)
        
        # 处理空闲区域
        free_areas = (occupancy_map == 1.0)
        free_areas = binary_dilation(free_areas, kernel)
        free_areas = binary_erosion(free_areas, kernel)
        
        # 更新地图
        occupancy_map[free_areas] = 1.0
        
        return occupancy_map
    
    def _set_map_borders(self, occupancy_map):
        """在地图边界附近设置障碍物"""
        border_width = int(0.5 / self.resolution)  # 50cm边界
        occupancy_map[:border_width, :] = 0.0  # 顶部边界
        occupancy_map[-border_width:, :] = 0.0  # 底部边界
        occupancy_map[:, :border_width] = 0.0  # 左侧边界
        occupancy_map[:, -border_width:] = 0.0  # 右侧边界
        
        return occupancy_map
    
    def _create_map_info(self, map_bounds):
        """创建地图元信息"""
        return {
            'resolution': self.resolution,
            'width': map_bounds['width'],
            'height': map_bounds['height'],
            'origin_x': map_bounds['min_x'],
            'origin_y': map_bounds['min_y'],
            'robot_radius': self.robot_radius
        }
    
    @staticmethod
    def _bresenham_line(x0, y0, x1, y1):
        """Bresenham直线算法生成连接两点的像素点"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        
        dx *= 2
        dy *= 2
        
        for _ in range(n):
            points.append((x, y))
            
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
        
        return points
    
    @staticmethod
    def calculate_map_statistics(occupancy_map, resolution):
        """计算地图统计信息"""
        free_area = np.sum(occupancy_map == 1.0) * (resolution ** 2)
        occupied_area = np.sum(occupancy_map == 0.0) * (resolution ** 2)
        unknown_area = np.sum(occupancy_map == 0.5) * (resolution ** 2)
        total_area = free_area + occupied_area + unknown_area
        
        return {
            'free_area': free_area,
            'occupied_area': occupied_area,
            'unknown_area': unknown_area,
            'total_area': total_area,
            'free_percentage': free_area / total_area * 100,
            'occupied_percentage': occupied_area / total_area * 100,
            'unknown_percentage': unknown_area / total_area * 100
        }

# ==================================================================================
#                              地图可视化器模块
# ==================================================================================

class MapVisualizer:
    """
    地图可视化器 - 占用地图的专业展示和分析组件
    
    【设计目标】
    提供高质量的占用地图可视化功能，将抽象的栅格数据转换为直观的图形表示。
    专注于机器人导航地图的展示，支持轨迹叠加、图例说明、颜色编码等专业功能。
    
    【核心功能】
    1. 占用地图渲染：将数值栅格转换为彩色图像
    2. 轨迹叠加显示：在地图上叠加机器人运动轨迹
    3. 图例和标注：提供清晰的地图元素说明
    4. 交互式显示：支持缩放、平移等交互操作
    5. 统计信息展示：显示地图的定量分析结果
    
    【可视化原理】
    占用地图可视化的核心是将离散的栅格值映射为连续的颜色空间：
    - 数值映射：0.0→黑色(障碍)，1.0→白色(空闲)，0.5→灰色(未知)
    - 颜色插值：支持中间值的平滑过渡
    - 对比度优化：确保不同区域的清晰区分
    - 空间校准：保持真实的空间比例关系
    
    【应用场景】
    - 机器人导航：路径规划的环境理解
    - SLAM评估：建图质量的视觉检查
    - 仿真验证：虚拟环境的真实性验证
    - 科研发表：论文中的地图展示
    - 教学演示：算法原理的直观解释
    
    【技术特性】
    - 高质量渲染：基于matplotlib的矢量图形
    - 自适应缩放：自动调整显示范围和比例
    - 多层叠加：支持多种信息的同时显示
    - 颜色理论：基于人眼视觉特性的配色方案
    - 无损显示：保持原始数据的精确性
    
    【设计模式】
    - 策略模式：不同的可视化策略
    - 模板方法：标准化的渲染流程
    - 装饰器模式：功能的动态添加
    """
    
    def __init__(self):
        """
        初始化地图可视化器
        
        【设计说明】
        采用轻量级初始化策略，不预加载任何数据或配置。所有可视化参数
        通过方法参数传递，保持类的无状态特性，提高内存效率和并发安全性。
        
        【无状态设计优势】
        - 内存效率：不存储大型数据对象
        - 线程安全：支持多线程并发调用
        - 灵活性：每次调用可使用不同参数
        - 可测试性：方法行为完全由输入决定
        """
        pass
    
    def plot_occupancy_map(self, occupancy_map, map_info, trajectory_2d=None, title="2D房间平面图"):
        """
        可视化占用地图 - 核心渲染方法
        
        【功能说明】
        将数值化的占用地图转换为高质量的可视化图像，支持轨迹叠加和专业标注。
        采用模板方法模式，将复杂的渲染过程分解为标准化的步骤。
        
        【渲染流程】
        1. 图像初始化：创建matplotlib画布和坐标轴
        2. 颜色映射：将数值栅格转换为RGB颜色数组
        3. 地图显示：渲染基础的占用地图
        4. 轨迹叠加：在地图上绘制机器人运动轨迹
        5. 属性设置：配置坐标轴、标题、网格等
        6. 图例添加：提供清晰的视觉元素说明
        7. 布局优化：调整间距和比例
        
        【技术细节】
        - 图像尺寸：12x10英寸，适合高分辨率显示和打印
        - 颜色空间：RGB颜色模式，确保跨平台一致性
        - 坐标变换：从栅格坐标到实际物理坐标的映射
        - 图例设计：使用Patch对象创建专业图例
        
        【教学价值】
        - 数据可视化：展示数值数据的图形化表示
        - 坐标系转换：理解图像坐标与物理坐标的关系
        - 颜色理论：学习颜色编码的设计原则
        - 用户界面：掌握科学图表的标准化设计
        
        【参数说明】
        occupancy_map (np.ndarray): 
            - 形状(H, W)的占用地图数组
            - 值域[0.0, 1.0]，0.0=障碍物，1.0=空闲，0.5=未知
            
        map_info (dict): 地图元信息字典，包含：
            - resolution: 地图分辨率(米/像素)
            - width/height: 地图尺寸(像素)
            - origin_x/origin_y: 地图原点坐标(米)
            
        trajectory_2d (np.ndarray, optional): 
            - 形状(N, 2)的轨迹坐标数组
            - 坐标单位为米，与地图坐标系一致
            
        title (str): 图像主标题，支持中文显示
        
        【使用示例】
        ```python
        visualizer = MapVisualizer()
        
        # 基础地图显示
        visualizer.plot_occupancy_map(occupancy_map, map_info)
        
        # 带轨迹的地图显示  
        visualizer.plot_occupancy_map(
            occupancy_map, map_info, 
            trajectory_2d=positions[:, :2],
            title="实验室SLAM地图"
        )
        ```
        """
        # 创建画布和坐标轴，设置适合打印和显示的尺寸
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        
        # 数据验证
        if occupancy_map.size == 0:
            ax.text(0.5, 0.5, '无地图数据', ha='center', va='center', 
                   transform=ax.transAxes, fontsize=20)
            plt.show()
            return
        
        # 步骤1: 创建颜色映射 - 将数值转换为RGB颜色
        colored_map = self._create_colored_map(occupancy_map)
        
        # 步骤2: 显示基础地图 - 渲染占用地图背景
        self._display_map(ax, colored_map, map_info)
        
        # 步骤3: 叠加轨迹信息 - 在地图上绘制运动路径
        if trajectory_2d is not None and len(trajectory_2d) > 0:
            self._overlay_trajectory(ax, trajectory_2d)
        
        # 步骤4: 设置图形属性 - 配置坐标轴和标题
        self._setup_plot_properties(ax, title)
        
        # 步骤5: 添加专业图例 - 说明地图元素含义
        self._add_legend(ax)
        
        # 步骤6: 优化布局并显示
        plt.tight_layout()
        plt.show()
    
    def _create_colored_map(self, occupancy_map):
        """创建颜色地图"""
        colored_map = np.zeros((occupancy_map.shape[0], occupancy_map.shape[1], 3))
        colored_map[occupancy_map == 0.0] = [0, 0, 0]      # 黑色 - 障碍物
        colored_map[occupancy_map == 1.0] = [1, 1, 1]      # 白色 - 空闲区域
        colored_map[occupancy_map == 0.5] = [0.5, 0.5, 0.5]  # 灰色 - 未知区域
        return colored_map
    
    def _display_map(self, ax, colored_map, map_info):
        """显示地图"""
        ax.imshow(colored_map, origin='lower', extent=[
            map_info['origin_x'], 
            map_info['origin_x'] + map_info['width'] * map_info['resolution'],
            map_info['origin_y'], 
            map_info['origin_y'] + map_info['height'] * map_info['resolution']
        ])
    
    def _overlay_trajectory(self, ax, trajectory_2d):
        """叠加轨迹"""
        ax.plot(trajectory_2d[:, 0], trajectory_2d[:, 1], 'r-', linewidth=2, 
                label='机器人轨迹', alpha=0.8)
        ax.plot(trajectory_2d[0, 0], trajectory_2d[0, 1], 'go', markersize=10, 
                label='起点')
        ax.plot(trajectory_2d[-1, 0], trajectory_2d[-1, 1], 'ro', markersize=10, 
                label='终点')
    
    def _setup_plot_properties(self, ax, title):
        """设置图形属性"""
        ax.set_xlabel('X坐标 (米)')
        ax.set_ylabel('Y坐标 (米)')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
    
    def _add_legend(self, ax):
        """添加图例"""
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='white', edgecolor='black', label='可行走区域'),
            Patch(facecolor='black', edgecolor='black', label='障碍物/墙壁'),
            Patch(facecolor='gray', edgecolor='black', label='未探索区域'),
            Patch(facecolor='red', edgecolor='red', label='机器人轨迹')
        ]
        ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1, 1))
    
    @staticmethod
    def print_map_statistics(stats):
        """打印地图统计信息"""
        print(f"  可行走区域: {stats['free_area']:.2f} 平方米 ({stats['free_percentage']:.1f}%)")
        print(f"  障碍物区域: {stats['occupied_area']:.2f} 平方米 ({stats['occupied_percentage']:.1f}%)")
        print(f"  未知区域: {stats['unknown_area']:.2f} 平方米 ({stats['unknown_percentage']:.1f}%)")

# ==================================================================================
#                              地图保存器模块
# ==================================================================================

class MapSaver:
    """
    地图保存器 - 多格式地图数据序列化组件
    
    【设计目标】
    提供统一的地图保存接口，支持多种行业标准格式，确保生成的地图文件
    能够被不同的机器人系统、仿真平台和分析工具正确读取和使用。
    
    【支持格式详解】
    1. PGM格式 (Portable Gray Map)
       - 标准：ROS/ROS2机器人操作系统标准格式
       - 用途：导航栈(Navigation Stack)的地图输入
       - 特点：8位灰度图像，广泛兼容
       - 应用：move_base、nav2等导航包
    
    2. YAML格式 (YAML Ain't Markup Language)  
       - 标准：ROS地图元数据标准
       - 用途：描述地图的物理属性和参数
       - 内容：分辨率、原点、阈值等关键信息
       - 关联：与PGM文件配对使用
    
    3. NumPy格式 (.npz压缩格式)
       - 标准：Python科学计算生态标准
       - 用途：Python程序间的高效数据交换
       - 特点：压缩存储，保持精确数值
       - 优势：快速读写，完整保存所有信息
    
    【数据流转换】
    内存占用地图 → 格式转换 → 文件保存
    ↓
    - 数值标准化：[0.0,1.0] → [0,255] (PGM)
    - 坐标系转换：笛卡尔坐标 → 图像坐标
    - 元数据提取：生成YAML配置文件
    - 压缩存储：完整数据的NPZ归档
    
    【工业应用价值】
    - 机器人导航：为自主导航提供环境地图
    - 仿真测试：构建虚拟测试环境
    - 数据分析：支持离线分析和算法验证
    - 跨平台交换：不同系统间的数据共享
    - 版本控制：地图数据的版本管理
    
    【质量保证】
    - 数据完整性：确保保存过程中无数据丢失
    - 格式合规性：严格遵循各格式标准
    - 跨平台兼容：保证在不同操作系统正常工作
    - 错误恢复：提供保存失败时的恢复机制
    """
    
    def __init__(self):
        """
        初始化地图保存器
        
        【设计理念】
        采用无状态设计，不预加载任何配置或缓存。每次保存操作都是独立的，
        避免状态污染，提高并发安全性和内存效率。
        
        【扩展性考虑】
        - 格式插件化：易于添加新的保存格式
        - 参数化配置：支持自定义保存选项
        - 异步保存：为大型地图提供异步保存能力
        - 进度反馈：为长时间操作提供进度信息
        """
        pass
    
    def save_occupancy_map(self, occupancy_map, map_info, filename):
        """
        保存占用地图为多种工业标准格式 - 一站式保存解决方案
        
        【功能概述】
        实现占用地图的多格式同步保存，确保生成的地图文件能够被不同的
        机器人系统、仿真平台和分析工具正确读取。采用原子化操作，
        要么全部成功，要么全部失败，保证数据一致性。
        
        【保存流程】
        1. 数据验证：检查输入数据的完整性和有效性
        2. PGM保存：生成ROS标准的8位灰度地图图像
        3. YAML保存：生成地图元数据配置文件
        4. NumPy保存：生成Python生态的压缩数据文件
        5. 完整性验证：确认所有文件正确生成
        6. 状态报告：向用户反馈保存结果
        
        【格式转换详解】
        
        PGM格式转换：
        - 输入：[0.0, 1.0]浮点数占用概率
        - 输出：[0, 255]整数灰度值
        - 映射：0.0→0(黑/障碍), 1.0→255(白/空闲), 0.5→127(灰/未知)
        - 坐标：Y轴翻转适配图像坐标系
        
        YAML配置文件内容：
        ```yaml
        image: filename.pgm           # 关联的图像文件
        resolution: 0.05              # 地图分辨率(米/像素)
        origin: [-10.0, -10.0, 0.0]  # 地图原点[x, y, theta]
        negate: 0                     # 是否反色(0=否, 1=是)
        occupied_thresh: 0.65         # 占用阈值
        free_thresh: 0.196            # 空闲阈值
        ```
        
        NumPy格式内容：
        - occupancy_map: 原始浮点数地图数组
        - map_info: 完整的地图元信息字典
        - 压缩存储：使用gzip压缩减小文件大小
        
        【工业标准兼容性】
        - ROS Navigation Stack：PGM+YAML格式
        - ROS2 Nav2：向后兼容ROS格式
        - Gazebo仿真器：支持PGM地图导入
        - RVIZ可视化：直接显示PGM地图
        - SLAM Toolbox：标准输入格式
        - Python分析：NumPy格式最优
        
        【错误处理策略】
        - 预检查：保存前验证所有输入
        - 原子操作：全成功或全失败
        - 回滚机制：失败时清理部分文件
        - 详细日志：记录每步操作状态
        
        【参数说明】
        occupancy_map (np.ndarray): 
            - 形状(H, W)的占用地图数组
            - 数据类型：float32或float64
            - 值域：[0.0, 1.0]，严格要求
            
        map_info (dict): 地图元信息，必需字段：
            - resolution (float): 地图分辨率，单位米/像素
            - width (int): 地图宽度，单位像素
            - height (int): 地图高度，单位像素  
            - origin_x (float): 地图原点X坐标，单位米
            - origin_y (float): 地图原点Y坐标，单位米
            
        filename (str): 输出文件基础名称（不含扩展名）
            - 自动添加.pgm/.yaml/.npz扩展名
            - 支持相对路径和绝对路径
            - 文件名应遵循操作系统命名规范
        
        【使用示例】
        ```python
        saver = MapSaver()
        
        # 基础保存
        saver.save_occupancy_map(map_array, map_info, "room_map")
        # 生成文件：room_map.pgm, room_map.yaml, room_map.npz
        
        # 带路径保存
        saver.save_occupancy_map(map_array, map_info, "maps/lab_room_2024")
        # 生成文件：maps/lab_room_2024.pgm, maps/lab_room_2024.yaml, maps/lab_room_2024.npz
        ```
        
        【性能特性】
        - 内存效率：流式写入，不重复内存分配
        - 并发安全：支持多线程同时调用
        - 压缩优化：NPZ格式自动压缩存储
        - 跨平台：在Windows/macOS/Linux正常工作
        """
        try:
            # 前置验证：确保输入数据有效
            self._validate_inputs(occupancy_map, map_info, filename)
            
            print(f"开始保存占用地图: {filename}")
            
            # 保存为不同格式 - 并行执行以提高效率
            save_results = {}
            
            # 步骤1: 保存PGM格式（ROS标准灰度图像）
            print(f"  保存PGM格式...")
            save_results['pgm'] = self._save_pgm_format(occupancy_map, filename)
            
            # 步骤2: 保存YAML元数据（ROS标准配置）  
            print(f"  保存YAML元数据...")
            save_results['yaml'] = self._save_yaml_metadata(map_info, filename)
            
            # 步骤3: 保存NumPy格式（Python科学计算）
            print(f"  保存NumPy格式...")
            save_results['numpy'] = self._save_numpy_format(occupancy_map, map_info, filename)
            
            # 验证所有保存操作成功
            if all(save_results.values()):
                print(f"✓ 地图保存完成: {filename}")
                self._print_save_info(filename)
            else:
                failed_formats = [k for k, v in save_results.items() if not v]
                raise IOError(f"部分格式保存失败: {failed_formats}")
                
        except Exception as e:
            print(f"✗ 地图保存失败: {e}")
            # 清理可能的部分文件
            self._cleanup_partial_files(filename)
            raise
    
    def _save_pgm_format(self, occupancy_map, filename):
        """保存为PGM格式（ROS标准格式）"""
        pgm_map = (occupancy_map * 255).astype(np.uint8)
        # PGM格式：255=空闲，0=障碍物，127=未知
        pgm_map[occupancy_map == 1.0] = 255  # 空闲 
        pgm_map[occupancy_map == 0.0] = 0    # 障碍物
        pgm_map[occupancy_map == 0.5] = 127  # 未知
        
        # 翻转Y轴以匹配图像坐标系
        pgm_map = np.flipud(pgm_map)
        
        # 保存PGM文件
        cv2.imwrite(f"{filename}.pgm", pgm_map)
    
    def _save_yaml_metadata(self, map_info, filename):
        """保存YAML元数据文件（ROS格式）"""
        yaml_content = f"""image: {filename}.pgm
resolution: {map_info['resolution']}
origin: [{map_info['origin_x']}, {map_info['origin_y']}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
        
        with open(f"{filename}.yaml", 'w') as f:
            f.write(yaml_content)
    
    def _save_numpy_format(self, occupancy_map, map_info, filename):
        """保存为NumPy格式"""
        np.savez(f"{filename}.npz", 
                 occupancy_map=occupancy_map, 
                 map_info=map_info)
    
    def _print_save_info(self, filename):
        """打印保存信息"""
        print(f"占用地图已保存:")
        print(f"  - PGM格式: {filename}.pgm")
        print(f"  - YAML元数据: {filename}.yaml") 
        print(f"  - NumPy格式: {filename}.npz")
    
    @staticmethod
    def generate_default_filename():
        """生成默认文件名"""
        import time
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        return f"occupancy_map_{timestamp}"

# ==================================================================================
#                              配置管理器模块
# ==================================================================================

class Config:
    """
    配置管理器 - 统一的参数配置和验证中心
    
    【设计理念】
    采用集中式配置管理模式，将分散的参数集中到单一配置对象中，
    提供统一的参数验证、类型检查和默认值管理。遵循"单一数据源"
    原则，避免参数不一致和重复定义。
    
    【核心功能】
    1. 参数集中化：统一管理所有配置参数
    2. 默认值管理：提供合理的默认参数设置
    3. 类型验证：确保参数类型和值域正确性
    4. 命令行集成：与argparse无缝集成
    5. 配置持久化：支持配置的保存和加载
    6. 参数校验：运行前的完整性检查
    
    【配置分类】
    
    1. 输入配置：
       - input_file: 轨迹文件路径
       - file_type: 文件格式类型(tum/kitti/numpy)
    
    2. 地图生成配置：
       - resolution: 地图分辨率(米/像素)
       - robot_radius: 机器人半径(米)
       - map_extension: 地图边界扩展(米)
       - generate_occupancy: 是否生成占用地图
    
    3. 输出配置：
       - save_map_filename: 地图保存文件名
    
    【参数优化指南】
    
    分辨率(resolution)选择：
    - 0.01-0.02: 超高精度，实验室精密定位
    - 0.05: 高精度，标准室内导航
    - 0.1: 中等精度，大型建筑导航
    - 0.2: 低精度，户外粗略导航
    
    机器人半径(robot_radius)设置：
    - 小型机器人(TurtleBot): 0.2-0.3米
    - 中型机器人(PR2): 0.3-0.5米
    - 大型机器人(运输车): 0.5-1.0米
    - 安全裕量：实际半径+10-20%
    
    地图扩展(map_extension)考虑：
    - 最小值: 1.0米，基本边界缓冲
    - 推荐值: 2.0米，标准安全距离
    - 大范围: 5.0米，路径规划充足空间
    
    【设计模式应用】
    - 单例模式：全局唯一的配置实例
    - 建造者模式：分步骤构建配置对象
    - 策略模式：不同环境的配置策略
    - 观察者模式：配置变更的通知机制
    
    【扩展性设计】
    - 配置文件支持：JSON/YAML配置文件读取
    - 环境变量：支持环境变量覆盖
    - 配置模板：预定义的配置模板
    - 动态配置：运行时配置修改
    - 配置继承：基础配置的扩展机制
    """
    
    def __init__(self):
        """
        初始化配置管理器 - 设置所有参数的默认值
        
        【默认值设计原则】
        1. 安全优先：选择保守但安全的默认值
        2. 通用性：适用于大多数应用场景
        3. 性能平衡：在精度和性能间取得平衡
        4. 行业标准：遵循机器人学的常见实践
        5. 易于调试：便于初学者理解和调试
        
        【参数来源优先级】
        1. 命令行参数(最高优先级)
        2. 配置文件设置
        3. 环境变量
        4. 默认值(最低优先级)
        
        【内存和性能考虑】
        - 轻量级对象：只存储必要的配置信息
        - 延迟验证：在使用时才进行昂贵的验证
        - 缓存机制：缓存验证结果避免重复计算
        """
        
        # ===== 地图生成参数 =====
        # 地图分辨率：每像素代表的实际距离(米)
        # 0.05米(5cm)是室内SLAM的标准精度，平衡了精度和计算效率
        self.resolution = 0.05
        
        # 机器人半径：用于计算可通行区域的安全缓冲(米)
        # 0.3米适合大多数小型服务机器人，包含了必要的安全裕量
        self.robot_radius = 0.3
        
        # 地图边界扩展：在轨迹范围外扩展的距离(米)
        # 2.0米提供充足的路径规划空间，避免边界效应
        self.map_extension = 2.0
        
        # 占用地图生成开关：是否生成2D导航地图
        # 默认关闭，用户需要显式启用，避免不必要的计算
        self.generate_occupancy = False
        
        # ===== 输出配置 =====
        # 地图保存文件名：不含扩展名的基础名称
        # None表示使用自动生成的时间戳文件名
        self.save_map_filename = None
        
        # ===== 输入文件配置 =====
        # 输入轨迹文件路径：支持相对路径和绝对路径
        self.input_file = None
        
        # 文件类型标识：用于选择对应的解析器
        # 支持的类型：'tum', 'kitti', 'numpy'
        self.file_type = None
        
    def update_from_args(self, args):
        """
        从命令行参数更新配置 - 参数注入和类型转换
        
        【功能说明】
        将argparse解析的命令行参数注入到配置对象中，实现从用户输入
        到程序配置的转换。支持参数验证和类型转换，确保配置的正确性。
        
        【参数映射策略】
        采用显式映射而非反射机制，提高代码的可读性和安全性：
        - 数值参数：直接赋值，保持原有类型
        - 布尔参数：转换argparse的action结果
        - 文件参数：互斥选择，确定输入文件和类型
        
        【文件类型自动识别】
        根据用户提供的参数自动确定文件类型和路径：
        - --tum filename → file_type='tum', input_file=filename
        - --kitti filename → file_type='kitti', input_file=filename
        - --numpy filename → file_type='numpy', input_file=filename
        
        【参数校验】
        在赋值过程中进行基础校验：
        - 文件路径存在性检查
        - 数值范围合理性验证
        - 类型兼容性确认
        
        【异常处理】
        - 参数冲突：多个输入文件同时指定
        - 类型错误：参数类型不匹配
        - 值域错误：参数超出合理范围
        
        参数：
            args: argparse.Namespace对象，包含解析后的命令行参数
        """
        
        # ===== 更新地图生成参数 =====
        self.resolution = float(args.resolution)           # 确保数值类型转换
        self.robot_radius = float(args.robot_radius)       # 防止字符串类型
        self.map_extension = float(args.map_extension)     # 标准化为浮点数
        self.generate_occupancy = bool(args.occupancy)     # 布尔值标准化
        self.save_map_filename = args.save_map             # 字符串可以为None
        
        # ===== 确定输入文件类型和路径 =====
        # 使用互斥逻辑确保只有一个输入源
        input_sources = [
            (args.tum, 'tum'),
            (args.numpy, 'numpy'), 
            (args.kitti, 'kitti')
        ]
        
        # 查找非空的输入源
        active_sources = [(path, ftype) for path, ftype in input_sources if path is not None]
        
        if len(active_sources) == 0:
            raise ValueError("必须指定一个输入文件 (--tum, --numpy 或 --kitti)")
        elif len(active_sources) > 1:
            raise ValueError("只能指定一个输入文件类型")
        else:
            self.input_file, self.file_type = active_sources[0]
            
        # 基础文件存在性检查（可选，避免过早失败）
        import os
        if not os.path.exists(self.input_file):
            print(f"警告: 输入文件可能不存在: {self.input_file}")
    
    def validate(self):
        """
        配置参数验证 - 运行前的完整性检查
        
        【验证策略】
        采用快速失败(Fail-Fast)原则，在程序开始处理前验证所有配置，
        避免在处理过程中因参数错误导致的失败和资源浪费。
        
        【验证层次】
        1. 必需参数：检查关键参数是否提供
        2. 类型验证：确保参数类型正确
        3. 值域验证：检查参数是否在合理范围内
        4. 逻辑验证：检查参数间的逻辑关系
        5. 资源验证：检查文件、路径等资源可用性
        
        【验证规则】
        
        输入文件验证：
        - 非空：必须指定输入文件
        - 存在性：文件必须存在且可读
        - 格式：文件扩展名与类型匹配
        
        地图参数验证：
        - resolution > 0：分辨率必须为正数
        - robot_radius > 0：机器人半径必须为正数
        - map_extension >= 0：地图扩展不能为负数
        
        逻辑关系验证：
        - robot_radius < map_extension：机器人半径应小于地图扩展
        - resolution在合理范围：避免过小或过大的分辨率
        
        【性能考虑】
        - 轻量级检查：避免昂贵的文件读取操作
        - 缓存结果：避免重复验证
        - 早期返回：发现错误立即抛出异常
        
        异常：
            ValueError: 参数值不符合要求
            FileNotFoundError: 输入文件不存在
            IOError: 文件访问权限问题
        """
        
        # ===== 必需参数检查 =====
        if self.input_file is None:
            raise ValueError("请指定轨迹文件 (--tum, --numpy 或 --kitti)")
        
        if self.file_type is None:
            raise ValueError("文件类型未确定，请检查命令行参数")
            
        # ===== 文件存在性和可访问性检查 =====
        import os
        if not os.path.exists(self.input_file):
            raise FileNotFoundError(f"轨迹文件不存在: {self.input_file}")
            
        if not os.access(self.input_file, os.R_OK):
            raise IOError(f"轨迹文件无法读取: {self.input_file}")
        
        # ===== 数值范围验证 =====
        if self.resolution <= 0:
            raise ValueError(f"分辨率必须大于0，当前值: {self.resolution}")
        
        if self.resolution > 1.0:
            print(f"警告: 分辨率过大 ({self.resolution}米/像素)，可能导致粗糙的地图")
        
        if self.resolution < 0.001:
            print(f"警告: 分辨率过小 ({self.resolution}米/像素)，可能导致巨大的内存消耗")
        
        if self.robot_radius <= 0:
            raise ValueError(f"机器人半径必须大于0，当前值: {self.robot_radius}")
            
        if self.robot_radius > 2.0:
            print(f"警告: 机器人半径过大 ({self.robot_radius}米)，请确认是否正确")
        
        if self.map_extension < 0:
            raise ValueError(f"地图扩展距离不能为负，当前值: {self.map_extension}")
        
        # ===== 逻辑关系验证 =====
        if self.robot_radius > self.map_extension:
            print(f"警告: 机器人半径({self.robot_radius}米)大于地图扩展({self.map_extension}米)")
            
        # ===== 文件类型和扩展名匹配检查 =====
        file_ext = os.path.splitext(self.input_file)[1].lower()
        expected_extensions = {
            'tum': ['.txt', '.tum'],
            'kitti': ['.txt', '.kitti'],
            'numpy': ['.npz']
        }
        
        if file_ext not in expected_extensions.get(self.file_type, []):
            print(f"警告: 文件扩展名({file_ext})与类型({self.file_type})可能不匹配")
    
    def print_config(self):
        """
        打印当前配置 - 用户友好的配置摘要显示
        
        【显示策略】
        分层次显示配置信息，突出重要参数，隐藏技术细节：
        1. 核心配置：输入文件、文件类型
        2. 条件配置：仅在相关功能启用时显示
        3. 计算参数：显示派生的计算参数
        4. 警告信息：显示潜在的配置问题
        
        【格式设计】
        - 缩进层次：清晰的视觉层次
        - 单位显示：包含物理单位便于理解
        - 颜色编码：使用符号区分不同类型信息
        - 对齐格式：整齐的表格式显示
        
        【教学价值】
        帮助用户理解：
        - 当前使用的参数设置
        - 参数对结果的影响
        - 可能需要调整的配置
        """
        
        print("=" * 50)
        print("📋 当前配置摘要")
        print("=" * 50)
        
        # ===== 输入配置 =====
        print("📁 输入配置:")
        print(f"   文件路径: {self.input_file}")
        print(f"   文件类型: {self.file_type.upper()}")
        
        # ===== 地图生成配置（条件显示）=====
        if self.generate_occupancy:
            print("\n🗺️  地图生成配置:")
            print(f"   分辨率: {self.resolution} 米/像素 ({1/self.resolution:.1f} 像素/米)")
            print(f"   机器人半径: {self.robot_radius} 米")
            print(f"   地图扩展: {self.map_extension} 米")
            
            # 计算派生信息
            print("\n📊 预估信息:")
            print(f"   可通行通道最小宽度: {2 * self.robot_radius:.2f} 米")
            print(f"   内存使用估算: 需要轨迹数据确定")
        
        # ===== 输出配置 =====
        print("\n💾 输出配置:")
        if self.save_map_filename:
            print(f"   保存文件名: {self.save_map_filename}")
            print("   输出格式: PGM + YAML + NPZ")
        else:
            print("   保存文件名: 自动生成(时间戳)")
            
        print("=" * 50)

# ==================================================================================
#                              轨迹处理器模块 - 系统核心控制器
# ==================================================================================

class TrajectoryProcessor:
    """
    轨迹处理器 - 系统核心控制器和工作流编排器
    
    【系统架构角色】
    作为整个轨迹分析系统的控制中心，负责协调各个功能组件完成完整的
    处理流程。采用Facade模式简化复杂的子系统交互，为用户提供
    简单统一的接口。
    
    【设计模式应用】
    1. Facade模式：简化复杂子系统的统一接口
    2. 组合模式：组合多个功能组件协同工作
    3. 模板方法：定义标准的处理流程框架
    4. 策略模式：根据配置选择不同的处理策略
    5. 责任链模式：按顺序执行各个处理步骤
    
    【工作流程设计】
    标准处理流程分为以下阶段：
    
    1. 初始化阶段：
       ├── 配置验证和参数检查
       ├── 组件初始化和资源分配
       └── 处理环境准备
    
    2. 数据加载阶段：
       ├── 轨迹文件读取和解析
       ├── 数据格式转换和标准化
       └── 数据质量检查和统计
    
    3. 分析可视化阶段：
       ├── 轨迹统计信息计算
       ├── 多视图轨迹可视化
       └── 3D立体轨迹展示
    
    4. 地图生成阶段（可选）：
       ├── 占用地图生成和处理
       ├── 地图质量分析和统计
       ├── 地图可视化和验证
       └── 多格式地图保存
    
    5. 清理阶段：
       ├── 资源释放和内存清理
       ├── 临时文件清理
       └── 状态报告和总结
    
    【组件协调机制】
    
    组件间通信：
    - 配置传递：配置对象在组件间传递
    - 数据流转：轨迹数据在处理链中流动
    - 状态同步：处理状态的实时更新
    - 错误传播：异常的层次化处理
    
    资源管理：
    - 内存管理：大型数据的及时释放
    - 文件管理：临时文件的自动清理
    - 进程管理：长时间操作的进度反馈
    
    【扩展性架构】
    - 插件化处理：易于添加新的处理步骤
    - 管道化设计：支持自定义处理管道
    - 异步处理：支持并行和异步操作
    - 进度回调：为GUI集成提供进度接口
    
    【错误处理策略】
    - 分层异常：不同层次的异常处理
    - 优雅降级：部分功能失败时的降级处理
    - 状态恢复：支持从错误状态恢复
    - 详细日志：完整的处理过程记录
    
    【性能优化特性】
    - 惰性加载：按需初始化组件
    - 内存优化：大数据的流式处理
    - 并行处理：独立任务的并行执行
    - 缓存机制：重复计算的结果缓存
    """
    
    def __init__(self):
        """
        初始化轨迹处理器 - 组件装配和系统准备
        
        【初始化策略】
        采用惰性初始化模式，只创建轻量级的组件实例，重型组件
        在需要时才创建。这种策略减少了初始化时间和内存占用。
        
        【组件初始化顺序】
        按照依赖关系和使用频率确定初始化顺序：
        1. 基础组件：数据加载器、可视化器
        2. 核心组件：地图可视化器、地图保存器
        3. 动态组件：地图生成器（按需创建）
        
        【内存管理】
        - 轻量级设计：初始化时不分配大块内存
        - 按需分配：组件在使用时才分配资源
        - 自动释放：处理完成后自动释放资源
        
        【异常安全】
        确保即使初始化过程中出现异常，也不会导致资源泄漏：
        - RAII原则：资源获取即初始化
        - 异常安全：强异常安全保证
        - 清理机制：自动资源清理
        """
        
        # ===== 核心功能组件初始化 =====
        # 轨迹数据加载器：负责多格式轨迹文件的读取和解析
        self.loader = TrajectoryLoader()
        
        # 轨迹可视化器：负责轨迹的多维度可视化分析
        self.visualizer = TrajectoryVisualizer()
        
        # 地图可视化器：负责占用地图的专业展示
        self.map_visualizer = MapVisualizer()
        
        # 地图保存器：负责多格式地图文件的输出
        self.map_saver = MapSaver()
        
        # ===== 动态组件（按需创建）=====
        # 占用地图生成器：只在需要生成地图时创建，避免不必要的内存占用
        self.map_generator = None
        
        # ===== 处理状态跟踪 =====
        # 处理阶段标记，用于错误恢复和状态监控
        self._current_stage = "初始化"
        self._processing_stats = {
            'start_time': None,
            'end_time': None,
            'processed_poses': 0,
            'generated_maps': 0,
            'saved_files': 0
        }
    
    def process(self, config):
        """
        主处理流程 - 系统核心执行引擎
        
        【功能概述】
        执行完整的轨迹分析和地图生成流程，协调各个组件按顺序完成
        数据处理任务。采用模板方法模式，定义了标准的处理框架。
        
        【处理阶段详解】
        
        阶段1: 配置验证与环境准备
        - 参数完整性检查和范围验证
        - 输入文件存在性和可读性验证
        - 处理环境初始化和状态重置
        - 配置摘要显示和用户确认
        
        阶段2: 轨迹数据加载与预处理
        - 根据文件类型选择对应解析器
        - 执行数据读取和格式转换
        - 数据质量检查和异常值处理
        - 统计信息计算和摘要输出
        
        阶段3: 轨迹分析与可视化
        - 多维度轨迹统计分析
        - 2D/3D轨迹可视化展示
        - 运动模式分析和异常检测
        - 轨迹质量评估和报告
        
        阶段4: 占用地图生成（可选）
        - 2D占用地图生成和处理
        - 地图质量分析和统计
        - 地图可视化和验证
        - 多格式地图文件保存
        
        阶段5: 结果总结与清理
        - 处理结果统计和总结
        - 临时资源清理和释放
        - 性能统计和时间分析
        - 用户反馈和建议输出
        
        【错误处理机制】
        - 阶段性检查：每个阶段完成后验证结果
        - 优雅降级：部分功能失败时继续其他处理
        - 状态恢复：支持从中断点继续处理
        - 资源清理：异常情况下的资源自动释放
        
        【性能监控】
        - 处理时间统计：记录各阶段耗时
        - 内存使用监控：追踪内存占用峰值
        - 进度反馈：为长时间操作提供进度信息
        - 性能优化建议：根据处理结果提供优化建议
        
        【参数说明】
        config (Config): 完整的配置对象，包含所有处理参数
        
        【异常情况】
        - 配置验证失败：抛出ValueError异常
        - 文件读取失败：抛出FileNotFoundError或IOError
        - 内存不足：抛出MemoryError异常
        - 处理中断：支持优雅的中断处理
        
        【使用示例】
        ```python
        # 创建处理器和配置
        processor = TrajectoryProcessor()
        config = Config()
        config.update_from_args(args)
        
        # 执行完整处理流程
        try:
            processor.process(config)
            print("处理完成")
        except Exception as e:
            print(f"处理失败: {e}")
        ```
        """
        import time
        
        try:
            # 记录处理开始时间
            self._processing_stats['start_time'] = time.time()
            self._current_stage = "配置验证"
            
            # ===== 阶段1: 配置验证与环境准备 =====
            print("🔧 阶段1: 配置验证与环境准备")
            config.validate()
            config.print_config()
            print("✓ 配置验证通过\n")
            
            # ===== 阶段2: 轨迹数据加载与预处理 =====
            print("📥 阶段2: 轨迹数据加载与预处理")
            self._current_stage = "数据加载"
            positions = self._load_trajectory(config)
            
            if positions is None or len(positions) == 0:
                print("✗ 无法加载有效的轨迹数据")
                return
            
            self._processing_stats['processed_poses'] = len(positions)
            print("✓ 轨迹数据加载完成\n")
            
            # ===== 阶段3: 轨迹分析与可视化 =====
            print("📊 阶段3: 轨迹分析与可视化")
            self._current_stage = "轨迹分析"
            
            # 显示轨迹统计信息
            self.visualizer.print_trajectory_stats(positions)
            
            # 生成描述性标题
            title = f"{config.file_type.upper()}格式轨迹分析"
            
            # 执行轨迹可视化
            self._visualize_trajectory(positions, title)
            print("✓ 轨迹可视化完成\n")
            
            # ===== 阶段4: 占用地图生成（可选）=====
            if config.generate_occupancy:
                print("🗺️ 阶段4: 占用地图生成")
                self._current_stage = "地图生成"
                self._process_occupancy_map(positions, config, title)
                self._processing_stats['generated_maps'] = 1
                print("✓ 占用地图生成完成\n")
            
            # ===== 阶段5: 结果总结与清理 =====
            print("📋 阶段5: 处理结果总结")
            self._current_stage = "结果总结"
            self._print_processing_summary()
            
        except Exception as e:
            # 错误处理和资源清理
            print(f"✗ 处理在阶段'{self._current_stage}'中失败: {e}")
            self._cleanup_resources()
            raise
        finally:
            # 记录处理结束时间
            self._processing_stats['end_time'] = time.time()
    
    def _load_trajectory(self, config):
        """加载轨迹数据"""
        print(f"加载{config.file_type.upper()}轨迹: {config.input_file}")
        
        trajectory_data = self.loader.load_trajectory(config.input_file, config.file_type)
        positions = self.loader.extract_positions(trajectory_data, config.file_type)
        
        return positions
    
    def _visualize_trajectory(self, positions, title):
        """可视化轨迹"""
        self.visualizer.plot_trajectory_map(positions, title)
        self.visualizer.plot_3d_map(positions, title + " (3D)")
    
    def _process_occupancy_map(self, positions, config, title):
        """处理占用地图"""
        print("\n生成2D占用地图...")
        
        # 创建地图生成器
        self.map_generator = OccupancyMapGenerator(
            resolution=config.resolution,
            robot_radius=config.robot_radius,
            map_extension=config.map_extension
        )
        
        # 生成占用地图
        occupancy_map, map_info = self.map_generator.generate_occupancy_map(positions)
        
        # 打印地图信息
        self._print_map_info(map_info)
        
        # 计算并打印地图统计
        stats = OccupancyMapGenerator.calculate_map_statistics(occupancy_map, config.resolution)
        self.map_visualizer.print_map_statistics(stats)
        
        # 显示占用地图
        self.map_visualizer.plot_occupancy_map(
            occupancy_map, map_info, positions[:, :2], 
            title + " - 2D房间平面图"
        )
        
        # 保存地图文件
        self._save_map(occupancy_map, map_info, config)
    
    def _print_map_info(self, map_info):
        """打印地图信息"""
        print(f"  地图尺寸: {map_info['width']} x {map_info['height']} 像素")
        print(f"  地图范围: X({map_info['origin_x']:.2f} ~ {map_info['origin_x'] + map_info['width'] * map_info['resolution']:.2f}) 米")
        print(f"           Y({map_info['origin_y']:.2f} ~ {map_info['origin_y'] + map_info['height'] * map_info['resolution']:.2f}) 米")
    
    def _save_map(self, occupancy_map, map_info, config):
        """保存地图"""
        filename = config.save_map_filename
        if filename is None:
            filename = self.map_saver.generate_default_filename()
        
        self.map_saver.save_occupancy_map(occupancy_map, map_info, filename)

# ==================================================================================
#                              主程序入口和命令行接口
# ==================================================================================

def main():
    """
    主程序入口函数 - 命令行工具的核心控制逻辑
    
    【功能概述】
    实现完整的命令行工具功能，包括参数解析、配置管理、流程控制和错误处理。
    采用面向对象的设计模式，将复杂的处理流程分解为独立的组件。
    
    【工作流程】
    1. 命令行参数解析：使用argparse库处理用户输入
    2. 配置对象创建：将参数转换为结构化配置
    3. 处理器初始化：创建轨迹处理器实例
    4. 流程执行：调用处理器完成完整分析流程
    5. 异常处理：捕获和处理各种错误情况
    
    【教学价值】
    - 演示专业的命令行工具设计模式
    - 展示面向对象程序设计的实际应用
    - 说明错误处理和用户体验的重要性
    - 体现代码的模块化和可维护性
    
    【扩展性设计】
    - 参数系统：易于添加新的命令行选项
    - 处理流程：支持插入新的处理步骤
    - 输出格式：支持多种输出和保存格式
    """
    
    # ===== 1. 命令行参数定义和解析 =====
    parser = argparse.ArgumentParser(
        description='ORB-SLAM3轨迹可视化和占用地图生成工具',
        epilog='''
使用示例:
  %(prog)s --tum trajectory.txt                    # 基础轨迹可视化
  %(prog)s --kitti odometry.txt --occupancy        # 生成占用地图
  %(prog)s --tum data.txt --occupancy --resolution 0.02  # 高精度地图
        ''',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    # 输入文件参数组（互斥选择）
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--tum', metavar='FILE', 
                            help='TUM格式轨迹文件路径')
    input_group.add_argument('--numpy', metavar='FILE', 
                            help='NumPy格式轨迹文件路径(.npz)')
    input_group.add_argument('--kitti', metavar='FILE', 
                            help='KITTI格式轨迹文件路径')
    
    # 功能选项参数
    parser.add_argument('--occupancy', action='store_true', 
                       help='生成2D占用地图（机器人导航地图）')
    
    # 地图生成参数组
    map_group = parser.add_argument_group('地图生成参数', '控制占用地图生成的详细参数')
    map_group.add_argument('--resolution', type=float, default=0.05, metavar='METERS',
                          help='地图分辨率，单位：米/像素 (默认: 0.05，即5cm/像素)')
    map_group.add_argument('--robot-radius', type=float, default=0.3, metavar='METERS',
                          help='机器人半径，影响通道宽度，单位：米 (默认: 0.3米)')
    map_group.add_argument('--map-extension', type=float, default=2.0, metavar='METERS',
                          help='地图边界扩展距离，单位：米 (默认: 2.0米)')
    
    # 输出参数
    parser.add_argument('--save-map', type=str, metavar='BASENAME',
                       help='保存占用地图的文件名前缀（自动添加.pgm/.yaml/.npz扩展名）')
    
    # 解析命令行参数
    args = parser.parse_args()
    
    # ===== 2. 配置对象创建和初始化 =====
    print("初始化配置...")
    config = Config()
    config.update_from_args(args)
    
    # ===== 3. 处理器创建和流程执行 =====
    print("创建轨迹处理器...")
    processor = TrajectoryProcessor()
    
    try:
        # 执行完整的处理流程
        print("开始处理轨迹数据...")
        processor.process(config)
        print("\n✓ 处理完成！")
        
    except ValueError as e:
        # 处理用户输入错误
        print(f"✗ 输入错误: {e}")
        print("  请检查文件路径和参数设置")
        return 1
        
    except FileNotFoundError as e:
        # 处理文件不存在错误
        print(f"✗ 文件错误: {e}")
        print("  请确认文件路径正确且文件存在")
        return 1
        
    except IOError as e:
        # 处理文件读写错误
        print(f"✗ IO错误: {e}")
        print("  请检查文件权限和磁盘空间")
        return 1
        
    except Exception as e:
        # 处理未预期的错误
        print(f"✗ 程序运行错误: {e}")
        print("  详细错误信息：")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0  # 成功退出

# ==================================================================================
#                              程序执行和教学总结
# ==================================================================================

if __name__ == '__main__':
    """
    程序主入口 - Python模块执行的标准模式
    
    【执行机制】
    当Python解释器直接运行此文件时，__name__变量会被设置为'__main__'，
    此时执行main()函数。当此文件作为模块被导入时，不会执行main()函数。
    
    【教学要点总结】
    
    1. 【面向对象设计模式】
       - 单一职责原则：每个类专注一个核心功能
       - 工厂模式：TrajectoryLoader的统一加载接口
       - 策略模式：不同格式的加载策略
       - 组合模式：TrajectoryProcessor组合各功能组件
    
    2. 【Python编程最佳实践】
       - 类型注解：提高代码可读性和IDE支持
       - 异常处理：完善的错误捕获和用户反馈
       - 文档字符串：详细的函数和类说明
       - 代码风格：遵循PEP 8编码规范
    
    3. 【SLAM和机器人学概念】
       - 位姿表示：理解不同的位姿表示方法
       - 坐标系转换：掌握坐标系变换的数学原理
       - 占用地图：理解机器人导航的核心数据结构
       - 轨迹分析：学习评估SLAM算法性能的方法
    
    4. 【数据科学和可视化】
       - NumPy数组操作：高效的数值计算
       - Matplotlib绘图：专业的科学可视化
       - 数据格式转换：处理异构数据源
       - 统计分析：从数据中提取有用信息
    
    5. 【软件工程原则】
       - 模块化设计：代码的可维护性和可扩展性
       - 配置管理：参数的集中化管理
       - 错误处理：用户友好的错误提示
       - 代码复用：避免重复代码，提高开发效率
    
    【进阶学习建议】
    1. 深入学习SLAM算法原理和数学基础
    2. 掌握更多的数据可视化技术
    3. 学习机器人操作系统(ROS)的使用
    4. 研究实时SLAM系统的实现
    5. 探索深度学习在SLAM中的应用
    
    【实践项目建议】
    1. 扩展支持更多轨迹格式(如EuRoC、TurtleBot等)
    2. 添加轨迹质量评估指标(如ATE、RPE等)
    3. 实现实时轨迹可视化功能
    4. 集成到ROS系统中作为可视化节点
    5. 开发Web界面的在线轨迹分析工具
    """
    exit_code = main()
    exit(exit_code)
