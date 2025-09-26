#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ORB-SLAM3轨迹可视化和占用地图生成工具

支持的轨迹格式：
- TUM格式：timestamp tx ty tz qx qy qz qw
- KITTI格式：3x4变换矩阵的12个元素
- NumPy格式：.npz文件包含poses, timestamps, frame_ids

功能特性：
1. 轨迹可视化：3D轨迹、俯视图、侧视图、高度编码地图
2. 占用地图生成：从轨迹数据生成2D房间平面图，显示可行走区域
3. 多种输出格式：PGM（ROS标准）、YAML元数据、NumPy

使用方法：
# 基本轨迹可视化
python visualize_trajectory.py --tum trajectory.txt
python visualize_trajectory.py --kitti trajectory.txt
python visualize_trajectory.py --numpy trajectory.npz

# 生成占用地图（2D房间平面图）
python visualize_trajectory.py --tum trajectory.txt --occupancy
python visualize_trajectory.py --kitti trajectory.txt --occupancy --resolution 0.02 --robot-radius 0.4

# 自定义参数生成占用地图
python visualize_trajectory.py --tum trajectory.txt --occupancy \\
    --resolution 0.05 \\           # 地图分辨率（米/像素）
    --robot-radius 0.3 \\          # 机器人半径（米）
    --map-extension 1.5 \\         # 地图边界扩展（米）
    --save-map room_map            # 保存地图文件
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

# 配置中文字体
def setup_chinese_font():
    """配置matplotlib中文字体"""
    # 获取系统所有可用字体
    try:
        font_list = [f.name for f in fm.fontManager.ttflist]
        
        # 查找支持中文的字体（按优先级排序）
        chinese_font_keywords = [
            'PingFang', 'Heiti', 'STHeiti', 'STFangsong', 'STSong', 'STKaiti',
            'SimHei', 'Microsoft YaHei', 'KaiTi', 'FangSong', 'SimSun',
            'WenQuanYi', 'Noto Sans CJK', 'Noto Serif CJK', 'Source Han',
            'Arial Unicode MS'
        ]
        
        chinese_fonts_available = []
        for keyword in chinese_font_keywords:
            for font_name in font_list:
                if keyword in font_name and font_name not in chinese_fonts_available:
                    chinese_fonts_available.append(font_name)
        
        # 尝试设置第一个可用的中文字体
        if chinese_fonts_available:
            selected_font = chinese_fonts_available[0]
            plt.rcParams['font.sans-serif'] = [selected_font] + plt.rcParams['font.sans-serif']
            plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
            print(f"检测到并使用中文字体: {selected_font}")
            
            # 验证字体是否真正可用（简单测试）
            
            return True
        else:
            # 备用方案：使用系统默认字体并警告
            print("警告: 未找到中文字体，将使用系统默认字体")
            print("可用字体列表前10个:", font_list[:10])
            plt.rcParams['axes.unicode_minus'] = False
            return False
            
    except Exception as e:
        print(f"字体配置出错: {e}")
        plt.rcParams['axes.unicode_minus'] = False
        return False

# 初始化中文字体
setup_chinese_font()

def load_tum_trajectory(filename):
    """加载TUM格式轨迹文件"""
    data = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) >= 8:
                timestamp = float(parts[0])
                tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                data.append([timestamp, tx, ty, tz, qx, qy, qz, qw])
    return np.array(data)

def load_numpy_trajectory(filename):
    """加载NumPy格式轨迹文件"""
    data = np.load(filename)
    return data['poses'], data['timestamps'], data['frame_ids']

def load_kitti_trajectory(filename):
    """加载KITTI格式轨迹文件
    
    KITTI格式说明：
    - 每行包含12个浮点数，表示3x4变换矩阵
    - 格式：r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
    - 表示从第0帧到当前帧的累积变换
    """
    poses = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) >= 12:
                # 解析12个元素为3x4变换矩阵
                values = [float(x) for x in parts[:12]]
                # 重构为3x4矩阵
                transform_matrix = np.array([
                    [values[0], values[1], values[2], values[3]],   # [r11 r12 r13 tx]
                    [values[4], values[5], values[6], values[7]],   # [r21 r22 r23 ty]
                    [values[8], values[9], values[10], values[11]]  # [r31 r32 r33 tz]
                ])
                poses.append(transform_matrix)
    
    if len(poses) == 0:
        return np.array([])
    
    # 转换为numpy数组，形状为(N, 3, 4)
    poses_array = np.array(poses)
    # 提取位置信息（最后一列）
    positions = poses_array[:, :, 3]  # 形状为(N, 3)
    
    return positions

def plot_trajectory_map(positions, title="相机轨迹地图"):
    """绘制轨迹地图"""
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle(title, fontsize=16)
    
    # 1. 2D俯视图
    axes[0, 0].plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='轨迹')
    axes[0, 0].plot(positions[0, 0], positions[0, 1], 'go', markersize=10, label='起点')
    axes[0, 0].plot(positions[-1, 0], positions[-1, 1], 'ro', markersize=10, label='终点')
    axes[0, 0].scatter(positions[::20, 0], positions[::20, 1], c='red', s=20, alpha=0.5)
    axes[0, 0].set_xlabel('X (米)')
    axes[0, 0].set_ylabel('Y (米)')
    axes[0, 0].set_title('俯视图')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axis('equal')
    
    # 2. 侧视图 (XZ)
    axes[0, 1].plot(positions[:, 0], positions[:, 2], 'r-', linewidth=2)
    axes[0, 1].plot(positions[0, 0], positions[0, 2], 'go', markersize=10, label='起点')
    axes[0, 1].plot(positions[-1, 0], positions[-1, 2], 'ro', markersize=10, label='终点')
    axes[0, 1].set_xlabel('X (米)')
    axes[0, 1].set_ylabel('Z (米)')
    axes[0, 1].set_title('侧视图 (XZ)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # 3. 高度编码的轨迹
    colors = plt.cm.viridis((positions[:, 2] - positions[:, 2].min()) / 
                           (positions[:, 2].max() - positions[:, 2].min() + 1e-8))
    scatter = axes[1, 0].scatter(positions[:, 0], positions[:, 1], c=positions[:, 2], 
                                cmap='viridis', s=15, alpha=0.8)
    axes[1, 0].plot(positions[0, 0], positions[0, 1], 'wo', markersize=8, label='起点')
    axes[1, 0].plot(positions[-1, 0], positions[-1, 1], 'ko', markersize=8, label='终点')
    plt.colorbar(scatter, ax=axes[1, 0], label='高度 Z (米)')
    axes[1, 0].set_xlabel('X (米)')
    axes[1, 0].set_ylabel('Y (米)')
    axes[1, 0].set_title('高度编码地图')
    axes[1, 0].legend()
    axes[1, 0].axis('equal')
    
    # 4. 速度分析
    if len(positions) > 1:
        velocities = np.diff(positions, axis=0)
        speeds = np.linalg.norm(velocities, axis=1)
        axes[1, 1].plot(range(len(speeds)), speeds, 'purple', linewidth=2)
        axes[1, 1].set_xlabel('时间步')
        axes[1, 1].set_ylabel('速度 (米/步)')
        axes[1, 1].set_title('运动速度')
        axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def generate_occupancy_map(positions, resolution=0.05, robot_radius=0.3, map_extension=2.0):
    """
    从轨迹数据生成占用地图（2D房间平面图）
    
    参数：
        positions: 相机/机器人位置数组 (N, 3)
        resolution: 地图分辨率（米/像素）
        robot_radius: 机器人半径（米）
        map_extension: 地图边界扩展距离（米）
    
    返回：
        occupancy_map: 占用地图 (0=占用, 1=空闲, 0.5=未知)
        map_info: 地图元信息字典
    """
    # 提取XY平面的轨迹点
    trajectory_2d = positions[:, :2]  # 只取x, y坐标
    
    # 计算地图边界
    min_x, min_y = trajectory_2d.min(axis=0) - map_extension
    max_x, max_y = trajectory_2d.max(axis=0) + map_extension
    
    # 计算地图尺寸
    width = int((max_x - min_x) / resolution) + 1
    height = int((max_y - min_y) / resolution) + 1
    
    # 初始化占用地图（0.5表示未知区域）
    occupancy_map = np.full((height, width), 0.5, dtype=np.float32)
    
    # 将轨迹点转换为像素坐标
    trajectory_pixels = np.zeros((len(trajectory_2d), 2), dtype=int)
    trajectory_pixels[:, 0] = ((trajectory_2d[:, 0] - min_x) / resolution).astype(int)
    trajectory_pixels[:, 1] = ((trajectory_2d[:, 1] - min_y) / resolution).astype(int)
    
    # 确保像素坐标在地图范围内
    trajectory_pixels[:, 0] = np.clip(trajectory_pixels[:, 0], 0, width - 1)
    trajectory_pixels[:, 1] = np.clip(trajectory_pixels[:, 1], 0, height - 1)
    
    # 标记轨迹经过的区域为空闲区域
    for i in range(len(trajectory_pixels)):
        x, y = trajectory_pixels[i]
        # 在轨迹点周围创建圆形空闲区域
        radius_pixels = int(robot_radius / resolution)
        y_coords, x_coords = np.ogrid[:height, :width]
        mask = (x_coords - x)**2 + (y_coords - y)**2 <= radius_pixels**2
        occupancy_map[mask] = 1.0  # 空闲区域
    
    # 连接轨迹点之间的路径
    for i in range(len(trajectory_pixels) - 1):
        x1, y1 = trajectory_pixels[i]
        x2, y2 = trajectory_pixels[i + 1]
        
        # 使用Bresenham算法连接两点
        line_points = bresenham_line(x1, y1, x2, y2)
        for px, py in line_points:
            if 0 <= px < width and 0 <= py < height:
                # 在线段周围创建空闲走廊
                radius_pixels = int(robot_radius / resolution)
                y_coords, x_coords = np.ogrid[:height, :width]
                mask = (x_coords - px)**2 + (y_coords - py)**2 <= radius_pixels**2
                occupancy_map[mask] = 1.0
    
    # 使用形态学操作平滑地图
    # 先膨胀再腐蚀以填充小洞
    kernel_size = max(1, int(robot_radius / resolution / 2))
    kernel = disk(kernel_size)
    
    # 处理空闲区域
    free_areas = (occupancy_map == 1.0)
    free_areas = binary_dilation(free_areas, kernel)
    free_areas = binary_erosion(free_areas, kernel)
    
    # 更新地图
    occupancy_map[free_areas] = 1.0
    
    # 基于轨迹边界推断障碍物
    # 在地图边界附近设置障碍物
    border_width = int(0.5 / resolution)  # 50cm边界
    occupancy_map[:border_width, :] = 0.0  # 顶部边界
    occupancy_map[-border_width:, :] = 0.0  # 底部边界
    occupancy_map[:, :border_width] = 0.0  # 左侧边界
    occupancy_map[:, -border_width:] = 0.0  # 右侧边界
    
    # 地图元信息
    map_info = {
        'resolution': resolution,
        'width': width,
        'height': height,
        'origin_x': min_x,
        'origin_y': min_y,
        'robot_radius': robot_radius
    }
    
    return occupancy_map, map_info

def bresenham_line(x0, y0, x1, y1):
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

def plot_occupancy_map(occupancy_map, map_info, trajectory_2d=None, title="2D房间平面图"):
    """
    可视化占用地图
    
    参数：
        occupancy_map: 占用地图数组
        map_info: 地图元信息
        trajectory_2d: 轨迹2D坐标（可选）
        title: 图像标题
    """
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    
    # 创建颜色地图：黑色=障碍物，白色=空闲，灰色=未知
    colored_map = np.zeros((occupancy_map.shape[0], occupancy_map.shape[1], 3))
    colored_map[occupancy_map == 0.0] = [0, 0, 0]      # 黑色 - 障碍物
    colored_map[occupancy_map == 1.0] = [1, 1, 1]      # 白色 - 空闲区域
    colored_map[occupancy_map == 0.5] = [0.5, 0.5, 0.5]  # 灰色 - 未知区域
    
    # 显示地图（注意Y轴翻转以匹配常规地图显示）
    ax.imshow(colored_map, origin='lower', extent=[
        map_info['origin_x'], 
        map_info['origin_x'] + map_info['width'] * map_info['resolution'],
        map_info['origin_y'], 
        map_info['origin_y'] + map_info['height'] * map_info['resolution']
    ])
    
    # 叠加轨迹
    if trajectory_2d is not None:
        ax.plot(trajectory_2d[:, 0], trajectory_2d[:, 1], 'r-', linewidth=2, 
                label='机器人轨迹', alpha=0.8)
        ax.plot(trajectory_2d[0, 0], trajectory_2d[0, 1], 'go', markersize=10, 
                label='起点')
        ax.plot(trajectory_2d[-1, 0], trajectory_2d[-1, 1], 'ro', markersize=10, 
                label='终点')
    
    ax.set_xlabel('X坐标 (米)')
    ax.set_ylabel('Y坐标 (米)')
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # 添加色条说明
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='white', edgecolor='black', label='可行走区域'),
        Patch(facecolor='black', edgecolor='black', label='障碍物/墙壁'),
        Patch(facecolor='gray', edgecolor='black', label='未探索区域'),
        Patch(facecolor='red', edgecolor='red', label='机器人轨迹')
    ]
    ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1, 1))
    
    plt.tight_layout()
    plt.show()

def save_occupancy_map(occupancy_map, map_info, filename):
    """
    保存占用地图为常用格式
    
    参数：
        occupancy_map: 占用地图数组
        map_info: 地图元信息
        filename: 输出文件名（不含扩展名）
    """
    # 保存为PGM格式（ROS标准格式）
    pgm_map = (occupancy_map * 255).astype(np.uint8)
    # PGM格式：255=空闲，0=障碍物，127=未知
    pgm_map[occupancy_map == 1.0] = 255  # 空闲 
    pgm_map[occupancy_map == 0.0] = 0    # 障碍物
    pgm_map[occupancy_map == 0.5] = 127  # 未知
    
    # 翻转Y轴以匹配图像坐标系
    pgm_map = np.flipud(pgm_map)
    
    # 保存PGM文件
    cv2.imwrite(f"{filename}.pgm", pgm_map)
    
    # 保存YAML元数据文件（ROS格式）
    yaml_content = f"""image: {filename}.pgm
resolution: {map_info['resolution']}
origin: [{map_info['origin_x']}, {map_info['origin_y']}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    
    with open(f"{filename}.yaml", 'w') as f:
        f.write(yaml_content)
    
    # 保存为NumPy格式
    np.savez(f"{filename}.npz", 
             occupancy_map=occupancy_map, 
             map_info=map_info)
    
    print(f"占用地图已保存:")
    print(f"  - PGM格式: {filename}.pgm")
    print(f"  - YAML元数据: {filename}.yaml") 
    print(f"  - NumPy格式: {filename}.npz")

def plot_3d_map(positions, title="3D轨迹地图"):
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

def main():
    parser = argparse.ArgumentParser(description='轨迹可视化和占用地图生成工具')
    parser.add_argument('--tum', help='TUM格式轨迹文件')
    parser.add_argument('--numpy', help='NumPy格式轨迹文件')
    parser.add_argument('--kitti', help='KITTI格式轨迹文件')
    parser.add_argument('--occupancy', action='store_true', 
                       help='生成2D占用地图（房间平面图）')
    parser.add_argument('--resolution', type=float, default=0.05, 
                       help='地图分辨率（米/像素，默认0.05）')
    parser.add_argument('--robot-radius', type=float, default=0.3, 
                       help='机器人半径（米，默认0.3）')
    parser.add_argument('--map-extension', type=float, default=2.0, 
                       help='地图边界扩展距离（米，默认2.0）')
    parser.add_argument('--save-map', type=str, 
                       help='保存占用地图文件名（不含扩展名）')
    
    args = parser.parse_args()
    
    positions = None
    
    if args.tum:
        print(f"加载TUM轨迹: {args.tum}")
        tum_data = load_tum_trajectory(args.tum)
        positions = tum_data[:, 1:4]  # 提取位置
        title = "TUM轨迹地图"
    elif args.numpy:
        print(f"加载NumPy轨迹: {args.numpy}")
        poses, timestamps, frame_ids = load_numpy_trajectory(args.numpy)
        positions = poses[:, :3, 3]  # 提取位置
        title = "NumPy轨迹地图"
    elif args.kitti:
        print(f"加载KITTI轨迹: {args.kitti}")
        positions = load_kitti_trajectory(args.kitti)
        title = "KITTI轨迹地图"
    else:
        print("请指定轨迹文件 (--tum, --numpy 或 --kitti)")
        return
    
    if positions is None or len(positions) == 0:
        print("无法加载轨迹数据")
        return
    
    print(f"轨迹统计:")
    print(f"  位姿数量: {len(positions)}")
    print(f"  总距离: {np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)):.2f} 米")
    print(f"  起点: ({positions[0, 0]:.2f}, {positions[0, 1]:.2f}, {positions[0, 2]:.2f})")
    print(f"  终点: ({positions[-1, 0]:.2f}, {positions[-1, 1]:.2f}, {positions[-1, 2]:.2f})")
    
    # 显示轨迹地图
    plot_trajectory_map(positions, title)
    plot_3d_map(positions, title + " (3D)")
    
    # 生成占用地图
    if args.occupancy:
        print("\n生成2D占用地图...")
        print(f"  分辨率: {args.resolution} 米/像素")
        print(f"  机器人半径: {args.robot_radius} 米")
        print(f"  地图扩展: {args.map_extension} 米")
        
        # 生成占用地图
        occupancy_map, map_info = generate_occupancy_map(
            positions, 
            resolution=args.resolution,
            robot_radius=args.robot_radius,
            map_extension=args.map_extension
        )
        
        print(f"  地图尺寸: {map_info['width']} x {map_info['height']} 像素")
        print(f"  地图范围: X({map_info['origin_x']:.2f} ~ {map_info['origin_x'] + map_info['width'] * map_info['resolution']:.2f}) 米")
        print(f"           Y({map_info['origin_y']:.2f} ~ {map_info['origin_y'] + map_info['height'] * map_info['resolution']:.2f}) 米")
        
        # 计算地图统计信息
        free_area = np.sum(occupancy_map == 1.0) * (args.resolution ** 2)
        occupied_area = np.sum(occupancy_map == 0.0) * (args.resolution ** 2)
        unknown_area = np.sum(occupancy_map == 0.5) * (args.resolution ** 2)
        total_area = free_area + occupied_area + unknown_area
        
        print(f"  可行走区域: {free_area:.2f} 平方米 ({free_area/total_area*100:.1f}%)")
        print(f"  障碍物区域: {occupied_area:.2f} 平方米 ({occupied_area/total_area*100:.1f}%)")
        print(f"  未知区域: {unknown_area:.2f} 平方米 ({unknown_area/total_area*100:.1f}%)")
        
        # 显示占用地图
        plot_occupancy_map(occupancy_map, map_info, positions[:, :2], 
                          title + " - 2D房间平面图")
        
        # 保存地图文件
        if args.save_map:
            save_occupancy_map(occupancy_map, map_info, args.save_map)
        else:
            # 默认文件名
            import time
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            default_filename = f"occupancy_map_{timestamp}"
            save_occupancy_map(occupancy_map, map_info, default_filename)

if __name__ == '__main__':
    main()
