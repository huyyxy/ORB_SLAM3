#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
占用地图生成测试脚本

生成示例轨迹数据并测试占用地图生成功能
"""

import numpy as np
import matplotlib.pyplot as plt
from visualize_trajectory import generate_occupancy_map, plot_occupancy_map, save_occupancy_map

def generate_sample_trajectory():
    """生成示例机器人轨迹（模拟房间环境）"""
    
    # 创建一个矩形房间轨迹
    trajectory_points = []
    
    # 1. 沿着房间边缘移动（顺时针）
    # 底边：从(0,0)到(8,0)
    for x in np.linspace(0, 8, 50):
        trajectory_points.append([x, 0.5, 0.0])  # 距离墙壁0.5米
    
    # 右边：从(8,0)到(8,6)
    for y in np.linspace(0.5, 5.5, 30):
        trajectory_points.append([7.5, y, 0.0])
    
    # 顶边：从(8,6)到(0,6)
    for x in np.linspace(7.5, 0.5, 40):
        trajectory_points.append([x, 5.5, 0.0])
    
    # 左边：从(0,6)到(0,0)
    for y in np.linspace(5.5, 0.5, 30):
        trajectory_points.append([0.5, y, 0.0])
    
    # 2. 在房间内部探索
    # 添加一些内部轨迹点
    for i in range(3):
        for j in range(2):
            center_x = 2 + i * 2
            center_y = 2 + j * 2
            # 围绕中心点做小圈
            for angle in np.linspace(0, 2*np.pi, 20):
                x = center_x + 0.8 * np.cos(angle)
                y = center_y + 0.8 * np.sin(angle)
                trajectory_points.append([x, y, 0.0])
    
    # 3. 添加一些连接路径
    # 连接各个探索区域
    connection_paths = [
        # 从底部到中心
        [(1.5, 0.5), (3, 2)],
        [(5.5, 0.5), (5, 2)],
        # 从中心到顶部
        [(3, 4), (2.5, 5.5)],
        [(5, 4), (5.5, 5.5)],
        # 横向连接
        [(3, 2), (5, 2)],
        [(3, 4), (5, 4)]
    ]
    
    for path in connection_paths:
        start, end = path
        for t in np.linspace(0, 1, 15):
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            trajectory_points.append([x, y, 0.0])
    
    return np.array(trajectory_points)

def test_occupancy_map_generation():
    """测试占用地图生成功能"""
    
    print("生成示例轨迹数据...")
    positions = generate_sample_trajectory()
    
    print(f"轨迹点数: {len(positions)}")
    print(f"轨迹范围: X({positions[:, 0].min():.2f} ~ {positions[:, 0].max():.2f}) 米")
    print(f"         Y({positions[:, 1].min():.2f} ~ {positions[:, 1].max():.2f}) 米")
    
    # 测试不同分辨率
    resolutions = [0.1, 0.05, 0.02]
    
    for resolution in resolutions:
        print(f"\n测试分辨率: {resolution} 米/像素")
        
        # 生成占用地图
        occupancy_map, map_info = generate_occupancy_map(
            positions, 
            resolution=resolution,
            robot_radius=0.3,
            map_extension=1.0
        )
        
        print(f"地图尺寸: {map_info['width']} x {map_info['height']} 像素")
        
        # 计算统计信息
        free_area = np.sum(occupancy_map == 1.0) * (resolution ** 2)
        occupied_area = np.sum(occupancy_map == 0.0) * (resolution ** 2)
        unknown_area = np.sum(occupancy_map == 0.5) * (resolution ** 2)
        total_area = free_area + occupied_area + unknown_area
        
        print(f"可行走区域: {free_area:.2f} 平方米 ({free_area/total_area*100:.1f}%)")
        print(f"障碍物区域: {occupied_area:.2f} 平方米 ({occupied_area/total_area*100:.1f}%)")
        print(f"未知区域: {unknown_area:.2f} 平方米 ({unknown_area/total_area*100:.1f}%)")
        
        # 可视化地图
        plot_occupancy_map(occupancy_map, map_info, positions[:, :2], 
                          f"示例房间地图 (分辨率: {resolution}m)")
        
        # 保存地图（仅保存最高分辨率的）
        if resolution == 0.02:
            save_occupancy_map(occupancy_map, map_info, f"test_room_map_{resolution}")

def generate_tum_format_file():
    """生成TUM格式的示例轨迹文件用于测试"""
    
    positions = generate_sample_trajectory()
    
    # 生成时间戳和四元数（假设无旋转）
    timestamps = np.linspace(0, len(positions) * 0.1, len(positions))
    
    # TUM格式：timestamp tx ty tz qx qy qz qw
    with open("sample_trajectory.txt", "w") as f:
        f.write("# TUM trajectory format\n")
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        
        for i, pos in enumerate(positions):
            # 无旋转的四元数 (0, 0, 0, 1)
            f.write(f"{timestamps[i]:.6f} {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
                   f"0.000000 0.000000 0.000000 1.000000\n")
    
    print("已生成TUM格式示例文件: sample_trajectory.txt")
    return "sample_trajectory.txt"

if __name__ == '__main__':
    print("=== 占用地图生成测试 ===")
    
    # 1. 测试占用地图生成
    test_occupancy_map_generation()
    
    # 2. 生成TUM格式文件
    tum_file = generate_tum_format_file()
    
    print(f"\n=== 测试完成 ===")
    print(f"您可以使用以下命令测试完整功能：")
    print(f"python visualize_trajectory.py --tum {tum_file} --occupancy")
    print(f"python visualize_trajectory.py --tum {tum_file} --occupancy --resolution 0.02 --robot-radius 0.4")
