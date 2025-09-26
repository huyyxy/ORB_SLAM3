#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
占用地图生成快速开始脚本

这个脚本提供了一个简单的交互式界面来生成占用地图
"""

import os
import sys
import argparse

def print_banner():
    """打印欢迎界面"""
    print("=" * 60)
    print("    ORB-SLAM3 占用地图生成工具")
    print("    从轨迹数据生成2D房间平面图")
    print("=" * 60)

def check_dependencies():
    """检查依赖库"""
    print("检查依赖库...")
    
    required_libs = [
        ('numpy', 'NumPy'),
        ('matplotlib', 'Matplotlib'),
        ('cv2', 'OpenCV'),
        ('scipy', 'SciPy'),
        ('skimage', 'scikit-image')
    ]
    
    missing_libs = []
    for lib_name, display_name in required_libs:
        try:
            __import__(lib_name)
            print(f"✓ {display_name}")
        except ImportError:
            print(f"✗ {display_name} - 需要安装")
            missing_libs.append(display_name)
    
    if missing_libs:
        print(f"\n请安装缺失的库:")
        print(f"pip install {' '.join([lib.lower().replace('opencv', 'opencv-python').replace('scikit-image', 'scikit-image') for lib in missing_libs])}")
        return False
    
    print("✓ 所有依赖库都已安装")
    return True

def find_trajectory_files():
    """查找当前目录下的轨迹文件"""
    trajectory_files = []
    
    # 查找TUM格式文件
    for filename in os.listdir('.'):
        if filename.endswith('.txt') and 'trajectory' in filename.lower():
            trajectory_files.append(('tum', filename))
    
    # 查找NumPy格式文件
    for filename in os.listdir('.'):
        if filename.endswith('.npz'):
            trajectory_files.append(('numpy', filename))
    
    return trajectory_files

def interactive_mode():
    """交互式模式"""
    print("\n=== 交互式占用地图生成 ===")
    
    # 1. 查找轨迹文件
    trajectory_files = find_trajectory_files()
    
    if not trajectory_files:
        print("在当前目录未找到轨迹文件。")
        print("支持的格式：")
        print("  - TUM格式：*.txt (包含trajectory关键词)")
        print("  - NumPy格式：*.npz")
        print("  - KITTI格式：*.txt")
        
        # 询问是否生成示例文件
        response = input("\n是否生成示例轨迹文件进行测试？(y/n): ").lower()
        if response in ['y', 'yes', 'Y']:
            print("生成示例轨迹...")
            os.system("python3 test_occupancy_map.py > /dev/null 2>&1")
            if os.path.exists("sample_trajectory.txt"):
                print("✓ 已生成示例文件: sample_trajectory.txt")
                trajectory_files = [('tum', 'sample_trajectory.txt')]
            else:
                print("✗ 生成示例文件失败")
                return
        else:
            return
    
    # 2. 选择轨迹文件
    print(f"\n找到 {len(trajectory_files)} 个轨迹文件:")
    for i, (format_type, filename) in enumerate(trajectory_files):
        print(f"  {i+1}. {filename} ({format_type.upper()}格式)")
    
    try:
        choice = int(input(f"\n请选择文件 (1-{len(trajectory_files)}): ")) - 1
        format_type, filename = trajectory_files[choice]
    except (ValueError, IndexError):
        print("无效选择")
        return
    
    # 3. 设置参数
    print(f"\n选择的文件: {filename}")
    print("\n=== 参数设置 ===")
    
    try:
        resolution = float(input("地图分辨率 (米/像素，推荐0.02-0.1，默认0.05): ") or "0.05")
        robot_radius = float(input("机器人半径 (米，默认0.3): ") or "0.3")
        map_extension = float(input("地图边界扩展 (米，默认2.0): ") or "2.0")
    except ValueError:
        print("参数输入错误，使用默认值")
        resolution = 0.05
        robot_radius = 0.3
        map_extension = 2.0
    
    save_map = input("保存地图文件名前缀 (可选，默认自动生成): ").strip()
    
    # 4. 生成命令并执行
    cmd_parts = [
        "python3 visualize_trajectory.py",
        f"--{format_type} {filename}",
        "--occupancy",
        f"--resolution {resolution}",
        f"--robot-radius {robot_radius}",
        f"--map-extension {map_extension}"
    ]
    
    if save_map:
        cmd_parts.append(f"--save-map {save_map}")
    
    command = " ".join(cmd_parts)
    
    print(f"\n=== 执行命令 ===")
    print(command)
    print("\n开始生成占用地图...")
    
    # 执行命令
    exit_code = os.system(command)
    
    if exit_code == 0:
        print("\n✓ 占用地图生成成功！")
        print("\n生成的文件:")
        if save_map:
            base_name = save_map
        else:
            # 查找最新生成的文件
            import glob
            import time
            pattern = "occupancy_map_*.pgm"
            files = glob.glob(pattern)
            if files:
                latest_file = max(files, key=os.path.getctime)
                base_name = latest_file.replace('.pgm', '')
            else:
                base_name = "occupancy_map"
        
        for ext in ['.pgm', '.yaml', '.npz']:
            if os.path.exists(f"{base_name}{ext}"):
                print(f"  - {base_name}{ext}")
    else:
        print("\n✗ 生成失败，请检查错误信息")

def batch_mode(args):
    """批处理模式"""
    print("\n=== 批处理模式 ===")
    
    # 构建命令
    cmd_parts = ["python3 visualize_trajectory.py"]
    
    if args.tum:
        cmd_parts.append(f"--tum {args.tum}")
    elif args.kitti:
        cmd_parts.append(f"--kitti {args.kitti}")
    elif args.numpy:
        cmd_parts.append(f"--numpy {args.numpy}")
    
    cmd_parts.append("--occupancy")
    
    if args.resolution:
        cmd_parts.append(f"--resolution {args.resolution}")
    if args.robot_radius:
        cmd_parts.append(f"--robot-radius {args.robot_radius}")
    if args.map_extension:
        cmd_parts.append(f"--map-extension {args.map_extension}")
    if args.save_map:
        cmd_parts.append(f"--save-map {args.save_map}")
    
    command = " ".join(cmd_parts)
    print(f"执行命令: {command}")
    
    # 执行命令
    exit_code = os.system(command)
    
    if exit_code == 0:
        print("✓ 占用地图生成成功！")
    else:
        print("✗ 生成失败")

def main():
    parser = argparse.ArgumentParser(
        description='占用地图生成快速开始工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法：
  # 交互式模式
  python3 quick_start_occupancy.py
  
  # 批处理模式
  python3 quick_start_occupancy.py --tum trajectory.txt
  python3 quick_start_occupancy.py --tum trajectory.txt --resolution 0.02 --robot-radius 0.4
        """
    )
    
    # 轨迹文件参数
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--tum', help='TUM格式轨迹文件')
    group.add_argument('--kitti', help='KITTI格式轨迹文件')
    group.add_argument('--numpy', help='NumPy格式轨迹文件')
    
    # 参数设置
    parser.add_argument('--resolution', type=float, help='地图分辨率（米/像素）')
    parser.add_argument('--robot-radius', type=float, help='机器人半径（米）')
    parser.add_argument('--map-extension', type=float, help='地图边界扩展（米）')
    parser.add_argument('--save-map', help='保存地图文件名前缀')
    
    args = parser.parse_args()
    
    print_banner()
    
    # 检查依赖库
    if not check_dependencies():
        return
    
    # 选择模式
    if any([args.tum, args.kitti, args.numpy]):
        batch_mode(args)
    else:
        interactive_mode()

if __name__ == '__main__':
    main()
