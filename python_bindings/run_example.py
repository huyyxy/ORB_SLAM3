#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运行ORB-SLAM3 RGB-D数据集示例的便捷脚本

该脚本简化了运行example_rgbd.py的过程，自动设置正确的路径和参数。
"""

import os
import sys
import subprocess

def main():
    # 获取项目根目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # 设置文件路径
    vocab_file = os.path.join(project_root, "Vocabulary", "ORBvoc.txt")
    settings_file = os.path.join(project_root, "Examples", "RGB-D", "TUM1.yaml")
    dataset_path = os.path.join(project_root, "rgbd_dataset_freiburg1_xyz")
    example_script = os.path.join(script_dir, "example_rgbd.py")
    
    # 检查文件是否存在
    files_to_check = [
        (vocab_file, "ORB词汇表文件"),
        (settings_file, "设置文件"),
        (dataset_path, "数据集目录"),
        (example_script, "示例脚本")
    ]
    
    print("正在检查必要文件...")
    for file_path, description in files_to_check:
        if os.path.exists(file_path):
            print(f"✓ {description}: {file_path}")
        else:
            print(f"✗ {description}不存在: {file_path}")
            sys.exit(1)
    
    # 构建命令
    cmd = [
        "python3",
        example_script,
        "--vocab", vocab_file,
        "--settings", settings_file, 
        "--dataset", dataset_path
    ]
    
    print("\n正在启动ORB-SLAM3 RGB-D处理...")
    print("命令:", " ".join(cmd))
    print("\n" + "="*60)
    
    # 运行命令
    try:
        subprocess.run(cmd, cwd=project_root)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"\n运行出错: {e}")

if __name__ == "__main__":
    main()
