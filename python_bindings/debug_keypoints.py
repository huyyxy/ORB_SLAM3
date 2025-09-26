#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ORB-SLAM3关键点数据结构调试工具

该脚本用于诊断get_tracked_keypoints()返回的数据结构问题
"""

import numpy as np
import cv2
import argparse
from orb_slam3_python import ORBSLAMSystem, SensorType, TrackingState

def debug_keypoints_structure(vocab_file, settings_file, dataset_path):
    """
    调试关键点数据结构
    """
    print("正在初始化ORB-SLAM3系统...")
    
    try:
        slam = ORBSLAMSystem(
            vocab_file=vocab_file,
            settings_file=settings_file,
            sensor_type=SensorType.RGBD,
            use_viewer=False
        )
        print("系统初始化成功！")
    except Exception as e:
        print(f"系统初始化失败: {e}")
        return
    
    # 加载一帧数据进行测试
    import os
    rgb_file = os.path.join(dataset_path, 'rgb.txt')
    depth_file = os.path.join(dataset_path, 'depth.txt')
    
    # 读取第一张图像
    with open(rgb_file, 'r') as f:
        for line in f:
            if not line.startswith('#'):
                parts = line.strip().split()
                if len(parts) >= 2:
                    rgb_path = os.path.join(dataset_path, parts[1])
                    break
    
    with open(depth_file, 'r') as f:
        for line in f:
            if not line.startswith('#'):
                parts = line.strip().split()
                if len(parts) >= 2:
                    depth_path = os.path.join(dataset_path, parts[1])
                    break
    
    # 加载图像
    rgb_frame = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
    depth_frame = cv2.imread(depth_path, cv2.IMREAD_ANYDEPTH)
    
    if rgb_frame is None or depth_frame is None:
        print("无法加载测试图像")
        return
    
    rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB)
    depth_frame = depth_frame.astype(np.float32) / 1000.0
    
    print("\n开始调试关键点数据结构...")
    
    # 执行跟踪
    import time
    timestamp = time.time()
    try:
        pose = slam.track_rgbd(rgb_frame, depth_frame, timestamp)
        print(f"跟踪结果: {'成功' if pose is not None else '失败'}")
    except Exception as e:
        print(f"跟踪失败: {e}")
        slam.shutdown()
        return
    
    # 调试关键点数据
    try:
        print("\n=== 调试get_tracked_keypoints()返回值 ===")
        keypoints = slam.get_tracked_keypoints()
        
        print(f"返回值类型: {type(keypoints)}")
        print(f"返回值长度: {len(keypoints) if keypoints else 'None'}")
        
        if keypoints and len(keypoints) > 0:
            # 分析第一个关键点
            first_kp = keypoints[0]
            print(f"\n第一个关键点的类型: {type(first_kp)}")
            print(f"第一个关键点的内容: {first_kp}")
            
            # 尝试不同的访问方式
            print("\n=== 尝试不同的数据访问方式 ===")
            
            # 方式1：字典访问
            try:
                x = first_kp['x']
                print(f"字典访问成功: x = {x}")
            except Exception as e:
                print(f"字典访问失败: {e}")
            
            # 方式2：属性访问
            try:
                x = first_kp.x
                print(f"属性访问成功: x = {x}")
            except Exception as e:
                print(f"属性访问失败: {e}")
            
            # 方式3：索引访问
            try:
                if hasattr(first_kp, '__getitem__'):
                    x = first_kp[0]
                    print(f"索引访问成功: x = {x}")
                else:
                    print("不支持索引访问")
            except Exception as e:
                print(f"索引访问失败: {e}")
            
            # 方式4：numpy数组访问
            try:
                if isinstance(first_kp, np.void):
                    print("检测到numpy.void类型")
                    print(f"dtype: {first_kp.dtype}")
                    if first_kp.dtype.names:
                        print(f"字段名: {first_kp.dtype.names}")
                        for name in first_kp.dtype.names:
                            print(f"  {name}: {first_kp[name]}")
            except Exception as e:
                print(f"numpy处理失败: {e}")
            
            # 方式5：dir()查看可用属性/方法
            print(f"\n可用属性/方法: {[attr for attr in dir(first_kp) if not attr.startswith('_')]}")
        
    except Exception as e:
        print(f"调试过程中发生错误: {e}")
        print(f"错误类型: {type(e).__name__}")
    
    slam.shutdown()
    print("\n调试完成！")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ORB-SLAM3关键点调试工具')
    parser.add_argument('--vocab', required=True, help='ORB词汇表文件路径')
    parser.add_argument('--settings', required=True, help='设置文件路径') 
    parser.add_argument('--dataset', required=True, help='数据集路径')
    
    args = parser.parse_args()
    debug_keypoints_structure(args.vocab, args.settings, args.dataset)
