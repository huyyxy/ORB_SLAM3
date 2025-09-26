#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ORB-SLAM3 RGB-D SLAM Python示例程序

本程序演示了如何使用ORB-SLAM3的Python绑定来进行RGB-D SLAM（同时定位与地图构建）。
RGB-D SLAM使用RGB图像和深度图像来估计相机位姿并构建环境的3D地图。

== 程序架构 ==
1. 命令行参数解析：处理用户输入的配置文件路径和运行参数
2. SLAM系统初始化：加载词汇表和配置文件，创建ORB-SLAM3实例
3. 数据源设置：配置相机或数据集作为输入源
4. 主处理循环：
   - 获取RGB-D图像帧
   - 调用SLAM跟踪算法
   - 提取并显示位姿信息
   - 可视化跟踪结果
   - 处理用户交互
5. 资源清理：保存轨迹数据并释放系统资源

== 核心概念 ==
• ORB特征：Oriented FAST and Rotated BRIEF，具有旋转不变性的快速特征
• 位姿估计：通过特征匹配估计相机在3D空间中的位置和姿态
• 地图构建：根据多帧观测建立环境的3D点云地图
• 回环检测：识别相机重新访问之前的位置，用于全局一致性优化
• 局部优化：优化当前帧及其邻近关键帧的位姿和地图点

== 使用方法 ==
# 基本用法（使用相机）
python example_rgbd.py --vocab path/to/ORBvoc.txt --settings path/to/settings.yaml

# 指定相机设备
python example_rgbd.py --vocab path/to/ORBvoc.txt --settings path/to/settings.yaml --camera 1

# 禁用可视化界面（适合服务器环境）
python example_rgbd.py --vocab path/to/ORBvoc.txt --settings path/to/settings.yaml --no-viewer

# 使用数据集（需要实现数据集加载逻辑）
python example_rgbd.py --vocab path/to/ORBvoc.txt --settings path/to/settings.yaml --dataset path/to/dataset

== 依赖要求 ==
- Python 3.6+
- OpenCV (cv2)
- NumPy
- ORB-SLAM3 Python绑定

== 输出文件格式 ==
1. TUM格式：trajectory_tum_YYYYMMDD_HHMMSS.txt
   格式：timestamp tx ty tz qx qy qz qw
2. KITTI格式：trajectory_kitti_YYYYMMDD_HHMMSS.txt
   格式：每行12个数值（3x4变换矩阵）
3. NumPy格式：trajectory_numpy_YYYYMMDD_HHMMSS.npz
   包含：poses(位姿矩阵)、timestamps(时间戳)、frame_ids(帧ID)

作者：ORB-SLAM3团队
维护者：Python绑定社区
用途：教学、研究和开发
许可证：GPLv3
"""

# 导入必要的库
import numpy as np      # 用于数值计算和数组操作
import cv2             # OpenCV库，用于图像处理和相机操作
import time            # 用于时间戳和帧率控制
import argparse        # 用于命令行参数解析
from orb_slam3_python import ORBSLAMSystem, SensorType, TrackingState  # ORB-SLAM3 Python绑定

def main():
    """
    主函数：处理命令行参数，初始化SLAM系统，执行SLAM流程
    
    该函数是程序的入口点，负责：
    1. 解析命令行参数
    2. 初始化ORB-SLAM3系统
    3. 设置数据源（相机或数据集）
    4. 执行SLAM主循环
    5. 清理资源
    """
    
    # 创建命令行参数解析器
    # ArgumentParser允许我们定义程序需要的命令行参数
    parser = argparse.ArgumentParser(description='ORB-SLAM3 RGB-D Python示例程序')
    
    # 添加必需的参数：ORB词汇表文件路径
    # 词汇表是预训练的ORB特征描述符集合，用于回环检测和重定位
    parser.add_argument('--vocab', required=True, help='ORB词汇表文件路径（.txt格式）')
    
    # 添加必需的参数：设置文件路径
    # 设置文件包含相机内参、畸变参数、ORB特征参数等配置
    parser.add_argument('--settings', required=True, help='SLAM系统设置文件路径（.yaml格式）')
    
    # 添加可选参数：数据集路径
    # 如果指定数据集，程序将从数据集读取数据而不是实时相机
    parser.add_argument('--dataset', help='RGB-D数据集路径（可选，如TUM或自定义格式）')
    
    # 添加可选参数：相机设备ID
    # 默认使用0号相机设备（通常是笔记本电脑的内置摄像头）
    parser.add_argument('--camera', type=int, default=0, help='相机设备ID（默认：0）')
    
    # 添加可选参数：禁用可视化界面
    # 在没有显示器的服务器环境中运行时很有用
    parser.add_argument('--no-viewer', action='store_true', help='禁用可视化界面')
    
    # 解析命令行参数
    args = parser.parse_args()
    
    print("正在初始化ORB-SLAM3系统...")
    
    # 创建ORB-SLAM3系统实例
    # 这是整个SLAM系统的核心组件，负责特征提取、跟踪、建图等功能
    try:
        slam = ORBSLAMSystem(
            vocab_file=args.vocab,           # ORB词汇表文件，用于特征匹配和回环检测
            settings_file=args.settings,    # 系统配置文件，包含相机参数和算法参数
            sensor_type=SensorType.RGBD,    # 传感器类型为RGB-D（彩色+深度）
            use_viewer=not args.no_viewer   # 是否启用3D可视化界面
        )
        print("ORB-SLAM3系统初始化成功！")
    except Exception as e:
        print(f"ORB-SLAM3系统初始化失败: {e}")
        print("请检查词汇表文件和设置文件路径是否正确")
        return
    
    # 初始化数据源：相机或数据集
    if args.dataset:
        # 使用数据集模式
        # 注意：这里是占位符，实际使用时需要实现数据集加载逻辑
        print(f"使用数据集模式: {args.dataset}")
        print("提示：数据集模式需要您实现相应的数据加载逻辑")
        print("常见的RGB-D数据集格式包括TUM、NYU Depth等")
        cap = None
    else:
        # 使用实时相机模式
        print(f"正在打开相机设备 {args.camera}...")
        cap = cv2.VideoCapture(args.camera)
        
        # 检查相机是否成功打开
        if not cap.isOpened():
            print(f"无法打开相机设备 {args.camera}")
            print("请检查：")
            print("1. 相机设备是否已连接")
            print("2. 相机是否被其他程序占用")
            print("3. 设备ID是否正确")
            return
        
        # 设置相机属性（根据实际相机调整参数）
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # 设置图像宽度为640像素
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置图像高度为480像素
        cap.set(cv2.CAP_PROP_FPS, 30)            # 设置帧率为30FPS
        
        print("相机初始化成功！")
    
    # 初始化跟踪变量
    frame_id = 0        # 帧计数器，记录处理的帧数
    trajectory = []     # 轨迹列表，存储每一帧的位姿信息
    
    print("开始SLAM处理...")
    print("控制说明：")
    print("  - 按 'q' 键退出程序")
    print("  - 按 's' 键保存当前轨迹")
    print("  - 按 'r' 键重置SLAM系统")
    print("  - Ctrl+C 强制退出")
    
    try:
        # SLAM主循环：持续处理图像帧
        while True:
            start_time = time.time()  # 记录帧处理开始时间，用于帧率控制
            
            if cap:
                # ===== 从相机读取数据 =====
                ret, rgb_frame = cap.read()  # 读取一帧RGB图像
                if not ret:
                    print("无法从相机读取图像帧，可能相机已断开")
                    break
                
                # 颜色空间转换：OpenCV默认使用BGR格式，而ORB-SLAM3需要RGB格式
                rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB)
                
                # ===== 创建深度图像 =====
                # 注意：这里创建的是虚拟深度图（所有像素深度为1000mm）
                # 在实际应用中，您需要：
                # 1. 如果使用RGB-D相机（如RealSense、Kinect），直接获取真实深度数据
                # 2. 如果使用双目相机，通过立体匹配计算深度
                # 3. 如果使用单目相机，ORB-SLAM3会通过运动估计深度（但需要使用单目模式）
                depth_frame = np.ones((rgb_frame.shape[0], rgb_frame.shape[1]), dtype=np.float32) * 1000.0
                print("警告：正在使用虚拟深度数据！实际应用中请使用真实深度传感器数据")
                
            else:
                # ===== 从数据集读取数据 =====
                # 这是数据集模式的占位符
                # 实际实现时，您需要：
                # 1. 解析数据集的目录结构
                # 2. 按时间戳顺序读取RGB和深度图像
                # 3. 加载相机内参和外参
                print("数据集模式尚未实现，请先实现数据集加载逻辑")
                break
            
            # 生成时间戳
            # 时间戳用于SLAM系统的时序处理和数据关联
            timestamp = time.time()
            
            # ===== 执行SLAM跟踪 =====
            try:
                # 调用ORB-SLAM3的RGB-D跟踪函数
                # 这是SLAM系统的核心功能，包括：
                # 1. ORB特征提取和匹配
                # 2. 相机位姿估计
                # 3. 地图点投影和搜索
                # 4. 局部优化
                pose = slam.track_rgbd(rgb_frame, depth_frame, timestamp)
                
                if pose is not None:
                    # 跟踪成功：保存位姿信息
                    trajectory.append({
                        'frame_id': frame_id,    # 帧ID
                        'timestamp': timestamp,   # 时间戳
                        'pose': pose.copy()      # 4x4变换矩阵（旋转+平移）
                    })
                    
                    # 提取并显示相机位置（平移向量）
                    # pose矩阵的前3行第4列是相机在世界坐标系中的位置
                    translation = pose[:3, 3]
                    print(f"帧 {frame_id}: 相机位置 ({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}) 米")
                else:
                    # 跟踪失败：可能由于特征不足、运动过快或遮挡
                    print(f"帧 {frame_id}: 跟踪失败")
                
                # ===== 获取跟踪状态 =====
                # SLAM系统的状态指示当前跟踪质量
                state = slam.get_tracking_state()
                if state == TrackingState.LOST:
                    print("警告: 跟踪丢失！尝试缓慢移动相机以重新初始化")
                elif state == TrackingState.NOT_INITIALIZED:
                    print("系统尚未初始化...请移动相机以观察场景中的特征点")
                
                # ===== 获取地图点信息 =====
                # 地图点是3D环境中的特征点，用于位姿估计和建图
                map_points = slam.get_tracked_map_points()
                if len(map_points) > 0:
                    print(f"  跟踪到 {len(map_points)} 个地图点")
                
                # ===== 获取关键点信息 =====
                # 关键点是当前帧中检测到的2D特征点
                keypoints = slam.get_tracked_keypoints()
                if len(keypoints) > 0:
                    print(f"  检测到 {len(keypoints)} 个关键点")
                
            except Exception as e:
                print(f"跟踪过程中发生错误: {e}")
                print("这可能是由于：")
                print("1. 输入图像格式不正确")
                print("2. 系统配置参数有误")
                print("3. 内存不足")
                break
            
            # ===== 图像显示和用户交互 =====
            if cap and not args.no_viewer:
                # 准备显示图像：将RGB格式转回BGR（OpenCV显示格式）
                display_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                
                # 在图像上绘制跟踪到的关键点
                # 关键点显示为绿色圆点，有助于直观了解特征检测效果
                keypoints = slam.get_tracked_keypoints()
                for kp in keypoints:
                    # 在关键点位置绘制半径为2的绿色圆点
                    cv2.circle(display_frame, (int(kp['x']), int(kp['y'])), 2, (0, 255, 0), -1)
                
                # 显示图像窗口
                cv2.imshow('ORB-SLAM3 RGB-D实时跟踪', display_frame)
                
                # 处理键盘输入
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    # 用户按'q'键退出
                    print("用户请求退出程序")
                    break
                elif key == ord('s'):
                    # 用户按's'键保存轨迹
                    print("保存当前轨迹...")
                    save_trajectory(slam, trajectory)
                elif key == ord('r'):
                    # 用户按'r'键重置SLAM系统
                    print("重置SLAM系统...")
                    slam.reset()
                    trajectory.clear()
                    frame_id = 0
                    print("系统已重置，轨迹已清空")
            
            # 更新帧计数器
            frame_id += 1
            
            # ===== 帧率控制 =====
            # 控制处理速度，避免过快消耗CPU资源
            elapsed = time.time() - start_time  # 计算本帧处理时间
            target_fps = 30.0                   # 目标帧率
            if elapsed < 1.0/target_fps:
                # 如果处理速度过快，主动等待以维持稳定帧率
                time.sleep(1.0/target_fps - elapsed)
    
    except KeyboardInterrupt:
        # 处理Ctrl+C中断信号
        print("\n用户中断程序执行")
    
    finally:
        # ===== 程序清理和退出 =====
        print("正在关闭系统...")
        
        # 关闭SLAM系统，释放内存和线程资源
        slam.shutdown()
        
        # 释放相机资源
        if cap:
            cap.release()
        
        # 关闭所有OpenCV窗口
        cv2.destroyAllWindows()
        
        # 保存最终轨迹（如果有数据）
        if trajectory:
            print("保存最终轨迹数据...")
            save_trajectory(slam, trajectory)
        
        print("程序已正常退出！")

def save_trajectory(slam, trajectory):
    """
    保存轨迹数据的函数
    
    该函数将SLAM系统计算得到的相机轨迹保存为多种格式，便于后续分析和可视化：
    1. TUM格式：常用于学术研究，格式为 timestamp tx ty tz qx qy qz qw
    2. KITTI格式：常用于自动驾驶研究，格式为3x4变换矩阵的12个元素
    3. NumPy格式：Python友好的二进制格式，包含完整的位姿矩阵和元数据
    
    参数：
        slam: ORB-SLAM3系统实例
        trajectory: 包含位姿数据的轨迹列表
    """
    
    # 生成带时间戳的文件名，避免覆盖之前保存的文件
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    
    # ===== 保存TUM格式轨迹 =====
    # TUM格式是RGB-D SLAM基准测试中常用的格式
    # 格式：timestamp tx ty tz qx qy qz qw（时间戳 + 位置 + 四元数旋转）
    tum_filename = f"trajectory_tum_{timestamp_str}.txt"
    slam.save_trajectory_tum(tum_filename)
    print(f"轨迹已保存为TUM格式: {tum_filename}")
    print("  TUM格式说明：每行包含时间戳、位置坐标(xyz)和四元数旋转(qxqyqzqw)")
    
    # ===== 保存KITTI格式轨迹 =====
    # KITTI格式是自动驾驶研究中常用的格式
    # 格式：每行包含3x4变换矩阵的12个元素（按行展开）
    kitti_filename = f"trajectory_kitti_{timestamp_str}.txt"
    slam.save_trajectory_kitti(kitti_filename)
    print(f"轨迹已保存为KITTI格式: {kitti_filename}")
    print("  KITTI格式说明：每行包含3x4变换矩阵的12个元素")
    
    # ===== 保存NumPy格式轨迹 =====
    # 这是自定义的Python友好格式，保存完整信息便于后续处理
    if trajectory:
        numpy_filename = f"trajectory_numpy_{timestamp_str}.npz"
        
        # 提取位姿矩阵数组（4x4变换矩阵）
        poses = np.array([t['pose'] for t in trajectory])
        
        # 提取时间戳数组
        timestamps = np.array([t['timestamp'] for t in trajectory])
        
        # 提取帧ID数组
        frame_ids = np.array([t['frame_id'] for t in trajectory])
        
        # 使用NumPy的压缩格式保存多个数组
        np.savez(numpy_filename, 
                poses=poses,           # 位姿矩阵数组 (N, 4, 4)
                timestamps=timestamps, # 时间戳数组 (N,)
                frame_ids=frame_ids)   # 帧ID数组 (N,)
        
        print(f"轨迹已保存为NumPy格式: {numpy_filename}")
        print("  NumPy格式说明：包含poses(位姿矩阵)、timestamps(时间戳)、frame_ids(帧ID)")
        print(f"  总计保存了 {len(trajectory)} 个位姿")
    else:
        print("警告：轨迹数据为空，跳过NumPy格式保存")

if __name__ == '__main__':
    """
    程序入口点
    
    当该Python文件作为主程序运行时（而不是作为模块导入），执行main()函数。
    这是Python程序的标准写法，确保代码只在直接运行时执行，而不在被导入时执行。
    
    使用示例：
    python example_rgbd.py --vocab ../Vocabulary/ORBvoc.txt --settings ../Examples/RGB-D/RealSense_D435i.yaml
    """
    main()
