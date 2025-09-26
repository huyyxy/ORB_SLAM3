#!/usr/bin/env python3
"""
=============================================================================
ORB-SLAM3 立体视觉SLAM Python示例程序
=============================================================================

【程序概述】
这个程序演示了如何使用ORB-SLAM3的Python绑定来实现基于立体视觉的同时定位与地图构建(SLAM)。
立体视觉使用两个摄像头来获取深度信息，从而实现更精确的三维重建和位姿估计。

【什么是立体视觉SLAM？】
- SLAM (Simultaneous Localization and Mapping): 同时定位与地图构建
- 立体视觉：使用两个摄像头模拟人眼的双目视觉，通过视差计算深度
- 相比单目SLAM，立体视觉SLAM可以直接获得尺度信息，定位更准确

【核心技术原理】
1. ORB特征提取：在图像中检测和描述稳定的角点特征
2. 立体匹配：在左右图像间匹配对应的特征点
3. 深度估计：通过视差计算特征点的三维坐标
4. 位姿估计：基于3D-2D对应关系估计相机位置和姿态
5. 地图构建：将稳定的特征点添加到全局地图中
6. 回环检测：识别重复访问的场景以消除累积误差

【程序功能】
✓ 使用双摄像头进行实时立体视觉SLAM
✓ 显示相机轨迹和跟踪状态
✓ 可视化特征点和地图信息
✓ 保存轨迹为TUM和KITTI标准格式
✓ 支持系统重置和交互式控制

【使用方法】
python example_stereo.py --vocab [词汇表文件] --settings [配置文件] 
                        [--left-camera 0] [--right-camera 1] [--no-viewer]

【键盘控制】
- 'q': 退出程序
- 's': 保存当前轨迹
- 'r': 重置SLAM系统

【注意事项】
1. 确保左右摄像头已正确连接并校准
2. 在纹理丰富的环境中使用以获得更好的效果
3. 避免过快的相机运动，特别是在系统初始化阶段
4. 确保有足够的光照条件

作者: ORB-SLAM3团队
教学版注释: 中文详细注释版本
版本: 1.0
日期: 2024
=============================================================================
"""

# 导入必要的库
import numpy as np      # 用于数值计算和数组操作
import cv2             # OpenCV库，用于图像处理和摄像头控制
import time            # 时间处理库，用于时间戳和帧率控制
import argparse        # 命令行参数解析库
# 从ORB-SLAM3 Python绑定中导入核心类
from orb_slam3_python import ORBSLAMSystem, SensorType, TrackingState

def main():
    """
    主函数：立体视觉SLAM的完整流程
    
    这个函数包含了从系统初始化到运行SLAM的完整流程：
    1. 解析命令行参数
    2. 初始化ORB-SLAM3系统
    3. 配置立体摄像头
    4. 运行SLAM主循环
    5. 清理资源并保存结果
    """
    
    # ===== 步骤1: 命令行参数解析 =====
    # 创建参数解析器，用于处理用户输入的各种配置参数
    parser = argparse.ArgumentParser(description='ORB-SLAM3立体视觉SLAM Python示例')
    
    # 必需参数：ORB词汇表文件路径
    # 词汇表包含预训练的ORB特征描述符，用于特征匹配和回环检测
    parser.add_argument('--vocab', required=True, help='ORB词汇表文件路径')
    
    # 必需参数：相机配置文件路径
    # 包含相机内参、外参、畸变参数等重要配置信息
    parser.add_argument('--settings', required=True, help='相机设置配置文件路径')
    
    # 可选参数：左摄像头设备ID（默认为0）
    parser.add_argument('--left-camera', type=int, default=0, help='左摄像头设备ID')
    
    # 可选参数：右摄像头设备ID（默认为1）
    parser.add_argument('--right-camera', type=int, default=1, help='右摄像头设备ID')
    
    # 可选参数：禁用可视化界面的标志
    parser.add_argument('--no-viewer', action='store_true', help='禁用可视化界面')
    
    # 解析所有命令行参数
    args = parser.parse_args()
    
    print("正在初始化ORB-SLAM3系统...")
    
    # ===== 步骤2: 创建并初始化ORB-SLAM3系统 =====
    try:
        # 创建ORB-SLAM3系统实例
        # 参数说明：
        # - vocab_file: ORB词汇表文件，包含预训练的特征描述符
        # - settings_file: 相机配置文件，定义相机参数和SLAM设置
        # - sensor_type: 传感器类型，这里使用立体视觉(STEREO)
        # - use_viewer: 是否启用3D可视化界面
        slam = ORBSLAMSystem(
            vocab_file=args.vocab,           # 词汇表文件路径
            settings_file=args.settings,     # 设置文件路径
            sensor_type=SensorType.STEREO,   # 立体视觉传感器类型
            use_viewer=not args.no_viewer    # 根据命令行参数决定是否启用查看器
        )
        print("ORB-SLAM3系统初始化成功！")
    except Exception as e:
        print(f"ORB-SLAM3系统初始化失败: {e}")
        print("请检查词汇表文件和设置文件是否存在且格式正确")
        return
    
    # ===== 步骤3: 初始化立体摄像头 =====
    print(f"正在打开立体摄像头 {args.left_camera} 和 {args.right_camera}...")
    
    # 创建左右摄像头的VideoCapture对象
    # VideoCapture是OpenCV中用于捕获视频流的类
    left_cap = cv2.VideoCapture(args.left_camera)   # 左摄像头
    right_cap = cv2.VideoCapture(args.right_camera) # 右摄像头
    
    # 检查左摄像头是否成功打开
    if not left_cap.isOpened():
        print(f"无法打开左摄像头 {args.left_camera}")
        print("请检查摄像头是否连接正确，设备ID是否正确")
        return
    
    # 检查右摄像头是否成功打开
    if not right_cap.isOpened():
        print(f"无法打开右摄像头 {args.right_camera}")
        print("请检查摄像头是否连接正确，设备ID是否正确")
        return
    
    # 为两个摄像头设置相同的属性参数
    # 立体视觉要求左右摄像头具有相同的分辨率和帧率
    print("配置摄像头参数...")
    for cap in [left_cap, right_cap]:
        # 设置图像宽度为640像素
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # 设置图像高度为480像素  
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # 设置帧率为30FPS
        cap.set(cv2.CAP_PROP_FPS, 30)
    
    # ===== 步骤4: 初始化SLAM运行所需的变量 =====
    frame_id = 0        # 帧计数器，用于跟踪处理的帧数
    trajectory = []     # 存储相机轨迹的列表，每个元素包含帧ID、时间戳和位姿
    
    print("开始立体视觉SLAM...")
    print("操作说明:")
    print("  按 'q' 键退出程序")
    print("  按 's' 键保存当前轨迹")
    print("  按 'r' 键重置SLAM系统")
    
    # ===== 步骤5: SLAM主循环 =====
    try:
        while True:
            # 记录帧处理开始时间，用于帧率控制
            start_time = time.time()
            
            # 从立体摄像头读取图像帧
            # ret_* 返回布尔值，表示是否成功读取
            # *_frame 包含实际的图像数据
            ret_left, left_frame = left_cap.read()     # 读取左摄像头图像
            ret_right, right_frame = right_cap.read()  # 读取右摄像头图像
            
            # 检查是否成功从两个摄像头读取到图像
            if not ret_left or not ret_right:
                print("从立体摄像头读取图像失败")
                print("可能的原因：摄像头连接断开或设备繁忙")
                break
            
            # 将彩色图像转换为灰度图像
            # ORB-SLAM3使用灰度图像进行特征提取和匹配
            # COLOR_BGR2GRAY: 从BGR彩色格式转换为灰度格式
            left_gray = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)   # 左图灰度化
            right_gray = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY) # 右图灰度化
            
            # 获取当前时间戳
            # SLAM系统需要准确的时间信息来进行运动估计
            timestamp = time.time()
            
            # ===== 核心SLAM处理：立体图像跟踪 =====
            try:
                # 调用ORB-SLAM3的立体视觉跟踪函数
                # 输入：左图灰度图、右图灰度图、时间戳
                # 输出：4x4的变换矩阵，表示相机在世界坐标系中的位姿
                pose = slam.track_stereo(left_gray, right_gray, timestamp)
                
                # 检查是否成功获得位姿估计
                if pose is not None:
                    # 将成功的位姿结果保存到轨迹列表中
                    trajectory.append({
                        'frame_id': frame_id,      # 帧编号
                        'timestamp': timestamp,    # 时间戳
                        'pose': pose.copy()        # 位姿矩阵的副本
                    })
                    
                    # 从4x4变换矩阵中提取平移部分（相机位置）
                    # pose[:3, 3] 表示变换矩阵的前3行、第4列，即xyz坐标
                    translation = pose[:3, 3]
                    print(f"帧 {frame_id}: 位置 ({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f})")
                else:
                    print(f"帧 {frame_id}: 跟踪失败")
                
                # 获取并显示SLAM系统的跟踪状态
                state = slam.get_tracking_state()
                if state == TrackingState.LOST:
                    print("警告: 跟踪丢失!")
                    print("建议：尝试移动到特征丰富的区域或重置系统")
                elif state == TrackingState.NOT_INITIALIZED:
                    print("系统尚未初始化...")
                    print("提示：请在纹理丰富的环境中缓慢移动相机")
                
                # 获取当前跟踪到的地图点数量
                # 地图点是三维空间中的特征点，用于构建环境地图
                map_points = slam.get_tracked_map_points()
                if len(map_points) > 0:
                    print(f"  跟踪到 {len(map_points)} 个地图点")
                
            except Exception as e:
                print(f"跟踪过程中发生错误: {e}")
                print("错误可能由图像质量、系统配置或硬件问题引起")
                break
            
            # ===== 图像显示和用户交互（可选） =====
            if not args.no_viewer:
                # 水平拼接左右图像以便同时显示
                # np.hstack: 沿水平方向堆叠数组
                combined_frame = np.hstack([left_frame, right_frame])
                
                # 在左图上绘制跟踪到的特征点（关键点）
                # 特征点是SLAM系统用于跟踪和定位的重要视觉特征
                keypoints = slam.get_tracked_keypoints()
                for kp in keypoints:
                    # 在左图上画绿色圆圈标记特征点位置
                    # 参数：图像, 中心坐标, 半径, 颜色(BGR), 填充(-1表示实心)
                    cv2.circle(left_frame, (int(kp['x']), int(kp['y'])), 2, (0, 255, 0), -1)
                
                # 显示三个窗口：左图、右图、拼接图
                cv2.imshow('ORB-SLAM3立体视觉 - 左摄像头', left_frame)
                cv2.imshow('ORB-SLAM3立体视觉 - 右摄像头', right_frame)
                cv2.imshow('ORB-SLAM3立体视觉 - 组合视图', combined_frame)
                
                # 检测用户按键输入
                # waitKey(1): 等待1毫秒获取按键，0xFF用于掩码处理
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    print("用户请求退出程序")
                    break
                elif key == ord('s'):
                    # 保存当前轨迹到文件
                    print("保存轨迹...")
                    save_trajectory(slam, trajectory)
                elif key == ord('r'):
                    # 重置SLAM系统
                    print("重置SLAM系统...")
                    print("注意：这将清除所有地图信息和轨迹数据")
                    slam.reset()
                    trajectory.clear()  # 清空轨迹列表
            
            # 更新帧计数器
            frame_id += 1
            
            # ===== 帧率控制 =====
            # 计算当前帧的处理时间
            elapsed = time.time() - start_time
            
            # 控制程序以30FPS运行
            # 如果处理时间少于1/30秒，则休眠剩余时间
            target_fps = 30.0
            frame_time = 1.0 / target_fps
            if elapsed < frame_time:
                time.sleep(frame_time - elapsed)
    
    except KeyboardInterrupt:
        print("\n程序被用户中断 (Ctrl+C)")
    
    finally:
        # ===== 步骤6: 资源清理和结果保存 =====
        print("正在关闭系统...")
        
        # 安全关闭ORB-SLAM3系统
        # 这会停止所有后台线程并保存必要的数据
        slam.shutdown()
        
        # 释放摄像头资源
        left_cap.release()   # 释放左摄像头
        right_cap.release()  # 释放右摄像头
        
        # 关闭所有OpenCV窗口
        cv2.destroyAllWindows()
        
        # 保存最终轨迹（如果有数据）
        if trajectory:
            print("保存最终轨迹数据...")
            save_trajectory(slam, trajectory)
        else:
            print("没有轨迹数据需要保存")
        
        print("程序已完成！")

def save_trajectory(slam, trajectory):
    """
    保存轨迹到文件
    
    这个函数将SLAM系统生成的相机轨迹保存为两种常用的格式：
    1. TUM格式：时间戳 x y z qx qy qz qw
    2. KITTI格式：3x4变换矩阵的12个元素
    
    参数:
        slam: ORB-SLAM3系统实例
        trajectory: 轨迹数据列表（实际上这里主要使用slam对象的内置保存功能）
    """
    
    # 生成基于当前时间的文件名后缀
    # 格式：YYYYMMDD_HHMMSS
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    
    try:
        # ===== 保存TUM格式轨迹 =====
        # TUM格式是一种常用的轨迹表示格式，每行包含：
        # timestamp tx ty tz qx qy qz qw
        # 其中：tx,ty,tz是平移向量，qx,qy,qz,qw是四元数表示的旋转
        tum_filename = f"stereo_trajectory_tum_{timestamp_str}.txt"
        slam.save_trajectory_tum(tum_filename)
        print(f"轨迹已保存为TUM格式: {tum_filename}")
        print("TUM格式说明：每行为 时间戳 x y z qx qy qz qw")
        
        # ===== 保存KITTI格式轨迹 =====
        # KITTI格式将每个位姿表示为3x4变换矩阵的12个元素
        # 格式：r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
        kitti_filename = f"stereo_trajectory_kitti_{timestamp_str}.txt"
        slam.save_trajectory_kitti(kitti_filename)
        print(f"轨迹已保存为KITTI格式: {kitti_filename}")
        print("KITTI格式说明：每行为变换矩阵的12个元素")
        
    except Exception as e:
        print(f"保存轨迹时发生错误: {e}")
        print("请检查文件写入权限和磁盘空间")

# ===== 程序入口点 =====
if __name__ == '__main__':
    """
    程序的入口点
    
    当脚本被直接运行时（而不是被导入），会执行main()函数
    这是Python程序的标准模式
    """
    main()
