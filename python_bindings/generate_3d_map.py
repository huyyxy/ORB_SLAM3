#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D地图生成器 - 支持多种输出格式
================================

本脚本基于ORB-SLAM3 Python绑定，从RGB-D数据集生成多种格式的3D地图。

支持的输出格式：
1. OctoMap格式 (.ot) - 八叉树占用地图
2. PLY格式 (.ply) - 点云数据
3. PCD格式 (.pcd) - PCL点云数据
4. OBJ格式 (.obj) - 3D网格模型
5. STL格式 (.stl) - 3D打印模型
6. HDF5格式 (.h5) - 科学计算数据
7. 3D栅格数组 (.npy) - NumPy数组
8. 点云文本格式 (.txt) - 简单文本格式

作者：ORB-SLAM3 Python绑定扩展
用途：RGB-D数据集3D地图生成
"""

import numpy as np
import cv2
import os
import sys
import time
import argparse
import struct
import h5py
import yaml
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 导入ORB-SLAM3 Python绑定
try:
    from orb_slam3_python import ORBSLAMSystem, SensorType
except ImportError as e:
    print(f"❌ 无法导入ORB-SLAM3 Python绑定: {e}")
    print("请确保已正确编译ORB-SLAM3 Python绑定")
    sys.exit(1)


class ConfigParser:
    """配置文件解析器"""
    
    def __init__(self, config_path: str):
        """
        初始化配置解析器
        
        参数:
            config_path: 配置文件路径
        """
        self.config_path = config_path
        self.config = self._load_config()
    
    def _load_config(self) -> Dict[str, Any]:
        """加载YAML配置文件"""
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 预处理文件内容，移除YAML指令行
            lines = content.split('\n')
            processed_lines = []
            
            for line in lines:
                # 跳过YAML指令行（以%开头的行）
                if line.strip().startswith('%'):
                    continue
                processed_lines.append(line)
            
            processed_content = '\n'.join(processed_lines)
            
            # 使用更宽松的YAML解析器
            try:
                config = yaml.safe_load(processed_content)
            except yaml.YAMLError as yaml_error:
                # 如果还是有问题，尝试使用yaml.load
                try:
                    config = yaml.load(processed_content, Loader=yaml.SafeLoader)
                except yaml.YAMLError:
                    raise yaml_error
            
            if config is None:
                config = {}
                
            return config
        except Exception as e:
            print(f"❌ 无法加载配置文件 {self.config_path}: {e}")
            raise
    
    def get_camera_intrinsics(self) -> Tuple[float, float, float, float]:
        """
        获取相机内参
        
        返回:
            tuple: (fx, fy, cx, cy)
        """
        try:
            fx = float(self.config['Camera1.fx'])
            fy = float(self.config['Camera1.fy'])
            cx = float(self.config['Camera1.cx'])
            cy = float(self.config['Camera1.cy'])
            return fx, fy, cx, cy
        except KeyError as e:
            print(f"❌ 配置文件中缺少相机内参: {e}")
            raise
    
    def get_depth_scale_factor(self) -> float:
        """
        获取深度缩放因子
        
        返回:
            float: 深度缩放因子
        """
        try:
            return float(self.config.get('RGBD.DepthMapFactor', 5000.0))
        except (ValueError, TypeError):
            print("⚠️ 使用默认深度缩放因子: 5000.0")
            return 5000.0
    
    def get_camera_resolution(self) -> Tuple[int, int]:
        """
        获取相机分辨率
        
        返回:
            tuple: (width, height)
        """
        try:
            width = int(self.config.get('Camera.width', 640))
            height = int(self.config.get('Camera.height', 480))
            return width, height
        except (ValueError, TypeError):
            print("⚠️ 使用默认分辨率: 640x480")
            return 640, 480


class DatasetLoader:
    """RGB-D数据集加载器"""
    
    def __init__(self, dataset_path: str):
        """
        初始化数据集加载器
        
        参数:
            dataset_path: 数据集根目录路径
        """
        self.dataset_path = dataset_path
        self.rgb_files = []
        self.depth_files = []
        self.timestamps = []
        self.ground_truth = []
        
    def load_tum_dataset(self) -> Tuple[List[str], List[str], List[float], List[np.ndarray]]:
        """
        加载TUM RGBD数据集
        
        返回:
            tuple: (rgb_files, depth_files, timestamps, ground_truth)
        """
        print("📁 加载TUM RGBD数据集...")
        
        if not os.path.exists(self.dataset_path):
            raise FileNotFoundError(f"数据集目录不存在: {self.dataset_path}")
        
        # 读取RGB和深度文件列表
        rgb_file = os.path.join(self.dataset_path, "rgb.txt")
        depth_file = os.path.join(self.dataset_path, "depth.txt")
        gt_file = os.path.join(self.dataset_path, "groundtruth.txt")
        
        if not os.path.exists(rgb_file) or not os.path.exists(depth_file):
            raise FileNotFoundError("缺少rgb.txt或depth.txt文件")
        
        # 加载RGB文件
        with open(rgb_file, 'r') as f:
            rgb_lines = f.readlines()
        
        # 加载深度文件
        with open(depth_file, 'r') as f:
            depth_lines = f.readlines()
        
        # 解析文件列表
        rgb_files = []
        depth_files = []
        timestamps = []
        
        for line in rgb_lines:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) >= 2:
                timestamp = float(parts[0])
                filename = parts[1]
                rgb_files.append(os.path.join(self.dataset_path, filename))
                timestamps.append(timestamp)
        
        for line in depth_lines:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) >= 2:
                filename = parts[1]
                depth_files.append(os.path.join(self.dataset_path, filename))
        
        # 加载真值轨迹（如果存在）
        ground_truth = []
        if os.path.exists(gt_file):
            with open(gt_file, 'r') as f:
                gt_lines = f.readlines()
            
            for line in gt_lines:
                if line.startswith('#'):
                    continue
                parts = line.strip().split()
                if len(parts) >= 8:
                    # TUM格式: timestamp tx ty tz qx qy qz qw
                    pose = np.array([float(x) for x in parts[1:8]])
                    ground_truth.append(pose)
        
        print(f"✅ 加载完成: {len(rgb_files)} 个RGB图像, {len(depth_files)} 个深度图像")
        return rgb_files, depth_files, timestamps, ground_truth


class Map3DGenerator:
    """3D地图生成器"""
    
    def __init__(self, slam_system: ORBSLAMSystem, config_parser: ConfigParser):
        """
        初始化3D地图生成器
        
        参数:
            slam_system: ORB-SLAM3系统实例
            config_parser: 配置解析器实例
        """
        self.slam_system = slam_system
        self.config_parser = config_parser
        self.point_cloud = []
        self.poses = []
        self.keyframes = []
        
        # 从配置文件获取相机内参
        self.fx, self.fy, self.cx, self.cy = config_parser.get_camera_intrinsics()
        self.depth_scale = config_parser.get_depth_scale_factor()
        
        print(f"📷 相机内参 (从配置文件读取):")
        print(f"   fx: {self.fx:.6f}, fy: {self.fy:.6f}")
        print(f"   cx: {self.cx:.6f}, cy: {self.cy:.6f}")
        print(f"   深度缩放因子: {self.depth_scale}")
        
    def process_dataset(self, rgb_files: List[str], depth_files: List[str], 
                       timestamps: List[float]) -> None:
        """
        处理RGB-D数据集
        
        参数:
            rgb_files: RGB图像文件路径列表
            depth_files: 深度图像文件路径列表
            timestamps: 时间戳列表
        """
        print("🔄 开始处理RGB-D数据集...")
        
        total_frames = min(len(rgb_files), len(depth_files))
        
        for i in range(total_frames):
            if i % 50 == 0:
                print(f"处理进度: {i}/{total_frames} ({i/total_frames*100:.1f}%)")
            
            # 读取图像
            rgb_image = cv2.imread(rgb_files[i], cv2.IMREAD_COLOR)
            depth_image = cv2.imread(depth_files[i], cv2.IMREAD_UNCHANGED)
            
            if rgb_image is None or depth_image is None:
                print(f"⚠️ 跳过无效图像: {i}")
                continue
            
            # 确保深度图像是float32类型
            if depth_image.dtype != np.float32:
                depth_image = depth_image.astype(np.float32)
            
            # 处理帧
            pose = self.slam_system.track_rgbd(rgb_image, depth_image, timestamps[i])
            
            if pose is not None:
                self.poses.append(pose)
                
                # 提取点云
                points = self._extract_point_cloud(rgb_image, depth_image, pose)
                if len(points) > 0:
                    self.point_cloud.extend(points)
                
                if i % 100 == 0:  # 每100帧打印一次状态
                    print(f"  📊 帧 {i}: 位姿有效, 点云数量: {len(points)}")
            else:
                if i % 100 == 0:  # 每100帧打印一次状态
                    print(f"  ⚠️ 帧 {i}: 位姿无效")
        
        print(f"✅ 处理完成: 获得 {len(self.poses)} 个有效位姿, {len(self.point_cloud)} 个3D点")
    
    def _extract_point_cloud(self, rgb_image: np.ndarray, depth_image: np.ndarray, 
                           pose: np.ndarray) -> List[np.ndarray]:
        """
        从RGB-D图像提取点云
        
        参数:
            rgb_image: RGB图像
            depth_image: 深度图像
            pose: 相机位姿 (4x4矩阵)
            
        返回:
            List[np.ndarray]: 3D点列表
        """
        points = []
        height, width = depth_image.shape
        
        # 使用从配置文件读取的相机内参
        fx = self.fx
        fy = self.fy
        cx = self.cx
        cy = self.cy
        depth_scale = self.depth_scale
        
        for v in range(0, height, 4):  # 降采样以提高效率
            for u in range(0, width, 4):
                depth = depth_image[v, u] / depth_scale
                
                if depth > 0.1 and depth < 10.0:  # 有效深度范围
                    # 计算3D点
                    x = (u - cx) * depth / fx
                    y = (v - cy) * depth / fy
                    z = depth
                    
                    # 转换到世界坐标系
                    point_3d = np.array([x, y, z, 1.0])
                    world_point = pose @ point_3d
                    
                    # 获取颜色
                    color = rgb_image[v, u]
                    
                    # 存储点云数据 [x, y, z, r, g, b]
                    points.append(np.array([world_point[0], world_point[1], world_point[2], 
                                          color[2], color[1], color[0]]))  # BGR -> RGB
        
        return points


class MapExporter:
    """地图导出器 - 支持多种格式"""
    
    def __init__(self, point_cloud: List[np.ndarray], poses: List[np.ndarray]):
        """
        初始化地图导出器
        
        参数:
            point_cloud: 点云数据列表
            poses: 位姿列表
        """
        self.point_cloud = np.array(point_cloud) if point_cloud else np.array([])
        self.poses = np.array(poses) if poses else np.array([])
    
    def export_ply(self, output_path: str) -> None:
        """导出PLY格式点云"""
        print(f"💾 导出PLY格式: {output_path}")
        
        if len(self.point_cloud) == 0:
            print("⚠️ 没有点云数据可导出")
            return
        
        with open(output_path, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(self.point_cloud)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")
            
            for point in self.point_cloud:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                       f"{int(point[3])} {int(point[4])} {int(point[5])}\n")
        
        print(f"✅ PLY文件已保存: {output_path}")
    
    def export_pcd(self, output_path: str) -> None:
        """导出PCD格式点云"""
        print(f"💾 导出PCD格式: {output_path}")
        
        if len(self.point_cloud) == 0:
            print("⚠️ 没有点云数据可导出")
            return
        
        with open(output_path, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z rgb\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F U\n")
            f.write("COUNT 1 1 1 1\n")
            f.write(f"WIDTH {len(self.point_cloud)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(self.point_cloud)}\n")
            f.write("DATA ascii\n")
            
            for point in self.point_cloud:
                # 将RGB转换为PCD格式的整数
                rgb = (int(point[3]) << 16) | (int(point[4]) << 8) | int(point[5])
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {rgb}\n")
        
        print(f"✅ PCD文件已保存: {output_path}")
    
    def export_obj(self, output_path: str) -> None:
        """导出OBJ格式3D网格"""
        print(f"💾 导出OBJ格式: {output_path}")
        
        if len(self.point_cloud) == 0:
            print("⚠️ 没有点云数据可导出")
            return
        
        with open(output_path, 'w') as f:
            f.write("# OBJ file generated by ORB-SLAM3 Python binding\n")
            f.write("# Point cloud as vertices\n")
            
            for i, point in enumerate(self.point_cloud):
                f.write(f"v {point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                       f"{point[3]/255.0:.3f} {point[4]/255.0:.3f} {point[5]/255.0:.3f}\n")
        
        print(f"✅ OBJ文件已保存: {output_path}")
    
    def export_stl(self, output_path: str) -> None:
        """导出STL格式3D模型"""
        print(f"💾 导出STL格式: {output_path}")
        
        if len(self.point_cloud) == 0:
            print("⚠️ 没有点云数据可导出")
            return
        
        # 简单的STL导出 - 将点云作为三角形网格
        with open(output_path, 'wb') as f:
            # STL头部
            header = b"STL file generated by ORB-SLAM3" + b"\x00" * 80
            f.write(header[:80])
            
            # 三角形数量 (简化：每个点作为一个三角形)
            num_triangles = len(self.point_cloud)
            f.write(struct.pack('<I', num_triangles))
            
            # 写入三角形数据
            for point in self.point_cloud:
                # 法向量 (简化)
                normal = np.array([0.0, 0.0, 1.0])
                f.write(struct.pack('<3f', *normal))
                
                # 三个顶点 (简化：使用同一个点)
                vertex = point[:3]
                f.write(struct.pack('<3f', *vertex))
                f.write(struct.pack('<3f', *vertex))
                f.write(struct.pack('<3f', *vertex))
                
                # 属性字节计数
                f.write(struct.pack('<H', 0))
        
        print(f"✅ STL文件已保存: {output_path}")
    
    def export_hdf5(self, output_path: str) -> None:
        """导出HDF5格式数据"""
        print(f"💾 导出HDF5格式: {output_path}")
        
        with h5py.File(output_path, 'w') as f:
            if len(self.point_cloud) > 0:
                f.create_dataset('point_cloud', data=self.point_cloud)
                f.create_dataset('point_cloud_xyz', data=self.point_cloud[:, :3])
                f.create_dataset('point_cloud_rgb', data=self.point_cloud[:, 3:6])
            
            if len(self.poses) > 0:
                f.create_dataset('poses', data=self.poses)
            
            # 添加元数据
            f.attrs['dataset_type'] = 'ORB-SLAM3_3D_Map'
            f.attrs['num_points'] = len(self.point_cloud)
            f.attrs['num_poses'] = len(self.poses)
            f.attrs['created_time'] = time.strftime('%Y-%m-%d %H:%M:%S')
        
        print(f"✅ HDF5文件已保存: {output_path}")
    
    def export_numpy(self, output_path: str) -> None:
        """导出NumPy格式数据"""
        print(f"💾 导出NumPy格式: {output_path}")
        
        data = {}
        if len(self.point_cloud) > 0:
            data['point_cloud'] = self.point_cloud
            data['point_cloud_xyz'] = self.point_cloud[:, :3]
            data['point_cloud_rgb'] = self.point_cloud[:, 3:6]
        
        if len(self.poses) > 0:
            data['poses'] = self.poses
        
        np.savez_compressed(output_path, **data)
        print(f"✅ NumPy文件已保存: {output_path}")
    
    def export_octomap(self, output_path: str, resolution: float = 0.05) -> None:
        """导出OctoMap格式 (简化版本)"""
        print(f"💾 导出OctoMap格式: {output_path}")
        
        if len(self.point_cloud) == 0:
            print("⚠️ 没有点云数据可导出")
            return
        
        # 简化的八叉树实现
        points = self.point_cloud[:, :3]
        
        # 计算边界
        min_bound = np.min(points, axis=0)
        max_bound = np.max(points, axis=0)
        
        # 创建栅格
        size = max_bound - min_bound
        grid_size = np.ceil(size / resolution).astype(int)
        
        # 创建占用栅格
        occupancy_grid = np.zeros(grid_size, dtype=np.uint8)
        
        # 填充占用栅格
        for point in points:
            idx = np.floor((point - min_bound) / resolution).astype(int)
            if np.all(idx >= 0) and np.all(idx < grid_size):
                occupancy_grid[tuple(idx)] = 1
        
        # 保存为二进制格式
        with open(output_path, 'wb') as f:
            # 写入头部信息
            f.write(b'OCTOMAP')
            f.write(struct.pack('<f', resolution))
            f.write(struct.pack('<3f', *min_bound))
            f.write(struct.pack('<3f', *max_bound))
            f.write(struct.pack('<3I', *grid_size))
            
            # 写入栅格数据
            f.write(occupancy_grid.tobytes())
        
        print(f"✅ OctoMap文件已保存: {output_path}")
    
    def export_txt(self, output_path: str) -> None:
        """导出文本格式点云"""
        print(f"💾 导出文本格式: {output_path}")
        
        if len(self.point_cloud) == 0:
            print("⚠️ 没有点云数据可导出")
            return
        
        with open(output_path, 'w') as f:
            f.write("# Point cloud data: x y z r g b\n")
            for point in self.point_cloud:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                       f"{int(point[3])} {int(point[4])} {int(point[5])}\n")
        
        print(f"✅ 文本文件已保存: {output_path}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='ORB-SLAM3 3D地图生成器')
    parser.add_argument('dataset_path', help='RGB-D数据集路径')
    parser.add_argument('--output', '-o', default='output', help='输出目录')
    parser.add_argument('--format', '-f', 
                       choices=['ply', 'pcd', 'obj', 'stl', 'hdf5', 'numpy', 'octomap', 'txt', 'all'],
                       default='all', help='输出格式')
    parser.add_argument('--config', '-c', 
                       default='../Examples/RGB-D/TUM1.yaml', 
                       help='ORB-SLAM3配置文件路径')
    parser.add_argument('--vocab', '-v', 
                       default='../Vocabulary/ORBvoc.txt', 
                       help='ORB词汇表路径')
    
    args = parser.parse_args()
    
    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)
    
    print("🚀 ORB-SLAM3 3D地图生成器启动")
    print(f"📁 数据集路径: {args.dataset_path}")
    print(f"📁 输出目录: {args.output}")
    print(f"📄 输出格式: {args.format}")
    
    try:
        # 检查词汇表文件是否存在
        if not os.path.exists(args.vocab):
            print(f"❌ 词汇表文件不存在: {args.vocab}")
            return 1
        
        # 检查配置文件是否存在
        if not os.path.exists(args.config):
            print(f"❌ 配置文件不存在: {args.config}")
            return 1
            
        print(f"📚 词汇表文件: {args.vocab}")
        print(f"⚙️ 配置文件: {args.config}")
        
        # 加载配置文件
        print("📄 加载配置文件...")
        config_parser = ConfigParser(args.config)
        print("✅ 配置文件加载完成")
        
        # 初始化ORB-SLAM3系统
        print("🔧 初始化ORB-SLAM3系统...")
        # 在服务器环境中禁用可视化以避免X11错误
        slam_system = ORBSLAMSystem(args.vocab, args.config, SensorType.RGBD, use_viewer=False)
        print("✅ ORB-SLAM3系统初始化完成")
        
        # 加载数据集
        print("📁 开始加载数据集...")
        loader = DatasetLoader(args.dataset_path)
        rgb_files, depth_files, timestamps, ground_truth = loader.load_tum_dataset()
        
        # 生成3D地图
        generator = Map3DGenerator(slam_system, config_parser)
        generator.process_dataset(rgb_files, depth_files, timestamps)
        
        # 导出地图
        exporter = MapExporter(generator.point_cloud, generator.poses)
        
        base_name = os.path.basename(args.dataset_path.rstrip('/'))
        
        if args.format == 'all':
            formats = ['ply', 'pcd', 'obj', 'stl', 'hdf5', 'numpy', 'octomap', 'txt']
        else:
            formats = [args.format]
        
        for fmt in formats:
            output_path = os.path.join(args.output, f"{base_name}_map.{fmt}")
            
            if fmt == 'ply':
                exporter.export_ply(output_path)
            elif fmt == 'pcd':
                exporter.export_pcd(output_path)
            elif fmt == 'obj':
                exporter.export_obj(output_path)
            elif fmt == 'stl':
                exporter.export_stl(output_path)
            elif fmt == 'hdf5':
                exporter.export_hdf5(output_path)
            elif fmt == 'numpy':
                exporter.export_numpy(output_path)
            elif fmt == 'octomap':
                exporter.export_octomap(output_path)
            elif fmt == 'txt':
                exporter.export_txt(output_path)
        
        print("🎉 3D地图生成完成!")
        
    except FileNotFoundError as e:
        print(f"❌ 文件未找到错误: {e}")
        print("请检查文件路径是否正确")
        return 1
    except ImportError as e:
        print(f"❌ 导入错误: {e}")
        print("请确保ORB-SLAM3 Python绑定已正确编译")
        return 1
    except RuntimeError as e:
        print(f"❌ 运行时错误: {e}")
        print("可能是词汇表文件损坏或配置参数错误")
        return 1
    except Exception as e:
        print(f"❌ 未知错误: {e}")
        print(f"错误类型: {type(e).__name__}")
        import traceback
        print("详细错误信息:")
        traceback.print_exc()
        return 1
    
    finally:
        # 清理资源
        if 'slam_system' in locals():
            slam_system.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
