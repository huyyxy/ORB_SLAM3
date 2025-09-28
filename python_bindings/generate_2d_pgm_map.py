#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D PGM占用地图生成器 - 从3D地图数据生成指定高度范围的2D占用地图
================================================================

本脚本基于ORB-SLAM3 Python绑定，从3D地图数据生成指定高度范围的2D PGM占用地图。

支持的输入格式：
1. PLY格式 (.ply) - 点云数据
2. PCD格式 (.pcd) - PCL点云数据
3. NPY格式 (.npy) - NumPy数组
4. TXT格式 (.txt) - 点云文本格式
5. HDF5格式 (.h5) - 科学计算数据

支持的输出格式：
1. PGM格式 (.pgm) - 2D占用地图
2. YAML格式 (.yaml) - 地图元数据配置
3. PNG格式 (.png) - 可视化预览

作者：ORB-SLAM3 Python绑定扩展
用途：从3D地图生成2D占用地图
"""

import numpy as np
import cv2
import os
import sys
import time
import argparse
import h5py
import yaml
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any, Union

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class PointCloudLoader:
    """点云数据加载器 - 支持多种格式"""
    
    def __init__(self):
        """初始化点云加载器"""
        self.supported_formats = ['.ply', '.pcd', '.npy', '.txt', '.h5']
    
    def load_point_cloud(self, file_path: str) -> np.ndarray:
        """
        加载点云数据
        
        参数:
            file_path: 点云文件路径
            
        返回:
            np.ndarray: 点云数据数组 [N, 6] (x, y, z, r, g, b)
        """
        file_ext = os.path.splitext(file_path)[1].lower()
        
        if file_ext not in self.supported_formats:
            raise ValueError(f"不支持的文件格式: {file_ext}。支持的格式: {self.supported_formats}")
        
        print(f"📁 加载点云文件: {file_path}")
        
        if file_ext == '.ply':
            return self._load_ply(file_path)
        elif file_ext == '.pcd':
            return self._load_pcd(file_path)
        elif file_ext == '.npy':
            return self._load_npy(file_path)
        elif file_ext == '.txt':
            return self._load_txt(file_path)
        elif file_ext == '.h5':
            return self._load_hdf5(file_path)
        else:
            raise ValueError(f"未实现的文件格式: {file_ext}")
    
    def _load_ply(self, file_path: str) -> np.ndarray:
        """加载PLY格式点云"""
        points = []
        
        with open(file_path, 'r') as f:
            lines = f.readlines()
        
        # 查找数据开始位置
        data_start = 0
        num_vertices = 0
        
        for i, line in enumerate(lines):
            if line.startswith('element vertex'):
                num_vertices = int(line.split()[-1])
            elif line.startswith('end_header'):
                data_start = i + 1
                break
        
        # 解析点云数据
        for i in range(data_start, data_start + num_vertices):
            if i < len(lines):
                parts = lines[i].strip().split()
                if len(parts) >= 6:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                    points.append([x, y, z, r, g, b])
        
        return np.array(points)
    
    def _load_pcd(self, file_path: str) -> np.ndarray:
        """加载PCD格式点云"""
        points = []
        
        with open(file_path, 'r') as f:
            lines = f.readlines()
        
        # 查找数据开始位置
        data_start = 0
        num_points = 0
        
        for i, line in enumerate(lines):
            if line.startswith('POINTS'):
                num_points = int(line.split()[-1])
            elif line.startswith('DATA ascii'):
                data_start = i + 1
                break
        
        # 解析点云数据
        for i in range(data_start, data_start + num_points):
            if i < len(lines):
                parts = lines[i].strip().split()
                if len(parts) >= 4:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    rgb = int(parts[3])
                    r = (rgb >> 16) & 0xFF
                    g = (rgb >> 8) & 0xFF
                    b = rgb & 0xFF
                    points.append([x, y, z, r, g, b])
        
        return np.array(points)
    
    def _load_npy(self, file_path: str) -> np.ndarray:
        """加载NumPy格式点云"""
        data = np.load(file_path, allow_pickle=True)
        
        if isinstance(data, np.ndarray):
            if data.ndim == 2 and data.shape[1] >= 3:
                # 如果是2D数组，直接返回
                if data.shape[1] == 3:
                    # 只有xyz，添加默认颜色
                    colors = np.zeros((data.shape[0], 3), dtype=np.uint8)
                    return np.hstack([data, colors])
                elif data.shape[1] >= 6:
                    return data[:, :6]
            else:
                raise ValueError("NumPy数组格式不正确，期望形状为 [N, 3+] 或 [N, 6+]")
        elif isinstance(data, dict):
            # 如果是字典格式，查找点云数据
            if 'point_cloud' in data:
                return data['point_cloud']
            elif 'point_cloud_xyz' in data and 'point_cloud_rgb' in data:
                return np.hstack([data['point_cloud_xyz'], data['point_cloud_rgb']])
            else:
                raise ValueError("NumPy文件中未找到点云数据")
        else:
            raise ValueError("NumPy文件格式不正确")
    
    def _load_txt(self, file_path: str) -> np.ndarray:
        """加载文本格式点云"""
        points = []
        
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('#') or not line:
                    continue
                
                parts = line.split()
                if len(parts) >= 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    if len(parts) >= 6:
                        r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                    else:
                        r, g, b = 128, 128, 128  # 默认灰色
                    points.append([x, y, z, r, g, b])
        
        return np.array(points)
    
    def _load_hdf5(self, file_path: str) -> np.ndarray:
        """加载HDF5格式点云"""
        with h5py.File(file_path, 'r') as f:
            if 'point_cloud' in f:
                return f['point_cloud'][:]
            elif 'point_cloud_xyz' in f and 'point_cloud_rgb' in f:
                xyz = f['point_cloud_xyz'][:]
                rgb = f['point_cloud_rgb'][:]
                return np.hstack([xyz, rgb])
            else:
                raise ValueError("HDF5文件中未找到点云数据")


class OccupancyMap2DGenerator:
    """2D占用地图生成器"""
    
    def __init__(self, resolution: float = 0.05, robot_radius: float = 0.2):
        """
        初始化2D占用地图生成器
        
        参数:
            resolution: 地图分辨率（米/像素）
            robot_radius: 机器人半径（米）
        """
        self.resolution = resolution
        self.robot_radius = robot_radius
        
    def generate_occupancy_map(self, point_cloud: np.ndarray, 
                             height_min: float, height_max: float,
                             map_extension: float = 2.0) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        从3D点云生成2D占用地图
        
        参数:
            point_cloud: 3D点云数据 [N, 6] (x, y, z, r, g, b)
            height_min: 最小高度（米）
            height_max: 最大高度（米）
            map_extension: 地图扩展范围（米）
            
        返回:
            tuple: (occupancy_map, map_info)
        """
        print(f"🗺️ 生成2D占用地图...")
        print(f"   高度范围: {height_min:.2f} - {height_max:.2f} 米")
        print(f"   分辨率: {self.resolution:.3f} 米/像素")
        print(f"   机器人半径: {self.robot_radius:.2f} 米")
        
        # 过滤高度范围内的点
        height_mask = (point_cloud[:, 2] >= height_min) & (point_cloud[:, 2] <= height_max)
        filtered_points = point_cloud[height_mask]
        
        print(f"   原始点数: {len(point_cloud)}")
        print(f"   过滤后点数: {len(filtered_points)}")
        
        if len(filtered_points) == 0:
            print("⚠️ 警告: 高度范围内没有点云数据")
            # 返回空地图
            empty_map = np.full((100, 100), 205, dtype=np.uint8)
            map_info = {
                'width': 100, 'height': 100, 'resolution': self.resolution,
                'origin_x': 0.0, 'origin_y': 0.0, 'height_range': (height_min, height_max)
            }
            return empty_map, map_info
        
        # 计算地图边界
        x_min = np.min(filtered_points[:, 0]) - map_extension
        x_max = np.max(filtered_points[:, 0]) + map_extension
        y_min = np.min(filtered_points[:, 1]) - map_extension
        y_max = np.max(filtered_points[:, 1]) + map_extension
        
        # 计算地图尺寸（像素）
        map_width = int((x_max - x_min) / self.resolution)
        map_height = int((y_max - y_min) / self.resolution)
        
        print(f"   地图尺寸: {map_width} x {map_height} 像素")
        print(f"   实际尺寸: {map_width * self.resolution:.2f} x {map_height * self.resolution:.2f} 米")
        
        # 创建占用地图 (0=占用, 254=空闲, 205=未知)
        occupancy_map = np.full((map_height, map_width), 205, dtype=np.uint8)
        
        # 将3D点投影到2D网格
        occupied_cells = set()
        
        for point in filtered_points:
            x, y = point[0], point[1]
            
            # 转换为像素坐标
            pixel_x = int((x - x_min) / self.resolution)
            pixel_y = int((y - y_min) / self.resolution)
            
            # 确保在边界内
            if 0 <= pixel_x < map_width and 0 <= pixel_y < map_height:
                occupied_cells.add((pixel_x, pixel_y))
        
        # 标记占用区域
        for pixel_x, pixel_y in occupied_cells:
            occupancy_map[pixel_y, pixel_x] = 0
        
        # 膨胀操作（考虑机器人半径）
        if self.robot_radius > 0:
            kernel_size = int(2 * self.robot_radius / self.resolution) + 1
            if kernel_size > 1:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
                occupied_mask = (occupancy_map == 0)
                dilated_mask = cv2.dilate(occupied_mask.astype(np.uint8), kernel, iterations=1)
                occupancy_map[dilated_mask > 0] = 0
        
        # 创建地图信息
        map_info = {
            'width': map_width,
            'height': map_height,
            'resolution': self.resolution,
            'origin_x': x_min,
            'origin_y': y_min,
            'height_range': (height_min, height_max),
            'robot_radius': self.robot_radius,
            'map_extension': map_extension,
            'num_points_original': len(point_cloud),
            'num_points_filtered': len(filtered_points)
        }
        
        print(f"✅ 2D占用地图生成完成")
        return occupancy_map, map_info


class MapExporter:
    """地图导出器"""
    
    def __init__(self, occupancy_map: np.ndarray, map_info: Dict[str, Any]):
        """
        初始化地图导出器
        
        参数:
            occupancy_map: 占用地图数组
            map_info: 地图元信息
        """
        self.occupancy_map = occupancy_map
        self.map_info = map_info
    
    def export_pgm(self, output_path: str) -> None:
        """导出PGM格式占用地图"""
        print(f"💾 导出PGM格式: {output_path}")
        
        # 翻转Y轴以符合图像坐标系
        pgm_map = np.flipud(self.occupancy_map)
        
        # 使用OpenCV保存PGM格式
        cv2.imwrite(output_path, pgm_map)
        
        print(f"✅ PGM文件已保存: {output_path}")
    
    def export_yaml(self, output_path: str, pgm_filename: str) -> None:
        """导出YAML配置文件"""
        print(f"💾 导出YAML配置: {output_path}")
        
        yaml_content = f"""image: {pgm_filename}
resolution: {self.map_info['resolution']}
origin: [{self.map_info['origin_x']}, {self.map_info['origin_y']}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
        
        with open(output_path, 'w') as f:
            f.write(yaml_content)
        
        print(f"✅ YAML文件已保存: {output_path}")
    
    def export_png(self, output_path: str) -> None:
        """导出PNG格式可视化地图"""
        print(f"💾 导出PNG格式: {output_path}")
        
        # 创建可视化地图
        png_map = self.occupancy_map.copy()
        png_map[self.occupancy_map == 254] = 255  # 空闲 - 白色
        png_map[self.occupancy_map == 0] = 0      # 占用 - 黑色
        png_map[self.occupancy_map == 205] = 128  # 未知 - 灰色
        
        cv2.imwrite(output_path, png_map)
        print(f"✅ PNG文件已保存: {output_path}")
    
    def export_all(self, output_dir: str, base_name: str) -> Dict[str, str]:
        """导出所有格式"""
        print(f"💾 导出所有格式到: {output_dir}")
        
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        
        # 生成文件名
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        full_base_name = f"{base_name}_{timestamp}"
        
        # 导出PGM
        pgm_path = os.path.join(output_dir, f"{full_base_name}.pgm")
        self.export_pgm(pgm_path)
        
        # 导出YAML
        yaml_path = os.path.join(output_dir, f"{full_base_name}.yaml")
        self.export_yaml(yaml_path, f"{full_base_name}.pgm")
        
        # 导出PNG
        png_path = os.path.join(output_dir, f"{full_base_name}.png")
        self.export_png(png_path)
        
        # 导出地图信息
        info_path = os.path.join(output_dir, f"{full_base_name}_info.txt")
        self._export_info(info_path)
        
        return {
            'pgm': pgm_path,
            'yaml': yaml_path,
            'png': png_path,
            'info': info_path
        }
    
    def _export_info(self, output_path: str) -> None:
        """导出地图信息"""
        print(f"💾 导出地图信息: {output_path}")
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write("2D占用地图信息\n")
            f.write("=" * 30 + "\n\n")
            f.write(f"地图尺寸: {self.map_info['width']} x {self.map_info['height']} 像素\n")
            f.write(f"分辨率: {self.map_info['resolution']:.6f} 米/像素\n")
            f.write(f"实际尺寸: {self.map_info['width'] * self.map_info['resolution']:.2f} x {self.map_info['height'] * self.map_info['resolution']:.2f} 米\n")
            f.write(f"原点坐标: ({self.map_info['origin_x']:.3f}, {self.map_info['origin_y']:.3f})\n")
            f.write(f"高度范围: {self.map_info['height_range'][0]:.2f} - {self.map_info['height_range'][1]:.2f} 米\n")
            f.write(f"机器人半径: {self.map_info['robot_radius']:.2f} 米\n")
            f.write(f"地图扩展: {self.map_info['map_extension']:.2f} 米\n")
            f.write(f"原始点数: {self.map_info['num_points_original']}\n")
            f.write(f"过滤后点数: {self.map_info['num_points_filtered']}\n")
        
        print(f"✅ 地图信息已保存: {output_path}")


def analyze_occupancy_map(occupancy_map: np.ndarray, map_info: Dict[str, Any]) -> None:
    """分析占用地图统计信息"""
    print("\n📊 占用地图统计分析:")
    
    # 计算各种区域的像素数量
    free_pixels = np.sum(occupancy_map == 254)      # 空闲区域
    occupied_pixels = np.sum(occupancy_map == 0)    # 占用区域
    unknown_pixels = np.sum(occupancy_map == 205)   # 未知区域
    total_pixels = occupancy_map.size
    
    # 计算实际面积（平方米）
    resolution = map_info['resolution']
    pixel_area = resolution ** 2
    free_area = free_pixels * pixel_area
    occupied_area = occupied_pixels * pixel_area
    unknown_area = unknown_pixels * pixel_area
    total_area = total_pixels * pixel_area
    
    # 打印统计信息
    print(f"  总面积: {total_area:.2f} 平方米")
    print(f"  可通行区域: {free_area:.2f} 平方米 ({free_pixels/total_pixels*100:.1f}%)")
    print(f"  占用区域: {occupied_area:.2f} 平方米 ({occupied_pixels/total_pixels*100:.1f}%)")
    print(f"  未知区域: {unknown_area:.2f} 平方米 ({unknown_pixels/total_pixels*100:.1f}%)")
    print(f"  地图尺寸: {map_info['width']} x {map_info['height']} 像素")
    print(f"  分辨率: {resolution:.6f} 米/像素")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='从3D地图数据生成2D PGM占用地图')
    parser.add_argument('input_file', help='输入点云文件路径')
    parser.add_argument('--output', '-o', default='./2d_maps', help='输出目录')
    parser.add_argument('--height-min', type=float, default=0.0, help='最小高度（米）')
    parser.add_argument('--height-max', type=float, default=2.0, help='最大高度（米）')
    parser.add_argument('--resolution', type=float, default=0.05, help='地图分辨率（米/像素）')
    parser.add_argument('--robot-radius', type=float, default=0.2, help='机器人半径（米）')
    parser.add_argument('--map-extension', type=float, default=2.0, help='地图扩展范围（米）')
    parser.add_argument('--prefix', type=str, default='occupancy_map', help='输出文件名前缀')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("🗺️ 2D PGM占用地图生成器")
    print("=" * 60)
    
    try:
        # 检查输入文件
        if not os.path.exists(args.input_file):
            print(f"❌ 输入文件不存在: {args.input_file}")
            return 1
        
        # 加载点云数据
        loader = PointCloudLoader()
        point_cloud = loader.load_point_cloud(args.input_file)
        
        if len(point_cloud) == 0:
            print("❌ 点云数据为空")
            return 1
        
        print(f"✅ 成功加载点云数据: {len(point_cloud)} 个点")
        
        # 生成2D占用地图
        generator = OccupancyMap2DGenerator(
            resolution=args.resolution,
            robot_radius=args.robot_radius
        )
        
        occupancy_map, map_info = generator.generate_occupancy_map(
            point_cloud=point_cloud,
            height_min=args.height_min,
            height_max=args.height_max,
            map_extension=args.map_extension
        )
        
        # 分析地图
        analyze_occupancy_map(occupancy_map, map_info)
        
        # 导出地图
        exporter = MapExporter(occupancy_map, map_info)
        saved_files = exporter.export_all(args.output, args.prefix)
        
        print(f"\n🎯 地图文件已保存到: {args.output}")
        print(f"   PGM地图: {saved_files['pgm']}")
        print(f"   YAML配置: {saved_files['yaml']}")
        print(f"   PNG预览: {saved_files['png']}")
        print(f"   地图信息: {saved_files['info']}")
        print(f"\n   ROS使用: rosrun map_server map_server {saved_files['yaml']}")
        
        print("\n✅ 2D占用地图生成完成!")
        
    except FileNotFoundError as e:
        print(f"❌ 文件未找到错误: {e}")
        return 1
    except ValueError as e:
        print(f"❌ 数值错误: {e}")
        return 1
    except Exception as e:
        print(f"❌ 未知错误: {e}")
        print(f"错误类型: {type(e).__name__}")
        import traceback
        print("详细错误信息:")
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
