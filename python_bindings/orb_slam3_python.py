"""
ORB-SLAM3 Python绑定库 - 教学版本
=================================

本文件提供了ORB-SLAM3的Python接口，使用ctypes库与C++底层代码进行交互。
ORB-SLAM3是一个实时的SLAM（同时定位与地图构建）系统，支持多种传感器类型。

文件结构概述：
1. 库加载部分：动态加载ORB-SLAM3的共享库
2. 枚举定义：定义传感器类型、跟踪状态等常量
3. 数据结构：定义与C++对应的数据结构
4. 函数签名：设置ctypes与C++函数的接口
5. 主要类：ORBSLAMSystem - 封装所有SLAM功能的Python类

作者：ORB-SLAM3团队
许可证：GPLv3
版本：教学注释版本
"""

# 导入必要的Python标准库和第三方库
import ctypes          # 用于与C/C++代码交互的Python标准库
import numpy as np     # 数值计算库，用于处理图像和数组数据
from ctypes import Structure, POINTER, c_void_p, c_char_p, c_int, c_float, c_double  # ctypes数据类型
import os             # 操作系统接口，用于文件路径操作
import sys            # 系统相关参数和函数
from enum import IntEnum  # 用于创建整数枚举类型

# ============================================================================
# 第一部分：动态库加载
# ============================================================================

def load_orb_slam3_library():
    """
    加载ORB-SLAM3 Python绑定的动态库
    
    功能说明：
    - 这个函数负责找到并加载ORB-SLAM3的共享库文件
    - 支持多种操作系统的库文件格式（.so, .dylib, .dll）
    - 使用ctypes.CDLL来加载C++编译的动态库
    
    加载顺序：
    1. 首先获取当前Python文件所在的目录
    2. 尝试不同的库文件名（适配不同操作系统）
    3. 找到第一个存在且可加载的库文件就返回
    
    返回值：
        ctypes.CDLL: 加载成功的动态库对象
        
    异常：
        RuntimeError: 如果找不到或无法加载任何库文件
    """
    # 获取当前Python文件的绝对路径目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 定义不同操作系统下的可能库文件名
    # .so: Linux动态库文件扩展名
    # .dylib: macOS动态库文件扩展名  
    # .dll: Windows动态库文件扩展名
    lib_names = [
        'orb_slam3_python.so',       # Linux标准格式
        'liborb_slam3_python.so',    # Linux带lib前缀格式
        'orb_slam3_python.dylib',    # macOS格式
        'orb_slam3_python.dll'       # Windows格式
    ]
    
    # 遍历所有可能的库文件名
    for lib_name in lib_names:
        lib_path = os.path.join(current_dir, lib_name)
        if os.path.exists(lib_path):
            try:
                # 使用ctypes.CDLL加载C++动态库
                # CDLL假设函数使用标准C调用约定
                return ctypes.CDLL(lib_path)
            except OSError as e:
                # 如果加载失败，打印错误信息并继续尝试下一个
                print(f"Failed to load {lib_path}: {e}")
                continue
    
    # 如果所有库文件都无法加载，抛出运行时错误
    raise RuntimeError("Could not find ORB-SLAM3 Python bindings library. Please build it first.")

# 全局变量：存储加载的动态库对象
# 这个变量在整个模块中都会被使用，用于调用C++函数
try:
    _lib = load_orb_slam3_library()
except RuntimeError as e:
    print(f"Error: {e}")
    sys.exit(1)  # 如果库加载失败，直接退出程序

# ============================================================================
# 第二部分：枚举类型定义
# ============================================================================

class SensorType(IntEnum):
    """
    传感器类型枚举
    
    定义了ORB-SLAM3支持的所有传感器配置类型。
    不同的传感器类型影响SLAM系统的初始化和跟踪算法。
    
    传感器类型说明：
    - MONOCULAR: 单目相机，只能恢复相对尺度
    - STEREO: 双目相机，可以恢复真实尺度  
    - RGBD: RGB-D相机（如Kinect），同时提供颜色和深度信息
    - IMU_*: 带IMU（惯性测量单元）的传感器组合，提供更稳定的跟踪
    
    IMU的优势：
    - 在快速运动时提供更好的跟踪稳定性
    - 可以估计真实的尺度和重力方向
    - 在视觉特征缺失时保持跟踪连续性
    """
    MONOCULAR = 0        # 单目相机：仅使用一个摄像头
    STEREO = 1          # 双目相机：使用左右两个摄像头
    RGBD = 2            # RGB-D相机：提供彩色图像和深度图像
    IMU_MONOCULAR = 3   # 单目相机+IMU：单目视觉+惯性测量单元
    IMU_STEREO = 4      # 双目相机+IMU：双目视觉+惯性测量单元  
    IMU_RGBD = 5        # RGB-D相机+IMU：RGB-D视觉+惯性测量单元

class TrackingState(IntEnum):
    """
    跟踪状态枚举
    
    表示当前SLAM系统的跟踪状态，用于监控系统运行状况。
    这些状态帮助用户了解系统是否正常工作以及可能遇到的问题。
    
    状态转换流程：
    SYSTEM_NOT_READY -> NO_IMAGES_YET -> NOT_INITIALIZED -> OK
                                                        -> LOST (可能回到OK)
    
    状态说明：
    - SYSTEM_NOT_READY: 系统未准备好，通常是初始化阶段
    - NO_IMAGES_YET: 系统已准备但还未接收到图像数据
    - NOT_INITIALIZED: 已接收图像但还未完成初始化（如单目需要足够的视差）
    - OK: 正常跟踪状态，系统工作正常
    - LOST: 跟踪丢失，通常由于运动过快、遮挡或环境变化导致
    """
    SYSTEM_NOT_READY = -1   # 系统未准备好：SLAM系统还未完全初始化
    NO_IMAGES_YET = 0       # 未接收图像：系统准备好但还没有接收到任何图像
    NOT_INITIALIZED = 1     # 未初始化：接收到图像但地图还未初始化完成
    OK = 2                  # 正常跟踪：系统正常工作，成功跟踪相机位姿
    LOST = 3                # 跟踪丢失：无法跟踪当前位姿，可能需要重定位

# ============================================================================
# 第三部分：数据结构定义
# ============================================================================

class Point3D(Structure):
    """
    三维点结构
    
    表示三维空间中的一个点，通常用于表示地图点(MapPoint)的位置。
    在SLAM系统中，这些点构成了环境的三维地图。
    
    ctypes.Structure说明：
    - 这个类继承自ctypes.Structure，用于与C++代码交换数据
    - _fields_定义了结构体的内存布局，必须与C++结构体完全一致
    - c_float对应C++中的float类型（32位浮点数）
    
    应用场景：
    - 存储从立体视觉或RGBD传感器获取的3D特征点
    - 表示SLAM地图中的landmark位置
    - 用于可视化和导出地图数据
    """
    _fields_ = [
        ("x", c_float),  # X坐标：通常表示前方向（相机坐标系）
        ("y", c_float),  # Y坐标：通常表示左方向（相机坐标系）
        ("z", c_float)   # Z坐标：通常表示上方向（相机坐标系）
    ]

class KeyPoint2D(Structure):
    """
    二维关键点结构
    
    表示图像中的一个特征点，包含位置、方向、响应强度等信息。
    这些关键点是ORB-SLAM3进行特征匹配和跟踪的基础。
    
    ORB特征说明：
    - ORB (Oriented FAST and Rotated BRIEF) 是一种快速的特征检测器
    - 具有旋转不变性和一定的尺度不变性
    - 在移动机器人和实时SLAM中广泛使用
    
    字段详解：
    - x, y: 像素坐标，表示特征点在图像中的位置
    - angle: 特征点的主方向角度，用于旋转不变性
    - response: 特征响应强度，值越大表示特征越稳定
    - octave: 金字塔层级，用于尺度不变性
    """
    _fields_ = [
        ("x", c_float),        # 特征点的X像素坐标
        ("y", c_float),        # 特征点的Y像素坐标
        ("angle", c_float),    # 特征点主方向角度（弧度）
        ("response", c_float), # 特征响应强度（Harris角点响应或其他）
        ("octave", c_int)      # 特征点所在的金字塔层级
    ]

class CameraPose(Structure):
    """
    相机位姿结构
    
    表示相机在世界坐标系中的位姿（位置+姿态）。
    使用4x4变换矩阵来表示，这是计算机视觉中的标准表示方法。
    
    变换矩阵格式：
    [R11 R12 R13 tx]    R: 3x3旋转矩阵
    [R21 R22 R23 ty]    t: 3x1平移向量  
    [R31 R32 R33 tz]    [0 0 0 1]: 齐次坐标
    [0   0   0   1 ]
    
    存储顺序：
    - data数组按行优先顺序存储矩阵元素
    - 索引0-3对应第一行，4-7对应第二行，以此类推
    
    坐标系说明：
    - 通常使用右手坐标系
    - 相机坐标系：Z轴向前，Y轴向下，X轴向右
    """
    _fields_ = [
        ("data", c_float * 16),  # 4x4变换矩阵，按行存储（16个浮点数）
        ("valid", c_int)         # 有效性标志：1表示有效，0表示无效/丢失
    ]

class MapPoints(Structure):
    """
    地图点集合结构
    
    用于批量传递地图点数据，避免单个传递的开销。
    这种设计模式在需要传递大量数据时很常见。
    
    内存管理说明：
    - points指向Point3D数组的首地址
    - num_points指定数组的长度
    - 这种设计允许动态分配任意数量的点
    - 使用完毕后需要调用相应的释放函数
    
    应用场景：
    - 获取当前帧看到的所有地图点
    - 获取整个地图的所有点用于可视化
    - 导出地图数据进行离线处理
    """
    _fields_ = [
        ("points", POINTER(Point3D)),  # 指向Point3D数组的指针
        ("num_points", c_int)          # 数组中点的数量
    ]

class TrackedKeyPoints(Structure):
    """
    跟踪到的关键点集合结构
    
    表示当前帧中成功跟踪到的所有关键点。
    这些点已经与地图中的特征建立了对应关系。
    
    跟踪过程说明：
    1. 从当前图像中提取ORB特征点
    2. 与前一帧的特征点进行匹配
    3. 通过几何验证筛选出可靠的匹配
    4. 这些匹配的特征点就是"跟踪到的关键点"
    
    数据用途：
    - 可视化显示当前帧的跟踪结果
    - 分析跟踪质量（点的数量和分布）
    - 调试特征匹配算法
    """
    _fields_ = [
        ("keypoints", POINTER(KeyPoint2D)),  # 指向KeyPoint2D数组的指针
        ("num_keypoints", c_int)             # 数组中关键点的数量
    ]

# ============================================================================
# 第四部分：函数签名设置
# ============================================================================

def setup_function_signatures():
    """
    设置ctypes函数签名
    
    功能说明：
    这个函数为动态库中的每个C++函数设置正确的参数类型和返回值类型。
    这是使用ctypes调用C++函数的必要步骤，确保数据类型转换的正确性。
    
    ctypes类型映射：
    - c_char_p: 对应C++的const char*（字符串）
    - c_void_p: 对应C++的void*（通用指针）
    - c_int: 对应C++的int（32位整数）
    - c_float: 对应C++的float（32位浮点数）
    - c_double: 对应C++的double（64位浮点数）
    - POINTER(type): 对应C++的type*（指向特定类型的指针）
    
    为什么需要设置签名：
    1. 确保参数传递时的类型安全
    2. 让Python知道如何正确解释返回值
    3. 避免内存访问错误和数据损坏
    """
    
    # ========================================================================
    # 系统管理相关函数
    # ========================================================================
    
    # 创建SLAM系统实例
    # 参数：词汇文件路径、设置文件路径、传感器类型、是否启用可视化
    # 返回：系统句柄（void指针）
    _lib.orb_slam3_create_system.argtypes = [c_char_p, c_char_p, c_int, c_int]
    _lib.orb_slam3_create_system.restype = c_void_p
    
    # 销毁SLAM系统实例
    # 参数：系统句柄
    # 返回：无
    _lib.orb_slam3_destroy_system.argtypes = [c_void_p]
    _lib.orb_slam3_destroy_system.restype = None
    
    # 关闭SLAM系统（停止所有线程但不销毁对象）
    # 参数：系统句柄
    # 返回：无
    _lib.orb_slam3_shutdown.argtypes = [c_void_p]
    _lib.orb_slam3_shutdown.restype = None
    
    # 检查系统是否已关闭
    # 参数：系统句柄
    # 返回：布尔值（0表示未关闭，非0表示已关闭）
    _lib.orb_slam3_is_shutdown.argtypes = [c_void_p]
    _lib.orb_slam3_is_shutdown.restype = c_int
    
    # ========================================================================
    # 视觉跟踪相关函数
    # ========================================================================
    
    # RGB-D跟踪函数
    # 参数：系统句柄、RGB图像指针、深度图像指针、图像宽度、图像高度、时间戳
    # 返回：相机位姿结构体（包含4x4变换矩阵和有效性标志）
    _lib.orb_slam3_track_rgbd.argtypes = [c_void_p, POINTER(ctypes.c_ubyte), POINTER(c_float), c_int, c_int, c_double]
    _lib.orb_slam3_track_rgbd.restype = CameraPose
    
    # 双目立体跟踪函数
    # 参数：系统句柄、左图像指针、右图像指针、图像宽度、图像高度、时间戳
    # 返回：相机位姿结构体
    _lib.orb_slam3_track_stereo.argtypes = [c_void_p, POINTER(ctypes.c_ubyte), POINTER(ctypes.c_ubyte), c_int, c_int, c_double]
    _lib.orb_slam3_track_stereo.restype = CameraPose
    
    # 单目跟踪函数
    # 参数：系统句柄、图像指针、图像宽度、图像高度、时间戳
    # 返回：相机位姿结构体
    _lib.orb_slam3_track_monocular.argtypes = [c_void_p, POINTER(ctypes.c_ubyte), c_int, c_int, c_double]
    _lib.orb_slam3_track_monocular.restype = CameraPose
    
    # ========================================================================
    # 状态查询相关函数
    # ========================================================================
    
    # 获取当前跟踪状态
    # 参数：系统句柄
    # 返回：跟踪状态枚举值（-1到3的整数）
    _lib.orb_slam3_get_tracking_state.argtypes = [c_void_p]
    _lib.orb_slam3_get_tracking_state.restype = c_int
    
    # 检查跟踪是否丢失
    # 参数：系统句柄
    # 返回：布尔值（0表示未丢失，非0表示丢失）
    _lib.orb_slam3_is_lost.argtypes = [c_void_p]
    _lib.orb_slam3_is_lost.restype = c_int
    
    # 检查地图是否发生变化
    # 参数：系统句柄
    # 返回：布尔值（0表示未变化，非0表示有变化）
    _lib.orb_slam3_map_changed.argtypes = [c_void_p]
    _lib.orb_slam3_map_changed.restype = c_int
    
    # ========================================================================
    # 地图数据获取相关函数
    # ========================================================================
    
    # 获取当前帧跟踪到的地图点
    # 参数：系统句柄
    # 返回：地图点集合结构体
    _lib.orb_slam3_get_tracked_map_points.argtypes = [c_void_p]
    _lib.orb_slam3_get_tracked_map_points.restype = MapPoints
    
    # 获取当前帧跟踪到的关键点
    # 参数：系统句柄
    # 返回：关键点集合结构体
    _lib.orb_slam3_get_tracked_keypoints.argtypes = [c_void_p]
    _lib.orb_slam3_get_tracked_keypoints.restype = TrackedKeyPoints
    
    # 获取所有地图点（整个地图）
    # 参数：系统句柄
    # 返回：地图点集合结构体
    _lib.orb_slam3_get_all_map_points.argtypes = [c_void_p]
    _lib.orb_slam3_get_all_map_points.restype = MapPoints
    
    # ========================================================================
    # 内存管理相关函数
    # ========================================================================
    
    # 释放地图点集合的内存
    # 参数：指向地图点集合结构体的指针
    # 返回：无
    # 注意：必须调用此函数释放get_*_map_points返回的内存
    _lib.orb_slam3_free_map_points.argtypes = [POINTER(MapPoints)]
    _lib.orb_slam3_free_map_points.restype = None
    
    # 释放关键点集合的内存
    # 参数：指向关键点集合结构体的指针
    # 返回：无
    # 注意：必须调用此函数释放get_tracked_keypoints返回的内存
    _lib.orb_slam3_free_keypoints.argtypes = [POINTER(TrackedKeyPoints)]
    _lib.orb_slam3_free_keypoints.restype = None
    
    # ========================================================================
    # 模式控制相关函数
    # ========================================================================
    
    # 激活纯定位模式（停止建图，只进行定位）
    # 参数：系统句柄
    # 返回：无
    # 用途：在已知地图中进行纯定位，节省计算资源
    _lib.orb_slam3_activate_localization_mode.argtypes = [c_void_p]
    _lib.orb_slam3_activate_localization_mode.restype = None
    
    # 停用纯定位模式（恢复SLAM建图功能）
    # 参数：系统句柄
    # 返回：无
    _lib.orb_slam3_deactivate_localization_mode.argtypes = [c_void_p]
    _lib.orb_slam3_deactivate_localization_mode.restype = None
    
    # 重置SLAM系统（清除地图和跟踪状态）
    # 参数：系统句柄
    # 返回：无
    # 注意：这将删除所有构建的地图数据
    _lib.orb_slam3_reset.argtypes = [c_void_p]
    _lib.orb_slam3_reset.restype = None
    
    # ========================================================================
    # 轨迹保存相关函数
    # ========================================================================
    
    # 以TUM格式保存相机轨迹
    # 参数：系统句柄、输出文件路径
    # 返回：无
    # TUM格式：timestamp tx ty tz qx qy qz qw
    _lib.orb_slam3_save_trajectory_tum.argtypes = [c_void_p, c_char_p]
    _lib.orb_slam3_save_trajectory_tum.restype = None
    
    # 以KITTI格式保存相机轨迹  
    # 参数：系统句柄、输出文件路径
    # 返回：无
    # KITTI格式：12个元素的变换矩阵（3x4）
    _lib.orb_slam3_save_trajectory_kitti.argtypes = [c_void_p, c_char_p]
    _lib.orb_slam3_save_trajectory_kitti.restype = None

# 调用函数签名设置，完成ctypes绑定初始化
setup_function_signatures()

# ============================================================================
# 第五部分：主要Python封装类
# ============================================================================

class ORBSLAMSystem:
    """
    ORB-SLAM3系统的Python封装类
    
    这个类是整个Python绑定的核心，提供了完整的SLAM功能接口。
    它封装了底层的C++实现，使得Python用户可以方便地使用ORB-SLAM3。
    
    设计模式：
    - 采用了Facade（外观）模式，简化了复杂的C++接口
    - 使用RAII（资源获取即初始化）管理C++对象的生命周期
    - 提供了Pythonic的接口，隐藏了ctypes的复杂性
    
    主要功能：
    1. SLAM系统的初始化和配置
    2. 多种传感器类型的支持（单目、双目、RGB-D、IMU组合）
    3. 实时相机跟踪和位姿估计
    4. 地图构建和管理
    5. 系统状态监控
    6. 数据导出和可视化支持
    
    使用流程：
    1. 创建系统实例：指定词汇文件、配置文件、传感器类型
    2. 循环处理图像帧：调用相应的track_*方法
    3. 获取结果：位姿、地图点、跟踪状态等
    4. 保存数据：轨迹、地图等（可选）
    5. 系统清理：自动调用析构函数
    
    注意事项：
    - 确保在创建实例前已正确编译并放置动态库文件
    - 词汇文件和配置文件路径必须正确
    - 图像数据格式必须符合要求（类型、尺寸等）
    - 时间戳应该单调递增且精确
    """
    
    def __init__(self, vocab_file, settings_file, sensor_type=SensorType.RGBD, use_viewer=True):
        """
        初始化ORB-SLAM3系统
        
        这是系统的构造函数，负责创建和配置SLAM系统实例。
        这个过程包括加载词汇文件、解析配置文件、初始化各个子系统。
        
        参数详解：
            vocab_file (str): ORB特征词汇文件路径
                - 这是一个预训练的词汇树文件，通常是ORBvoc.txt
                - 用于快速的特征匹配和回环检测
                - 文件较大（几百MB），加载需要一些时间
                - 必须与ORB-SLAM3版本兼容
                
            settings_file (str): 系统配置文件路径
                - YAML格式的配置文件，包含相机参数、ORB参数等
                - 相机内参：fx, fy, cx, cy
                - 畸变参数：k1, k2, p1, p2, k3
                - ORB参数：特征点数量、尺度因子、金字塔层数等
                - 传感器特定参数：深度阈值、基线等
                
            sensor_type (SensorType): 传感器类型枚举
                - 决定了系统使用的跟踪算法和初始化策略
                - RGBD: 适用于Kinect、RealSense等深度相机
                - STEREO: 适用于双目立体相机
                - MONOCULAR: 适用于单个相机（需要运动初始化）
                - IMU_*: 包含IMU的传感器组合
                
            use_viewer (bool): 是否启用可视化窗口
                - True: 显示实时跟踪结果和3D地图
                - False: 纯后台运行，适合服务器环境
                - 可视化使用Pangolin库
        
        初始化过程：
        1. 参数验证和预处理
        2. 字符串编码转换（Python str -> C char*）
        3. 调用C++构造函数创建系统实例
        4. 验证创建是否成功
        
        可能的异常：
        - RuntimeError: 系统创建失败
            * 词汇文件路径错误或文件损坏
            * 配置文件格式错误或参数无效
            * 依赖库缺失或版本不兼容
            * 内存不足
        
        内存管理：
        - _system_handle存储C++对象的指针
        - 析构函数会自动释放C++对象
        - 使用RAII模式确保资源安全
        """
        # 初始化系统句柄为空，用于存储C++对象指针
        self._system_handle = None
        
        # 字符串编码转换：Python的str需要转换为bytes才能传递给C++
        # UTF-8编码确保支持各种文件路径字符
        vocab_bytes = vocab_file.encode('utf-8')
        settings_bytes = settings_file.encode('utf-8')
        
        # 调用底层C++函数创建SLAM系统实例
        # 参数顺序：词汇文件、配置文件、传感器类型、是否启用可视化
        # int()转换确保枚举值被正确传递为整数
        self._system_handle = _lib.orb_slam3_create_system(
            vocab_bytes, settings_bytes, int(sensor_type), int(use_viewer)
        )
        
        # 验证系统创建是否成功
        # C++函数失败时返回NULL指针，在Python中表示为False
        if not self._system_handle:
            raise RuntimeError("Failed to create ORB-SLAM3 system")
    
    def __del__(self):
        """
        析构函数（对象销毁时自动调用）
        
        负责清理C++对象和释放相关资源。这是RAII模式的重要组成部分，
        确保即使Python对象被垃圾回收，C++资源也能正确释放。
        
        执行时机：
        - Python对象引用计数归零时
        - 程序退出时
        - 显式删除对象时（del object）
        
        清理内容：
        - 调用C++的销毁函数释放SLAM系统对象
        - 停止所有后台线程（跟踪、建图、回环检测）
        - 释放内存中的地图数据
        - 关闭可视化窗口
        
        注意：
        - 这个方法通常不需要手动调用
        - 如果需要立即释放资源，可以调用shutdown()方法
        """
        if self._system_handle:
            _lib.orb_slam3_destroy_system(self._system_handle)
    
    def track_rgbd(self, rgb_image, depth_image, timestamp):
        """
        RGB-D图像跟踪处理
        
        这是RGB-D SLAM的核心函数，处理彩色图像和深度图像来估计相机位姿。
        RGB-D传感器同时提供颜色信息和深度信息，使得SLAM系统能够直接获得3D结构。
        
        RGB-D SLAM优势：
        - 直接获得尺度信息，无需初始化过程
        - 深度辅助特征匹配，提高鲁棒性
        - 可以处理低纹理环境
        - 实时性能好
        
        算法流程：
        1. 特征提取：从RGB图像中提取ORB特征点
        2. 深度关联：为特征点分配深度值（来自深度图像）
        3. 特征匹配：与前一帧或地图点进行匹配
        4. 位姿估计：使用PnP算法计算相机位姿
        5. 地图更新：添加新的地图点或更新现有点
        6. 关键帧决策：判断是否需要插入新的关键帧
        
        参数详解：
            rgb_image (numpy.ndarray): RGB彩色图像
                - 形状：(H, W, 3) 或 (H, W) 对于灰度图
                - 数据类型：uint8 (0-255)
                - 颜色通道顺序：通常是BGR（OpenCV格式）
                - 用于特征提取和可视化
                
            depth_image (numpy.ndarray): 深度图像
                - 形状：(H, W) 必须与RGB图像尺寸一致
                - 数据类型：float32，单位通常是米
                - 无效深度用0或NaN表示
                - 深度值范围：通常0.1m到10m之间
                
            timestamp (float): 时间戳
                - 单位：秒（支持小数）
                - 必须单调递增
                - 用于运动模型预测和IMU融合
                - 精度影响跟踪稳定性
        
        返回值：
            numpy.ndarray 或 None: 相机位姿矩阵
                - 成功时：4x4齐次变换矩阵（世界坐标系到相机坐标系）
                - 失败时：None（跟踪丢失或初始化未完成）
                - 矩阵格式：SE(3)变换，包含旋转和平移
        
        常见问题和解决方案：
        1. 返回None：
           - 检查图像质量和特征点数量
           - 确认深度图像有效性
           - 验证相机标定参数
           
        2. 跟踪不稳定：
           - 调整ORB特征参数
           - 检查深度图像噪声
           - 优化光照条件
           
        3. 性能问题：
           - 降低图像分辨率
           - 减少特征点数量
           - 关闭可视化
        
        使用示例：
            pose = slam.track_rgbd(rgb_img, depth_img, time.time())
            if pose is not None:
                print(f"相机位置: {pose[:3, 3]}")
        """
        # 系统状态检查：确保SLAM系统已正确初始化
        if not self._system_handle:
            raise RuntimeError("System not initialized")
        
        # 输入数据验证和类型转换
        # RGB图像必须是uint8类型（0-255范围）
        if rgb_image.dtype != np.uint8:
            rgb_image = rgb_image.astype(np.uint8)
        # 深度图像必须是float32类型（精度要求）
        if depth_image.dtype != np.float32:
            depth_image = depth_image.astype(np.float32)
        
        # 获取图像尺寸：RGB图像可能是3通道，所以取前两个维度
        height, width = rgb_image.shape[:2]
        
        # 数据指针转换：将numpy数组转换为ctypes可用的指针
        # 这样C++函数就可以直接访问Python的内存数据
        rgb_ptr = rgb_image.ctypes.data_as(POINTER(ctypes.c_ubyte))
        depth_ptr = depth_image.ctypes.data_as(POINTER(c_float))
        
        # 调用底层C++跟踪函数
        # 参数顺序：系统句柄、RGB指针、深度指针、宽度、高度、时间戳
        pose = _lib.orb_slam3_track_rgbd(
            self._system_handle, rgb_ptr, depth_ptr, width, height, timestamp
        )
        
        # 结果处理：检查跟踪是否成功
        if pose.valid:
            # 成功：将C结构体转换为numpy矩阵
            # data是16个浮点数，reshape成4x4矩阵
            pose_matrix = np.array(pose.data).reshape(4, 4)
            return pose_matrix
        else:
            # 失败：返回None表示跟踪丢失
            return None
    
    def track_stereo(self, left_image, right_image, timestamp):
        """
        双目立体视觉跟踪处理
        
        处理来自双目立体相机的图像对，通过视差计算获得深度信息并估计相机位姿。
        双目视觉是经典的3D视觉方法，通过两个相机的视角差异恢复场景的三维结构。
        
        双目SLAM原理：
        - 利用左右相机的视差计算深度
        - 无需额外的深度传感器
        - 适用于室外大范围环境
        - 对光照变化相对鲁棒
        
        算法流程：
        1. 特征提取：分别从左右图像提取ORB特征
        2. 立体匹配：在左右图像间匹配特征点
        3. 深度计算：根据视差和基线计算深度
        4. 位姿估计：使用3D-2D对应关系估计位姿
        5. 地图更新：三角化新的地图点
        
        参数说明：
            left_image (numpy.ndarray): 左目图像
                - 形状：(H, W) 灰度图像
                - 数据类型：uint8
                - 通常是已矫正的图像
                
            right_image (numpy.ndarray): 右目图像  
                - 形状：(H, W) 必须与左图一致
                - 数据类型：uint8
                - 与左图已进行立体矫正
                
            timestamp (float): 时间戳（秒）
        
        返回值：
            numpy.ndarray 或 None: 4x4相机位姿矩阵
        
        注意事项：
        - 需要准确的双目标定参数
        - 图像必须经过立体矫正
        - 基线距离影响深度精度
        - 远距离目标的深度精度较低
        """
        if not self._system_handle:
            raise RuntimeError("System not initialized")
        
        # 输入验证和类型转换
        if left_image.dtype != np.uint8:
            left_image = left_image.astype(np.uint8)
        if right_image.dtype != np.uint8:
            right_image = right_image.astype(np.uint8)
        
        height, width = left_image.shape
        
        # 转换为ctypes指针
        left_ptr = left_image.ctypes.data_as(POINTER(ctypes.c_ubyte))
        right_ptr = right_image.ctypes.data_as(POINTER(ctypes.c_ubyte))
        
        # 调用C++双目跟踪函数
        pose = _lib.orb_slam3_track_stereo(
            self._system_handle, left_ptr, right_ptr, width, height, timestamp
        )
        
        if pose.valid:
            pose_matrix = np.array(pose.data).reshape(4, 4)
            return pose_matrix
        else:
            return None
    
    def track_monocular(self, image, timestamp):
        """
        单目视觉跟踪处理
        
        处理单个相机的图像来估计相机位姿，是最基础但也最具挑战性的SLAM模式。
        单目SLAM只能恢复相对尺度，需要特殊的初始化过程。
        
        单目SLAM特点：
        - 硬件要求最低，只需一个相机
        - 尺度不确定性（scale ambiguity）
        - 需要运动初始化过程
        - 对快速旋转敏感
        
        初始化过程：
        1. 特征检测：检测足够多的特征点
        2. 特征匹配：在连续帧间建立对应关系
        3. 基础矩阵估计：使用RANSAC估计几何关系
        4. 三角化：计算初始地图点
        5. 捆绑调整：优化初始地图和位姿
        
        参数说明：
            image (numpy.ndarray): 输入图像
                - 形状：(H, W) 灰度图像
                - 数据类型：uint8
                - 建议有足够的纹理特征
                
            timestamp (float): 时间戳（秒）
                - 用于运动模型预测
                - 必须单调递增
        
        返回值：
            numpy.ndarray 或 None: 4x4相机位姿矩阵
                - 初始化期间可能返回None
                - 跟踪丢失时返回None
                - 成功时返回位姿矩阵（任意尺度）
        
        使用建议：
        - 初始化时缓慢移动相机
        - 避免纯旋转运动
        - 确保场景有足够纹理
        - 保持适当的运动速度
        """
        if not self._system_handle:
            raise RuntimeError("System not initialized")
        
        # 输入验证和类型转换
        if image.dtype != np.uint8:
            image = image.astype(np.uint8)
        
        height, width = image.shape
        
        # 转换为ctypes指针
        image_ptr = image.ctypes.data_as(POINTER(ctypes.c_ubyte))
        
        # 调用C++单目跟踪函数
        pose = _lib.orb_slam3_track_monocular(
            self._system_handle, image_ptr, width, height, timestamp
        )
        
        if pose.valid:
            pose_matrix = np.array(pose.data).reshape(4, 4)
            return pose_matrix
        else:
            return None
    
    def get_tracking_state(self):
        """
        获取当前跟踪状态
        
        返回系统的当前状态，用于监控SLAM系统的运行情况。
        这对于调试和用户界面反馈非常重要。
        
        状态含义：
        - SYSTEM_NOT_READY(-1): 系统未就绪
        - NO_IMAGES_YET(0): 还未接收图像
        - NOT_INITIALIZED(1): 未完成初始化
        - OK(2): 正常跟踪
        - LOST(3): 跟踪丢失
        
        返回值：
            TrackingState: 当前跟踪状态枚举值
        """
        if not self._system_handle:
            return TrackingState.SYSTEM_NOT_READY
        return TrackingState(_lib.orb_slam3_get_tracking_state(self._system_handle))
    
    def is_lost(self):
        """
        检查跟踪是否丢失
        
        这是一个便捷函数，快速判断是否需要采取恢复措施。
        跟踪丢失可能由多种原因引起：运动过快、遮挡、光照变化等。
        
        返回值：
            bool: True表示跟踪丢失，False表示正常跟踪
            
        恢复策略：
        - 减慢移动速度
        - 回到已知区域
        - 改善光照条件
        - 或调用reset()重新开始
        """
        if not self._system_handle:
            return True
        return bool(_lib.orb_slam3_is_lost(self._system_handle))
    
    def map_changed(self):
        """
        检查地图是否发生变化
        
        用于检测地图是否有更新，这对于实时可视化和数据同步很有用。
        地图变化包括：新增地图点、删除无效点、关键帧更新等。
        
        返回值：
            bool: True表示地图有变化，False表示无变化
            
        应用场景：
        - 实时地图可视化更新
        - 地图数据导出时机判断
        - 系统性能监控
        """
        if not self._system_handle:
            return False
        return bool(_lib.orb_slam3_map_changed(self._system_handle))
    
    def get_tracked_map_points(self):
        """
        获取当前帧跟踪到的地图点
        
        返回当前帧中成功跟踪到的所有3D地图点的坐标。
        这些点是已经在地图中存在并被当前帧观测到的特征点。
        
        用途：
        - 可视化当前帧的跟踪结果
        - 分析跟踪质量（点的数量和分布）
        - 导出部分地图数据
        - 调试跟踪算法
        
        返回值：
            numpy.ndarray: 3D点坐标数组
                - 形状：(N, 3) 其中N是点的数量
                - 每行包含一个点的(x, y, z)坐标
                - 坐标系：世界坐标系
                - 如果没有跟踪到点，返回空数组
        
        注意事项：
        - 返回的点数量反映跟踪质量
        - 点的分布影响位姿估计精度
        - 内存会被自动管理和释放
        """
        if not self._system_handle:
            return np.array([])
        
        # 调用C++函数获取地图点数据
        map_points = _lib.orb_slam3_get_tracked_map_points(self._system_handle)
        
        if map_points.num_points > 0:
            # 将C结构体数组转换为numpy数组
            # ctypeslib.as_array创建数组视图，避免数据复制
            points_array = np.ctypeslib.as_array(map_points.points, shape=(map_points.num_points,))
            # 提取x,y,z坐标并组织成numpy数组
            result = np.array([(p.x, p.y, p.z) for p in points_array])
            
            # 释放C++分配的内存，防止内存泄漏
            _lib.orb_slam3_free_map_points(ctypes.byref(map_points))
            return result
        else:
            return np.array([])
    
    def get_tracked_keypoints(self):
        """
        获取当前帧跟踪到的关键点
        
        返回当前帧中成功跟踪到的所有2D关键点信息。
        这些关键点已经与地图中的特征建立了对应关系，是SLAM跟踪的基础。
        
        ORB关键点信息：
        - 位置：图像中的像素坐标(x, y)
        - 角度：特征点的主方向，用于旋转不变性
        - 响应：特征强度，值越大表示特征越稳定
        - 层级：金字塔层级，用于尺度不变性
        
        用途：
        - 可视化跟踪结果（在图像上绘制特征点）
        - 分析特征分布质量
        - 调试特征检测和匹配算法
        - 评估跟踪稳定性
        
        返回值：
            list: 关键点字典列表
                - 每个字典包含以下键值：
                  * 'x' (float): X像素坐标
                  * 'y' (float): Y像素坐标  
                  * 'angle' (float): 主方向角度（弧度）
                  * 'response' (float): 特征响应强度
                  * 'octave' (int): 金字塔层级
                - 如果没有跟踪到关键点，返回空列表
        
        数据解释：
        - x, y: 特征点在当前图像中的精确位置
        - angle: 用于描述子计算和匹配的方向信息
        - response: Harris角点响应或其他强度度量
        - octave: 0表示原始分辨率，1表示1/2分辨率，以此类推
        
        质量指标：
        - 关键点数量：通常需要50+个点才能稳定跟踪
        - 分布均匀性：点应该在图像中均匀分布
        - 响应强度：高响应值表示更稳定的特征
        
        使用示例：
            keypoints = slam.get_tracked_keypoints()
            print(f"跟踪到 {len(keypoints)} 个关键点")
            for kp in keypoints:
                print(f"位置: ({kp['x']:.1f}, {kp['y']:.1f}), "
                      f"强度: {kp['response']:.3f}")
        """
        if not self._system_handle:
            return []
        
        # 调用C++函数获取关键点数据
        keypoints = _lib.orb_slam3_get_tracked_keypoints(self._system_handle)
        
        if keypoints.num_keypoints > 0:
            # 将C结构体数组转换为Python字典列表
            # 这种格式更符合Python的使用习惯
            kps_array = np.ctypeslib.as_array(keypoints.keypoints, shape=(keypoints.num_keypoints,))
            result = []
            for kp in kps_array:
                # 将每个C结构体转换为Python字典
                result.append({
                    'x': kp.x,              # X像素坐标
                    'y': kp.y,              # Y像素坐标
                    'angle': kp.angle,      # 主方向角度
                    'response': kp.response, # 特征响应强度
                    'octave': kp.octave     # 金字塔层级
                })
            
            # 释放C++分配的内存，防止内存泄漏
            _lib.orb_slam3_free_keypoints(ctypes.byref(keypoints))
            return result
        else:
            return []
    
    def activate_localization_mode(self):
        """
        激活纯定位模式
        
        切换到定位模式后，系统将停止建图，只进行相机位姿跟踪。
        这在已知环境中进行导航时很有用，可以节省计算资源。
        
        定位模式特点：
        - 不再创建新的地图点
        - 不插入新的关键帧
        - 计算资源需求降低
        - 适用于重定位和导航
        
        应用场景：
        - 在已构建的地图中导航
        - 长期运行的导航系统
        - 计算资源受限的环境
        - 地图已经足够完整的情况
        
        注意：
        - 可以通过deactivate_localization_mode()恢复建图
        - 定位精度依赖于现有地图质量
        """
        if self._system_handle:
            _lib.orb_slam3_activate_localization_mode(self._system_handle)
    
    def deactivate_localization_mode(self):
        """
        停用纯定位模式
        
        恢复完整的SLAM功能，系统将重新开始建图和地图扩展。
        这允许系统适应环境变化和探索新区域。
        
        恢复功能：
        - 重新启用地图点创建
        - 恢复关键帧插入
        - 启用地图扩展和更新
        - 恢复回环检测功能
        
        适用情况：
        - 探索新的未知区域
        - 环境发生了显著变化
        - 需要更新和扩展现有地图
        - 提高定位精度
        """
        if self._system_handle:
            _lib.orb_slam3_deactivate_localization_mode(self._system_handle)
    
    def reset(self):
        """
        重置SLAM系统
        
        清除所有地图数据和跟踪状态，系统回到初始状态。
        这是一个"重新开始"的操作，会丢失所有已构建的地图。
        
        重置内容：
        - 清除所有地图点
        - 删除所有关键帧
        - 重置跟踪状态
        - 清空关键帧数据库
        - 重置回环检测器
        
        使用场景：
        - 跟踪完全丢失且无法恢复
        - 切换到完全不同的环境
        - 地图质量严重下降
        - 开始新的建图任务
        
        注意：
        - 这是一个不可逆操作
        - 所有历史数据将丢失
        - 需要重新进行初始化
        """
        if self._system_handle:
            _lib.orb_slam3_reset(self._system_handle)
    
    def shutdown(self):
        """
        关闭SLAM系统
        
        停止所有SLAM相关的后台线程但保留对象，为程序安全退出做准备。
        与reset()不同，这个操作是为了安全关闭系统。
        
        关闭内容：
        - 停止跟踪线程
        - 停止局部建图线程
        - 停止回环检测线程
        - 关闭可视化窗口
        - 保存必要的数据
        
        使用时机：
        - 程序准备退出时
        - 长时间暂停SLAM功能
        - 系统资源紧张时
        
        注意：
        - 关闭后系统无法继续处理图像
        - 地图数据仍然保留在内存中
        - 可以在关闭前保存轨迹数据
        """
        if self._system_handle:
            _lib.orb_slam3_shutdown(self._system_handle)
    
    def is_shutdown(self):
        """
        检查系统是否已关闭
        
        返回系统的关闭状态，用于确认shutdown()操作是否成功完成。
        
        返回值：
            bool: True表示系统已关闭，False表示仍在运行
            
        用途：
        - 验证关闭操作完成状态
        - 程序退出前的状态检查
        - 系统状态监控
        """
        if not self._system_handle:
            return True
        return bool(_lib.orb_slam3_is_shutdown(self._system_handle))
    
    def save_trajectory_tum(self, filename):
        """
        以TUM格式保存相机轨迹
        
        TUM (Technical University of Munich) 格式是一种常用的轨迹数据格式，
        特别适用于与其他SLAM系统进行比较和评估。
        
        TUM格式特点：
        - 每行包含：timestamp tx ty tz qx qy qz qw
        - timestamp: 时间戳（秒）
        - tx, ty, tz: 平移向量（米）
        - qx, qy, qz, qw: 四元数表示的旋转
        - 使用空格分隔
        - 文本格式，易于读取和处理
        
        参数：
            filename (str): 输出文件路径
                - 建议使用.txt扩展名
                - 文件会被覆盖如果已存在
                - 路径必须是可写的
        
        使用场景：
        - 与TUM数据集进行比较
        - 轨迹质量评估
        - 与其他SLAM算法比较
        - 数据集制作和分享
        
        注意事项：
        - 确保有足够的磁盘空间
        - 文件路径必须有写权限
        - 建议在关闭系统前调用
        """
        if self._system_handle:
            filename_bytes = filename.encode('utf-8')
            _lib.orb_slam3_save_trajectory_tum(self._system_handle, filename_bytes)
    
    def save_trajectory_kitti(self, filename):
        """
        以KITTI格式保存相机轨迹
        
        KITTI格式是自动驾驶领域广泛使用的轨迹数据格式，
        特别适用于车载SLAM系统的评估。
        
        KITTI格式特点：
        - 每行包含12个浮点数（3x4变换矩阵）
        - 格式：r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
        - 表示从第0帧到当前帧的累积变换
        - 不包含时间戳信息
        - 矩阵按行展开存储
        
        变换矩阵含义：
        [r11 r12 r13 tx]   - 旋转矩阵R和平移向量t组成
        [r21 r22 r23 ty]   - 表示相机在世界坐标系中的位姿
        [r31 r32 r33 tz]
        
        参数：
            filename (str): 输出文件路径
                - 建议使用.txt扩展名
                - 与KITTI数据集格式兼容
        
        使用场景：
        - 与KITTI数据集比较
        - 自动驾驶应用评估
        - 室外大场景SLAM评估
        - 车载导航系统测试
        
        与TUM格式对比：
        - KITTI: 矩阵形式，无时间戳，适合车载
        - TUM: 四元数形式，有时间戳，适合室内
        """
        if self._system_handle:
            filename_bytes = filename.encode('utf-8')
            _lib.orb_slam3_save_trajectory_kitti(self._system_handle, filename_bytes)

# ============================================================================
# 文件结束：教学级注释完成
# ============================================================================

# 使用示例（可选，供教学参考）：
"""
基本使用流程：

1. 创建SLAM系统
slam = ORBSLAMSystem(
    vocab_file="ORBvoc.txt",
    settings_file="config.yaml", 
    sensor_type=SensorType.RGBD,
    use_viewer=True
)

2. 处理图像序列
for rgb_img, depth_img, timestamp in image_sequence:
    pose = slam.track_rgbd(rgb_img, depth_img, timestamp)
    
    if pose is not None:
        print(f"相机位置: {pose[:3, 3]}")
        
        # 获取地图点
        map_points = slam.get_tracked_map_points()
        print(f"跟踪到 {len(map_points)} 个地图点")
    else:
        print("跟踪丢失")

3. 保存结果并清理
slam.save_trajectory_tum("trajectory.txt")
slam.shutdown()
"""
