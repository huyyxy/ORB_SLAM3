#!/bin/bash

echo "开始清理 ORB_SLAM3 构建文件..."

# 设置脚本在出错时退出
set -e

# 函数：安全删除目录或文件
safe_remove() {
    if [ -e "$1" ]; then
        echo "删除: $1"
        rm -rf "$1"
    else
        echo "未找到: $1 (跳过)"
    fi
}

# 进入脚本所在目录
cd "$(dirname "$0")"

echo ""
echo "1. 清理主构建目录..."
safe_remove "build"

echo ""
echo "2. 清理第三方库构建目录..."
safe_remove "Thirdparty/DBoW2/build"
safe_remove "Thirdparty/g2o/build"
safe_remove "Thirdparty/Sophus/build"

echo ""
echo "3. 清理库文件..."
safe_remove "lib"
safe_remove "Thirdparty/DBoW2/lib"
safe_remove "Thirdparty/g2o/lib"

echo ""
echo "4. 清理可执行文件..."

# RGB-D 示例
safe_remove "Examples/RGB-D/rgbd_tum"
safe_remove "Examples/RGB-D/rgbd_realsense_D435i"

# RGB-D Inertial 示例
safe_remove "Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i"

# Stereo 示例
safe_remove "Examples/Stereo/stereo_kitti"
safe_remove "Examples/Stereo/stereo_euroc"
safe_remove "Examples/Stereo/stereo_tum_vi"
safe_remove "Examples/Stereo/stereo_realsense_t265"
safe_remove "Examples/Stereo/stereo_realsense_D435i"

# Monocular 示例
safe_remove "Examples/Monocular/mono_tum"
safe_remove "Examples/Monocular/mono_kitti"
safe_remove "Examples/Monocular/mono_euroc"
safe_remove "Examples/Monocular/mono_tum_vi"
safe_remove "Examples/Monocular/mono_realsense_t265"
safe_remove "Examples/Monocular/mono_realsense_D435i"

# Monocular Inertial 示例
safe_remove "Examples/Monocular-Inertial/mono_inertial_euroc"
safe_remove "Examples/Monocular-Inertial/mono_inertial_tum_vi"
safe_remove "Examples/Monocular-Inertial/mono_inertial_realsense_t265"
safe_remove "Examples/Monocular-Inertial/mono_inertial_realsense_D435i"

# Stereo Inertial 示例
safe_remove "Examples/Stereo-Inertial/stereo_inertial_euroc"
safe_remove "Examples/Stereo-Inertial/stereo_inertial_tum_vi"
safe_remove "Examples/Stereo-Inertial/stereo_inertial_realsense_t265"
safe_remove "Examples/Stereo-Inertial/stereo_inertial_realsense_D435i"

# Calibration 示例
safe_remove "Examples/Calibration/recorder_realsense_D435i"
safe_remove "Examples/Calibration/recorder_realsense_T265"

echo ""
echo "5. 清理旧版本示例可执行文件..."

# RGB-D 旧版本示例
safe_remove "Examples_old/RGB-D/rgbd_tum_old"
safe_remove "Examples_old/RGB-D/rgbd_realsense_D435i_old"

# RGB-D Inertial 旧版本示例
safe_remove "Examples_old/RGB-D-Inertial/rgbd_inertial_realsense_D435i_old"

# Stereo 旧版本示例
safe_remove "Examples_old/Stereo/stereo_kitti_old"
safe_remove "Examples_old/Stereo/stereo_euroc_old"
safe_remove "Examples_old/Stereo/stereo_tum_vi_old"
safe_remove "Examples_old/Stereo/stereo_realsense_t265_old"
safe_remove "Examples_old/Stereo/stereo_realsense_D435i_old"

# Monocular 旧版本示例
safe_remove "Examples_old/Monocular/mono_tum_old"
safe_remove "Examples_old/Monocular/mono_kitti_old"
safe_remove "Examples_old/Monocular/mono_euroc_old"
safe_remove "Examples_old/Monocular/mono_tum_vi_old"
safe_remove "Examples_old/Monocular/mono_realsense_t265_old"
safe_remove "Examples_old/Monocular/mono_realsense_D435i_old"

# Monocular Inertial 旧版本示例
safe_remove "Examples_old/Monocular-Inertial/mono_inertial_euroc_old"
safe_remove "Examples_old/Monocular-Inertial/mono_inertial_tum_vi_old"
safe_remove "Examples_old/Monocular-Inertial/mono_inertial_realsense_t265_old"
safe_remove "Examples_old/Monocular-Inertial/mono_inertial_realsense_D435i_old"

# Stereo Inertial 旧版本示例
safe_remove "Examples_old/Stereo-Inertial/stereo_inertial_euroc_old"
safe_remove "Examples_old/Stereo-Inertial/stereo_inertial_tum_vi_old"
safe_remove "Examples_old/Stereo-Inertial/stereo_inertial_realsense_t265_old"
safe_remove "Examples_old/Stereo-Inertial/stereo_inertial_realsense_D435i_old"

echo ""
echo "6. 清理词汇表解压文件..."
safe_remove "Vocabulary/ORBvoc.txt"

echo ""
echo "7. 清理Python绑定构建文件..."
safe_remove "python_bindings/build"

echo ""
echo "8. 清理CMake缓存和其他临时文件..."
find . -name "CMakeCache.txt" -type f -delete 2>/dev/null || true
find . -name "CMakeFiles" -type d -exec rm -rf {} + 2>/dev/null || true
find . -name "cmake_install.cmake" -type f -delete 2>/dev/null || true
find . -name "Makefile" -type f -delete 2>/dev/null || true
find . -name "*.so" -type f -delete 2>/dev/null || true
find . -name "*.a" -type f -delete 2>/dev/null || true

echo ""
echo "9. 清理编译器生成的文件..."
find . -name "*.o" -type f -delete 2>/dev/null || true
find . -name "*.obj" -type f -delete 2>/dev/null || true

echo ""
echo "ORB_SLAM3 构建文件清理完成！"
echo ""
echo "提示："
echo "- 如需重新构建，请运行 ./build.sh 或 ./build_new.sh"
echo "- 如需清理后重新初始化环境，请运行 ./init.sh"
