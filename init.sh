sudo apt update
sudo aptitude install cmake
sudo aptitude install gcc
sudo aptitude install g++
sudo aptitude install pkgconf
sudo aptitude install libopencv-core-dev libopencv-highgui-dev libopencv-imgproc-dev libopencv-calib3d-dev libopencv-features2d-dev
sudo aptitude install libopencv-dev python3-opencv
sudo aptitude install libeigen3-dev
sudo aptitude install libgl1-mesa-dev libglew-dev libwayland-dev libxkbcommon-dev wayland-protocols
sudo aptitude install libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols
sudo aptitude install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev
sudo aptitude install libpng-dev libjpeg-dev libeigen3-dev
sudo aptitude install python3-dev python3-pip
sudo aptitude install libeigen3-dev libopencv-dev cmake gcc g++ git libglew-dev libboost-all-dev
sudo aptitude install clang
sudo aptitude install patchelf
sudo aptitude install libepoxy-dev
sudo aptitude install libssl-dev

apt install cmake gcc g++ libopencv-core-dev libopencv-highgui-dev libopencv-imgproc-dev libopencv-calib3d-dev libopencv-features2d-dev libopencv-dev python3-opencv libeigen3-dev libgl1-mesa-dev libglew-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libpng-dev libjpeg-dev python3-dev python3-pip libboost-all-dev clang libepoxy-dev libssl-dev


# 完整的Pangolin安装步骤
# Get Pangolin
cd ~/your_fav_code_directory
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin

# Install dependencies
./scripts/install_prerequisites.sh recommended

# Configure and build
cmake -B build -GNinja
cmake --build build

# 添加安装步骤（重要！）
sudo cmake --build build --target install

# 或者如果不想使用sudo，可以安装到用户目录
cmake --build build --target install -- DESTDIR=~/.local

# Python绑定安装
cmake --build build -t pypangolin_pip_install

# 可选：运行测试
cmake -B build -G Ninja -D BUILD_TESTS=ON
cmake --build build
cd build
ctest