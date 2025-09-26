#!/bin/bash

# Build script for ORB-SLAM3 Python bindings

set -e

echo "Building ORB-SLAM3 Python bindings..."

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ORB_SLAM3_ROOT="$SCRIPT_DIR/.."

# Check if ORB-SLAM3 is built
if [ ! -f "$ORB_SLAM3_ROOT/lib/libORB_SLAM3.so" ]; then
    echo "Error: ORB-SLAM3 library not found!"
    echo "Please build ORB-SLAM3 first by running ./build.sh in the root directory"
    exit 1
fi

# Create build directory
BUILD_DIR="$SCRIPT_DIR/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with CMake
echo "Configuring with CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-O3 -march=native"

# Build
echo "Building..."
make -j$(nproc)

# Copy the library to the bindings directory
if [ -f "orb_slam3_python.so" ]; then
    cp orb_slam3_python.so "$SCRIPT_DIR/"
    echo "Build successful! Library copied to $SCRIPT_DIR/orb_slam3_python.so"
else
    echo "Error: Build failed - library not found"
    exit 1
fi

echo "Python bindings built successfully!"
echo ""
echo "To test the bindings, you can run:"
echo "  cd $SCRIPT_DIR"
echo "  python3 example_rgbd.py --vocab /path/to/ORBvoc.txt --settings /path/to/settings.yaml"
echo ""
echo "Make sure you have the required Python packages installed:"
echo "  pip install numpy opencv-python"
