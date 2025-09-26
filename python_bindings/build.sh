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

# Check if the library was built successfully
# The library should be in the python_bindings directory due to LIBRARY_OUTPUT_DIRECTORY setting
if [ -f "$SCRIPT_DIR/orb_slam3_python.so" ]; then
    echo "Build successful! Library found at $SCRIPT_DIR/orb_slam3_python.so"
elif [ -f "orb_slam3_python.so" ]; then
    # If it's in the build directory, copy it to the bindings directory
    cp orb_slam3_python.so "$SCRIPT_DIR/"
    echo "Build successful! Library copied to $SCRIPT_DIR/orb_slam3_python.so"
else
    echo "Error: Build failed - library not found"
    echo "Checked locations:"
    echo "  - $SCRIPT_DIR/orb_slam3_python.so"
    echo "  - $BUILD_DIR/orb_slam3_python.so"
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
