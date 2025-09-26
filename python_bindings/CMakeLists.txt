cmake_minimum_required(VERSION 3.5)
project(ORB_SLAM3_Python_Bindings)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

# Find ORB-SLAM3 components
set(ORB_SLAM3_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Include directories
include_directories(
    ${ORB_SLAM3_ROOT_DIR}
    ${ORB_SLAM3_ROOT_DIR}/include
    ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/g2o
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

# Set library directories
link_directories(
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
    ${ORB_SLAM3_ROOT_DIR}/lib
)

# Find ORB-SLAM3 library
find_library(ORB_SLAM3_LIBRARY
    NAMES ORB_SLAM3
    PATHS ${ORB_SLAM3_ROOT_DIR}/lib
    NO_DEFAULT_PATH
)

if(NOT ORB_SLAM3_LIBRARY)
    message(FATAL_ERROR "ORB-SLAM3 library not found. Please build ORB-SLAM3 first.")
endif()

# Find third-party libraries
find_library(DBoW2_LIBRARY
    NAMES DBoW2
    PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
    NO_DEFAULT_PATH
)

find_library(G2O_LIBRARY
    NAMES g2o
    PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
    NO_DEFAULT_PATH
)

# Create shared library for Python bindings
add_library(orb_slam3_python SHARED
    orb_slam3_wrapper.cpp
)

# Set library properties
set_target_properties(orb_slam3_python PROPERTIES
    PREFIX ""
    SUFFIX ".so"
    POSITION_INDEPENDENT_CODE ON
)

# Link libraries
target_link_libraries(orb_slam3_python
    ${ORB_SLAM3_LIBRARY}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${DBoW2_LIBRARY}
    ${G2O_LIBRARY}
    GL
    GLU
    GLEW
    pthread
    -lboost_serialization
    -lcrypto
)

# Compiler-specific options
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# Add definitions
add_definitions(-DCOMPILED_WITH_CMAKE)

# Print information
message(STATUS "ORB-SLAM3 library found at: ${ORB_SLAM3_LIBRARY}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Eigen3 version: ${EIGEN3_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Installation
install(TARGETS orb_slam3_python
    LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
)

install(FILES orb_slam3_wrapper.h
    DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
)
