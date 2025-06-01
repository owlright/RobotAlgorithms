cmake_minimum_required(VERSION 3.10.0)
project(RobotAlgorithms VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

find_package(Eigen3 REQUIRED)
# 查找PCL (包含KdTreeFLANN)
find_package(PCL 1.8 REQUIRED COMPONENTS common kdtree io)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)
include_directories(include/)

# 根据构建类型设置编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Build type: Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0") # 启用调试符号，禁用优化
else()
    message(STATUS "Build type: Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") # 启用优化
endif()

add_executable(test_icp test_icp.cpp src/icp3d.cpp)
add_executable(matrix_solver_stability_demo matrix_solver_stability_demo.cpp)
# 添加Eigen3库
target_link_libraries(test_icp
    PRIVATE Eigen3::Eigen
    PRIVATE ${PCL_LIBRARIES}
    PRIVATE Sophus::Sophus
    PRIVATE glog::glog
    PRIVATE gflags
    PRIVATE TBB::tbb
)

target_link_libraries(matrix_solver_stability_demo
    PRIVATE Eigen3::Eigen
)
