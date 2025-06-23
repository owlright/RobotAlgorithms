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
find_package(TBB)
include_directories(include/)

# 根据构建类型设置编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Build type: Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0") # 启用调试符号，禁用优化
else()
    message(STATUS "Build type: Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") # 启用优化
endif()

add_executable(matrix_solver_stability_demo matrix_solver_stability_demo.cpp)
target_link_libraries(matrix_solver_stability_demo
    PRIVATE Eigen3::Eigen
)

add_executable(test_icp test_icp.cpp src/icp3d.cpp)
add_executable(test_ndt test_ndt.cpp src/ndt3d.cpp)
add_executable(run_imu_integration run_imu_integration.cpp)

# 自动查找所有的 .cpp 文件
file(GLOB_RECURSE LIB_CPP_FILES ${CMAKE_SOURCE_DIR}/src/*.cpp)
add_library(ra ${LIB_CPP_FILES})
target_link_libraries(ra
    PUBLIC Eigen3::Eigen
    PUBLIC ${PCL_LIBRARIES}
    PUBLIC Sophus::Sophus
    PUBLIC glog::glog
    PUBLIC gflags
    PUBLIC pthread
)
# 添加Eigen3库
target_link_libraries(test_icp ra)
target_link_libraries(test_ndt ra)
target_link_libraries(run_imu_integration ra)

