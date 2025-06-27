# 设置最低的 CMake 版本要求
cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(RobotAlgorithmsTests)

# 启用测试支持
enable_testing()

find_package(GTest REQUIRED)

# 包含顶层目录的头文件
include_directories(${CMAKE_SOURCE_DIR}/include)

# 查找测试源文件
file(GLOB TEST_SOURCES *.cpp)

# 为每个测试源文件创建一个可执行文件和测试
foreach(TEST_SOURCE ${TEST_SOURCES})
    # 获取测试文件名（不带扩展名）
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    # 添加可执行文件
    add_executable(${TEST_NAME} ${TEST_SOURCE})

    # 链接必要的库（根据项目需求修改）
    target_link_libraries(${TEST_NAME} PRIVATE GTest::GTest GTest::Main ra)

    # 添加测试
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()