cmake_minimum_required(VERSION 3.10)
project(dog_tool)

set(CMAKE_CXX_STANDARD 14)

# 包含头文件目录
include_directories(include)

# 查找线程库 (pthread)
find_package(Threads REQUIRED)

# 生成可执行文件
add_executable(dog_tool
    src/main.cpp
    src/UdpClient.cpp
    src/McuUpdater.cpp
    src/JointUpdater.cpp
    src/JointControl.cpp
    src/BoardBasicTester.cpp
    src/ws2812_control.cpp
    src/device_creator.cpp
    src/ProdTestServer.cpp
    src/TestLogger.cpp
    src/TestConfig.cpp
)

# 链接线程库
target_link_libraries(dog_tool PRIVATE Threads::Threads)

