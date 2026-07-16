# D:/my code/ota/ota_pro/toolchain.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 指定交叉编译器路径（基于你提供的 PATH 信息）
#set(TOOLCHAIN_PATH "/home/js/share/SOC/trunk/MainSoc/A733-back/out/toolchain/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu")

set(TOOLCHAIN_PATH "/home/lv/桌面/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu")
set(CMAKE_C_COMPILER "${TOOLCHAIN_PATH}/bin/aarch64-none-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}/bin/aarch64-none-linux-gnu-g++")
#set(CMAKE_C_COMPILER "gcc")
#set(CMAKE_CXX_COMPILER "g++")
# 设置查找模式
set(CMAKE_FIND_ROOT_PATH "${TOOLCHAIN_PATH}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
