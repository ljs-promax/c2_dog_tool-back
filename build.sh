#!/bin/bash

# 获取脚本所在目录，确保路径正确
SRC_DIR=$(cd "$(dirname "$0")"; pwd)

# 检查是否存在 build 目录
if [ ! -d "$SRC_DIR/build" ]; then
    mkdir "$SRC_DIR/build"
fi

cd "$SRC_DIR/build"

# 执行编译配置
echo "Configuring with CMake..."
cmake .. -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake

# 编译
echo "Building dog_tool..."
make -j$(nproc)

# 检查编译结果
if [ $? -eq 0 ]; then
    # 注意：这里的名称应与 CMakeLists.txt 中的项目名称对齐
    BIN_NAME="dog_tool"
    echo "---------------------------------------"
    echo "Compile Success: ./build/$BIN_NAME"
    echo "---------------------------------------"
else
    echo "Compile Failed!"
    exit 1
fi