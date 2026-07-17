#!/bin/bash
set -e

# 获取脚本所在目录，确保路径正确
SRC_DIR=$(cd "$(dirname "$0")"; pwd)
BUILD_DIR="$SRC_DIR/build-aarch64"
TOOLCHAIN_FILE="$SRC_DIR/toolchain.cmake"
BIN_NAME="dog_tool"

if [ ! -f "$TOOLCHAIN_FILE" ]; then
    echo "Toolchain file not found: $TOOLCHAIN_FILE"
    exit 1
fi

# 执行编译配置
echo "Configuring with CMake..."
cmake -S "$SRC_DIR" -B "$BUILD_DIR" -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE"

# 编译
echo "Building dog_tool..."
cmake --build "$BUILD_DIR" --parallel "$(nproc)"

# 检查编译结果
echo "---------------------------------------"
echo "Compile Success: $BUILD_DIR/$BIN_NAME"
echo "---------------------------------------"
