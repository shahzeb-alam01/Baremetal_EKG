#!/bin/bash
# CMake Build Script for Baremetal ECG

set -e

# Configuration
BUILD_DIR="cmake_build"
CMAKE_TOOLCHAIN="cmake/arm-toolchain.cmake"

echo "=== Building Baremetal ECG ==="

# Create build directory
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# Configure with CMake
cmake .. \
    -DCMAKE_TOOLCHAIN_FILE=$CMAKE_TOOLCHAIN \
    -DCMAKE_BUILD_TYPE=Release \
    -GNinja

# Build
cmake --build . --config Release

cd ..

echo "=== Build Complete ==="
echo "Output files:"
echo "  - ELF: $BUILD_DIR/Baremetal_ECG.elf"
echo "  - HEX: $BUILD_DIR/Baremetal_ECG.hex"
echo "  - BIN: $BUILD_DIR/Baremetal_ECG.bin"
