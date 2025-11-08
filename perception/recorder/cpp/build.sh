#!/bin/bash

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${SCRIPT_DIR}"

# Set build and install directories
BUILD_ROOT_DIR="/opt/${ORGANIZATION}/build"
BUILD_DIR="${BUILD_ROOT_DIR}/recorder_cpp_build"
INSTALL_DIR="${BUILD_ROOT_DIR}/install"

echo "Building C++ recorder module..."
echo "Source directory: ${SOURCE_DIR}"
echo "Build directory: ${BUILD_DIR}"
echo "Install directory: ${INSTALL_DIR}"

# Create build and install directories
mkdir -p "${BUILD_DIR}"
mkdir -p "${INSTALL_DIR}"

# Change to build directory
cd "${BUILD_DIR}"

# Configure with CMake
echo "Configuring with CMake..."
cmake "${SOURCE_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DORGANIZATION="${ORGANIZATION}"

# Build the project
echo "Building project..."
make -j$(nproc)

# Install the project
echo "Installing project..."
make install

# Create a symlink for easier importing
echo "Creating symlink for Python import..."
cd "${INSTALL_DIR}"
rm -f recorder_cpp.so || true
ln -sf recorder_cpp.cpython-*.so recorder_cpp.so
echo "Created symlink: recorder_cpp.so -> $(ls recorder_cpp.cpython-*.so)"
