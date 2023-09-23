#!/bin/bash

set -e

cd /tmp
sudo apt-get update
sudo apt-get install -y build-essential libtool autoconf zip unzip wget zstd
sudo rm -rf /var/lib/apt/lists/*
download cmake.sh https://cmake.org/files/v3.23/cmake-3.23.1-linux-aarch64.sh
sudo mkdir -p /opt/cmake
sudo chown -R 1000:1000 /opt/cmake/
sh cmake.sh --prefix=/opt/cmake --exclude-subdir --skip-license
sudo mv /usr/bin/cmake /usr/bin/cmake-old
sudo ln -s /opt/cmake/bin/cmake /usr/bin/cmake

echo "CMake upgrade script complete"

