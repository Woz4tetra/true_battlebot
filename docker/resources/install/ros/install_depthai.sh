#!/bin/bash

set -e

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp

cd /tmp
sudo apt-get update
sudo apt-get install -y cmake git libopencv-dev
git clone --recursive https://github.com/luxonis/depthai-core.git --branch main
cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build depthai-core/build --target install

# clean up
sudo ldconfig
rm -r /tmp/* || true

echo "Built and installed depthai libraries"

cd /tmp
