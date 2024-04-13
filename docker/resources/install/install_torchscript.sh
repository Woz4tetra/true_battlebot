#!/bin/bash

set -e

sudo chown -R 1000:1000 /tmp

cd /tmp

download torchscript.zip https://download.pytorch.org/libtorch/cu${CUDA_VERSION_MAJOR}${CUDA_VERSION_MINOR}/libtorch-cxx11-abi-shared-with-deps-2.2.2%2Bcu${CUDA_VERSION_MAJOR}${CUDA_VERSION_MINOR}.zip
unzip torchscript.zip
sudo mv libtorch /usr/local/libtorch


# clean up
sudo ldconfig
rm -r /tmp/*

echo "Built and installed torchscript"
