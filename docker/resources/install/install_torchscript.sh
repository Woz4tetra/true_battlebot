#!/bin/bash

set -e

sudo chown -R 1000:1000 /tmp

cd /tmp

download torchscript.zip https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip
unzip torchscript.zip
sudo mv libtorch /usr/local/libtorch


# clean up
sudo ldconfig
rm -r /tmp/*

echo "Built and installed torchscript"
