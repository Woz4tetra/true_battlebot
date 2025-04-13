#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

LLVM_VERSION=12
sudo apt-get update
sudo apt-get install -y llvm-${LLVM_VERSION}*
sudo ln -s /usr/lib/llvm-${LLVM_VERSION}/bin/llvm-config /usr/bin

sudo python -m pip install --no-cache-dir --upgrade pip==24.2 setuptools==75.2.0
sudo python -m pip install --no-cache-dir -r ${BASE_DIR}/requirements.txt

echo "Installed python dependencies"
