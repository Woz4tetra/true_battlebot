#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

source /opt/${ORGANIZATION}/venv/bin/activate

LLVM_VERSION=20
sudo apt-get update
sudo apt-get install -y llvm-${LLVM_VERSION}*
sudo ln -s /usr/lib/llvm-${LLVM_VERSION}/bin/llvm-config /usr/bin

python -m pip install --no-cache-dir -r ${BASE_DIR}/requirements.txt

echo "Installed python dependencies"
