#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

if ! /usr/bin/python3 --version | grep -q "${PYTHON_INSTALL_VERSION}"; then
    sudo add-apt-repository -y ppa:deadsnakes/ppa
    sudo apt-get install -y python${PYTHON_INSTALL_VERSION} python${PYTHON_INSTALL_VERSION}-dev python${PYTHON_INSTALL_VERSION}-venv
fi

sudo rm /usr/bin/python || true
sudo rm /usr/bin/python3 || true
sudo ln -s /usr/bin/python${PYTHON_INSTALL_VERSION} /usr/bin/python
sudo ln -s /usr/bin/python${PYTHON_INSTALL_VERSION} /usr/bin/python3

sudo mkdir -p /usr/lib/python${PYTHON_INSTALL_VERSION}/site-packages
sudo chown -R 1000:1000 /usr/lib/python${PYTHON_INSTALL_VERSION}/site-packages

sudo apt-get update
sudo apt-get install -y llvm-15*
sudo ln -s /usr/lib/llvm-10/bin/llvm-config /usr/bin

sudo python -m pip install --no-cache-dir --upgrade pip setuptools
sudo python -m pip install --no-cache-dir -r ${BASE_DIR}/requirements.txt

echo "Installed python dependencies"
