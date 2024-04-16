#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo apt-get install -y \
    v4l-utils  \
    libusb-1.0-0*  \
    libturbojpeg-dev \
    libv4l-dev \
    ffmpeg \
    libbluetooth-dev \
    libportaudio2

if ! /usr/bin/python3 --version | grep -q "${PYTHON_INSTALL_VERSION}"; then
    sudo add-apt-repository -y ppa:deadsnakes/ppa
    sudo apt-get install -y \
    python${PYTHON_INSTALL_VERSION} \
    python${PYTHON_INSTALL_VERSION}-dev \
    python${PYTHON_INSTALL_VERSION}-venv
fi

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp
cd /tmp
download get-pip.py https://bootstrap.pypa.io/get-pip.py
sudo python${PYTHON_INSTALL_VERSION} get-pip.py
rm -r /tmp/* || true
cd -

sudo rm /usr/bin/python || true
sudo rm /usr/bin/python3 || true
sudo ln -s /usr/bin/python${PYTHON_INSTALL_VERSION} /usr/bin/python
sudo ln -s /usr/bin/python${PYTHON_INSTALL_VERSION} /usr/bin/python3

sudo mkdir -p /usr/lib/python${PYTHON_INSTALL_VERSION}/site-packages
sudo chown -R 1000:1000 /usr/lib/python${PYTHON_INSTALL_VERSION}/site-packages

LLVM_VERSION=12
sudo apt-get update
sudo apt-get install -y llvm-${LLVM_VERSION}*
sudo ln -s /usr/lib/llvm-${LLVM_VERSION}/bin/llvm-config /usr/bin

sudo python -m pip install --no-cache-dir --upgrade pip setuptools
sudo python -m pip install --no-cache-dir -r ${BASE_DIR}/requirements.txt

echo "Installed python dependencies"
