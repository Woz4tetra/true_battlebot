#!/bin/bash

set -e


sudo apt-get update
sudo apt-get install -y llvm-10*
sudo ln -s /usr/lib/llvm-10/bin/llvm-config /usr/bin

sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt-get install -y python${PYTHON_INSTALL_VERSION} python${PYTHON_INSTALL_VERSION}-dev

sudo rm /usr/bin/python || true
sudo rm /usr/bin/python3 || true
sudo ln -s /usr/bin/python${PYTHON_INSTALL_VERSION} /usr/bin/python-torch
sudo ln -s /usr/bin/python3.8 /usr/bin/python
sudo ln -s /usr/bin/python3.8 /usr/bin/python3

sudo python -m pip install --no-cache-dir --upgrade pip setuptools
sudo python -m pip install --no-cache-dir \
    scipy==1.5.4 \
    shapely==1.6.4 \
    dataclasses \
    flask==2.0.3 \
    psutil \
    tqdm \
    v4l2-fix \
    numpy==1.24.4 \
    matplotlib==3.4.3 \
    python-dateutil \
    pillow==9.1.0

sudo python -m pip install Cython llvmlite==0.39.0 numba==0.56.4 --no-cache-dir

echo "Installed python dependencies"
