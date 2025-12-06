#!/bin/bash

set -e

sudo apt-get update
sudo apt-get install -y \
    v4l-utils  \
    libusb-1.0-0*  \
    libturbojpeg-dev \
    libv4l-dev \
    ffmpeg \
    libbluetooth-dev \
    libportaudio2

sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install -y \
    python${PYTHON_INSTALL_VERSION} \
    python${PYTHON_INSTALL_VERSION}-dev \
    python3-pip \
    python${PYTHON_INSTALL_VERSION}-venv

sudo rm /usr/bin/python || true
sudo rm /usr/bin/python3 || true
sudo ln -s /usr/bin/python${PYTHON_INSTALL_VERSION} /usr/bin/python
sudo ln -s /usr/bin/python${PYTHON_INSTALL_VERSION} /usr/bin/python3

sudo mkdir -p /usr/lib/python${PYTHON_INSTALL_VERSION}/site-packages
sudo chown -R 1000:1000 /usr/lib/python${PYTHON_INSTALL_VERSION}/site-packages

cd /opt/${ORGANIZATION}
python -m venv venv
source venv/bin/activate

echo "Installed python"
