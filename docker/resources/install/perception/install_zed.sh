#!/bin/bash

ZED_VERSION_MAJOR=5
ZED_VERSION_MINOR=1
ZED_PATCH_VERSION=1
TENSOR_RT_VERSION_MAJOR=10
TENSOR_RT_VERSION_MINOR=13
CUDA_VERSION_MAJOR=13
CUDA_VERSION_MINOR=0
UBUNTU_VERSION_MAJOR=24

set -e

sudo apt-get update || true
sudo apt-get install -y --no-install-recommends apt-utils dialog

sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime
echo $TZ | sudo tee /etc/timezone 
sudo apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libopencv-dev libpq-dev zstd usbutils

sudo mkdir -p /tmp
sudo sudo chown -R 1001:1001 /tmp

ZED_SDK_URL="https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/${ZED_VERSION_MAJOR}.${ZED_VERSION_MINOR}/ZED_SDK_Ubuntu${UBUNTU_VERSION_MAJOR}_cuda${CUDA_VERSION_MAJOR}.${CUDA_VERSION_MINOR}_tensorrt${TENSOR_RT_VERSION_MAJOR}.${TENSOR_RT_VERSION_MINOR}_v${ZED_VERSION_MAJOR}.${ZED_VERSION_MINOR}.${ZED_PATCH_VERSION}.zstd.run"
echo "Using ZED SDK URL: ${ZED_SDK_URL}"
cd /tmp
wget -q -O ZED_SDK_Linux_Ubuntu.run ${ZED_SDK_URL}
chmod +x ZED_SDK_Linux_Ubuntu.run 
sudo ./ZED_SDK_Linux_Ubuntu.run -- silent
sudo chown -R 1001:1001 /usr/local/zed/
sudo ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so 
sudo rm ZED_SDK_Linux_Ubuntu.run 

source /opt/${ORGANIZATION}/venv/bin/activate
sudo chown -R 1001:1001 /usr/local/lib/python${PYTHON_INSTALL_VERSION}/dist-packages
python -m pip install requests
cd /usr/local/zed
python get_python_api.py
