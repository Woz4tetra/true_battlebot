#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo python -m pip install --no-cache-dir -r ${BASE_DIR}/ros-extra-requirements.txt
sudo apt-get install -y python${PYTHON_INSTALL_VERSION}-tk

echo "Installed python extras"
