#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")
REQUIREMENTS_FILE=${BASE_DIR}/ros-extra-requirements.txt

sudo python -m pip install --no-cache-dir -r ${REQUIREMENTS_FILE}
sudo apt-get install -y python${PYTHON_INSTALL_VERSION}-tk

echo "Installed python extras"
