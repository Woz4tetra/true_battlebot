#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo python -m pip uninstall -y bson
sudo python -m pip install --ignore-installed --no-cache-dir -r ${BASE_DIR}/perception-extra-requirements.txt
sudo python -m pip uninstall -y numpy
sudo python -m pip install numpy==1.26.4

echo "Installed python extras"
