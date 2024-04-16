#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo python -m pip uninstall -y bson
sudo python -m pip install --no-cache-dir -r ${BASE_DIR}/perception-extra-requirements.txt

echo "Installed python extras"
