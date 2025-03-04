#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo chown -R 1000:1000 ${HOME}/.local

sudo -H python -m pip cache purge
sudo -H python -m pip install --no-cache-dir --ignore-installed -r ${BASE_DIR}/perception-requirements.txt

echo "Installed python for perception"
