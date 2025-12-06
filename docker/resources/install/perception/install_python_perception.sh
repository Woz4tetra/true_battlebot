#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo chown -R 1000:1000 ${HOME}/.local

source /opt/${ORGANIZATION}/venv/bin/activate

python -m pip cache purge
python -m pip install --no-cache-dir --ignore-installed -r ${BASE_DIR}/perception-requirements.txt

echo "Installed python for perception"
