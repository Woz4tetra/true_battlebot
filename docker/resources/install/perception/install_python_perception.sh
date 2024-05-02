#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo chown -R 1000:1000 ${HOME}/.local

sudo -H python -m pip install --no-cache-dir --ignore-installed -r ${BASE_DIR}/perception-requirements.txt
sudo -H python -m pip install --no-cache-dir 'git+https://github.com/facebookresearch/detectron2.git'

python -c "import detectron2; print(detectron2.__version__)"
echo "Installed python for perception"
