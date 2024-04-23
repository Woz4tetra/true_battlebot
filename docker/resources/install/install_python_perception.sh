#!/bin/bash

set -ex

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo chown -R 1000:1000 ${HOME}/.local

sudo -H python -m pip install -r ${BASE_DIR}/perception-requirements.txt

sudo -H python -m pip uninstall -y kiwisolver
sudo -H python -m pip install kiwisolver

sudo -H python -m pip install 'git+https://github.com/facebookresearch/detectron2.git'

python -c "import detectron2; print(detectron2.__version__)"
echo "Installed python for perception"
