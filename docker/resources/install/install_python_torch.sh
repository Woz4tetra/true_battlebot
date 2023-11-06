#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo chown -R 1000:1000 ${HOME}/.local

sudo -H python -m pip install -r ${BASE_DIR}/torch-requirements.txt

sudo -H python -m pip uninstall -y kiwisolver
sudo -H python -m pip install kiwisolver

echo "Installed python torch"
