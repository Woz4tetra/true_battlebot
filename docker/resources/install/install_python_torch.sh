#!/bin/bash

set -e

sudo chown -R 1000:1000 ${HOME}/.local

python -m pip install -r ./torch-requirements.txt

echo "Installed python torch"
