#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

if ! which pip-compile | grep -q "pip-compile"; then
    python3 -m pip install pip-tools
fi

cd ${BASE_DIR}/../../docker/resources/install/
pip-compile -v requirements.in --output-file requirements.txt
