#!/bin/bash

set -e

rm -r ${HOME}/${PROJECT_NAME} || true
ln -s /opt/${ORGANIZATION}/${PROJECT_NAME} ${HOME}/${PROJECT_NAME}
