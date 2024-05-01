#!/bin/bash

set -e

rm -r ${HOME}/${PROJECT_NAME} || true
ln -s /opt/${ORGANIZATION}/${PROJECT_NAME} ${HOME}/${PROJECT_NAME}

sudo rm -r /usr/local/zed/settings || true
ln -s /opt/${ORGANIZATION}/${PROJECT_NAME}/perception/data/zed/settings /usr/local/zed/settings

sudo rm -r /usr/local/zed/resources || true
ln -s /opt/${ORGANIZATION}/${PROJECT_NAME}/perception/data/zed/resources /usr/local/zed/resources
