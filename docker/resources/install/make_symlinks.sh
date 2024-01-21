#!/bin/bash

set -e

ln -s /opt/${ORGANIZATION}/${PROJECT_NAME} ${HOME}/${PROJECT_NAME}

sudo rm -r /usr/local/zed/settings
ln -s /opt/${ORGANIZATION}/${PROJECT_NAME}/src/bw_data/data/zed/settings /usr/local/zed/settings

sudo rm -r /usr/local/zed/resources
ln -s /opt/${ORGANIZATION}/${PROJECT_NAME}/src/bw_data/data/zed/resources /usr/local/zed/resources
