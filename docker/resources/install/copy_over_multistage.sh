#!/bin/bash
set -e
sudo rsync -az /tmp/multistage_copy/* /
sudo rm -r /tmp/*
