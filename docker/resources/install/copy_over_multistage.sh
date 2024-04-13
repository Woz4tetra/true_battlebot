#!/bin/bash

sudo rm -r /tmp/ros_base/sys
sudo rsync -az /tmp/ros_base/* /
sudo rm -r /tmp/*
