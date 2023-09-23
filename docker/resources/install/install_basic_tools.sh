#!/bin/bash
sudo apt-get update
sudo apt-get install -y apt-utils \
    git nano tmux curl wget htop net-tools iproute2 iputils-ping \
    gdb dumb-init rsync entr build-essential libtool autoconf zip unzip zstd software-properties-common
