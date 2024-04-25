#!/bin/bash
echo "fs.inotify.max_user_watches=1000000" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
