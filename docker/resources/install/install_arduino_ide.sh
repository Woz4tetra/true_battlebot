#!/bin/bash

set -e

cd /tmp
download arduino.zip https://downloads.arduino.cc/arduino-ide/arduino-ide_2.2.1_Linux_64bit.zip

unzip arduino.zip
sudo mkdir -p /opt/arduino
sudo mv arduino-ide_2.2.1_Linux_64bit /opt/arduino/ide
sudo ln -s /opt/arduino/ide/arduino-ide /usr/local/bin/arduino-ide
chown 1000:1000 /opt/arduino/ide/arduino-ide

# clean up
rm -r /tmp/* || true
