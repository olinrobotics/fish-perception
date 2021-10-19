#!/bin/bash
source ~/.bashrc
apt install python-pip -y
apt install curl -y
apt install wireshark -y
# install ros packages
apt install ros-melodic-cv-bridge
apt install ros-melodic-sensor-msgs

# gpio interface with python
pip3 install --upgrade pip
pip3 install Jetson.GPIO -y
groupadd -f -r gpio
usermod -a -G gpio $USER