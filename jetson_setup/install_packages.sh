#!/bin/bash
source ~/.bashrc
apt install python-pip -y
apt install curl -y
apt install wireshark -y

# install ros packages
apt install ros-melodic-cv-bridge
apt install ros-melodic-sensor-msgs

# create link for opencv
ln -s /usr/include/opencv4/opencv2/ /usr/include/opencv4

# gpio interface with python
pip install --upgrade pip
pip install Jetson.GPIO -y
groupadd -f -r gpio
usermod -a -G gpio $USER