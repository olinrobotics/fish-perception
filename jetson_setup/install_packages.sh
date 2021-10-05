#!/bin/bash
source ~/.bashrc
apt install python3-pip -y
apt install curl -y
apt install wireshark -y

# gpio interface with python
pip3 install --upgrade pip
pip3 install Jetson.GPIO -y
groupadd -f -r gpio
usermod -a -G gpio $USER