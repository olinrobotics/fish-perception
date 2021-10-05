# Flashing and Setting Up NVIDIA Jetson

## Flashing and Wifi
To start with setup, reference [Nvidia's guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#intro) to setting up a Jetson Nano Dev Kit. This is easiest to do with a USB keyboard and mouse, but in theory it works with a headless connection over serial. 

Once you have flashed it, turned it on, and gone through the bootloader settings, you *may* notice that your USB wifi module doesn't work. That would probably be because of Nvidia's built-in driver for wifi not having compatibility with your module. When I set this up, I was using a rtl8723bu USB wifi module and had to install an open source wifi driver on the Jetson. The files you need to replicate this are in this folder (with some minor adjustments from the [Github project I used](https://github.com/lwfinger/rtl8723bu/tree/master)). I also have included the .deb file to download DKMS which is needed to install this driver. The .deb file is for Ubuntu 18.04 which was the version of Ubuntu that the Jetson's latest image was using at the time I installed this. If this changes, find the correct version of DKMS that aligns with the version of Ubuntu for the Jetson [here](https://launchpad.net/ubuntu/+source/dkms).

If your wifi isn't working follow these steps:
1. Move/copy the 'rtl8723bu' directory and the .deb (or newer version that you have downloaded) onto the Jetson's MicroUSB (or plug in an additional flash drive with this content to move it onto the Jetson). They can go anywhere, I just placed them in `home/USERNAME/Downloads`
2. On the Jetson, navigate to the location of the .deb file and install it (`sudo dpkg -i NAME_OF_DEB.deb`)
3. Navigate into the 'rtl8723bu' directory and run the `install.sh` script (`sudo ./install.sh`)
4. Edit `/etc/modprobe.d/blacklist.conf` and  append `blacklist rtl8xxxu`
5. reboot and you should be able to connect to wifi!

## Install Tools, Packages, and Libraries
Before installing anything else, you should update your Jetson and upgrade any packages. You can do this by running `sudo apt update` and `sudo apt upgrade -y`. It also doesn't hurt to run a `sudo apt autoremove` to clean up whatever you don't need.

You can then clone this repo onto your Jetson and run the `./install_packages.sh` script as root to install some packages that will be needed to use GPIO, pip, etc.

To Be Continued...

## ROS
Follow the ROS Melodic (optimal for Ubuntu 18.04) [installation guide](http://wiki.ros.org/melodic/Installation/Ubuntu) and install the ROS-base version (no GUI). Don't worry, you can still visualize and utilize ROS GUI tools on your own laptop while they run on the Jetson.

A general note about ROS: It is difficult to test nodes/scripts in your own environment and ensure they will work the same on the Jetson. The Jetson *in this case* has the bare-bones of ROS to avoid installing packages that are not used. However, many people have installed other packages locally that don't exist on the Jetson and will cause problems.

*TBD: Potentially create a development container for a common environment*

## Nice to Have
For nicer terminal searches, you can add these lines to your `~/.inputrc` file for good backwards scrolling and removing case sensitivity, then restart your terminal.
```
"\e[A": history-search-backward
set completion-ignore-case on
```

