%% Connect to the Raspberry Pi
% In a different script to allow only running it once
clear all;
rpi = raspi("192.168.32.116", "pi", "fish"); % You'll need to update the IP every time, because MATLAB is bad
cam = cameraboard(rpi,'Resolution','640x480');
load("camParams.mat");