clc;
clear all;
close all;

% rosshutdown
% rosinit('10.203.6.175');
% pause(2);
% chatterpub = rospublisher("/num","std_msgs/Float64","DataFormat","struct")
% pause(1)
% chattermsg = rosmessage(chatterpub);
% chattermsg.Data = 0.1;
% send(chatterpub,chattermsg)

rosshutdown;
rosinit
pause(2);


