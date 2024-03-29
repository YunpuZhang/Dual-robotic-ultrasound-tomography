%% Run First
% KDC101disconnect(KDC101);

clc;
clear all;
close all;

% Init the KDC101 Object
KDC101_init
% Init ROS core
rosshutdown;
% rosinit;
rosinit("10.203.70.157") 
        % for this laptop with hopkins, 10.203.215.162
        % for Bailan 10.203.169.211
        % for Bailan with roscore: "10.203.70.157"
pause(2);

TRUSsub = rossubscriber("/TRUS", "DataFormat", "struct");
pause(1);

KDC101.MoveTo(95, 15000);

%% Try
KDC101.MoveTo(95, 15000);
%% Wait for the other computer to connect to the ROS master, then RUN this Section

for i = -5:0.4:5
    angle = receive(TRUSsub, 15).Data;
    disp(angle);
    KDC101.MoveTo(angle, 2000);
end
% First arg in degree, second arg in ms

%% Finally
rosshutdown;
KDC101disconnect(KDC101);