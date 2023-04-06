clc;
% clear all;
% close all;

%% Run First
% Init the KDC101 Object
KDC101_init
% Init ROS core
rosshutdown;
rosinit % for hopkins, 10.203.215.162

pause(2);
TRUSpub = rospublisher("/TRUS","std_msgs/Float64","DataFormat","struct");
pause(1);
TRUSmsg = rosmessage(TRUSpub);




%% Wait for the other computer to connect to the ROS master, then RUN this Section
for i = 0:10:360
    KDC101.MoveTo(i*1.0,3000);
    TRUSmsg.Data = System.Decimal.ToDouble(KDC101.Position);
    send(TRUSpub, TRUSmsg)
end
% First arg in degree, second arg in ms

%% Finally
rosshutdown;
KDC101disconnect(KDC101);