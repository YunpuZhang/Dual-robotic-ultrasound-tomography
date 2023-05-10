%% Init Core and Publisher
clc;

rosshutdown
% rosinit('10.203.215.162');
rosinit("10.203.70.157");
pause(2);

% TRUSsub = rossubscriber("/TRUS", "DataFormat", "struct")
TRUSpub = rospublisher("/TRUS", "std_msgs/Float64", "DataFormat", "struct");
pause(1);
TRUSmsg = rosmessage(TRUSpub);

%% Publishing Data
TRUSmsg.Data = 135;
while (1)
    send(TRUSpub, TRUSmsg);
    TRUSmsg.Data = TRUSmsg.Data - 2;
    pause(4);
end


% for i=1:1:36
%     angle = receive(TRUSsub, 10).Data
% %     pause(3);
% end