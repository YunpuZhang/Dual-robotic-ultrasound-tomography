% clear, clc, close all;
% rosshutdown
% setenv('ROS_MASTER_URI','http://10.0.0.101:11311')
% setenv('ROS_IP','10.0.0.103')
% setenv('ROS_HOSTNAME','10.0.0.103')
% rosinit
%clear; rosshutdown; rosinit; node_matlab = ros.Node('/node_matlab', 'http://10.0.0.101:11311'); pub_angle = ros.Publisher(node_matlab, '/theta', 'std_msgs/Float64','DataFormat','struct', 'IsLatching', false);
% node_matlab = ros.Node('/node_matlab', 'http://10.0.0.101:11311');
% pub_angle = ros.Publisher(node_matlab, '/theta', 'std_msgs/Float64','DataFormat','struct', 'IsLatching', false);
% msg_rorate_angle = rosmessage(pub_angle);
% sub_angle = ros.Subscriber(node_matlab,'/real_angle','std_msgs/Float64','DataFormat','struct');

%% online search

left = -35;
right = 35;
num_acquisition = 0;

acquisited_angles = [];

algo = 't'; % gs
ratio = (sqrt(5) - 1) / 2;

it_num = 1;
output_list = [];

while (abs(left - right) > 0.1)
    fprintf('iteration %d \n', it_num);
    if strcmp(algo, 't')
        midl = left + roundn((right - left) / 3, -4);
        midr = right - roundn((right - left) / 3, -4);
    else
        midl = roundn(ratio * left + (1 - ratio) * right, -4);
        midr = roundn((1 - ratio) * left + ratio * right, -4);
    end
    
    fprintf('mid points %d, %d \n', midl, midr);
    % read data from DAQ
    % mid left point
    fprintf('press any key to rotate to angle: %d \n', midl);
    acquisited_angles = [acquisited_angles, midl];
    msg_rorate_angle.data = midl;
    send(pub_angle, msg_rorate_angle);
    pause(1);
    fprintf('Please download data using GUI! \n');
    pause;
    folder_num = string(midl);
    if strcmp(algo, 't')
        folder_name = 'D:\real_online_phantom\9_t\degree' + folder_num;
    else
        folder_name = 'D:\real_online_phantom\9_gs\degree' + folder_num;
    end
    mkdir(folder_name);
    copyfile('G:\JHU\EN. 601.656 Computer Integrated Surgery 2\PA_buf2000', folder_name);
    [~, ~, intensityl] = calculate_PA_coordinates(node_matlab);
    % mid right point
    fprintf('press any key to rotate to angle: %d \n', midr);
    acquisited_angles = [acquisited_angles, midr];
    msg_rorate_angle.data = midr;
    send(pub_angle, msg_rorate_angle);
    pause(1);
    fprintf('Please download data using GUI! \n');
    pause;
    folder_num = string(midr);
    if strcmp(algo, 't')
        folder_name = 'D:\real_online_phantom\9_t\degree' + folder_num;
    else
        folder_name = 'D:\real_online_phantom\9_gs\degree' + folder_num;
    end
    mkdir(folder_name);
    copyfile('G:\JHU\EN. 601.656 Computer Integrated Surgery 2\PA_buf2000', folder_name);
    [~, ~, intensityr] = calculate_PA_coordinates(node_matlab);
    num_acquisition = num_acquisition + 2;
    
    % mid_indicator = 0 -- update mid left point, mid_indicator = 1 --
    % update mid right point
    if intensityr <= intensityl 
        right = midr;
    else 
        left = midl;
    end
    
    fprintf('next iteration interval [%d, %d] \n', left, right);
    output = (left + right) / 2;
    output_list = [output_list, output];
    fprintf('output: %d \n', output);
    it_num = it_num +  1;
end

fprintf('# of acquisition: %d', num_acquisition);
fprintf('# of angle: %d', size(acquisited_angles, 2));

figure();
plot(1 : num_acquisition, acquisited_angles);
if strcmp(algo, 't')
    save acquisited_angles_9_t.txt acquisited_angles -ascii;
    save output_9_t.txt output_list -ascii;
    acquisited_angles_t = acquisited_angles;
else
    save acquisited_angles_9_gs.txt acquisited_angles -ascii;
    save output_9_gs.txt output_list -ascii;
    acquisited_angles_t_gs = acquisited_angles;
end
% 
% pos_calculated_idx = left - 1;
% pos_calculated = pos_calculated_idx;
% 
% fprintf('# of acquisition: %d, calculated angle: %d \n', num_acquisition, pos_calculated);
