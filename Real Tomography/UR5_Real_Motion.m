%% Init Core and Publisher
clc;
clear;
close all;

rosshutdown % Shut down any existing roscore

% Connect to the roscore from the launch file, the ip address would be
% different on each computer
rosinit("10.203.70.157");
pause(2);

% Create a publisher to tell the other computer how much the TRUS probe
% needs to rotates
TRUSpub = rospublisher("/TRUS", "std_msgs/Float64", "DataFormat", "struct");
% Create a subscriber to receive the readings from a force sensor
force_sub = rossubscriber("/robotiq_ft_wrench","DataFormat", "struct"); 
pause(1);

% Create a msg for the publisher
TRUSmsg = rosmessage(TRUSpub);


%% Getting to the Big Process
% (Run this after the first section on two computers)

% Define Variables
base_ref = "base_link";
T_step = 7;
T_step_draw = 0.5;
k = 0.001;
desired_force = -0;
force_z_offset = 3.6;

% The two below are for plot
mea_force = zeros(100, 1);
time = zeros(100, 1);

% XA is the transformation from "ee_link" to the abdominal probe
XA = [-0.0441 0.9571 -0.2865 0.2529066; 0.7949 0.2073 0.5703 -0.001771; 0.6051 -0.2026 -0.77 0.0076861; 0 0 0 1];

% Create the ur5 object
ur5 = ur5_interface();
pause(1);

% Create some frames in Rviz for visualization
TRUS_Frame = tf_frame(base_ref, "TRUS", eye(4));
pause(0.7);
Goal_Frame = tf_frame(base_ref, "Goal", eye(4));
pause(0.7);
Probe_Frame = tf_frame("ee_link", "Probe", XA);
pause(0.7);


% find P_I for angle from -5 to 5 (angle between abdominal probe and horizontal plane)
count = 0;
max_range = 95;
dtheta = 0.1;
t = 0;
for theta = -5:dtheta:5
    % Calculate and send what the TRUS probe should do
    TRUSmsg.Data = max_range - count*dtheta;
    count = count + 1;
    send(TRUSpub, TRUSmsg);
    % Record force for a plot
    wrench_z = receive(force_sub, 15).Wrench.Force.Z - force_z_offset;
    mea_force(count) = wrench_z;
    time(count) = t;
    t = t + 0.5;
    
    % A backup transformation
%     B_new = [0.6954   -0.5673    0.4412   -0.1315;
%                 0.2359   -0.3996   -0.8859    0.1851;
%                 0.6787    0.7201   -0.1441    0.7857;
%                      0         0         0    1.0000];

    % B_new is between the TRUS probe and "base_link" when the TRUS probe is
    % at 90 degrees
    B_new = [0.6917   -0.5521    0.4656   -0.1360;
                0.2734   -0.3965   -0.8765    0.1783; 
                0.6684    0.7335   -0.1233    0.7820;
                     0         0         0    1.0000];
    % Update the B_new with each the motor rotation
    X_trus = B_new * [ROTX((theta)*pi/180),[0 0 0]'; 0 0 0 1];
    % Get the transformation between "ee_link" and "base_link"
    X = ur5.get_current_transformation(base_ref, "ee_link");
    % Get the transformation between abdominal probe and "base_link"
    X = X*XA;
    % Current Joints and Jacobian (not used)
    q = ur5.get_current_joints();
    Jb = ur5BodyJacobian(q);
    
    % Find the desired position and orientation
    P_I = findPItest(X_trus,X);
    direction_zyx = rotm2eul(X_trus(1:3, 1:3));
    direction_rpy = [direction_zyx(3); direction_zyx(2); direction_zyx(1)];
    P_I6 = [P_I; direction_rpy];
    
    curr_dir_zyx = rotm2eul(X(1:3, 1:3));
    curr_dir_rpy = [curr_dir_zyx(3); curr_dir_zyx(2); curr_dir_zyx(1)];
    curr_pos = X(1:3, 4);
    
    initial = [curr_pos; curr_dir_rpy];
   
    
    % Form a transformation from P_I
    R = X_trus(1:3, 1:3);
    p = P_I(1:3);
    
    Goal = [        R,  p;
            zeros(1,3), 1];
        
    % Move frames for visualization
%     TRUS_Frame.move_frame(base_ref, X_trus);
%     pause(0.7);
%     Goal_Frame.move_frame(base_ref, Goal);
%     pause(0.7);
    
    % Force converted from probe to base_link, as well as with a factor k
    eelink2tool0 = quat2rotm([0.500, -0.500, 0.500, -0.500]);
    
    f = [0, wrench_z - desired_force, 0, 0, 0, 0]';
    f_p = (X_trus)*[f(1:3);0];
    f(1:3) = f_p(1:3);
    f = k*f;
    % Get needed movements from virtual fixture
    Jdq = VF_checkpoint(initial, P_I6 , f);
    % Convert it to transformation in the "base_link" frame
    Result = [ROTZ(initial(6) - Jdq(6))*ROTY(initial(5) - Jdq(5))*ROTX(initial(4) - Jdq(4)), initial(1:3) - Jdq(1:3, 1);
                                                                                 zeros(1,3),                          1];
 
    Result = Result * inv(XA) * [eelink2tool0 [0 0 0]'; 0 0 0 1];
    % Use inverse kinematics to get the joint angles and find closest one
    qtest = ur5InvKin(Result);
    q_test = getbest(qtest, q);
    % Move the robot, then next iteration
    ur5.move_joints(q_test, T_step);
    pause(T_step);
    T_step = T_step_draw;    
   
end