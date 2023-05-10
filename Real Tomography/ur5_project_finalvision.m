clear
clc
%% main
ur5 = ur5_interface();
% gdesired1 = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1];
% gdesired0 = [0 -1 0 0.8; -1 0 0 0.4; 0 0 -1 0.22; 0 0 0 1];
% start_location = gdesired1;
% target_location = gdesired0;
home = [0.43 -1.82 0.5 -1.57 0 0]';
% home = [-1.57 -1.57 0 -1.57 0 0]';
a  = input("Please move to the start position! Press enter");
start_location = ur5.get_current_joints();
qdesired1 = start_location;
start_location = ur5FwdKin(start_location);
gdesired1 = start_location;
b  = input("Please move to the target position! Press enter");
target_location = ur5.get_current_joints();
qdesired3 = target_location;
target_location = ur5FwdKin(target_location);
gdesired3 = target_location;
methods = input("Please input your method(1 for IK, 2 for BR and 3 for TJ) and then press enter:");
if methods == 1
    z_offset = 0.3;
    %parameter define 
    g_offset = [ 1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
    start_location = start_location*g_offset;
    target_location = target_location*g_offset;
    start_location_above = start_location*g_offset;
    start_location_above(3,4) = start_location_above(3,4) + z_offset;
    target_location_above = target_location *g_offset;
    target_location_above(3,4) = target_location_above(3,4) + z_offset;
    %step 0: move to the home position 
    ur5.move_joints(home,10)
    pause(10);
    disp("move to home")
    %step 1: move to the position above the start location
    [theta1] = ur5InvKin(start_location_above);
    current = ur5.get_current_joints();
    theta1 = getbest(theta1,current);
    ur5.move_joints(theta1,10)
    pause(10)
    disp("move to above")
    %step 2:move to the start location
    [theta2] = ur5InvKin(start_location);
    current = ur5.get_current_joints();
    theta2 = getbest(theta2,current);
    ur5.move_joints(theta2,10)
    pause(10)
    disp("move to start location")
    %step 3:move to the position above the start location
    [theta3] = ur5InvKin(start_location_above);
    current = ur5.get_current_joints();
    theta3 = getbest(theta3,current);
    ur5.move_joints(theta3,10)
    pause(10)
    disp("move to above")
    %step 4: move to the position above the target location
    [theta4] = ur5InvKin(target_location_above);
    current = ur5.get_current_joints();
    theta4 = getbest(theta4,current);
    ur5.move_joints(theta4,10)
    pause(10)    
    disp("move to above target location")
    %step 5: move to the target location
    [theta5] = ur5InvKin(target_location);
    current = ur5.get_current_joints();
    theta5 = getbest(theta5,current);
    ur5.move_joints(theta5,10)
    pause(10)
    disp("move to tagret location")
    %step 6: move to the position above the target location
    [theta6] = ur5InvKin(target_location_above);
    current = ur5.get_current_joints();
    theta6 = getbest(theta6,current);
    ur5.move_joints(theta6,10)
    pause(10)
    disp("move to above target location")
    %step 7 move to home
    ur5.move_joints(home,10)
    pause(10);
    disp("move to home")

elseif methods == 2
    K = 0.3;
    % move to start position(z=0.3)
    % qdesired1 = [-1.1458;-1.5629;1.779;-1.7893;-1.5687;-2.7143];
    % gdesired1 = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1];
    ur5.move_joints(home,10)
    pause(10);
    disp("Home position")
    qmedium1 =(qdesired1+home)/2;
    ur5.move_joints(qmedium1,10);
    pause(11)
%     qmedium2 = (qdesired1+home)*2/3;
%     ur5.move_joints(qmedium2,5);
%     pause(10)
    gdesired0 = gdesired1 + [0 0 0 0;0 0 0 0; 0 0 0 0.3;0 0 0 0];
    BR(gdesired0, K, ur5);
    disp("Arrived at the start position above")
    % down to start position
    BR(gdesired1, K, ur5);
    disp("Arrived at the start position")
    % up to start position
    BR(gdesired0, K, ur5);
    disp("Arrived at the start position above")
    % move to target position(z=0.3)
    % qdesired3 = [1.9868;-1.5693;1.7799;-1.7822;-1.5723;0.4240];
    % gdesired3 = [0 -1 0 -0.3; -1 0 0 0.39; 0 0 -1 0.22; 0 0 0 1];
    gdesired2 = gdesired3 + [0 0 0 0;0 0 0 0; 0 0 0 0.3;0 0 0 0];
    BR(gdesired2, K, ur5);
    disp("Arrived at the target position above")
    % down to target position
    BR(gdesired3, K, ur5);
    disp("Arrived at the target positionn")
    % up to target position
    BR(gdesired2, K, ur5);
    disp("Arrived at the target position above")
    ur5.move_joints(home,10)
    pause(10);
    disp("End")
elseif methods == 3
    K = 0.3;
    % move to start position(z=0.3)
    % qdesired1 = [-1.1458;-1.5629;1.779;-1.7893;-1.5687;-2.7143];
    % gdesired1 = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1];
    ur5.move_joints(home,10)
    pause(10);
    disp("Home position")
    qmedium1 =(qdesired1+home)/2;
    ur5.move_joints(qmedium1,10);
    pause(11)
%     qmedium2 = (qdesired1+home)*2/3;
%     ur5.move_joints(qmedium2,5);
%     pause(10)
    gdesired0 = gdesired1 + [0 0 0 0;0 0 0 0; 0 0 0 0.3;0 0 0 0];
    TJ2(gdesired0, K, ur5);
    disp("Arrived at the start position above")
    % down to start position
    TJ(gdesired1, K, ur5);
    disp("Arrived at the start position")
    % up to start position
    TJ(gdesired0, K, ur5);
    disp("Arrived at the start position above")
    % move to target position(z=0.3)
    %     gdesired3 = [0 -1 0 -0.3; -1 0 0 0.39; 0 0 -1 0.22; 0 0 0 1];
    %     qdesired3 = [1.9868;-1.5693;1.7799;-1.7822;-1.5723;0.4240];
    qas = ur5.get_current_joints();
    qm = (qas+qdesired3)/2;
    gm = ur5FwdKin(qm);
    TJ(gm, K, ur5);
    disp("Arrived at the medium point between the target position and the start positoin above")
    gdesired2 = gdesired3 + [0 0 0 0;0 0 0 0; 0 0 0 0.3;0 0 0 0];
    TJ(gdesired2, K, ur5);
    disp("Arrived at the target position above")
    % down to target position
    TJ(gdesired3, K, ur5);
    disp("Arrived at the target position")
    % up to target position
    TJ(gdesired2, K, ur5);
    disp("Arrived at the target position above")
    ur5.move_joints(home,10)
    pause(10);
    disp("End")
end

%% IK-based
function q = getbest(theta,current)
%function to find the best solution from the theta got from inverse
%kinematics
%output: thete 6*1 matrix
%input : thetalist 6*8 matrix represents all possibe solution
%input : current 6*1 matrix to thet current position
max_distance = 9999;
slist = [];
for i = 1:8
    if zcheck(theta(:,i)) ~= -1
        slist =[slist theta(:,i)];
        distance=sum(abs(theta(:,i)-current));
        if max_distance > distance
            max_distance = distance;
            q = theta(:,1);
        end
    else
        fprintf("fail the safety check")
    end
end
end

%safety check if this is in the shoulder link is above the table
function theta = zcheck(theta)
if (theta(2)>= -2.97 | theta(2) <=-0.17)
    theta = 0;
else
    theta = -1;
end
end

%% BR-based
function finalerr = BR(gdesired, K, ur5)
fprintf("callBR: ")

qk = ur5.get_current_joints();
% read the initial angles of UR5
Tstep = 2;
iterations = Tstep * 1000;
R_gdesired = gdesired(1:3,1:3);
P_gdesired = gdesired(1:3,4);
gdesired_inv = [transpose(R_gdesired) -transpose(R_gdesired)*P_gdesired; 0 0 0 1];
% gdesired is a constant
for k = 0: Tstep: iterations

    gst = ur5FwdKin(qk);
    g_error = gdesired_inv * gst;
    a = getXi(g_error);
    body_Jacobian = ur5BodyJacobian(qk);
    
    % find the vk and wk (theta_error and meter_error) in each step
    R_error = g_error(1:3,1:3);
    P_error = g_error(1:3,4);
    trace_R_error = R_error(1,1) + R_error(2,2) + R_error(3,3);
    theta_error = acos((trace_R_error - 1)/2);
    meter_error = sqrt(P_error(1)^2 + P_error(2)^2 + P_error(3)^2);

    if det(body_Jacobian) == 0
        finalerr = -1;
        return
        % check for singularities in each step, if failure, return -1
    else
        if theta_error <= deg2rad(5) && meter_error <= 0.03
            Tstep_update = 5;
            qk = qk - K * Tstep_update * inv(body_Jacobian) * a;
            ur5.move_joints(qk,5);
            pause(5)
        else
            qk = qk - K * Tstep * inv(body_Jacobian) * a;
            ur5.move_joints(qk,5);
            pause(5)
        end       
    end

    
    % define the threshold be 15 degree and 5cm
    % if less than the threshold, return final positional error in cm
    % if not, failure and return -1
    
    % check error
    if theta_error <= deg2rad(3) && meter_error <= 0.01
        finalerr = meter_error * 100;
        R_real = gst(1:3,1:3);
        Trace_R_error = (R_real - R_gdesired)*transpose(R_real - R_gdesired);
        Trace_R_error_so3 = sqrt(Trace_R_error(1,1) + Trace_R_error(2,2) + Trace_R_error(3,3));
        return
    end
    % safety test
    qc = ur5.get_current_joints();
    gc = ur5FwdKin(qc);
    if gc(3,4) <= 0.001
        error("Hit the table! Please set the position again and run this file again!")
    end
    %
end
finalerr = -1;

end
%% TJ-based
function finalerr = TJ(gdesired, K, ur5)
fprintf("callTJ: ")
qk = ur5.get_current_joints();
% read the initial angles of UR5
Tstep = 1;
iterations = Tstep * 100;
R_gdesired = gdesired(1:3,1:3);
P_gdesired = gdesired(1:3,4);
gdesired_inv = [transpose(R_gdesired) -transpose(R_gdesired)*P_gdesired; 0 0 0 1];
% gdesired is a constant
for k = 0: Tstep: iterations

    gst = ur5FwdKin(qk);
    g_error = gdesired_inv * gst;
    a = getXi(g_error);

    body_Jacobian = ur5BodyJacobian(qk);
    
    % find the vk and wk (theta_error and meter_error) in each step
    R_error = g_error(1:3,1:3);
    P_error = g_error(1:3,4);
    trace_R_error = R_error(1,1) + R_error(2,2) + R_error(3,3);
    theta_error = acos((trace_R_error - 1)/2);
    meter_error = sqrt(P_error(1)^2 + P_error(2)^2 + P_error(3)^2);

    if det(body_Jacobian) == 0
        finalerr = -1;
        return
        % check for singularities in each step, if failure, return -1
    else
        qk = qk - K * Tstep * transpose(body_Jacobian) * a;
        ur5.move_joints(qk,2-k*0.03);
        pause(2-k*0.03)
    end       
    
    
    % define the threshold be 15 degree and 5cm
    % if less than the threshold, return final positional error in cm
    % if not, failure and return -1

    if theta_error <= deg2rad(3) && meter_error <= 0.01
        finalerr = meter_error * 100;
        R_real = gst(1:3,1:3);
        Trace_R_error = (R_real - R_gdesired)*transpose(R_real - R_gdesired);
        Trace_R_error_so3 = sqrt(Trace_R_error(1,1) + Trace_R_error(2,2) + Trace_R_error(3,3));
        return
    end
    % safety test
    qc = ur5.get_current_joints();
    gc = ur5FwdKin(qc);
    if gc(3,4) <= 0.001
        error("Hit the table! Please set the position again and run this file again!")
    end
    %
end
finalerr = -1;
end

function finalerr = TJ2(gdesired, K, ur5)
fprintf("callTJ: ")
qk = ur5.get_current_joints();
% read the initial angles of UR5
Tstep = 1;
iterations = Tstep * 100;
R_gdesired = gdesired(1:3,1:3);
P_gdesired = gdesired(1:3,4);
gdesired_inv = [transpose(R_gdesired) -transpose(R_gdesired)*P_gdesired; 0 0 0 1];
% gdesired is a constant
for k = 0: Tstep: iterations

    gst = ur5FwdKin(qk);
    g_error = gdesired_inv * gst;
    a = getXi(g_error);

    body_Jacobian = ur5BodyJacobian(qk);
    
    % find the vk and wk (theta_error and meter_error) in each step
    R_error = g_error(1:3,1:3);
    P_error = g_error(1:3,4);
    trace_R_error = R_error(1,1) + R_error(2,2) + R_error(3,3);
    theta_error = acos((trace_R_error - 1)/2);
    meter_error = sqrt(P_error(1)^2 + P_error(2)^2 + P_error(3)^2);

    if det(body_Jacobian) == 0
        finalerr = -1;
        return
        % check for singularities in each step, if failure, return -1
    else
        qk = qk - K * Tstep * transpose(body_Jacobian) * a;
        ur5.move_joints(qk,2-k*0.009);
        pause(2-k*0.009)
    end       
    
    
    % define the threshold be 15 degree and 5cm
    % if less than the threshold, return final positional error in cm
    % if not, failure and return -1

    if theta_error <= deg2rad(3) && meter_error <= 0.01
        finalerr = meter_error * 100;
        R_real = gst(1:3,1:3);
        Trace_R_error = (R_real - R_gdesired)*transpose(R_real - R_gdesired);
        Trace_R_error_so3 = sqrt(Trace_R_error(1,1) + Trace_R_error(2,2) + Trace_R_error(3,3));
        return
    end
    % safety test
    qc = ur5.get_current_joints();
    gc = ur5FwdKin(qc);
    if gc(3,4) <= 0.05
        error("Hit the table! Please set the position again and run this file again!")
    end
    %
end
finalerr = -1;
end
