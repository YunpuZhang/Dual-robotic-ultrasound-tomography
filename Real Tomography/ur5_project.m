clear
clc
%% main
ur5 = ur5_interface();
% gdesired1 = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1];
% gdesired0 = [0 -1 0 0.8; -1 0 0 0.4; 0 0 -1 0.22; 0 0 0 1];
% start_location = gdesired1;
% target_location = gdesired0;
 home = [0.43 -1.82 0.5 -1.57 0 0]';
%home = [-1.57 -1.57 0 -1.57 0 0]';
a  = input("Please move to the start position! Press enter");
start_location = ur5.get_current_joints();
start_location = ur5FwdKin(start_location);
b  = input("Please move to the target position! Press enter");
target_location = ur5.get_current_joints();
target_location = ur5FwdKin(target_location);
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
    ur5.move_joints(home,10)
    K = 10;    
    % move to start position(z=0.3)
    %     gdesired1 = ur5FwdKin(start_location);

    BR(gdesired0, K, ur5);
    % down to start position
    BR(gdesired1, K, ur5);
    % up to start position
    BR(gdesired0, K, ur5);
    % move to target position(z=0.3)
%     gdesired3 = ur5FwdKin(target_location);
    gdesired3 = [0 -1 0 -0.3; -1 0 0 0.39; 0 0 -1 0.22; 0 0 0 1];
    gdesired2 = gdesired3 + [0 0 0 0;0 0 0 0; 0 0 0 0.3;0 0 0 0];
    BR(gdesired2, K, ur5);
    % down to target position
    BR(gdesired3, K, ur5);
    % up to target position
    BR(gdesired2, K, ur5);

elseif methods == 3
    ur5.move_joints(home,10)
    K = 0.00003;
    % move to start position(z=0.3)
    BR(gdesired0, K, ur5);
    % down to start position
    BR(gdesired1, K, ur5);
    % up to start position
    BR(gdesired0, K, ur5);
    % move to target position(z=0.3)
    gdesired3 = ur5FwdKin(target_location);
    gdesired2 = gdesired3 + [eye(3),[0; 0; 0.3];0 0 0 1];
    BR(gdesired2, K, ur5);
    % down to target position
    BR(gdesired3, K, ur5);
    % up to target position
    BR(gdesired2, K, ur5);

else
    fprintf("False input! Run this file again!\n")

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

ur5 = ur5_interface();

home = [-pi/2;-pi/2;0;-pi/2;0;0];
qk = ur5.get_current_joints()-home; 

gst = ur5FwdKin(qk);
body_Jacobian = ur5BodyJacobian(qk);

R_gdesired = gdesired(1:3,1:3);
P_gdesired = gdesired(1:3,4);
gdesired_inv = [transpose(R_gdesired) -transpose(R_gdesired)*P_gdesired; 0 0 0 1];

g_error = gdesired_inv * gst;
a = getXi(g_error);

wk = norm(a(1:3));
vk = norm(a(4:6));

Tstep = 0.01;
% read the initial angles of UR5

while norm(vk) > 0.05 || norm(wk) > deg2rad(15)
    qk = qk - K * Tstep * transpose(body_Jacobian) * a;
    BJ = ur5BodyJacobian(qk);
    mu = manipulability(BJ,'sigmamin');
    
    if abs(mu) <= 0.01
        finalerr = -1;
        disp(finalerr)
        error('singularity happens!');
    end
       
     if qk(2) >= 0 && qk(2) <= -pi
        finalerr = -1;
        disp(finalerr)
        error('exceeds working range');
     end
     
     ur5.move_joints(qk+home,2);
     pause(2);
     gst_k = ur5FwdKin(qk);
     BJ = ur5BodyJacobian(qk);
       
     error_g = gdesired \ gst_k;
     xi_k = getXi(error_g);
     v_k = xi_k(1:3);
     w_k = xi_k(4:6);
end
end
%% TJ-based
function finalerr = TJ(gdesired, K, ur5)

ur5 = ur5_interface();

home = [-pi/2;-pi/2;0;-pi/2;0;0];
qk = ur5.get_current_joints()-home;
% read the initial angles of UR5
Tstep = 0.01;
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

    if det(body_Jacobian) == 0
        finalerr = -1;
        return
        % check for singularities in each step, if failure, return -1
    else
        qk = qk - K * Tstep * transpose(body_Jacobian) * a;
        ur5.move_joints(qk,3);
        pause(1)
    end

    % find the vk and wk (theta_error and meter_error) in each step
    R_error = g_error(1:3,1:3);
    P_error = g_error(1:3,4);
    trace_R_error = R_error(1,1) + R_error(2,2) + R_error(3,3);
    theta_error = acos((trace_R_error - 1)/2);
    meter_error = sqrt(P_error(1)^2 + P_error(2)^2 + P_error(3)^2);
    %wk = norm(a(1:3));
    %vk = norm(a(4:6));

    % define the threshold be 15 degree and 5cm
    % if less than the threshold, return final positional error in cm
    % if not, failure and return -1

    if theta_error <= deg2rad(1) && meter_error <= 0.01
        finalerr = meter_error * 100;
        return
    end
end
finalerr = -1;
end

