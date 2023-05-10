%% Logics
% 1. Create an UR5 object
% NEED: 
% ** Virtual Fixture need: current position, desired position, as well as
% jacobian
% ** Motion Planning needs X_TRUS, TODO
% 2. Mock a transformation X_TRUS 
% 3. Get transformation B_A, Body Jacobian, joint angles
% 4. Given the transformations to findPi_Test -> Get desired XYZRPY
% 5. Find the current position XYZRPY 
% 6. Feed the current and desired RPY to virtual fixture -> get q
% 7. Move the robot with the result q from virtual fixture
% 8. Update the transformation X_TRUS by algorithm

%% Verifying the Motion Planning
clc;
clear;
close all;

base_ref = "base_link";
T_step = 4;
k = 0.05;

ur5 = ur5_interface();
pause(1);

% Move robot out of initial position
% r = 0.5;
% theta = 45;
% X = [ROTZ(-80-theta),[r*cos(theta*pi/180) 0 r*sin(theta*pi/180)]';0 0 0 1];
% qtest = ur5InvKin(X);
% q = ur5.get_current_joints();
% q_test = getbest(qtest, q);
% ur5.move_joints(q_test, T_step);
ur5.move_joints([0;-1;-2;0;0.5;1], T_step);
pause(T_step);

TRUS_Frame = tf_frame(base_ref, "TRUS", eye(4));
pause(0.7);
Goal_Frame = tf_frame(base_ref, "Goal", eye(4));
pause(0.7);

% neg_z = [0;0;-1;1];
% unit_z = [0; 0; 1; 1];
% find P_I for angle from 45 to 135 (angle between abdominal probe and horizontal plane)
for theta = 45:5:135
    X_trus = [ROTz(90-theta),[0.3 0.3 0.6]';0 0 0 1];
    
    X = ur5.get_current_transformation(base_ref, "tool0");
    q = ur5.get_current_joints();
    Jb = ur5BodyJacobian(q);
    
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
        
    TRUS_Frame.move_frame(base_ref, X_trus);
    pause(0.5);
    Goal_Frame.move_frame(base_ref, Goal);
    pause(1.5);
    
    % Force converted from tool to base, as well as with a factor k
    f = [0, 0, -1+2*rand(1), 0, 0, 0]';
    f_p = (X)*[f(1:3);0];
    f(1:3) = f_p(1:3);
    f = k*f;
    
    Jdq = VF_checkpoint(initial, P_I6, f);
    
    Result = [ROTZ(initial(6) - Jdq(6))*ROTY(initial(5) - Jdq(5))*ROTX(initial(4) - Jdq(4)), initial(1:3) - Jdq(1:3, 1);
                                                                                 zeros(1,3),                          1];
    disp("SHOW: ");
    offset = P_I6(1:3) - (initial(1:3) - Jdq(1:3, 1));
    offset_p = inv(X)*[offset; 0];
    offset_p = offset_p(1:3);
    disp(offset_p);
    pause(1);

    qtest = ur5InvKin(Result);
    q_test = getbest(qtest, q);
    ur5.move_joints(q_test, T_step);
    pause(T_step);
      
   
end