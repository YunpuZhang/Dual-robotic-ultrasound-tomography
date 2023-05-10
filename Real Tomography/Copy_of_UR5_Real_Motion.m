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
T_step = 5;
k = 0.05;

ur5 = ur5_interface();
pause(1);

% Move robot out of initial position
ur5.move_joints([0;-1;2;-1.5;0.5;0], T_step);
pause(T_step);


TRUS_Frame = tf_frame(base_ref, "TRUS", eye(4));
pause(0.7);
Goal_Frame = tf_frame(base_ref, "Goal", eye(4));
pause(0.7);
Probe_Frame = tf_frame(base_ref, "Probe", eye(4));
pause(0.7);


% neg_z = [0;0;-1;1];
% unit_z = [0; 0; 1; 1];
% find P_I for angle from 45 to 135 (angle between abdominal probe and horizontal plane)
for theta = -45:2:45
%     B_new = [0.4408 0.7526 -0.4894 0.2347288; 0.8729 -0.4863 0.0385 0.1900574; -0.209 -0.4442 -0.8713 0.7482927; 0 0 0 1];
    B_new = [-0.1591   -0.6965    0.6997   -0.1157;
               -0.9278    0.3478    0.1353    0.2065;
               -0.3376   -0.6276   -0.7015    0.7458;
                     0         0         0    1.0000];
    X_trus = B_new * [ROTX((theta)*pi/180),[0 0 0]'; 0 0 0 1];
%     X_trus = [ROTz(90-theta),[0.3 0.3 0.6]';0 0 0 1];
%     X = B_new * [eye(3) [0 0 0.4]'; 0 0 0 1] * [ROTX((theta-5)*pi/180),[0 0 0]'; 0 0 0 1];
    X = ur5.get_current_transformation(base_ref, "ee_link");
    XA = [ROTY(-90*pi/180)*ROTZ(180*pi/180) [0.2529066; 0; 0]; 0 0 0 1];
%     XA = [eye(3) [0 ; 0; 0.2529066]; 0 0 0 1];
%     XA = [-0.0441 0.9571 -0.2865 0.2529066; 0.7949 0.2073 0.5703 -0.001771; 0.6051 -0.2026 -0.77 0.0076861; 0 0 0 1];
%     XA = eye(4);
    X = X*XA;
    q = ur5.get_current_joints();
    Jb = ur5BodyJacobian(q);
    
    P_I = Copy_of_findPItest(X_trus,X);
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
    pause(0.5);
    
    % Force converted from tool to base, as well as with a factor k
    f = [0, 0, 0, 0, 0, 0]';
%     f = [0, 0, -1+2*rand(1), 0, 0, 0]';
    f_p = (X)*[f(1:3);0];
    f(1:3) = f_p(1:3);
    f = k*f;
    
    Jdq = VF_checkpoint(initial, P_I6, f);
    
    Result = [ROTZ(initial(6) - Jdq(6))*ROTY(initial(5) - Jdq(5))*ROTX(initial(4) - Jdq(4)), initial(1:3) - Jdq(1:3, 1);
                                                                                 zeros(1,3),                          1];
%     disp("SHOW: ");
%     offset = P_I6(1:3) - (initial(1:3) - Jdq(1:3, 1));
%     offset_p = inv(X)*[offset; 0];
%     offset_p = offset_p(1:3);
%     disp(offset_p);
%     pause(1);
    Probe_Frame.move_frame("ee_link", XA);
    pause(0.5);
 
    eelink2tool0 = quat2rotm([0.500, -0.500, 0.500, -0.500]);
    Result = Result * inv(XA) * [eelink2tool0 [0 0 0]'; 0 0 0 1];
    qtest = ur5InvKin(Result);
    q_test = getbest(qtest, q);
    ur5.move_joints(q_test, T_step);
    pause(T_step);
      
   
end