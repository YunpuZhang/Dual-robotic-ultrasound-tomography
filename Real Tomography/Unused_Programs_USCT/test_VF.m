clc;
clear;
close all;

base_ref = "base_link";
T_step = 4;

ur5 = ur5_interface();
pause(1);

ur5.move_joints(zeros(6, 1), 5);
pause(5);

q1 = ur5.get_current_joints();
f1 = ur5.get_current_transformation(base_ref, "tool0");
J1 = ur5BodyJacobian(q1);

dq = [0; 0; 0.1; 0.1; 0.1; 0.1];

ur5.move_joints(dq, T_step);
pause(T_step);

q2 = ur5.get_current_joints();
f2 = ur5.get_current_transformation(base_ref, "tool0");
J2 = ur5BodyJacobian(q2);

% What is the Jacobian? Converting from joint space to XYZRPY