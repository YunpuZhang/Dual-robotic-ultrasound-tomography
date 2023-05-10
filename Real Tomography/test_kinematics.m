clc;
clear;
close all;

base_ref = "base_link";
ur5 = ur5_interface();

TRUS_Frame = tf_frame(base_ref, "TRUS", eye(4));
pause(0.7);

% XA = [ROTY(-90*pi/180)*ROTZ(180*pi/180) [0.2529066; 0; 0]; 0 0 0 1];

XA = [-0.0441 0.9571 -0.2865 0.2529066; 0.7949 0.2073 0.5703 -0.001771; 0.6051 -0.2026 -0.77 0.0076861; 0 0 0 1];

% theta = 0;
% B_new = [0.4408 0.7526 -0.4894 0.2347288; 0.8729 -0.4863 0.0385 0.1900574; -0.209 -0.4442 -0.8713 0.7482927; 0 0 0 1];
% B_new = [0.4408, 0.4894, -0.7526, 0.2347; 
%          -0.8729, -0.0385, 0.4863, 0.1900574; 
%          0.2090, 0.8713, 0.4442, 0.7483;
%          0     ,      0,       0,     1];
% B_new = [0.7078, -0.5410,  0.4544, -0.1191;
%          0.2471, -0.4129, -0.8767,  0.2153;
%          0.6617,  0.7328, -0.1586,  0.7585;
%               0,       0,       0,       1];
% X_trus = B_new*[ROTX(pi) [0; 0; 0]; 0 0 0 1];

% X_trus = B_new * [ROTX((theta)*pi/180),[0 0 0]'; 0 0 0 1];

% TRUS_Frame.move_frame(base_ref, X_trus);
% pause(0.5);

bee = ur5.get_current_transformation(base_ref, "ee_link");

bnew = bee*XA
TRUS2_Frame = tf_frame(base_ref, "TRUS2",bnew);

