clc;
clear all;
close all;

%% Define Transformations

W = eye(6, 6);
% If there are six joints, then J should be 6*6
J = -pi + (2*pi)*rand(6, 6);
% J = [-70,   0,   0, 0,  0, 0;
%        0, 805, 445, 0, 65, 0;
%        0,   0,   0, 0,  0, 0;
%        0,  -1,  -1, 0, -1, 0;
%        0,   0,   0, 0,  0, 0;
%        1,   0,   0, 1,  0, 1];

% xp = [3, 3.1, 3, 0, 1, 0]';
% Pt = [3, 3, 3, 0, 0, 1]';
% xp = [0, 0, 0, 0, 0, 0]';
xp = [6.4524, 0, 7.6897, 0.6428, 0, 0.7660]';
Pt = [5.7577, 0, 8.2228, 0.5736, 0, 0.8192]';


delta = xp - Pt;
f = [0, 0, -1+2*rand(1), 0, 0, 0]';

%% Approximate sphere with n*m polyhedron
n = 4;
m = 4;

dt = 0.02;
epsilon = 0.05;

H = zeros(n*m, 6);
h = zeros(n*m, 1);

mc = 1;
nc = 1;
for i = 1:1:(n*m)
    if (i > 12)
        h(i) = -epsilon;
    else
        h(i) = -epsilon;
    end
%     h(i) = 0;
    H(i, :) = [-cos(2*pi*nc/n)*cos(2*pi*mc/m), -cos(2*pi*nc/n)*sin(2*pi*mc/m), -sin(2*pi*nc/n), 0, 0, 0];
    mc = mc + 1;
    if (mc > m)
        mc = 1;
        nc = nc + 1;
    end
end
% h(16) = -epsilon;
h = h - H*delta;




%% Solving the Problem
lb = zeros(6, 1);
ub = lb;
for i = 1:1:6
    lb(i) = -0.2;
    ub(i) = 0.2;
end
% dq = lsqlin(W*J, delta, H*J, -h, [], [], lb, ub);

dq = lsqlin(W*J, delta + f, H*J, -h);

Jdq = J*dq;
Pt
xp - Jdq

if (norm(xp - Jdq - Pt) <= 1.0e-5)
    disp("True");
end