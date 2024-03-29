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
f = [0, 0, 3, 0, 0, 0]';

%% Approximate sphere with n*m polyhedron
% For Stay as a Point
n = 4;
m = 4;

dt = 0.02;
epsilon1 = 0.05;

Hp = zeros(n*m, 6);
hp = zeros(n*m, 1);

mc = 1;
nc = 1;
for i = 1:1:(n*m)
    hp(i) = -epsilon1;
    Hp(i, :) = [-cos(2*pi*nc/n)*cos(2*pi*mc/m), -cos(2*pi*nc/n)*sin(2*pi*mc/m), -sin(2*pi*nc/n), 0, 0, 0];
    mc = mc + 1;
    if (mc > m)
        mc = 1;
        nc = nc + 1;
    end
end
hp = hp - Hp*delta;

% For Maintain a Direction
Hd = zeros(n*m, 6);
hd = zeros(n*m, 1);

epsilon2 = 0.0;

mc = 1;
nc = 1;
for i = 1:1:(n*m)
    hd(i) = -epsilon2;
    Hd(i, :) = [0, 0, 0, -cos(2*pi*nc/n)*cos(2*pi*mc/m), -cos(2*pi*nc/n)*sin(2*pi*mc/m), -sin(2*pi*nc/n)];
    mc = mc + 1;
    if (mc > m)
        mc = 1;
        nc = nc + 1;
    end
end
hd = hd - Hd*delta;

H = [Hp; Hd];
h = [hp; hd];


%% Solving the Problem
lb = zeros(6, 1);
ub = lb;
for i = 1:1:6
    lb(i) = -0.2;
    ub(i) = 0.2;
end
% dq = lsqlin(W*J, delta, H*J, -h, [], [], lb, ub);

dq = lsqlin(W*J, f, H*J, -h);

% [xp; zeros(3, 1)] + 
Jdq = J*dq;
xp - Jdq

if (norm(xp - Jdq - Pt) <= 1.0e-5)
    disp("True");
else
    disp("False");
    disp(norm(xp - Jdq - Pt))
end

% delta + 