function P_I = findPI(X_trus,B_A_r, B_A_p, X_A) 

% X_trus is assumed to the transformation of TRUS w.r.t base
% X_A is the 4*4 transformation matrix of abdominal probe w.r.t end-effector calculated by calibration
% B_A_r and B_A_p are the rotation and translation of end-effector w.r.t base obtained from ros


B_A_r = [B_A_r(1,4), B_A_r(1,1), B_A_r(1,2), B_A_r(1,3)]; % the rotation is a quaternion
B_A_r = quat2rotm(B_A_r);
B_A = [B_A_r, B_A_p'; 0 0 0 1];
 
T_A = inv(X_trus)*B_A*X_A;
P_A = T_A(1:3,4);
n_A = T_A(1:3,1:3) * [0,1,0]';
P_T = [0,0,0]';
n_T = [0,1,0]';

t = (dot(n_A, P_A) - dot(n_A, P_T))/ dot(n_A, n_T);
P_I = P_T + t*n_T;

% transfer the P_I from TRUS frame to base frame
P_I = X_trus*[P_I;1];
P_I = P_I(1:3,1);

end