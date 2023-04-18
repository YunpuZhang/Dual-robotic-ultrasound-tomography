function P_I = findPItest(X_trus,X) 

T_A = inv(X_trus)*X;
P_A = T_A(1:3,4);
n_A = T_A(1:3,1:3) * [0,0,1]';
P_T = [0,0,0]';
n_T = [0,0,1]';

t = (dot(n_A, P_A) - dot(n_A, P_T))/ dot(n_A, n_T);
P_I = P_T + t*n_T;

P_I = X_trus*[P_I;1];
P_I = P_I(1:3,1);

end