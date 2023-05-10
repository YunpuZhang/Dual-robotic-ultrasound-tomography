function J = ur5BodyJacobian(q)
w1 = [0 0 1];
w2 = [0 1 0];
w3 = [0 1 0];
w4 = [0 1 0];
w5 = [0 0 -1];
w6 = [0 1 0];
q1 =[0 0 0];
q2 =[0 0 0.0892];
q3 =[0.425 0 0.0892];
q4 =[0.817 0 0.0892];
q5 =[0.817 0.1093 0];
q6 =[0.817 0 -0.0055];

xi1 = [-cross(w1,q1) w1];
xi2 = [-cross(w2,q2) w2];
xi3 = [-cross(w3,q3) w3];
xi4 = [-cross(w4,q4) w4];
xi5 = [-cross(w5,q5) w5];
xi6 = [-cross(w6,q6) w6];
xi = [xi1' xi2' xi3' xi4' xi5' xi6'];
g0 = [ 0 1 0 0.8180;0 0 1 0.321;1 0 0 -0.004; 0 0 0 1];

J = zeros(6);
E = inv(g0);
for i = 6: -1 : 1  
    xi_hat = -[sk(xi(4:6,i)) xi(1:3,i); 0, 0, 0, 0];
    E = E * expm(xi_hat*q(i));
	J(:, i) = Adj(E) * xi(:, i);
end
end

function Adj = Adj(e)
%get a 6 by 6 adjoint matrix
R = e(1:3,1:3);
P = e(1:3,4);
Adj = [R sk(P)*R;zeros(3) R];
end

function skew = sk(x)
x1=x(1);
x2=x(2);
x3=x(3);
skew=[0 -x3 x2; x3 0 -x1; -x2 x1 0];
end


