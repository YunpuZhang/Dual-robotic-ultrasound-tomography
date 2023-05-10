function xi = getXi(g)
% Purpose: Take a homogenous transformation matrix and extract the unscaled twist
% Input: g: a homogeneous transformation
% Output: xi: the (un-normalized) twist in 6 × 1 vector or twist coordinate form such that
% g = exp(ˆξ)

[rows, cols] = size(g);
if ((rows ~= 4) | (cols ~= 4))
  error('getXi requires a 4x4 matrix argument. Check your dimensions.');
end

d = det(g);
if ((d <0.999) | (d >1.001))
  fprintf(1,'Error in getXi: the argument is not a rotation. Determinent is %f, should be 1.0\n',d);
  error('aborting');
end

xi_hat = logm(g);
v = xi_hat(1:3,4);
w_hat = xi_hat(1:3,1:3);
w = [w_hat(3,2);w_hat(1,3);w_hat(2,1)];
xi = [v;w];
end