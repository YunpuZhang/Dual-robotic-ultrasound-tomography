%q=[1 1 1 1 1 1];
% q=[1 1 1 1 1 1]
% 
% gst= ur5FwdKin(q)
a = eye(4)
a(3,4)= 0.8
a
[ theta ] = ur5InvKin( gst )
