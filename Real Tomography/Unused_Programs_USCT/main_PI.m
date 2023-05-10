clear;
clc;


unit_z = [0;0;1;0];


% find P_I for angle from 45 to 135 (angle between abdominal probe and horizontal plane)
for theta = 45:0.01:135
    X_trus = [ROTZ(85-theta),[0 0 0.6]';0 0 0 1];
    X = [ROTZ(-90-theta),[r*cos(theta*pi/180) 0 r*sin(theta*pi/180)]';0 0 0 1];
    P_I = findPItest(X_trus,X);
    direction = X_trus*unit_z;
    P_I_ = [P_I; direction];

end

% close(writeObj);
