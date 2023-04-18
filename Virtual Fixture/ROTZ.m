function outputz = ROTZ(theta)
theta = theta*pi/180;
outputz = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end