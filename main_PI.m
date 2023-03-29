clear;
clc;

% X_trus1 = [ROTZ(40),[0;0;0];0 0 0 1];
% X1 = [ROTZ(-135),[5;0;5];0 0 0 1];
% P_I1 = findPItest(X_trus1,X1);
% plotp3(P_I1)
% grid on
% hold on
% 
% X_trus2 = [ROTZ(-5),[0;0;0];0 0 0 1];
% X2 = [ROTZ(-180),[0;0;5*sqrt(2)];0 0 0 1];
% P_I2 = findPItest(X_trus2,X2);
% plotp3(P_I2)
% 
% X_trus3 = [ROTZ(-50),[0;0;0];0 0 0 1];
% X3 = [ROTZ(-225),[-5;0;5];0 0 0 1];
% P_I3 = findPItest(X_trus3,X3);
% plotp3(P_I3)

% plotr([ROTZ(30)])
% grid on
% hold on
% plotr([ROTZ(0)])

% simulate the phantom by a semi-circle
r = 10;
figure;


teta=0:0.01:pi;
x=r*cos(teta);
z=r*sin(teta);
plot3(x,zeros(1,numel(x)),z)
hold on;
grid on;


% find P_I for angle from 45 to 135 (angle between abdominal probe and horizontal plane)
for theta = 45:5:135
    X_trus = [ROTZ(85-theta),[0 0 0]';0 0 0 1];
    X = [ROTZ(-90-theta),[r*cos(theta*pi/180) 0 r*sin(theta*pi/180)]';0 0 0 1];
    P_I = findPItest(X_trus,X);
    plotp3(P_I)
    pause(0.1);
    drawnow;
end

