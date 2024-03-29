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

% writeObj = VideoWriter("PI.avi");
% open(writeObj);
r = 10;
figure('units','normalized','outerposition',[0 0 1 1])
xlim([-10 10])
ylim([-10 10])
zlim([0 12])



teta=0:0.01:pi;
x=r*cos(teta);
z=r*sin(teta);
plot3(x,zeros(1,numel(x)),z)
hold on;
grid on;
unit_z = [0;0;1;0];


% find P_I for angle from 45 to 135 (angle between abdominal probe and horizontal plane)
for theta = 45:1:135
    X_trus = [ROTZ(85-theta),[0 0 0.6]';0 0 0 1];
    X = [ROTZ(-90-theta),[r*cos(theta*pi/180) 0 r*sin(theta*pi/180)]';0 0 0 1];
    P_I = findPItest(X_trus,X);
    direction = X_trus*unit_z;
    h = plotv3([0;0;0], direction(1:3), 'green');
    pause(0.2);
    plotp3(P_I);
    pause(0.1);
    
    if (theta ~= 135)
        pause(0.1);
        set(h,'Visible','off')
    end
    
    drawnow;
%     frame = getframe;
%     writeVideo(writeObj, frame);
    
end

% close(writeObj);
