clc;
clear;
close all;



r = 10;
figure('units','normalized','outerposition',[0 0 1 1])


theta=0:0.01:pi;
x=r*cos(theta);
z=r*sin(theta);
plot3(x,zeros(1,numel(x)),z)
hold on;
grid on;
xlim([-10 10])
ylim([-10 10])
zlim([0 12])

unit_z = [0;0;1;0];
% J = -pi + (2*pi)*rand(6, 6);
J = eye(6);

% find P_I for angle from 45 to 135 (angle between abdominal probe and horizontal plane)
for theta = 45:5:135
    if (theta == 45)
        initial = [0;0;0;0;0;0];
    else
        initial = [P_I; direction(1:3)];
    end
    
    X_trus = [ROTz(85-theta),[0 0 0]';0 0 0 1];
    X = [ROTz(-90-theta),[r*cos(theta*pi/180) 0 r*sin(theta*pi/180)]';0 0 0 1];
    direction = X_trus*unit_z;
%     disp(direction);
    P_I = findPItest(X_trus,X);
    h = plotv3([0;0;0], direction(1:3), 'green');
    pause(0.2);
    
    plotp3(P_I);
    pause(0.3);
    
    dq = VF_checkpoint(initial, [P_I; direction(1:3)], J);
    Jdq = J*dq;
    fup = initial - Jdq;
    plotv3(fup(1:3), fup(1:3) + fup(4:6), 'red');
    
    
    if (theta ~= 135)
        pause(0.3);
        set(h,'Visible','off')
    end
    drawnow;
end