function q = getbest(theta,current)
%function to find the best solution from the theta got from inverse
%kinematics
%output: theta 6*1 matrix
%input : thetalist 6*8 matrix represents all possibe solution
%input : current 6*1 matrix to thet current position
max_distance = 9999;
slist = [];                  
for i = 1:8
%     if zcheck(theta(:,i)) ~= -1
        slist =[slist theta(:,i)];
        distance = abs(3*(abs(theta(1,i))-abs(current(1,1))) + sum(abs(theta(2:6,i)))-sum(abs(current(2:6,1))));
        %distance=abs(3*abs(theta(1,i))+sum(abs(theta(2:6,i)))-current);
        if max_distance > distance
            max_distance = distance;
            q = theta(:,i);
%             disp(distance)
        end
%     else
%         fprintf("fail the safety check")
%     end
end
% end

% %safety check if this is in the shoulder link is above the table
% function theta = zcheck(theta)
% if (theta(2)>= -2.97 | theta(2) <=-0.17)
%     theta = 0;
% else
%     theta = -1;
% end
% end