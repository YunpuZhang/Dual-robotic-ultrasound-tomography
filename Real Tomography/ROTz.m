function R_after = ROTZ(yaw)
            %rotation based on Z axis
            %input : scalar roll value
            %output : 3*3 matrix 
            R_after = [cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0; 0 0 1];
end   