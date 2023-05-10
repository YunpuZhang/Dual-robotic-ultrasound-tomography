function R_after = ROTX(roll)
            %rotation based on x axis
            %input : scalar roll value
            %output : 3*3 matrix 
            R_after = [1 0 0;0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
end