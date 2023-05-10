function finalerr = ur5RRcontrol(gdesired, K, ur5)

ur5 = ur5_interface();

qk = ur5.get_current_joints();
% read the initial angles of UR5 
Tstep = 1;
iterations = Tstep * 100;


R_gdesired = gdesired(1:3,1:3);
P_gdesired = gdesired(1:3,4);
gdesired_inv = [transpose(R_gdesired) -transpose(R_gdesired)*P_gdesired; 0 0 0 1];
% gdesired is a constant

for k = 0: Tstep: iterations 
        
    gst = ur5FwdKin(qk); 
    g_error = gdesired_inv * gst;
    a = getXi(g_error);
    body_Jacobian = ur5BodyJacobian(qk);
    
    if det(body_Jacobian) == 0
        finalerr = -1;
        return
        % check for singularities in each step, if failure, return -1
    else
        qk = qk - K * Tstep * inv(body_Jacobian) * a;
        ur5.move_joints(qk,3);
        pause(1)
    end
            
    % find the vk and wk (theta_error and meter_error) in each step
    R_error = g_error(1:3,1:3);
    P_error = g_error(1:3,4);
    trace_R_error = R_error(1,1) + R_error(2,2) + R_error(3,3);
    theta_error = acos((trace_R_error - 1)/2);
    meter_error = sqrt(P_error(1)^2 + P_error(2)^2 + P_error(3)^2);
    %wk = norm(a(1:3));
    %vk = norm(a(4:6));
    
    % define the threshold be 15 degree and 5cm
    % if less than the threshold, return final positional error in cm
    % if not, failure and return -1
    
    if theta_error <= deg2rad(1) && meter_error <= 0.01
        finalerr = meter_error * 100;
        return
    end    
end
    finalerr = -1;
end