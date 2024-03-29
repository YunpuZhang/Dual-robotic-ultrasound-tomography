function Jdq = VF_checkpoint(xp, Pt)
    %% Define Transformations
    W = eye(6, 6);
    % If there are six joints, then J should be 6*6
    J = -pi + (2*pi)*rand(6, 6);

%     xp = [3, 3.1, 3]';
%     Pt = [3, 3, 3]';
%     dir = zeros(3, 1);

%     delta = [xp - Pt; dir];
    delta = xp - Pt;
    f = [0, 0, 0, 0, 0, 0]';

    %% Approximate sphere with n*m polyhedron
    n = 4;
    m = 4;

    dt = 0.02;
    epsilon = 0.02;

    H = zeros(n*m, 6);
    h = zeros(n*m, 1);

    mc = 1;
    nc = 1;
    for i = 1:1:(n*m)
        h(i) = -epsilon;
        H(i, :) = [-cos(2*pi*nc/n)*cos(2*pi*mc/m), -cos(2*pi*nc/n)*sin(2*pi*mc/m), -sin(2*pi*nc/n), 0, 0, 0];
        mc = mc + 1;
        if (mc > m)
            mc = 1;
            nc = nc + 1;
        end
    end
    h = h - H*delta;




    %% Solving the Problem
    lb = zeros(6, 1);
    ub = lb;
    for i = 1:1:6
        lb(i) = -0.2;
        ub(i) = 0.2;
    end
    % dq = lsqlin(W*J, delta, H*J, -h, [], [], lb, ub);

    dq = lsqlin(W*J, delta + f, H*J, -h);
    
    Jdq = J*dq;
    

end

