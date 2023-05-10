ur5 = ur5_interface();
%transformation = [0.9024 0.3188 -0.2898 -100.6152; -0.2202 0.9196 0.3253 702.4821; 0.3702 -0.2297 0.9001 740.1838; 0 0 0 1]

%transformation = [0.9024 -0.2305 0.364 -143.6312; 0.3188 0.9255 -0.2044 18.8811; -0.2898 0.3005 0.9087 -773.6679; 0 0 0 1];
transformation = [0.9204 0.3839 0.0736 -41.9367; -0.3015 0.8169 -0.4918 196.0632;-0.2489 0.4305 0.8676 -764.6999; 0 0 0 1];


transformation = inv(transformation)
[theta1] = ur5InvKin(transformation);

current = ur5.get_current_joints()
theta1 = getbest(theta1,current)
ur5.move_joints(theta1,150)

18.16
