# Dual-robotic-ultrasound-tomography

The functionity implemented are the BXp calculation, motion planing algorithm, and virtual fixture. 

The BXp problem code could be found in 






The motion planning algorithm function is find_PI.m and find_PItest.m.

There is an example code for how to use the function named as main_PI.m, which would also produce a simulation. 



The virtual fixture code is in VF_Point_Dir.m. In this file, both 'Stay at a Point' and 'Maintain a Direction' are implemented in this file. 
The virtual fixture would find a desired delta q for joint angles, and it will print out the result of manipulator Jacobian multiplied this delta q. 

To visualize the result of virtual fixture, you may want to run main_VF.m, which would give a simulation. 
