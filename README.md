# Dual-robotic-ultrasound-tomography

The functionity implemented are the BXp calculation, motion planing algorithm, ROS control, and virtual fixture. They are in three folders and one zip file respectively. (The BXp calculation code could be found in bxp_RF_calibrater.zip.) 


MOTION PLANNING:

The motion planning algorithm function are find_PI.m and find_PItest.m. They would calculate a P_goal based on its input matrix.

There is an example code for how to use the function named as main_PI.m, which would also produce a simulation. 


VITRUAL FIXTURE:

The virtual fixture code is in VF_Point_Dir.m. Both 'Stay at a Point' and 'Maintain a Direction' are implemented in this file. 

The virtual fixture would find a desired delta q for joint angles, and it will print out the result of manipulator Jacobian multiplied this delta q. 

To visualize the result of virtual fixture, you may want to run main_VF.m, which would give a simulation. This file is a combination of the motion planning and the virtual fixture. Specifically, a P_goal will be calculted first, and then Virtual Fixture would provide joint angles to move the robot to that desired position with fixed orientation. In the simulation, the green part is the y-axis of the TRUS probe, then the find_PItest.m function would calculate a desired goal position and orientation as shown in black dots in simulation. Finally, the abdominal probe would move based on the virtual fixture result, as shown by the red components. 


ROS Control:

In this folder, a communication between two laptops were implemented, as well as using a KDC101 controller to control the rotary stage. The TRUS_Control.m file is the main code. 

To run this code, you need to implement a software for the KDC101 controller (Kinesis Thorlabs). The code depends on some .ddl files from the software. Put that software in default location would make the life easier, as that is where the code would look for those .ddl files. If not, the position of all the files needed to be changed in the code. -> Begin with KDC101.init. 
