# Dual-robotic-ultrasound-tomography

The functionality implemented is the BXp calculation, motion planning algorithm, ROS control, and virtual fixture. They are in three folders and one zip file respectively. (The BXp calculation code could be found in bxp_RF_calibrater.zip.) 

In addition to those functionalities, there is also a directory named Real Tomography, which contains the code for controlling the real robot and motor to achieve the desired ultrasound scan behavior. If you are not interested in MATLAB simulation and would like to focus on the final used programs, please skip the first three directories and jump to ROS Control. 

## Bxp_RF_calibrater.zip:
Instructions are inside the zip file. 


## MOTION PLANNING:

The motion planning algorithm functions are find_PI.m and find_PItest.m. They would calculate a P_goal based on its input matrix.

There is an example code for how to use the function named main_PI.m, which would also produce a MATLAB simulation. 


## VITRUAL FIXTURE:

The virtual fixture code is in VF_Point_Dir.m. Both 'Stay at a Point' and 'Maintain a Direction' are implemented in this file. 

The virtual fixture would find a desired delta q for joint angles, and it will print out the result of the manipulator Jacobian multiplied by this delta q. 

To visualize the result of the virtual fixture, you may want to run main_VF.m, which would give a MATLAB simulation. This file is a combination of motion planning and the virtual fixture. Specifically, a P_goal will be calculated first, and then Virtual Fixture would provide joint angles to move the robot to that desired position with a fixed orientation. In the simulation, the green part is the y-axis of the TRUS probe, then the find_PItest.m function would calculate a desired goal position and orientation as shown in black dots in the simulation. Finally, the abdominal probe would move based on the virtual fixture result, as shown by the red components. 


## ROS Control:

In this folder, a communication between two laptops was implemented (this folder only has one side of the code for the Windows computer, which is used to control the TRUS probe based on messages received from a ROS publisher on a Ubuntu computer, if you are looking for the rest, please read the documentation below for "Real Tomography"), as well as using a KDC101 controller to control the rotary stage. The TRUS_Control.m file is the main code. 

To run this code, you need to first implement software for the KDC101 controller (Kinesis Thorlabs). The code depends on some .ddl files from the software. Putting that software in the default location would make life easier, as that is where the code would look for those .ddl files. If not, the position of all the files needed to be changed in the code. -> Begin with KDC101.init. In addition, the softwares only support Windows system. 

Other than that, more instructions for TRUS_Control.m will be in the next section. 

## Real Tomography:
As indicated at the very top, this is the directory of everything we used at the very end. You will find some scripts existing in other directories in this one. 

### UR5_Real_Motion.m
This is the main program. 

To run this program, you need to have a computer with a Linux system. We used Ubuntu 18.04. Then, you need to install ROS1, multiple versions could work, but for reference, we used Melodic. 

Then, you also need a launch file for the UR5, which could be found in the ROS package ur_modern_driver/launch as ur5_bringup.launch. You are also welcome to write your own. The instruction for the ROS package could be found at https://github.com/ros-industrial/ur_modern_driver. 

After that, you need MATLAB on your computer with ROS_Toolbox installed. Be careful with the version of Python required based on your MATLAB version. You may need to also install a specific version of Python based on MATLAB documentation. 

The first section is to create the needed ROS enviroment, the second section is where all the algorithm comes together. 

As mentioned in the ROS Control section, the part on the Ubuntu computer is the first section of this code. The order of code running should be 
1. Run the first section of UR5_Real_Motion.m on the Ubuntu computer
2. Run the first section of TRUS_Control.m on the Windows computer
3. Run the second section of TRUS_Control.m on the Windows computer
4. Run the second section of UR5_Real_Motion on the Ubuntu computer.

The UR5_Real_Motion.m file could be run without the Windows computer, it would still publish messages; there is just no computer listen to the messages. 

For the logics of this program, refer to the comments inside the program. 

### Unused Programs USCT
There are a lot of functions made during the past months but ended up not being used, as well as some code served as unit test code. Those are sorted into this sub-directory. 
#### findPI.m
This function is very similar to the findPItest function, the only difference is the input it is asking for. 
#### getXi.m
This function would take a homogenous transformation function and return a 6 by 1 twist. 
#### test_VF.m
This program is used for testing the idea of Jacobian in a Virtual Fixture. More specifically, which type of jacobian should we use to convert {x, y, z, roll, pitch, yaw} to joint angles?
#### ur5BodyJacobian.m
This function is used to calculate the body jacobian of the UR5
#### ur5RRcontrol.m
This function is a resolved control for the UR5. It would take a homogenous desired transformation, a scaling factor K, and a ur5 object to control the UR5 robot to the desired transformation. 



