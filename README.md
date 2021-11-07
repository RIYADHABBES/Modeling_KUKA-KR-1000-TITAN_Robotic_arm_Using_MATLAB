# Modeling_KUKA-KR-1000-TITAN_Robotic_arm_Using_MATLAB
Applied robotics project

### Description
- 3d simulation of a 6DOF robotic arm motion.
- The motion is giving by manipulating the angles in the joint space to follow a desired trajectory.

### Objectives
The goal is to plan the trajectory of the KUKA KR 1000 TITAN robotic arm that have six degrees of freedom 
in a way its end effector moves from point A to point B and compare it with the line (AB) while varying
the angles of our links.

The validation of the model and the results will be done by a script programmed in MATLAB.
    
To achieve this goal, we went through intermediate stages:

- Read the datasheet of the robot.
- Establish a kinematic diagram of the robot (topology and geometric parameters).
- Determine the Denavit-Hartenberg parameters.
- Establish the direct geometric model.
- Establish a control law for a point-to-point type trajectory.
- Determine the trajectory of the effector.
          
![A* Search](https://github.com/RIYADHABBES/Modeling_KUKA-KR-1000-TITAN_Robotic_arm_Using_MATLAB/blob/main/KUKA_KR_1000_0001.jpg?raw=true) <img src="https://raw.githubusercontent.com/RIYADHABBES/Modeling_KUKA-KR-1000-TITAN_Robotic_arm_Using_MATLAB/main/kuka.gif" width="550" height="450"/>




### Motion Planner
- Angular trajectories were planned by interpolation of the order of 5.

## Notes
- Since the frame of this work is in french, the documentation and comments are in french as well.

### Dependencies
- *matlab*

### Version

- v 1.0 beta
