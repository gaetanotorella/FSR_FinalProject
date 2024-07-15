# Geometric Controller with APF method for UAV

This project presents a geometric controller to manage the position and attitude of an Unmanned Aerial Vehicle (UAV). 
The developed control system aims to ensure high stability and reliability during flight. 
The Artificial Potential Field (APF) method was implemented for obstacle avoidance, providing a solution both in offline mode, 
for preemptive path planning and in online mode, for real-time adaptation using exteroceptive sensors.
The simulations were conducted in two distinct development environments: Simulink and ROS/Gazebo. 
Simulink was used to model and simulate dynamic controls, allowing for a detailed analysis of system performance under controlled conditions. 
ROS/Gazebo provided a realistic simulation environment in which drone behavior was tested in complex scenarios that included various obstacles.
The simulation results indicate that the proposed system can maintain safe and precise navigation even in the presence of unexpected obstacles. 
The combined approach of geometric control and APF method has proven effective in avoiding collisions, significantly enhancing the drone’s 
capability to operate autonomously in dynamic and potentially complex environments. 
This study confirms the validity of the proposed method and suggests its applicability for future implementations in autonomous drones intended 
for critical missions such as surveillance, inspection, and goods delivery.


## Installation Instruction - MATLAB/Simulink
---------------------------------------------------------
To clone the Matlab branch of the repository

```
git clone https://github.com/gaetanotorella/FSR_FinalProject.git
```


## Installation Instructions - Ubuntu 20.04 with ROS Noetic
---------------------------------------------------------
To clone the ROS branch of the repository

```
cd ~/catkin_ws/src
git clone -b ROS --recursive –submodules https://github.com/gaetanotorella/FSR_FinalProject.git
cd ..
catkin build
source devel/setup.bash
```

## ROS simulation
---------------------------------------------------------
1. Launch Gazebo and spawn the UAV
   ```
   roslaunch geometric_controller UAV.launch world_name:=APF_world
   ```
   default world is "APF_world", other worlds are in "/geometric_controller/worlds"

2. Launch geometric controller
   ```
   roslaunch geometric_controller geometric_controller_apf.launch
   ```

3. Launch APF method
   ```
   roslaunch geometric_controller apf_method.launch version:=offline x_goal:=0 y_goal:=0 z_goal:=2
   ```
   available version are "offline" (only for "APF_world") and "online". It's possible to choose desired position by passing x y z coordinates.
