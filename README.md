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

---

## Project Overview

The system integrates:
- **Geometric control on SO(3)** for robust attitude and position regulation.
- **Artificial Potential Field (APF)** for goal attraction and obstacle repulsion.
- **Offline APF trajectory planning** when obstacle locations are known.
- **Online APF computation** using **LiDAR** data for unknown environments.
- **Disturbance and mass estimation** using a momentum-based observer.

Two simulation environments are used:
- **MATLAB/Simulink** for controlled dynamic validation and analysis.
- **ROS + Gazebo** for realistic testing in structured and unstructured environments.

### Key Characteristics
| Component | Role | Notes |
|---------|------|-------|
| Geometric Controller | Regulates UAV pose and velocity | Coordinate-free formulation avoids attitude singularities |
| APF Planner | Generates velocities/trajectories | Operates offline or online |
| LiDAR Online Sensing | Detects obstacles in real time | Enables navigation in unknown environments |
| Momentum-Based Estimator | Compensates model uncertainty | Improves height tracking and thrust accuracy |

---

## High-Level Workflow

1. **Goal position** is set either manually or via an autonomous mission profile.
2. APF computes a **desired total force direction**:
   - Attraction toward goal
   - Repulsion away from obstacles
3. The geometric controller converts the desired motion into:
   - **Thrust command**
   - **Torque command** for attitude control
4. Commands are mapped to motor speeds and executed in the UAV dynamics.

---

## Repository Structure (Conceptual)
```
FSR_FinalProject/
├── MATLAB/                        → Simulink implementation
├── geometric_controller/          → ROS package for UAV control + APF
├── worlds/                        → Gazebo simulation environments
├── models/                        → UAV and environment obstacle models
└── docs/                          → Documentation and project report
```

---

## Installation Instruction - MATLAB/Simulink
(To clone the Matlab branch of the repository)
```
git clone https://github.com/gaetanotorella/FSR_FinalProject.git
```

## Installation Instructions - Ubuntu 20.04 with ROS Noetic
(To clone the ROS branch of the repository)
```
cd ~/catkin_ws/src
git clone -b ROS --recursive –submodules https://github.com/gaetanotorella/FSR_FinalProject.git
cd ..
catkin build
source devel/setup.bash
```

## ROS Simulation
1. Launch Gazebo and spawn the UAV
   ```
   roslaunch geometric_controller UAV.launch world_name:=APF_world
   ```
   Default world: **APF_world** (others available in `/geometric_controller/worlds`).

2. Launch the geometric controller
   ```
   roslaunch geometric_controller geometric_controller_apf.launch
   ```

3. Launch APF method
   ```
   roslaunch geometric_controller apf_method.launch version:=offline x_goal:=0 y_goal:=0 z_goal:=2
   ```
   Versions:
   - `offline`: predefined map
   - `online`: real-time sensing using LiDAR

---

## Notes on Performance

- **Offline APF** yields smooth trajectories but requires known obstacle locations.
- **Online APF** adapts dynamically but may introduce force spikes in cluttered environments.
- Increasing influence distance **ηo** and applying filtering can smooth repulsive behavior.
- **Mass estimation** significantly improves steady-state altitude tracking.

---

## Demo Video

https://github.com/user-attachments/assets/4994c09c-fcf5-4eb4-8782-62c19b83f6e6

https://github.com/user-attachments/assets/fe69edc4-9da9-4946-91f7-0fea17f3d348










