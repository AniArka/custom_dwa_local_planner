# DWA Local Planner for ROS2 Humble

A custom implementation of the Dynamic Window Approach (DWA) local planner for TurtleBot3 in ROS2 Humble. This planner enables autonomous navigation with obstacle avoidance in Gazebo simulations.


https://github.com/user-attachments/assets/b076423b-ce86-45b4-aa45-1619451b5f42



## Features

- **Complete DWA Implementation**: Velocity sampling, trajectory prediction, and cost evaluation
- **Real-time Obstacle Avoidance**: Uses laser scan data for safe navigation
- **Adaptive Behavior**: Adjusts speed based on obstacle proximity and goal distance
- **Visualization**: Real-time trajectory visualization in RViz
- **Recovery Behaviors**: Automatic stuck detection and recovery
- **Configurable Parameters**: Easy tuning through ROS2 parameters

## Requirements

- ROS2 Humble
- Gazebo
- TurtleBot3 packages
- Python 3.8+

## Installation

### 1. Set up ROS2 Humble Environment
```bash
source /opt/ros/humble/setup.bash
```

### 2\. Install TurtleBot3 Packages

```bash

sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3\*
```
### 3\. Create and Build Workspace

```bash

mkdir -p ~/dwa_planner_ws/src
cd ~/dwa_planner_ws/src
git clone https://github.com/AniArka/custom_dwa_local_planner
cd ..
colcon build
source install/setup.bash
```
## Usage

### 1\. Launch Simulation Environment

```bash

export TURTLEBOT3\_MODEL\=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
### 2\. Run the DWA Planner

```bash

source ~/dwa_planner_ws/install/setup.bash
ros2 run custom_dwa_planner dwa_planner
```
### 3\. Visualize in RViz

```bash

rviz2
```
Add the following displays in RViz:

*   LaserScan (`/scan`)
    
*   Odometry (`/odom`)
    
*   MarkerArray (`/trajectory_markers`)
    
*   Marker (`/goal_marker`)
    

## Configuration Parameters

The planner can be configured via ROS2 parameters in the launch file or dynamically:

| Parameter | Default | Description |
| --- | --- | --- |
| max_vel | 0.12 | Maximum linear velocity (m/s) |
| min_vel | 0.05 | Minimum linear velocity (m/s) |
| max_rot_vel | 0.8 | Maximum angular velocity (rad/s) |
| max_accel | 0.3 | Maximum linear acceleration (m/s²) |
| max_rot_accel | 0.5 | Maximum angular acceleration (rad/s²) |
| predict_time | 2.0 | Trajectory prediction time (s) |
| goal_tolerance | 0.3 | Goal distance tolerance (m) |
| obstacle_safe_distance | 0.5 | Safe distance from obstacles (m) |
| robot_radius | 0.2 | Robot collision radius (m) |

## Node Structure

### Publishers

*   `/cmd_vel` (`geometry_msgs/Twist`) - Velocity commands for robot control
    
*   `/trajectory_markers` (`visualization_msgs/MarkerArray`) - Visualized trajectories in RViz
    
*   `/goal_marker` (`visualization_msgs/Marker`) - Goal position visualization
    

### Subscribers

*   `/odom` (`nav_msgs/Odometry`) - Robot pose and velocity information
    
*   `/scan` (`sensor_msgs/LaserScan`) - Obstacle detection data from laser
-----
## Planner Architecture & Approach

This project implements a **goal-directed Dynamic Window Approach (DWA)** local planner **from scratch** without Nav2, to demonstrate understanding of real robot motion planning fundamentals rather than relying on packaged stacks.

The planner continuously samples feasible velocity commands, predicts motion trajectories, evaluates them, and executes the best one based on weighted scores and safety rules.

**Core Architecture**

|**Component**|**Purpose**|
| :- | :- |
|Laser Processing|Extract obstacle clearance & safety radius|
|Odometry & TF|Compute robot state (x, y, yaw, v, ω)|
|Dynamic Window|Sample feasible (v, ω) based on accel limits|
|Trajectory Rollout|Predict forward motion arcs over time horizon|
|Cost Evaluation|Distance-to-goal, turn cost, forward speed bias|
|Collision Check|Laser-based forward collision cone validation|
|Recovery Logic|Escape local minima by controlled rotate-toward-goal|
|RViz Visualization|Live display of all candidate trajectories|

-----

## Algorithm Overview

While most DWA implementations fully rely on a global planner, this controller intentionally follows a **pure local DWA navigation philosophy** suitable for semi-structured environments like construction sites:

**Drive toward goal unless obstacle avoidance is required.\
If obstacles disrupt progress, locally optimize trajectory using DWA.**

This ensures:

- High-speed direct motion when path is clear
- Controlled turning only when needed
- Smooth and stable navigation instead of oscillatory motion
- Simplicity and robustness without a global costmap
-----
### Why Not Use Nav2?

This was a deliberate design choice:

-  Demonstrate understanding of core DWA mechanics
-  Prove ability to build a planner without pre-built stacks
-  Focus on dynamic feasibility, collision reasoning, and recovery
-  Suitable for cluttered job-site-like environments
-  Allows direct control and interpretation of trajectory scoring

In real robots, this architecture is ideal for:

- Relatively open, dynamic environments
- Construction-like navigation (dynamic workers/equipment)
- Robots without global maps or when map is unreliable
-----
### Cost Function Summary

The planner evaluates each candidate trajectory with:

|**Term**|**Meaning**|
| :- | :- |
|Distance to goal|Prioritizes forward progress|
|Turn penalty|Prefers straight or gentle curvature|
|Speed bonus|Encourages efficient forward motion|
|Safety check|Discards trajectories intersecting obstacles|

Collision avoidance is **strict, not probabilistic** — if a trajectory violates a safety envelope, it is rejected.

This ensures **smooth, safe, and directed navigation**.

-----
### Recovery Behavior

When progress stalls (local minima):

- Track distance-to-goal improvement
- Trigger rotational recovery aligned to goal heading
- Reset and resume DWA sampling

This prevents oscillation and stuck states without global planning.

-----
### Result

This planner reliably reaches goal positions in simulation while avoiding obstacles using:

- Selective straight-line bias
- Adaptive velocity control based on obstacle clearance
- Dynamic trajectory rollout
- Safety-gated sampling
- Minimal but effective recovery logic

Not just “move and avoid” — **predict, evaluate, choose, commit**.

-----
### Future Extensions (Open-Ended)

- Add clearance cost term to scoring (non-binary)
- Fuse simple inflation layer for smoother obstacle behavior
- Optional global waypoint assist for maze-like layouts

### File Structure

```text

custom_dwa_local_planner/
├── custom_dwa_local_planner/
│   └── dwa_planner.py    # Main DWA planner implementation
├── package.xml              # ROS2 package definition
├── setup.py                 # Python package setup
└── README.md               # This file
```
