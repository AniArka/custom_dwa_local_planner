# DWA Local Planner for ROS2 Humble

A custom implementation of the Dynamic Window Approach (DWA) local planner for TurtleBot3 in ROS2 Humble. This planner enables autonomous navigation with obstacle avoidance in Gazebo simulations.

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
# Copy your DWA planner package here
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
    

## Algorithm Overview

1.  Velocity Sampling: Generates admissible velocity pairs within dynamic constraints
    
2.  Trajectory Prediction: Simulates robot motion for each velocity sample over prediction horizon
    
3.  Cost Evaluation: Scores trajectories based on:
    
    *   Distance to goal
        
    *   Obstacle clearance
        
    *   Path smoothness
        
    *   Speed efficiency
        
4.  Best Trajectory Selection: Chooses optimal velocity command with lowest cost
    
5.  Safety Checks: Ensures collision-free navigation through obstacle validation

## File Structure

```text

custom_dwa_local_planner/
├── custom_dwa_local_planner/
│   └── dwa_planner.py    # Main DWA planner implementation
├── package.xml              # ROS2 package definition
├── setup.py                 # Python package setup
└── README.md               # This file
```
