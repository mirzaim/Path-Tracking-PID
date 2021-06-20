# PI Controller for Robot Path Tracking

This project provides a ROS-based implementation of a **PI controller** to control the movement of a robot along predefined paths such as elliptical and spiral trajectories. The robot's position and orientation are tracked, and the controller adjusts its movements to follow the desired path accurately.

## Overview
The PI controller is designed to ensure the robot can follow various paths using feedback from its current state. The project simulates the robot’s movements in the **Gazebo** simulator, and the results are visualized in **PlotJuggler**.

## Demo

Here is a demonstration of the robot following the path:

https://github.com/user-attachments/assets/1d0c0bd5-9ca6-4957-aa0c-44f48a4ae3c4

## File Structure
- **src/controller.py**: Implements the PI controller, computing velocity commands based on the robot's position and orientation.
- **src/pose_monitor.py**: Monitors the robot's pose and publishes the robot's path and orientation data. It also retrieves ground truth information from the Gazebo simulation.
- **CMakeLists.txt**: Configuration file for building the ROS package.
- **package.xml**: ROS package metadata and dependencies.

## Controller Details

The PI controller adjusts the robot's velocity and heading to minimize the error between the desired and actual paths. It consists of two main components:
- **Proportional Term (Kp)**: Corrects the error based on the distance to the target.
- **Integral Term (Ki)**: Corrects accumulated errors to improve long-term accuracy.

### Controller Parameters:
- **Elliptical Path:**
  - `Kp = 0.8`, `Ki = 0.1`, `Tp = 0.4`
- **Spiral Path:**
  - `Kp = 0.7`, `Ki = 0.1`, `Tp = 0.3`

## Simulation
The robot's movements are simulated in **Gazebo**, with trajectory data visualized using **PlotJuggler**. The blue line represents the robot's actual path, while the red line represents the desired trajectory.

## Dependencies
The project relies on the following ROS packages:
- `rospy`
- `std_msgs`

All dependencies are specified in the `package.xml` file.

## Usage

### Build the package:
1. Clone the repository:
   ```bash
   git clone https://github.com/mirzaim/Path-Tracking-PID.git
   ```
2. Navigate to the workspace and build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

### Run the package:
1. Source the workspace:
   ```bash
   source devel/setup.bash
   ```
2. Launch the controller and monitor:
   ```bash
   rosrun path_tracking controller.py
   rosrun path_tracking pose_monitor.py
   ```

### Visualize the Results:
Use **PlotJuggler** to visualize the robot’s path.
