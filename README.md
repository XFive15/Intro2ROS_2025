# Autonomous Driving Project

This project aims to navigate a self-driving car in a simulated urban environment using ROS Noetic. The system consists of perception, planning, decision-making, and control modules, enabling the car to obey traffic lights and avoid obstacles.

## Project Overview

The simulation handles various urban driving scenarios:
- Obstacle and traffic light detection using perception modules.
- Global and local path planning with ROS navigation stack.
- Finite State Machine for decision making.
- Real-time control with velocity command translation.
- Unity-based simulation providing semantic data and environment.

## Installation

### Prerequisites

Ensure the following are installed:
- [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- NTP tool: `ntpdate` (used for time synchronization)

### Steps

1. Install git:
   ```bash
   sudo apt install git
   ```

2. Clone the repository:
   ```bash
   git clone https://github.com/XFive15/Intro2ROS_2025.git
   cd Intro2ROS_2025
   ```

3. Create your catkin workspace and link packages:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ln -s /path/to/Intro2ROS_2025/src/* .
   cd ..
   catkin_make
   ```

4. Install required ROS packages:
   ```bash
   sudo apt update
   sudo apt install ros-noetic-map-server \
                    ros-noetic-move-base \
                    ros-noetic-teb-local-planner \
                    ros-noetic-tf2-ros \
                    ros-noetic-navigation \
                    ros-noetic-rviz \
                    ros-noetic-move-base-msgs \
                    ros-noetic-tf2-sensor-msgs \
                    ros-noetic-octomap-rviz-plugins \
                    ntpdate
   sudo ntpdate time.windows.com
   ```

5. Build and source workspace:
   ```bash
   catkin_make
   source devel/setup.bash
   ```

6. Launch the simulation:
   ```bash
   roslaunch simulation simulation_demo.launch
   ```

## System Modules

| Module       | Description                                  |
|--------------|----------------------------------------------|
| Perception   | Detects obstacles and traffic lights         |
| Planning     | Provides goal points and odometry            |
| FSM          | Safety decisions based on perception and map |
| Controller   | Translates velocity into actuation commands  |
| Simulation   | Provides sensor and pose data                |

## Visualization

- Launch RViz with predefined configurations.
- Use `rqt_graph` to inspect ROS node/topic structure.

## Known Issues

The following issues are rare and typically hardware- or rendering-related:

1. **Second traffic light malfunction**
   - *Symptom:* Vehicle fails to stop or resume after red light.
   - *Cause:* Delay in detection or Unity render lag.
   - *Fix:* Restart simulation; press forward key once to resume FSM.

2. **Random car freeze**
   - *Symptom:* Car gets stuck after stopping.
   - *Cause:* FSM missed green light trigger.
   - *Fix:* Wait a few seconds or restart Unity.

3. **Unity rendering lag at turns**
   - *Symptom:* Lag during sharp turns.
   - *Cause:* High GPU load at certain map areas.
   - *Fix:* No action needed unless freeze persists.

4. **Visual perception misidentification**
   - *Symptom:* Traffic light missed under certain view angles.
   - *Cause:* Angle-related recognition error or camera delay.
   - *Fix:* Buffer region added; reduce speed or restart if needed.

> **Note:** In extremely rare cases, the car may collide with the road edge due to initialization lag. Restarting the simulator generally resolves this.