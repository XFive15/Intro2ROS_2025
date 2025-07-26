
# Autonomous Driving Project

This project aims to navigate a car through a predefined circuit as quickly as possible while staying on the road, avoiding other vehicles, and obeying traffic signals. The project utilizes the Robot Operating System (ROS) for robot control, navigation, and sensor data processing.

## Project Overview

This project involves creating a self-driving car that navigates a circuit while handling various challenges such as staying on the road, avoiding collisions, and responding to traffic signals. The key components include:
- Path planning using Dijkstra's and DWA algorithms.
- Sensor integration using point clouds and depth information.
- Real-time vehicle control and navigation using ROS nodes.

## Installation

### Prerequisites

Ensure you have the following software installed:
- [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

### Steps

1. Install \`git\`:
   ```
   sudo apt install git
   ```

2. Clone the repository:
   ```
   git clone https://gitlab.lrz.de/00000000014B8BFA/project_i2ros.git
   cd AutonomousDriving
   ```

3. Install the \`ros-noetic-navigation\` package:
   ```
   sudo apt install ros-noetic-navigation
   ```

4. Install \`rospack tools\`:
   ```
   sudo apt install rospack-tools
   ```

5. Install \`catkin tools\`:
   ```
   sudo apt-get install ros-noetic-catkin python3-catkin-tools
   ```

6. Install \`ros-noetic-octomap-server\`:
   ```
   sudo apt install ros-noetic-octomap-server
   ```

7. Install \`ros-noetic-octomap-rviz-plugins\`:
   ```
   sudo apt install ros-noetic-octomap-rviz-plugins
   ```

8. Install \`python3-rosdep\`:
   ```
   sudo apt-get install python3-rosdep
   sudo rosdep init
   rosdep update
   catkin config --extend /opt/ros/noetic
   ```

9. Clean the workspace:
   ```
   catkin clean
   ```

10. Build the workspace:
    ```
    catkin build
    ```

11. Download the "AutonomousDriving.zip" folder from [this link](https://syncandshare.lrz.de/getlink/fiLvgiTXetubiN1i4PRjuR/)

12. Unzip the file and extract the contents into \`project_i2ros-main/AutonomousDriving/devel/lib/simulation\`

13. Right-click on the \`Car_build.x86_64\` file and go into properties. From the pop-up window, click on permissions on the top right and tick the "Allow executing file as program" box under the "Execute:" section.

14. Build the workspace again:
    ```
    catkin build
    ```

15. Source the folders:
    ```
    source devel/setup.bash
    ```

16. Run the simulation:
    ```
    roslaunch simulation simulation.launch
    ```
