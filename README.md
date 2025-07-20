
#  Intro2ROS_2025 Autonomous Driving Project

This is a ROS-based autonomous driving project implemented for simulation in a structured urban environment. It includes modules for perception, planning, decision-making, and vehicle control.

---

##  Installation Instructions

### 1. Clone the repository

```bash
git clone https://github.com/XFive15/Intro2ROS_2025.git
cd Intro2ROS_2025
```

### 2. Create your catkin workspace (if you haven't already)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s /path/to/Intro2ROS_2025-main/src/* .  # Link packages
cd ..
catkin_make
```

> Replace `/path/to/Intro2ROS_2025-main` with the actual directory path.

---

##  Dependencies

Make sure the following ROS packages are installed:

### ROS Packages:

Install via:

```bash
sudo apt update
sudo apt install ros-noetic-map-server \
                 ros-noetic-move-base \
                 ros-noetic-teb-local-planner \
                 ros-noetic-tf2-ros \
                 ros-noetic-navigation \
                 ros-noetic-rviz
```

---

##  Running the Project

### 1. Source your workspace

```bash
source ~/catkin_ws/devel/setup.bash
```

### 2. Launch the system

```bash
roslaunch simulation simulation_demo.launch
```

### 3. Run Runtime Nodes

After launching the system, you also need to start the key nodes manually in separate terminals:

```bash
rosrun auto_car_utils twist2vc
rosrun auto_car_perception traffic
rosrun auto_car_perception obstacle_detection_node
rosrun state_machine safety_monitor_node
rosrun auto_car_utils goal_pub

##  Behavior Overview

| Module       | Role                                     |
|--------------|------------------------------------------|
| Perception   | Detects obstacles and traffic lights     |
| Planning     | Publishes goal points and odometry       |
| FSM          | Makes safety decisions based on input    |
| Controller   | Translates velocity commands into controls |
| Simulation   | Provides semantic data (pose, sensors)   |

---

##  Visualization

- Launch with RViz using predefined config
- Use `rqt_graph` to visualize node/topic structure

---

Note: In extremely rare cases, the vehicle may collide with the road edge due to initialization delays or minor simulation timing issues. This behavior is highly uncommon — in the vast majority of runs, the system operates reliably without issues. If such a case occurs, restarting the project will typically resolve it.

