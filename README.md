
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
roslaunch auto_car_navigation_bringup navigation_bringup.launch
```

This will:
- Start odometry publisher
- Launch the map server
- Start `move_base` with TEB planner
- Initialize static TF transforms

---

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

##  Notes

- Make sure TF frames (`world`, `map`, `OurCar`, etc.) are consistent.
- If RViz does not show anything, check TF tree using:

```bash
rosrun tf view_frames
```
