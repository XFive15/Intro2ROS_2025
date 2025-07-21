
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
                 ros-noetic-move-base-msgs
                 ros-noetic-tf2-sensor-msgs
                 ros-noetic-octomap-rviz-plugins
                 ntpdate
sudo ntpdate time.windows.com

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

##  Known Issues and Fixes

During simulation, a few rare but observed issues may occur due to a combination of simulation timing, Unity environment imperfections, and hardware performance. These are not common and do not impact the majority of runs. We summarize the known issues below, along with potential causes and suggested workarounds:

1. Vehicle fails to stop or reaccelerate at the second red light
Symptom: The car either doesn't stop at the second traffic light, or stops but never resumes driving.

Possible Cause: Delay in traffic light detection or message reception due to image recognition latency or Unity's rendering lag.

Solution:

The detection zone has been tuned to reduce checking during turning.

If the car remains stuck, manually restart the simulation.

In some edge cases, pressing the forward key once can “unstick” the FSM.

Ensure that your PC meets minimum requirements; hardware with limited GPU or CPU can cause delays.

2. Random car freeze when stopping or starting
Symptom: After a stop, the car fails to move again.

Possible Cause: The FSM may be stuck waiting for a green light signal that was missed due to Unity delay.

Solution:

This usually self-resolves in a few seconds.

Restarting the Unity simulator typically clears the issue.

Adjust NTP time sync to reduce message timestamp mismatch (sudo ntpdate time.windows.com).

3. Unity rendering is occasionally laggy or stuck at turns
Symptom: Turning areas (especially sharp turns) show more rendering lag or freezing.

Possible Cause: Unity world not fully optimized; certain map regions trigger higher processing loads.

Solution:

This does not impact logic but may delay detection.

No action needed unless the car remains stuck; restart Unity if needed.

4. Visual Perception Delay or Misidentification
Symptom: Traffic light detection fails under certain angles or when approaching too fast.

Possible Cause: The camera perspective or image delay makes detection near red-light zone inaccurate.

Solution:

A detection buffer region was added.

If this fails, try slowing down approach or restarting the run.

Note: In extremely rare cases, the vehicle may collide with the road edge due to initialization delays or minor simulation timing issues. This behavior is highly uncommon — in the vast majority of runs, the system operates reliably without issues. If such a case occurs, restarting the project will typically resolve it.

