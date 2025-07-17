# Launching Command
## update 17.07
###
先给这个权限 
```bash
cd src #你的文件夹所在位置
chmod +x /simulation/unity_sim/Build_Ubuntu/AD_Sim.x86_64  
```
 (路径需要适应修改)

### 现在不需要其他launch文件，已经把所有整合到一个launch文件了
```bash
roslaunch simulation simulation_demo.launch
```

### 不再需要
roslaunch auto_car_navigation_bringup navigation_bringup.launch


```bash
rosrun auto_car_utils twist2vc
```
```bash
rosrun auto_car_perception traffic
```
```bash
rosrun auto_car_perception obstacle_detection_node 
```

```bash
rosrun state_machine safety_monitor_node 
```
```bash
rosrun auto_car_utils goal_pub
```



