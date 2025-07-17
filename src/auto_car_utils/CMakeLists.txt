cmake_minimum_required(VERSION 3.0.2)
project(auto_car_utils)

## Uncomment to enable C++11 if needed
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs
  actionlib_msgs
  nav_msgs
  simulation
  cv_bridge
  sensor_msgs
  tf
  tf2
)

## Find OpenCV if needed
find_package(OpenCV REQUIRED)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  include/auto_car_utils 
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ executables
add_executable(odom_pub src/odom_pub.cpp)
add_executable(goal_pub src/goal_pub.cpp)
add_executable(twist2vc src/twist2vc.cpp)

## Link executables against catkin libraries
target_link_libraries(odom_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(goal_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(twist2vc
  ${catkin_LIBRARIES}
)

