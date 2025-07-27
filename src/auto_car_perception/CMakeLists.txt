cmake_minimum_required(VERSION 3.0.2)
project(auto_car_perception)

## Uncomment to enable C++11 if needed
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  move_base_msgs
  actionlib_msgs
  image_transport
  simulation
  message_generation
  # roscpp
  # rospy
  # std_msgs
  # simulation
  # cv_bridge
  # sensor_msgs
  # tf
  # tf2
)

## Find OpenCV if needed
find_package(OpenCV REQUIRED)
  add_message_files(
  FILES
  ObstacleState.msg
  )
  generate_messages(
  DEPENDENCIES
  std_msgs
  )
 
## Declare a catkin package
catkin_package(
CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs message_runtime)

## Specify additional locations of header files
include_directories(
  include/auto_car_perception
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ executables
add_executable(traffic src/traffic.cpp)
add_executable(obstacle_detection_node src/obstacle_detection.cpp)

## Link the executable against catkin libraries
target_link_libraries(traffic
  ${catkin_LIBRARIES}
)
target_link_libraries(obstacle_detection_node ${catkin_LIBRARIES})

