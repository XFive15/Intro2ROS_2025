cmake_minimum_required(VERSION 3.0.2)
project(auto_car_navigation_bringup)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Example: declare a C++ executable (uncomment if needed)
# add_executable(goal_publisher src/goal_pub.cpp)

## Link the executable against catkin libraries (uncomment if needed)
# target_link_libraries(goal_publisher ${catkin_LIBRARIES})

## Example: install the executable (uncomment if needed)
# install(TARGETS goal_publisher
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

