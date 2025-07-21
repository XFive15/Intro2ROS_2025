cmake_minimum_required(VERSION 3.0.2)
project(state_machine)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)
add_message_files(
  FILES
  StateStatus.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )
  

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

include_directories(
  include/state_machine
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/safety_monitor.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(safety_monitor_node
  src/safety_monitor_node.cpp
)
target_link_libraries(safety_monitor_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS safety_monitor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
