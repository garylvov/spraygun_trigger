cmake_minimum_required(VERSION 3.0.2)
project(spraygun_trigger)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  interbotix_xs_msgs
  message_generation
  roscpp
)

add_service_files(
  FILES
  unsqueeze.srv 
  squeeze.srv
  partial_squeeze.srv 
  timed_squeeze.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(trigger_control_node src/trigger_control.cpp)
target_link_libraries(trigger_control_node ${catkin_LIBRARIES})
