cmake_minimum_required(VERSION 2.8.3)
project(ur_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

 add_service_files(
   FILES
   RG2.srv
   RG2_Grip.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur5_control
#  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
    roscpp
    std_msgs
    message_runtime
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
#add_executable(ur_control_node src/ur_control_node.cpp src/robotarm.cpp)

#target_link_libraries(ur_control_node
#   ${catkin_LIBRARIES}
#)
