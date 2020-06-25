cmake_minimum_required(VERSION 2.8.3)
project(bpmsg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  genmsg
  message_generation
)

add_message_files(
  FILES
  pose.msg
  arm_state.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES probot_msgs
  
#  DEPENDS system_lib
)
include_directories(
# include

)


