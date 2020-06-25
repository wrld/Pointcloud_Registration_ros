cmake_minimum_required(VERSION 2.8.3)
project(template_match)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(PCL_DIR "/usr/local/include/pcl-1.10/pcl") 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_ros
  cv_bridge
sensor_msgs
std_msgs
  message_generation
bpmsg
)

# generate_messages(
#    DEPENDENCIES
#    std_msgs
   
# )
# find_package(PCL 1.7 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
 catkin_package(
    INCLUDE_DIRS
   # include
    CATKIN_DEPENDS roscpp
                   pcl_conversions
                   pcl_ros
                   std_msgs
                  message_runtime
                 
                    # pcl_ros 
  )


# find_package(VTK REQUIRED)
# include(${VTK_USE_FILE})
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  #  ${template_match_SOURCE_DIR} include/
   
  #  ${PCL_INCLUDE_DIRS}
)

add_executable(output_clouds src/output.cpp)
target_link_libraries(output_clouds ${catkin_LIBRARIES})
# target_link_libraries (output_clouds libvtkCommonCore-8.2.so.1 libvtkRenderingOpenGL2-8.2.so.1 )
#add_dependencies(template_match ${catkin_EXPORTED_TARGETS})
add_executable(template_match src/listen.cpp src/template_match.cpp)
target_link_libraries(template_match ${catkin_LIBRARIES})
# target_link_libraries (template_match libvtkCommonCore-8.2.so.1 libvtkRenderingOpenGL2-8.2.so.1)
# target_link_libraries(template_match ${VTK_LIBRARIES} ${PCL_LIBRARIES})
catkin_package()