# Pointcloud_Registration_ros
## Introduction
A ROS package for pointcloud filtering, segmentation(cluster extraction), coarse registration and ICP registration, used in bin picking question.
## Procedure
- First, read the model file, and subscirbe the /camera/depth/points topic, convert ros topic of pointclouds to pcl format.
- Second, filter the pointclouds, save the clouds of target objects.
- Third, doing the cluster extraction for all the targets.
- Then do the coarse registration for each object, using FPFH and SCA-IA.
- Based on the result of coarse registration, doing the icp registration for each object, then get the results of pose.
## Environment
- ubuntu 16.04
- ROS kinetic
- pcl
## Usage
- First, set up a ROS workspace, and put the package in its /src folder.
- Second, make it.
``` shell
catkin_make
```
- Third, roslaunch it
``` shell
roslaunch template_match template_match.launch 
```
- Forth, publish rostopic for it
``` shell
rostopic pub /arm_state bpmsg/arm_state "pick_state: 0"
```
- Then, you will see the visual interface, and press 'q' to quit it.