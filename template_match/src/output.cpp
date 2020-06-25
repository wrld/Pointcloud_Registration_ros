#include <pcl/io/pcd_io.h>  //which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <string>
using namespace std;
main(int argc, char **argv) {
  ros::init(argc, argv, "output_clouds");
  ros::NodeHandle nh;
  string file_location;
  ros::param::get("~file_location", file_location);
  ros::Publisher pcl_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/kinect2/hd/points", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile(file_location, cloud);
  // Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output);
  output.header.frame_id =
      "odom";  // this has been done in order to be able to visualize our
               // PointCloud2 message on the RViz visualizer
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}