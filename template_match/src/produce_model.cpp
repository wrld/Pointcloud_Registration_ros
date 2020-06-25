#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include <iostream>

#define pi 3.1415926
void showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addPointCloud<pcl::PointXYZ>(cloud1, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->addPointCloud<pcl::PointXYZ>(cloud2, "sample cloud2", v2);
  viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
  viewer->addCoordinateSystem(1.0);

  viewer->initCameraParameters();
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  };
}
int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud.width = 2500;  // number of points from the cylinder

  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  size_t i = 0;

  // number of points from the cylinder = number of points fron a circle plane *
  // number of planes (number of points on the Z-axis)

  for (int j = 0; j <= 10 * pi; j++)

    for (int k = 0; k <= 30; k++)

    {
      cloud.points[i].x = cosf((float)j / 5) * 0.0075 + 0.1;
      cloud.points[i].y = sinf((float)j / 5) * 0.0075 + 0.1;
      cloud.points[i].z = (float)k * 5 / 3000 + 0.1;

      i++;
    }

  // number of points from the two planes of the cylinder
  for (int l = 0; l <= 10 * pi; l++)
    for (int m = 0; m <= 10; m++) {
      cloud.points[i].x = m * 0.00075 * cosf((float)l / 5) + 0.1;
      cloud.points[i].y = m * 0.00075 * sinf((float)l / 5) + 0.1;
      cloud.points[i].z = 0.1;

      i++;
    }
  for (int l = 0; l <= 10 * pi; l++)
    for (int m = 0; m <= 10; m++) {
      cloud.points[i].x = m * 0.00075 * cosf((float)l / 5) + 0.1;
      cloud.points[i].y = m * 0.00075 * sinf((float)l / 5) + 0.1;
      cloud.points[i].z = 0.15;

      i++;
    }

  pcl::io::savePCDFileASCII(
      "/home/gjx/orbslam/catkin_ws/src/ZJUBinPicking/pcd_files/cylinder.pcd",
      cloud);  // save the data to a PCD file

  pcl::PointCloud<pcl::PointXYZ>* ptr = &cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(ptr);

  showCloud(mycloud, mycloud);

  return (0);
}