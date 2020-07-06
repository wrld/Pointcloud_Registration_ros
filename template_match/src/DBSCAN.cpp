#include "DBSCAN/DBSCAN.hpp"
void showCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,
               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud1, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->addPointCloud<pcl::PointXYZ>(cloud2, "sample cloud2", v2);
  viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
  // viewer->addCoordinateSystem(1.0);

  viewer->initCameraParameters();
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  };
}
void DBSCAN::start_scan() {
  start = clock();
  select_kernel();
  find_independent();
}
void DBSCAN::select_kernel() {
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  float resolution = 0.0001f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
  if (method_ == KD_TREE) {
    kdtree.setInputCloud(cloud_);
  } else if (method_ == OCT_TREE) {
    octree.setInputCloud(cloud_);
    octree.addPointsFromInputCloud();
  }

  for (auto i = 0; i < cloud_->points.size(); i++) {
    if (method_ == KD_TREE) {
      kdtree.radiusSearch(cloud_->points[i], eps, neighbourPoints[i],
                          neighbourDistance[i]);
    } else if ((method_ == OCT_TREE)) {
      octree.radiusSearch(cloud_->points[i], eps, neighbourPoints[i],
                          neighbourDistance[i]);
    }
    if (neighbourPoints[i].size() >= MinPts) {
      cluster_type.push_back(CORE_POINT);
      core_points.push_back(i);
      //   cout << "core cluster" << neighbourPoints[i].size() << endl;
    } else if (neighbourPoints[i].size() > MinbPts) {
      cluster_type.push_back(BOUND_POINT);
      bound_points.push_back(i);
    } else {
      cluster_type.push_back(NOISE_POINT);
      // auto iter_1 = cloud_->begin() + i;
      // auto iter_2 = neighbourPoints.begin() + i;
      // auto iter_3 = neighbourDistance.begin() + i;

      // cloud_->erase(iter_1);
      // neighbourPoints.erase(iter_2);
      // neighbourDistance.erase(iter_3);
    }
  }
  cout << "core_points" << core_points.size() << endl;
  cout << "bound_points" << bound_points.size() << endl;
}
vector<int> DBSCAN::vectors_intersection(vector<int> v1, vector<int> v2) {
  vector<int> v;
  sort(v1.begin(), v1.end());
  sort(v2.begin(), v2.end());
  set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                   back_inserter(v));  //求交集
  return v;
}

//两个vector求并集
vector<int> DBSCAN::vectors_set_union(vector<int> v1, vector<int> v2) {
  vector<int> v;
  sort(v1.begin(), v1.end());
  sort(v2.begin(), v2.end());
  set_union(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));
  return v;
}

void DBSCAN::find_independent() {
  cout << "start find intersection of cluster and union them" << endl;
  for (int i = 0; i < core_points.size(); i++) {
    if (neighbourPoints[core_points[i]].size() == 0) continue;
    for (int j = 0; j < core_points.size(); j++) {
      if (j == i || neighbourPoints[core_points[j]].size() == 0) continue;
      vector<int> result;
      result = vectors_intersection(neighbourPoints[core_points[i]],
                                    neighbourPoints[core_points[j]]);
      if (result.size() > 0) {
        neighbourPoints[core_points[i]] = vectors_set_union(
            neighbourPoints[core_points[i]], neighbourPoints[core_points[j]]);
        neighbourPoints[core_points[j]].clear();
      }
    }
  }
  for (int i = 0; i < bound_points.size() && use_edge; i++) {
    int max_intersect = 0;
    int max_index = -1;
    for (int j = 0; j < core_points.size(); j++) {
      if (neighbourPoints[core_points[j]].size() == 0) continue;
      vector<int> result;
      result = vectors_intersection(neighbourPoints[bound_points[i]],
                                    neighbourPoints[core_points[j]]);
      if (result.size() > max_intersect) {
        max_intersect = result.size();
        max_index = core_points[j];
      }
    }
    if (max_index != -1) {
      neighbourPoints[max_index] = vectors_set_union(
          neighbourPoints[max_index], neighbourPoints[bound_points[i]]);
    }
  }
  vector<int> final_cluster;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (auto i = 0; i < core_points.size(); i++) {
    if (neighbourPoints[core_points[i]].size() == 0 ||
        neighbourPoints[core_points[i]].size() < 200)
      continue;
    cout << "find_cluster" << neighbourPoints[core_points[i]].size() << endl;
    auto iter_1 = cloud_->begin() + i;
    auto iter_2 = neighbourPoints.begin() + i;
    auto iter_3 = neighbourDistance.begin() + i;
    int R = rand() % 255;
    int G = rand() % 255;
    int B = rand() % 255;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (auto j = 0; j < neighbourPoints[core_points[i]].size(); j++) {
      pcl::PointXYZRGB point;
      point.r = R;
      point.g = G;
      point.b = B;
      point.x = cloud_->points[neighbourPoints[core_points[i]][j]].x;
      point.y = cloud_->points[neighbourPoints[core_points[i]][j]].y;
      point.z = cloud_->points[neighbourPoints[core_points[i]][j]].z;
      result_cloud->points.push_back(point);
      cloud_cluster->points.push_back(
          cloud_->points[neighbourPoints[core_points[i]][j]]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    this->result_cloud_.push_back(cloud_cluster);
  }

  cout << "the sum of clusters: " << this->result_cloud_.size() << endl;
  result_cloud->width = result_cloud->points.size();
  result_cloud->height = 1;
  result_cloud->is_dense = true;
  // pcl::visualization::PCLVisualizer viewer("example");
  // viewer.addPointCloud<pcl::PointXYZRGB>(result_cloud, "result");
  // while (!viewer.wasStopped()) {
  //   viewer.spinOnce();
  // }
  end = clock();
  double endtime = (double)(end - start) / CLOCKS_PER_SEC;
  cout << "Total time:" << end << "  " << start << "  " << endtime << "s"
       << endl;  // s为单位
  showCloud(result_cloud, origin_cloud_);
}
// int main() {
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
//   pcl::PointCloud<pcl::PointXYZ>);

//   pcl::io::loadPCDFile(
//       "/home/gjx/orbslam/catkin_ws/src/ZJUBinPicking/pcd_files/filter_1.pcd",
//       *cloud);
//   DBSCAN gather(cloud, 0.010f, 40);  // 0.011
//   gather.start_scan();
// }