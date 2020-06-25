#include "template_match.hpp"
int user_data;
void viewerOneOff(pcl::visualization::PCLVisualizer &viewer) {
  viewer.setBackgroundColor(1.0, 0.5, 1.0);
  std::cout << "i only run once" << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer &viewer) {
  static unsigned count = 0;
  std::stringstream ss;
  ss << "Once per viewer loop: " << count++;
  viewer.removeShape("text", 0);
  viewer.addText(ss.str(), 200, 300, "text", 0);
  user_data++;
}

template_match::template_match() { init(); }

void template_match::init() {
  ros::NodeHandle nh;
  bat_sub = nh.subscribe("/camera/depth/points", 1, &template_match::cloudCB,
                         this);  //接收点云
  arm_sub = nh.subscribe("/arm_state", 1, &template_match::armCB,
                         this);  //接收点云

  trans_pub = nh.advertise<bpmsg::pose>("/goal_translation", 30);

  ros::param::get("~min_x", min_x);
  ros::param::get("~min_y", min_y);
  ros::param::get("~min_z", min_z);
  ros::param::get("~max_x", max_x);
  ros::param::get("~max_y", max_y);
  ros::param::get("~max_z", max_z);
  ros::param::get("~model_size", model_size);
  ros::param::get("~theta", theta);
  ros::param::get("~dx", dx);
  ros::param::get("~dy", dy);
  ros::param::get("~dz", dz);
  this->model_ = PointCloud::Ptr(new PointCloud);
  pcl::io::loadPCDFile(
      "/home/gjx/orbslam/catkin_ws/src/ZJUBinPicking/pcd_files/cylinder.pcd",
      *model_);
  // ros::spinOnce();
}

void template_match::showCloud(pcl::PointCloud<PointT>::Ptr cloud1,
                               pcl::PointCloud<PointT>::Ptr cloud2) {
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
    // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  };
}
void template_match::armCB(const bpmsg::arm_state &msg) {
  this->arm_state = msg.pick_state;
}

void template_match::cloudCB(const sensor_msgs::PointCloud2 &input) {
  final_trans.clear();
  goals.clear();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> *cloudptr = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::fromROSMsg(input, *cloudptr);  //从ROS类型消息转为PCL类型消息
  pcl::PointCloud<PointT>::Ptr mycloud(cloudptr);
  pcl::console::TicToc tt;

  std::cout << "ReadImage...\n", tt.tic();

  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> crop;

  crop.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));  //给定立体空间
  crop.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
  //数据随意给的，具体情况分析
  crop.setInputCloud(mycloud);
  crop.setKeepOrganized(true);
  crop.setUserFilterValue(0.1f);
  crop.filter(*cloud_filtered);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");  //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits(0.5, 0.52);      //设置在过滤字段的范围
  pass.setFilterLimitsNegative(false);  //保留还是过滤掉范围内的点
  pass.filter(*cloud_filtered);
  std::cout << "The points data:  " << cloud_filtered->points.size()
            << std::endl;
  // showCloud(cloud_filtered, mycloud);
  mycloud = cloud_filtered;
  // cluster(mycloud);
  // trans_pub.publish(result_pose);

  // if arm is waiting for picking or fail in picking, the program will continue
  // detecting
  if (arm_state == 0 || arm_state == 2) {
    trans_pub.publish(result_pose);
    cluster(mycloud);
  }
  // if arm has already picked the object, the program will stop detect, and
  // restart for next detection
  else if (arm_state == 1) {
    result_pose.if_detect = 0;
    result_pose.object_num = 0;
    trans_pub.publish(result_pose);
  }
}

void template_match::cluster(pcl::PointCloud<PointT>::Ptr cloud_) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02);  // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_);
  ec.extract(cluster_indices);
  // showCloud(cloud,cloud_filtered);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);

    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud_->points[*pit]);  //*
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: "
              << cloud_cluster->points.size() << " data points." << std::endl;

    std::stringstream ss;

    ss << "cloud_cluster_" << j;
    goals.push_back(cloud_cluster);
    j++;
    match(cloud_cluster);
  }
  cout << goals.size() << endl;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      result_pose.translation[i * 4 + j] = final_trans.back()(i, j);
    }
  }
  result_pose.object_num = goals.size();
  if (goals.size())
    result_pose.if_detect = result_pose.DETECTSUCCESS;
  else
    result_pose.if_detect = result_pose.NOTHING;
  trans_pub.publish(result_pose);
}

void template_match::match(pcl::PointCloud<pcl::PointXYZ>::Ptr goal) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud = goal;
  // for (int i = 0; i < goals.size(); i++) {
  FeatureCloud template_cloud;
  template_cloud.setInputCloud(model_);
  object_templates.push_back(template_cloud);
  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size(); ++i) {
    template_align.addTemplateCloud(object_templates[i]);
  }
  // cloud.push_back(goals[i]);

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud(cloud);
  template_align.setTargetCloud(target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment(best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf("Best fitness score: %f\n", best_alignment.fitness_score);

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation =
      best_alignment.final_transformation.block<3, 3>(0, 0);
  Eigen::Vector3f translation =
      best_alignment.final_transformation.block<3, 1>(0, 3);
  printf("\n");
  printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1),
         rotation(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1),
         rotation(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1),
         rotation(2, 2));
  printf("\n");
  printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1),
         translation(2));

  pcl::PointCloud<pcl::PointXYZ> *transformed_cloud =
      new pcl::PointCloud<pcl::PointXYZ>;
  pcl::transformPointCloud(*best_template.getPointCloud(), *transformed_cloud,
                           best_alignment.final_transformation);
  pcl::PointCloud<pcl::PointXYZ>::Ptr f_cloud(transformed_cloud);

  final_trans.push_back(icp(f_cloud, cloud) *
                        best_alignment.final_transformation);

  pcl::PointCloud<pcl::PointXYZ> *transformed_cloud2 =
      new pcl::PointCloud<pcl::PointXYZ>;
  pcl::transformPointCloud(*best_template.getPointCloud(), *transformed_cloud2,
                           final_trans.back());

  // trans_pub.publish(result_pose);
  // pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);

  // showCloud(PointCloud::Ptr(&transformed_cloud), model_);

  pcl::visualization::PCLVisualizer viewer("example");
  // 设置坐标系系统
  viewer.addCoordinateSystem(0.5, "cloud", 0);
  // 设置背景色
  viewer.setBackgroundColor(0.05, 0.05, 0.05,
                            0);  // Setting background to a dark grey

  // 1. 旋转后的点云rotated --------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(transformed_cloud2);
  PCLHandler transformed_cloud_handler(t_cloud, 255, 255, 255);
  viewer.addPointCloud(t_cloud, transformed_cloud_handler, "transformed_cloud");
  // 设置渲染属性（点大小）
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

  // 2. 目标点云target --------------------------------
  PCLHandler target_cloud_handler(cloud, 255, 100, 100);
  viewer.addPointCloud(cloud, target_cloud_handler, "target_cloud");
  // 设置渲染属性（点大小）
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");

  // // 3. 模板点云template --------------------------------
  PCLHandler template_cloud_handler(f_cloud, 100, 255, 255);
  viewer.addPointCloud(f_cloud, template_cloud_handler, "template_cloud");
  // 设置渲染属性（点大小）
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "template_cloud");

  while (
      !viewer
           .wasStopped()) {  // Display the visualiser until 'q' key is pressed
    viewer.spinOnce();
  }
}

Eigen::Matrix4f template_match::icp(pcl::PointCloud<PointT>::Ptr cloud_in,
                                    pcl::PointCloud<PointT>::Ptr cloud_out) {
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::cout << "has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  pcl::PointCloud<pcl::PointXYZ> *transformed_cloud =
      new pcl::PointCloud<pcl::PointXYZ>;
  pcl::transformPointCloud(*cloud_in, *transformed_cloud,
                           icp.getFinalTransformation());
  // pcl::visualization::PCLVisualizer viewer("example");
  // // 设置坐标系系统
  // viewer.addCoordinateSystem(0.5, "cloud", 0);
  // // 设置背景色
  // viewer.setBackgroundColor(0.05, 0.05, 0.05,
  //                           0);  // Setting background to a dark grey

  // // 1. 旋转后的点云rotated --------------------------------
  // pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(transformed_cloud);
  // PCLHandler transformed_cloud_handler(t_cloud, 0, 255, 255);
  // viewer.addPointCloud(t_cloud, transformed_cloud_handler,
  // "transformed_cloud");
  // // 设置渲染属性（点大小）
  // viewer.setPointCloudRenderingProperties(
  //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

  // PCLHandler target_cloud_handler(cloud_out, 255, 100, 100);
  // viewer.addPointCloud(cloud_out, target_cloud_handler, "target_cloud");
  // // 设置渲染属性（点大小）
  // viewer.setPointCloudRenderingProperties(
  //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");

  // PCLHandler init_cloud_handler(cloud_in, 255, 255, 255);
  // viewer.addPointCloud(cloud_in, init_cloud_handler, "init");
  // // 设置渲染属性（点大小）
  // viewer.setPointCloudRenderingProperties(
  //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "init");

  // while (
  //     !viewer
  //          .wasStopped()) {  // Display the visualiser until 'q' key is
  //          pressed
  //   viewer.spinOnce();
  // }
  return icp.getFinalTransformation();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr template_match::transclouds(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, double theta, double dx,
    double dy, double dz) {
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  // Define a rotation matrix (see
  // https://en.wikipedia.org/wiki/Rotation_matrix)
  // float theta = M_PI / 2;  // The angle of rotation in radians
  transform_1(0, 0) = cos(theta);
  transform_1(0, 1) = -sin(theta);
  transform_1(1, 0) = sin(theta);
  transform_1(1, 1) = cos(theta);
  //    (row, column)

  // Define a translation of 2.5 meters on the x axis.
  transform_1(0, 3) = dx;
  transform_1(1, 3) = dy;
  transform_1(2, 3) = dz;

  // Print the transformation
  printf("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;
  /*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
  */
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << dx, dy, dz;
  // The same rotation matrix as before; theta radians arround Z axis
  transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
  // Print the transformation
  printf("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);
  transformed_cloud->resize(model_size);
  showCloud(source_cloud, transformed_cloud);
}

void template_match::mainloop() {
  // ros::Rate loop_rate_class(10);
  while (1) {
    // cout << "666" << endl;
    ros::spinOnce();
    // transclouds(this->model_, this->theta, this->dx, this->dy, this->dz);
    // loop_rate_class.sleep();
  }
}