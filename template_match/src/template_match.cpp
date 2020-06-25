#include "template_match.hpp"
int user_data;
bool cmp(const pair<int, double> &a, const pair<int, double> &b) {
  return a.second < b.second;
}
// 0 0.015 0.015
// 0.016 0.015 0.017

// 0.016 0.015 0.030
//  0 0.0148 0.029

// 0.014 0.014 0
// 0.0025 0.014 0
void viewerOneOff(pcl::visualization::PCLVisualizer &viewer, double x, double y,
                  double z, string name) {
  // viewer.setBackgroundColor(1.0, 0.5, 1.0);
  //球体坐标
  pcl::PointXYZ o;
  o.x = x;
  o.y = y;
  o.z = z;
  //添加球体
  viewer.addSphere(o, 0.001, 255, 0, 0, name, 0);
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
  ros::param::get("~max_num", max_num);
  ros::param::get("~min_num", min_num);
  ros::param::get("~view_on", view_on);
  ros::param::get("~model_file_", model_file_);
  ros::param::get("~model_file_1", model_file_1);
  ros::param::get("~model_file_2", model_file_2);
  ros::param::get("~simulation", simulation);
  ros::param::get("~vision_simulation", vision_simulation);
  ros::param::get("~voxel_grid_size", voxel_grid_size);
  ros::param::get("~side_max", side_max);
  ros::param::get("~side_min", side_min);
  if (simulation && vision_simulation) {
    bat_sub = nh.subscribe("/kinect2/hd/points", 1, &template_match::cloudCB,
                           this);  //接收点云
  } else if (simulation && !vision_simulation) {
    bat_sub = nh.subscribe("/camera/depth/points", 1, &template_match::cloudCB,
                           this);  //接收点云
  } else if (!simulation) {
    bat_sub = nh.subscribe("/kinect2/hd/points", 1, &template_match::cloudCB,
                           this);  //接收点云
  }

  arm_sub = nh.subscribe("/arm_state", 1, &template_match::armCB,
                         this);  //接收点云

  trans_pub = nh.advertise<bpmsg::pose>("/goal_translation", 30);
  this->model_cylinder = PointCloud::Ptr(new PointCloud);
  pcl::io::loadPCDFile(model_file_2, *model_cylinder);
  this->model_pipe = PointCloud::Ptr(new PointCloud);
  if (simulation) {
    pcl::io::loadPCDFile(model_file_1, *model_pipe);
  } else {
    pcl::io::loadPCDFile(model_file_, *model_pipe);
  }
  pcl::PointXYZ pipe_minpt, pipe_maxpt, cy_minpt, cy_maxpt;
  pcl::getMinMax3D(*model_pipe, pipe_minpt, pipe_maxpt);
  // pcl::getMinMax3D(*model_cylinder, cy_minpt, cy_maxpt);
  // cout << "pipe min" << pipe_minpt << endl;
  // cout << "pipe max" << pipe_maxpt << endl;
  // cout << "cylinder min" << cy_minpt << endl;
  // cout << "cylinder max" << cy_maxpt << endl;
  // 0.034 0.031 0.031
  // 0.005041 0.026258 0.030
  // 0.028836 0.029025 0
  // 0.005027 0.031329 0.005167
  // 0.029 0.031 0.062
  // 0.005164 0.029025 0.062
  // 0.005041 0.002501 0.030422
  // 0.028959 0.005002 0.030835
  if (simulation) {
    origin_pos << 0.008, 0.015, 0.016, 1;
    origin_angle << 0, 0, 1;

    Eigen::Matrix<float, 4, 1> temp;
    temp << 0.008, 0.015, 0.030, 1;
    grasp_pos.push_back(temp);
    Eigen::Matrix<float, 4, 1> temp2;
    temp2 << 0.008, 0.014, 0, 1;
    grasp_pos.push_back(temp2);
  } else {
    origin_pos << 0.0195205, 0.028629, 0.0305, 1;
    origin_angle << 0, 0, 1;

    Eigen::Matrix<float, 4, 1> temp;
    temp << 0.0169315, 0.030177, 0.0025835, 1;
    grasp_pos.push_back(temp);
    Eigen::Matrix<float, 4, 1> temp2;
    temp2 << 0.017082, 0.030, 0.062, 1;
    grasp_pos.push_back(temp2);
    Eigen::Matrix<float, 4, 1> temp3;
    temp3 << 0.017082, 0.00375, 0.03, 1;
    grasp_pos.push_back(temp3);
  }
  cout << origin_pos << endl;
  cout << origin_angle << endl;
  // box(model_cylinder);
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
  this->pick_index = msg.pick_index;
}

void template_match::cloudCB(const sensor_msgs::PointCloud2 &input) {
  final_trans.clear();
  goals.clear();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> *cloudptr = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::fromROSMsg(input, *cloudptr);  //从ROS类型消息转为PCL类型消息
  pcl::PointCloud<PointT>::Ptr mycloud(cloudptr);
  pcl::console::TicToc tt;

  // std::cout << "ReadImage...\n", tt.tic();

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
  pass.setFilterLimits(min_z, max_z);   //设置在过滤字段的范围
  pass.setFilterLimitsNegative(false);  //保留还是过滤掉范围内的点
  pass.filter(*cloud_filtered);
  // when gazebo simulation, not filter
  if (!simulation) {
    std::cout << "before: The points data:  " << cloud_filtered->points.size()
              << std::endl;
    // const float voxel_grid_size = 0.005f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(cloud_filtered);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    vox_grid.filter(*tempCloud);
    cloud_filtered = tempCloud;
    std::cout << "after: The points data:  " << cloud_filtered->points.size()
              << std::endl;
  }
  if (view_on) {
    // showCloud(cloud_filtered, mycloud);
    showCloud(cloud_filtered, mycloud);
    // pcl::io::savePCDFileASCII(
    //     "/home/gjx/orbslam/catkin_ws/src/ZJUBinPicking/pcd_files/"
    //     "gather.pcd",
    //     *cloud_filtered);
  }

  // mycloud = cloud_filtered;
  // cluster(mycloud);
  // trans_pub.publish(result_pose);

  // if arm is waiting for picking or fail in picking, the program will
  // continue detectingcom_flag
  if ((arm_state == 0 || arm_state == 2) && cloud_filtered->points.size() &&
      !com_flag) {
    std::cout << "The points data:  " << cloud_filtered->points.size()
              << std::endl;
    // showCloud(cloud_filtered, mycloud);

    trans_pub.publish(result_pose);
    cluster(cloud_filtered, mycloud);
    // match(cloud_filtered, mycloud);
  }
  if (vision_simulation) {
    std::cout << "Simulation!!!!The points data:  "
              << cloud_filtered->points.size() << std::endl;
    arm_state = 0;
    cluster(cloud_filtered, mycloud);
  }
  // if arm has already picked the object, the program will stop detect, and
  // restart for next detection
  // else if (arm_state == 1 && com_flag) {
  //   result_pose.if_detect = 0;
  //   result_pose.object_num = 0;
  //   trans_pub.publish(result_pose);
  // }
}

void template_match::cluster(pcl::PointCloud<PointT>::Ptr cloud_,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2) {
  if (arm_state == 0 || arm_state == 2) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(model_size);  // 2cm
    ec.setMinClusterSize(min_num);
    ec.setMaxClusterSize(max_num);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_);
    ec.extract(cluster_indices);
    // showCloud(cloud,cloud_filtered);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it =
             cluster_indices.begin();
         it != cluster_indices.end() && arm_state != 1; ++it) {
      start = clock();
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
      pcl::PointXYZ min, max, cy_minpt, cy_maxpt;
      pcl::getMinMax3D(*cloud_cluster, min, max);
      float temp = pow((min.x - max.x), 2) + pow((min.y - max.y), 2) +
                   pow((min.z - max.z), 2);
      cout << temp << endl;
      // if (temp > 0.001825)
      // box(cloud_cluster);
      match(cloud_cluster, cloud_2, model_pipe, j);
      // else
      //   match(cloud_cluster, cloud_2, model_cylinder, j);
      if (target_pos[j](0) <= side_min || target_pos[j](0) >= side_max) {
        height_map_side.insert(make_pair(j, target_pos[j](2)));
        ROS_ERROR("side object!!!!!");
      } else {
        height_map.insert(make_pair(j, target_pos[j](2)));
      }
      j++;
    }
    cout << goals.size() << endl;
    object_num = goals.size();
    com_flag = 1;
    /*
     for (int i = 0; i < 4; i++) {
       for (int j = 0; j < 4; j++) {
         result_pose.translation[i * 4 + j] = final_trans.back()(i, j);
       }
     }
     for (int i = 0; i < 3; i++) {
       result_pose.target_pos[i] = this->target_pos(i, 0);
       // result_pose.target_angle[2 - i] = this->euler_angles.transpose()(0,
     i);
     }
     result_pose.object_num = goals.size();
     if (goals.size())
       result_pose.if_detect = result_pose.DETECTSUCCESS;
     else
       result_pose.if_detect = result_pose.NOTHING;
     trans_pub.publish(result_pose);*/
  }
}

void template_match::match(pcl::PointCloud<pcl::PointXYZ>::Ptr goal,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr model,
                           int index) {
  if (arm_state == 0 || arm_state == 2) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    cloud = goal;
    // for (int i = 0; i < goals.size(); i++) {
    FeatureCloud template_cloud;
    template_cloud.setInputCloud(model);
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

    pcl::PointCloud<pcl::PointXYZ> *transformed_cloud =
        new pcl::PointCloud<pcl::PointXYZ>;
    pcl::transformPointCloud(*best_template.getPointCloud(), *transformed_cloud,
                             best_alignment.final_transformation);
    pcl::PointCloud<pcl::PointXYZ>::Ptr f_cloud(transformed_cloud);

    final_trans.push_back(icp(f_cloud, cloud) *
                          best_alignment.final_transformation);

    end = clock();
    double endtime = (double)(end - start) / CLOCKS_PER_SEC;
    cout << "Total time:" << end << "  " << start << "  " << endtime << "s"
         << endl;  // s为单位
    // cout << "Total time:" << endtime * 1000 << "ms" << endl;  // ms为单位

    pcl::PointCloud<pcl::PointXYZ> *transformed_cloud2 =
        new pcl::PointCloud<pcl::PointXYZ>;
    pcl::transformPointCloud(*best_template.getPointCloud(),
                             *transformed_cloud2, final_trans.back());

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = final_trans.back().block<3, 3>(0, 0);
    Eigen::Vector3f translation = final_trans.back().block<3, 1>(0, 3);
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
    target_vector = rotation * origin_angle;
    // if (index >= object_num) {
    Eigen::Matrix<float, 3, 1> target_angle_;

    Eigen::Matrix<float, 4, 1> target_pos_;
    target_pos_ = final_trans.back() * origin_pos;
    target_pos.push_back(target_pos_);
    cout << "target" << target_pos.back() << endl;

    // result_pose.target_angle[0] = atan(target_vector(1) /
    // target_vector(0)); result_pose.target_angle[1] = atan(target_vector(0)
    // / target_vector(1)); result_pose.target_angle[2] =
    //     atan(sqrt(pow(target_vector(0), 2) + pow(target_vector(1), 2)) /
    //          target_vector(2));
    for (int i = 0; i < grasp_pos.size(); i++) {
      grasp_projection.push_back(final_trans.back() * grasp_pos[i]);
    }
    target_angle_ = Eigen::Matrix<float, 3, 1>(
        atan(target_vector(1) / target_vector(0)),
        // atan(target_vector(2) / target_vector(0)),
        atan((grasp_projection.back()(1, 0) - target_pos.back()(1, 0)) /
             (grasp_projection.back()(0, 0) - target_pos.back()(0, 0))),
        atan(sqrt(pow(target_vector(0), 2) + pow(target_vector(1), 2)) /
             target_vector(2)));
    target_angle.push_back(target_angle_);
    cout << "angle2x angle2y angle2z" << target_angle.back() << endl;
    grasp_projection.clear();
    // } else {
    //   target_pos[index] = final_trans.back() * origin_pos;
    //   cout << "target" << target_pos[index] << endl;

    //   // result_pose.target_angle[0] = atan(target_vector(1) /
    //   // target_vector(0)); result_pose.target_angle[1] =
    //   atan(target_vector(0)
    //   // / target_vector(1)); result_pose.target_angle[2] =
    //   //     atan(sqrt(pow(target_vector(0), 2) + pow(target_vector(1), 2))
    //   /
    //   //          target_vector(2));
    //   target_angle[index] = Eigen::Matrix<float, 3, 1>(
    //       atan(target_vector(1) / target_vector(0)),
    //       atan(target_vector(0) / target_vector(1)),
    //       atan(sqrt(pow(target_vector(0), 2) + pow(target_vector(1), 2)) /
    //            target_vector(2)));
    //   cout << "angle2x angle2y angle2z" << target_angle[index] << endl;
    // }

    // euler_angles = rotation.eulerAngles(2, 1, 0);
    // cout << "yaw(Z) pitch(Y) roll(X)=\n"
    //      << euler_angles.transpose() << endl
    //      << endl;
    /*
        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 4; j++) {
            result_pose.translation[i * 4 + j] = final_trans.back()(i, j);
          }
        }
        for (int i = 0; i < 3; i++) {
          result_pose.target_pos[i] = this->target_pos(i, 0);
          // result_pose.target_angle[2 - i] =
       this->euler_angles.transpose()(0, i);
        }
        result_pose.object_num = goals.size();
        if (goals.size())
          result_pose.if_detect = result_pose.DETECTSUCCESS;
        else
          result_pose.if_detect = result_pose.NOTHING;
        trans_pub.publish(result_pose);*/
    // trans_pub.publish(result_pose);
    // pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);

    // showCloud(PointCloud::Ptr(&transformed_cloud), model_);
    if (view_on) {
      pcl::visualization::PCLVisualizer viewer("example");
      // // 设置坐标系系统
      // viewer.addCoordinateSystem(0.5, "cloud", 0);
      // // 设置背景色
      // viewer.setBackgroundColor(0.05, 0.05, 0.05,
      //                           0);  // Setting background to a dark grey

      int v1(0);
      viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
      viewer.setBackgroundColor(0, 0, 0, v1);
      // viewer.addPointCloud<pcl::PointXYZ>(cloud1, "sample cloud1", v1);

      viewerOneOff(viewer, this->target_pos[index](0, 0),
                   this->target_pos[index](1, 0), this->target_pos[index](2, 0),
                   "origin");

      for (int i = 0; i < grasp_pos.size(); i++) {
        grasp_projection.push_back(final_trans.back() * grasp_pos[i]);

        viewerOneOff(viewer, grasp_projection[i](0, 0),
                     grasp_projection[i](1, 0), grasp_projection[i](2, 0),
                     "gasp" + i);
      }
      // 1. 旋转后的点云rotated --------------------------------
      pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(transformed_cloud2);
      PCLHandler transformed_cloud_handler(t_cloud, 255, 255, 255);
      viewer.addPointCloud(t_cloud, transformed_cloud_handler,
                           "transformed_cloud", v1);
      // 设置渲染属性（点大小）
      viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud",
          v1);

      // 2. 目标点云target --------------------------------
      PCLHandler target_cloud_handler(cloud, 255, 100, 100);
      viewer.addPointCloud(cloud, target_cloud_handler, "target_cloud", v1);
      // 设置渲染属性（点大小）
      viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud", v1);

      // // 3. 模板点云template --------------------------------
      PCLHandler template_cloud_handler(f_cloud, 100, 255, 255);
      viewer.addPointCloud(f_cloud, template_cloud_handler, "template_cloud",
                           v1);
      // 设置渲染属性（点大小）
      viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "template_cloud",
          v1);
      int v2(0);
      viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
      viewer.addPointCloud<pcl::PointXYZ>(cloud_2, "samplecloud2", v2);
      viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
      viewer.addCoordinateSystem(1.0);

      while (!viewer.wasStopped()) {  // Display the visualiser until 'q'key is
        // pressed
        viewer.spinOnce();
      }
    }
  }
}

Eigen::Matrix4f template_match::icp(pcl::PointCloud<PointT>::Ptr cloud_in,
                                    pcl::PointCloud<PointT>::Ptr cloud_out) {
  double dist = 0.05;
  double rans = 0.05;
  int iter = 50;

  bool nonLinear = false;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(iter);
  icp.setMaxCorrespondenceDistance(dist);
  icp.setRANSACOutlierRejectionThreshold(rans);
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::cout << "has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
  // std::cout << icp.getFinalTransformation() << std::endl;
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
  //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
  //     "transformed_cloud");

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
  // transformed_cloud->resize(model_size);
  showCloud(source_cloud, transformed_cloud);
}

void template_match::mainloop() {
  // ros::Rate loop_rate_class(10);
  while (1) {
    // cout << "666" << endl;
    ros::spinOnce();
    if (com_flag) {
      vector<pair<int, double>> vec(height_map.begin(), height_map.end());
      vector<pair<int, double>> vec2(height_map_side.begin(),
                                     height_map_side.end());
      //对线性的vector进行排序
      sort(vec.begin(), vec.end(), cmp);
      sort(vec2.begin(), vec2.end(), cmp);
      vec.insert(vec.end(), vec2.begin(), vec2.end());
      for (int j = 0; j < vec.size(); ++j) {
        // cout << vec[j].first << "  " << vec[j].second << endl;
        ROS_WARN("vec index: %d vec height: %f", vec[j].first, vec[j].second);
        for (int i = 0; i < 3; i++) {
          result_pose.target_pos[i] = target_pos[vec[j].first](i);
          result_pose.target_angle[i] = target_angle[vec[j].first](i);
        }
        result_pose.object_num = object_num;
        result_pose.target_index = vec[j].first;
        if (object_num)
          result_pose.if_detect = result_pose.DETECTSUCCESS;
        else
          result_pose.if_detect = result_pose.NOTHING;

        while (this->pick_index != vec[j].first) {
          ros::spinOnce();
          trans_pub.publish(result_pose);
        }
      }
      height_map.clear();
      height_map_side.clear();
      target_angle.clear();
      target_pos.clear();
      com_flag = 0;
    }
    if (arm_state == 1) {
      result_pose.if_detect = 0;
      result_pose.object_num = 0;
    }
    trans_pub.publish(result_pose);
    // transclouds(this->model_, this->theta, this->dx, this->dy, this->dz);
    // loop_rate_class.sleep();
  }
}
void template_match::box(pcl::PointCloud<PointT>::Ptr cloud) {
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  std::vector<float> moment_of_inertia;
  std::vector<float> eccentricity;
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  // 获取惯性矩
  feature_extractor.getMomentOfInertia(moment_of_inertia);
  // 获取离心率
  feature_extractor.getEccentricity(eccentricity);
  // 获取AABB盒子
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  // 获取OBB盒子
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
                           rotational_matrix_OBB);
  feature_extractor.getEigenValues(major_value, middle_value, minor_value);
  // 获取主轴major_vector，中轴middle_vector，辅助轴minor_vector
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  // 获取质心
  feature_extractor.getMassCenter(mass_center);

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  // 添加AABB包容盒
  viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y,
                  max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0,
                  1.0, 0.0, "AABB");
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

  // 添加OBB包容盒
  Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat(rotational_matrix_OBB);
  // position：中心位置
  // quat：旋转矩阵
  // max_point_OBB.x - min_point_OBB.x  宽度
  // max_point_OBB.y - min_point_OBB.y  高度
  // max_point_OBB.z - min_point_OBB.z  深度
  viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x,
                  max_point_OBB.y - min_point_OBB.y,
                  max_point_OBB.z - min_point_OBB.z, "OBB");
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

  pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
  pcl::PointXYZ x_axis(major_vector(0) + mass_center(0),
                       major_vector(1) + mass_center(1),
                       major_vector(2) + mass_center(2));
  pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0),
                       middle_vector(1) + mass_center(1),
                       middle_vector(2) + mass_center(2));
  pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0),
                       minor_vector(1) + mass_center(1),
                       minor_vector(2) + mass_center(2));
  viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
  viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
  viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  };
}