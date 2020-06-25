#ifndef TEMPLATE_MATCH
#define TEMPLATE_MATCH
#include <pcl/console/time.h>
#include <std_msgs/Int8.h>
//#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/memory.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    PCLHandler;
#include <math.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

#include "bpmsg/arm_state.h"
#include "bpmsg/pose.h"

// #include "template_match/pose.h"
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;
using namespace std;

class FeatureCloud {
 public:
  // A bit of shorthand

  FeatureCloud()
      : search_method_xyz_(new SearchMethod),
        normal_radius_(0.05f),
        feature_radius_(0.05f) {}

  ~FeatureCloud() {}

  // Process the given cloud
  void setInputCloud(PointCloud::Ptr xyz) {
    xyz_ = xyz;
    processInput();
  }

  // Load and process the cloud in the given PCD file
  void loadInputCloud(const std::string &pcd_file) {
    xyz_ = PointCloud::Ptr(new PointCloud);
    pcl::io::loadPCDFile(pcd_file, *xyz_);
    processInput();
  }

  // Get a pointer to the cloud 3D points
  PointCloud::Ptr getPointCloud() const { return (xyz_); }

  // Get a pointer to the cloud of 3D surface normals
  SurfaceNormals::Ptr getSurfaceNormals() const { return (normals_); }

  // Get a pointer to the cloud of feature descriptors
  LocalFeatures::Ptr getLocalFeatures() const { return (features_); }

 protected:
  // Compute the surface normals and local features
  void processInput() {
    computeSurfaceNormals();
    computeLocalFeatures();
  }

  // Compute the surface normals
  void computeSurfaceNormals() {
    normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setInputCloud(xyz_);
    norm_est.setSearchMethod(search_method_xyz_);
    norm_est.setRadiusSearch(normal_radius_);
    norm_est.compute(*normals_);
  }

  // Compute the local feature descriptors
  void computeLocalFeatures() {
    features_ = LocalFeatures::Ptr(new LocalFeatures);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
        fpfh_est;
    fpfh_est.setInputCloud(xyz_);
    fpfh_est.setInputNormals(normals_);
    fpfh_est.setSearchMethod(search_method_xyz_);
    fpfh_est.setRadiusSearch(feature_radius_);
    fpfh_est.compute(*features_);
  }

 private:
  // Point cloud data
  PointCloud::Ptr xyz_;
  SurfaceNormals::Ptr normals_;
  LocalFeatures::Ptr features_;
  SearchMethod::Ptr search_method_xyz_;

  // Parameters
  float normal_radius_;
  float feature_radius_;
};

class TemplateAlignment {
 public:
  // A struct for storing alignment results
  struct Result {
    float fitness_score;
    Eigen::Matrix4f final_transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment()
      : min_sample_distance_(0.002f),               // 0.005f 0.002
        max_correspondence_distance_(0.1f * 0.1f),  // 0.1f * 0.1f 0.1*0.1
        nr_iterations_(1500) {                      // 1000 2000
    // Initialize the parameters in the Sample Consensus Initial Alignment
    // (SAC-IA) algorithm
    sac_ia_.setMinSampleDistance(min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
    sac_ia_.setMaximumIterations(nr_iterations_);
  }

  ~TemplateAlignment() {}

  // Set the given cloud as the target to which the templates will be aligned
  void setTargetCloud(FeatureCloud &target_cloud) {
    target_ = target_cloud;
    sac_ia_.setInputTarget(target_cloud.getPointCloud());
    sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
  }

  // Add the given cloud to the list of template clouds
  void addTemplateCloud(FeatureCloud &template_cloud) {
    templates_.push_back(template_cloud);
  }

  // Align the given template cloud to the target specified by setTargetCloud ()
  void align(FeatureCloud &template_cloud, TemplateAlignment::Result &result) {
    sac_ia_.setInputCloud(template_cloud.getPointCloud());
    sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sac_ia_.align(registration_output);

    result.fitness_score =
        (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
    result.final_transformation = sac_ia_.getFinalTransformation();
  }

  // Align all of template clouds set by addTemplateCloud to the target
  // specified by setTargetCloud ()
  void alignAll(std::vector<TemplateAlignment::Result,
                            Eigen::aligned_allocator<Result>> &results) {
    results.resize(templates_.size());
    for (size_t i = 0; i < templates_.size(); ++i) {
      align(templates_[i], results[i]);
    }
  }

  // Align all of template clouds to the target cloud to find the one with best
  // alignment score
  int findBestAlignment(TemplateAlignment::Result &result) {
    // Align all of the templates to the target cloud
    std::vector<Result, Eigen::aligned_allocator<Result>> results;
    alignAll(results);

    // Find the template with the best (lowest) fitness score
    float lowest_score = std::numeric_limits<float>::infinity();
    int best_template = 0;
    for (size_t i = 0; i < results.size(); ++i) {
      const Result &r = results[i];
      if (r.fitness_score < lowest_score) {
        lowest_score = r.fitness_score;
        best_template = (int)i;
      }
    }

    // Output the best alignment
    result = results[best_template];
    return (best_template);
  }

 private:
  // A list of template clouds and the target to which they will be aligned
  std::vector<FeatureCloud> templates_;
  FeatureCloud target_;

  // The Sample Consensus Initial Alignment (SAC-IA) registration routine and
  // its parameters
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                       pcl::FPFHSignature33>
      sac_ia_;
  float min_sample_distance_;
  float max_correspondence_distance_;
  int nr_iterations_;
};

class template_match {
 public:
  double min_x, min_y, min_z, max_x, max_y, max_z;
  double model_size, voxel_grid_size;
  int object_index = 0;
  double theta, dx, dy, dz,side_min,side_max;
  int arm_state;
  int max_num, min_num;
  int object_num = 0;
  int com_flag = 0;
  int pick_index;
  bool view_on;
  bool simulation, vision_simulation;
  clock_t start, end;
  string model_file_, model_file_2, model_file_1;
  Eigen::Vector3f euler_angles;
  Eigen::Matrix<float, 4, 1> origin_pos;
  std::vector<Eigen::Matrix<float, 4, 1>> grasp_pos;
  vector<Eigen::Matrix<float, 4, 1>> grasp_projection;
  Eigen::Matrix<float, 3, 1> origin_angle;
  std::vector<Eigen::Matrix<float, 3, 1>> target_angle;
  Eigen::Matrix<float, 3, 1> target_vector;
  std::vector<Eigen::Matrix<float, 4, 1>> target_pos;
  friend class TemplateAlignment;
  friend class FeatureCloud;
  ros::Subscriber bat_sub;
  ros::Subscriber arm_sub;
  ros::Publisher trans_pub;
  bpmsg::pose result_pose;
  PointCloud::Ptr model_pipe;
  PointCloud::Ptr model_cylinder;

  // pcl::PointCloud<PointT>::Ptr cloud_;
  map<int, double> height_map;
  map<int, double> height_map_side;
  vector<pcl::PointCloud<PointT>::Ptr> goals;
  // vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> t_cloud;
  std::vector<FeatureCloud> object_templates;
  vector<Eigen::Matrix4f> final_trans;
  template_match();
  void showCloud(pcl::PointCloud<PointT>::Ptr cloud1,
                 pcl::PointCloud<PointT>::Ptr cloud2);
  void cloudCB(const sensor_msgs::PointCloud2 &input);
  void armCB(const bpmsg::arm_state &msg);
  void init();
  void mainloop();
  void match(pcl::PointCloud<pcl::PointXYZ>::Ptr goal,
             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2,
             pcl::PointCloud<pcl::PointXYZ>::Ptr model, int index);
  void cluster(pcl::PointCloud<PointT>::Ptr cloud_,
               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2);
  void commandCB(const std_msgs::Int8 &msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transclouds(
      pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, double theta, double dx,
      double dy, double dz);
  Eigen::Matrix4f icp(pcl::PointCloud<PointT>::Ptr cloud_in,
                      pcl::PointCloud<PointT>::Ptr cloud_out);
  void box(pcl::PointCloud<PointT>::Ptr cloud);
};
#endif
