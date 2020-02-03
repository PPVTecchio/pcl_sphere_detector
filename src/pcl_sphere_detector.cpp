/*
 * COPYRIGHT Pedro Paulo Ventura Tecchio 2020
 */

#include <Eigen/Dense>

// ROS specific includes
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/pcl_macros.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


ros::Publisher pubPointCloud;
ros::Publisher pubPointStamped;
ros::Publisher pubFilteredCloud;

float min_radius = 0.49;
float max_radius = 0.51;

void cloud_cb(const sensor_msgs::PointCloud2& cloud_msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
  *cloudPtr = cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloudFilteredPtr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
  voxelFilter.setInputCloud(cloudPtr);
  voxelFilter.setLeafSize(0.1, 0.1, 0.1);
  voxelFilter.filter(*cloudFilteredPtr);

  // sensor_msgs::PointCloud2 output2Msg;
  // pcl::toROSMsg(*cloudFilteredPtr, output2Msg);
  // pub2.publish(output2Msg);

  pcl::PassThrough<pcl::PointXYZ>  passThroughFilter;
  passThroughFilter.setInputCloud(cloudFilteredPtr);
  passThroughFilter.setFilterFieldName("z");
  passThroughFilter.setFilterLimits(-0.5, 2);
  passThroughFilter.filter(*cloudFilteredPtr);

  passThroughFilter.setInputCloud(cloudFilteredPtr);
  passThroughFilter.setFilterFieldName("x");
  passThroughFilter.setFilterLimits(-10, 10);
  passThroughFilter.filter(*cloudFilteredPtr);

  passThroughFilter.setInputCloud(cloudFilteredPtr);
  passThroughFilter.setFilterFieldName("y");
  passThroughFilter.setFilterLimits(-10, 10);
  passThroughFilter.filter(*cloudFilteredPtr);

  sensor_msgs::PointCloud2 outputFilteredCloudMsg;
  pcl::toROSMsg(*cloudFilteredPtr, outputFilteredCloudMsg);
  pubFilteredCloud.publish(outputFilteredCloudMsg);

  Eigen::VectorXf modelCoefficientsSphere;
  std::vector<int> inliersSphere;

  // pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
  //   modelSphere(
  //     new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloudFilteredPtr));
  // modelSphere->setRadiusLimits(0.47, 0.53);

  // pcl::RandomSampleConsensus<pcl::PointXYZ> ransacSphere(modelSphere);
  // ransacSphere.setDistanceThreshold(0.01);
  // ransacSphere.setMaxIterations(100000);
  // ransacSphere.computeModel();
  // ransacSphere.getModelCoefficients(modelCoefficientsSphere);
  // ransacSphere.getInliers(inliersSphere);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr
    tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::Normal>::Ptr
    cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloudFilteredPtr);
  ne.setKSearch(100);
  ne.compute(*cloud_normals);

  pcl::SampleConsensusModelNormalSphere<pcl::PointXYZ, pcl::Normal>::Ptr
    modelSphere(new pcl::SampleConsensusModelNormalSphere<pcl::PointXYZ,
      pcl::Normal>(cloudFilteredPtr));
  modelSphere->setInputNormals(cloud_normals);
  modelSphere->setRadiusLimits(min_radius, max_radius);

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransacSphere(modelSphere);
  ransacSphere.setDistanceThreshold(0.01);
  ransacSphere.setMaxIterations(100000);
  ransacSphere.computeModel();
  ransacSphere.getModelCoefficients(modelCoefficientsSphere);
  ransacSphere.getInliers(inliersSphere);

  // ROS_INFO_STREAM("Sphere RANSAC max it: " << ransacSphere.getMaxIterations());
  // ROS_INFO_STREAM("Sphere model coeff: " << modelCoefficientsSphere);
  // ROS_INFO_STREAM("Sphere # inliers: " << inliersSphere.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr
    outputSpherePtr(new pcl::PointCloud<pcl::PointXYZ>);



  if (modelCoefficientsSphere[3] <= max_radius &
      modelCoefficientsSphere[3] >= min_radius &
      inliersSphere.size() > 50) {

    pcl::copyPointCloud(*cloudFilteredPtr, inliersSphere, *outputSpherePtr);
    sensor_msgs::PointCloud2 outputPointCloudMsg;
    pcl::toROSMsg(*outputSpherePtr, outputPointCloudMsg);
    pubPointCloud.publish(outputPointCloudMsg);

    std_msgs::Header outputHeaderMsg;
    outputHeaderMsg.seq = cloud_msg.header.seq;
    outputHeaderMsg.stamp = ros::Time::now();
    outputHeaderMsg.frame_id = cloud_msg.header.frame_id;


    geometry_msgs::PointStamped outputPointStampedMsg;
    outputPointStampedMsg.header = outputHeaderMsg;
    outputPointStampedMsg.point.x = modelCoefficientsSphere[0];
    outputPointStampedMsg.point.y = modelCoefficientsSphere[1];
    outputPointStampedMsg.point.z = modelCoefficientsSphere[2];

    pubPointStamped.publish(outputPointStampedMsg);
  }
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "pcl_sphere_detector");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>
    ("pcl_sphere_detector/point_cloud", 1);
  pubPointStamped = nh.advertise<geometry_msgs::PointStamped>
    ("pcl_sphere_detector/sphere_center", 1);

  pubFilteredCloud = nh.advertise<sensor_msgs::PointCloud2>
    ("pcl_sphere_detector/filtered_cloud", 1);

  // Spin
  ros::spin();
}