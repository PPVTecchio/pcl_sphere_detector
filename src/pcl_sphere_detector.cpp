/*
 * COPYRIGHT Pedro Paulo Ventura Tecchio 2020
 */

#include <Eigen/Dense>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/pcl_macros.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>




ros::Publisher pub;
ros::Publisher pub2;

// void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr& input) {
//   // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   pcl::fromROSMsg(*input, cloud);

//   pcl::ModelCoefficients coefficients;
//   pcl::PointIndices inliers;
//   // Create the segmentation object
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   // Optional
//   seg.setOptimizeCoefficients(true);
//   // Mandatory
//   seg.setModelType(pcl::SACMODEL_SPHERE);
//   seg.setMethodType(pcl::SAC_RANSAC);

//   seg.setDistanceThreshold(0.01);
//   seg.setRadiusLimits(0.4, 0.6);
//   seg.setMaxIterations(50000);

//   seg.setInputCloud(cloud.makeShared());
//   seg.segment(inliers, coefficients);



//   // Publish the model coefficients
//   pcl_msgs::ModelCoefficients ros_coefficients;
//   pcl_conversions::fromPCL(coefficients, ros_coefficients);
//   pub2.publish(ros_coefficients);
// }

// void cloud_cb(const sensor_msgs::PointCloud2& cloud_msg) {
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   pcl::fromROSMsg(cloud_msg, cloud);

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new
//     pcl::PointCloud<pcl::PointXYZ>);

//   *cloudPtr = cloud;

//   pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(
//     new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloudPtr));
//   model_s->setRadiusLimits(1.4, 1.6);

//   Eigen::VectorXf model_coefficients;
//   std::vector<int> inliers;

//   pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
//   ransac.setDistanceThreshold(0.05);
//   ransac.setMaxIterations(100000);
//   // ransac.setNumberOfThreads(2);
//   ransac.computeModel();
//   ransac.getModelCoefficients(model_coefficients);
//   ransac.getInliers(inliers);


//   ROS_INFO_STREAM("RANSAC max it: " << ransac.getMaxIterations());
//   ROS_INFO_STREAM("Sphere coeff: " << model_coefficients);
//   ROS_INFO_STREAM("# inliers: " << inliers.size());

//   pcl::PointCloud<pcl::PointXYZ>::Ptr
//     outputPtr(new pcl::PointCloud<pcl::PointXYZ>);

//   pcl::copyPointCloud(*cloudPtr, inliers, *outputPtr);


//   pcl::PointCloud<pcl::PointXYZ> output = *outputPtr;
//   // Convert to ROS data type
//   sensor_msgs::PointCloud2 output_msg;

//   pcl::toROSMsg(output, output_msg);

//   // Publish the data
//   pub.publish(output_msg);
// }



void cloud_cb(const sensor_msgs::PointCloud2& cloud_msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new
    pcl::PointCloud<pcl::PointXYZ>);

  *cloudPtr = cloud;

  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(
    new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloudPtr));
  model_s->setRadiusLimits(1.4, 1.6);

  Eigen::VectorXf model_coefficients;
  std::vector<int> inliers;

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
  ransac.setDistanceThreshold(0.05);
  ransac.setMaxIterations(100000);
  // ransac.setNumberOfThreads(2);
  ransac.computeModel();
  ransac.getModelCoefficients(model_coefficients);
  ransac.getInliers(inliers);


  ROS_INFO_STREAM("RANSAC max it: " << ransac.getMaxIterations());
  ROS_INFO_STREAM("Sphere coeff: " << model_coefficients);
  ROS_INFO_STREAM("# inliers: " << inliers.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr
    outputPtr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloudPtr, inliers, *outputPtr);


  pcl::PointCloud<pcl::PointXYZ> output = *outputPtr;
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output_msg;

  pcl::toROSMsg(output, output_msg);

  // Publish the data
  pub.publish(output_msg);
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "pcl_sphere_detector");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub2 = nh.advertise<pcl_msgs::ModelCoefficients> ("output2", 1);

  // Spin
  ros::spin();
}