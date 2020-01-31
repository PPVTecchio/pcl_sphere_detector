/* https://stackoverflow.com/questions/46826720/pclransac-segmentation-get-all-planes-in-cloud */

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


int main(int argc, char **argv) {
  pcl::visualization::CloudViewer viewer("viewer1");

  pcl::PCLPointCloud2::Ptr
    cloud_blob(new pcl::PCLPointCloud2),
    cloud_filtered_blob(new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_p(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read("clouds/table.pcd", *cloud_blob);

  // Create the filtering object: downsample
  // the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_blob);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: "
    << cloud_filtered->width * cloud_filtered->height
    << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = static_cast<int>(cloud_filtered->points.size());
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size() > 0.3 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    pcl::ScopeTime scopeTime("Test loop");
    {
      seg.segment(*inliers, *coefficients);
    }
    if (inliers->indices.size() == 0) {
      std::cerr <<
        "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);
    std::cerr <<
      "PointCloud representing the planar component: "
      << cloud_p->width * cloud_p->height
      << " data points." << std::endl;
  }

  viewer.showCloud(cloud_p, "viewer1");
  while (!viewer.wasStopped()) {
  }

  return (0);
}