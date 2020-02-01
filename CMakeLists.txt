cmake_minimum_required(VERSION 2.8.3)
project(pcl_sphere_detector)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
)

find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_sphere_detector
 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
 DEPENDS EIGEN3 PCL
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcl_sphere_detector src/pcl_sphere_detector.cpp)
target_link_libraries(pcl_sphere_detector ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
