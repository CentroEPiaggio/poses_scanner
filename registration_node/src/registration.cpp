// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

//PCL headers
#include <pcl/registration/lum.h>
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <boost/filesystem.hpp>
//#include <boost/algorithm/string/split.hpp>
//#include <boost/algorithm/string/trim.hpp>

#define D2R 0.017453293 //degrees to radians conversion

int main (int argc, char *argv[])
{
  
}
