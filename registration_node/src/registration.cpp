// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

//ROS generated headers
#include "registration_node/set_poses.h"
#include "registration_node/execute.h"

//PCL headers
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/search/kdtree.h>
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

typedef pcl::PointXYZRGBA PT; //default point type
typedef pcl::PointCloud<PT> PC; //default point cloud
typedef std::pair<boost::filesystem::path, PC> pose;

using namespace boost::filesystem;

class register_poses
{
  public: 
    register_poses();
    ros::NodeHandle nh;
  private:
    ros::ServiceServer srv_set, srv_execute;
    bool setPoses(registration_node::set_poses::Request& req, registration_node::set_poses::Response& res);
    bool execute(registration_node::execute::Request& req, registration_node::execute::Response& res);
    
    std::vector<pose> original_poses;
    std::vector<pose> registered_poses;

    pcl::registration::LUM<PT> lum;
};

register_poses::register_poses()
{
  nh = ros::NodeHandle("registration_node");
  srv_set = nh.advertiseService("set_poses", &register_poses::setPoses, this);
  srv_execute = nh.advertiseService("exec_registration", &register_poses::execute, this);
}

bool register_poses::setPoses (registration_node::set_poses::Request& req, registration_node::set_poses::Response& res)
{
  original_poses.clear();
  path directory (req.directory);
  if ( !exists(directory) && !is_directory(directory) )
  {
    ROS_ERROR ("[Registration_Node] %s does not exists or it is not a directory! Aborting...\n", directory.string().c_str());
    return -1;
  }
  std::vector<path> files;
  copy (directory_iterator(directory), directory_iterator(), back_inserter(files));
  sort (files.begin(), files.end(),
      [&req](path const &a, path const &b)
      {
      //fancy lambda function to simultaneously sort files and put reference pose at first position
        if (a.stem().string().compare(req.reference_pose) == 0)
          return true;
        else if (b.stem().string().compare(req.reference_pose) == 0)
          return false;
        else
        {
          if (a.compare(b) > 0)
            return false;
          else
            return true;
        }
      });
  int i(0);
  for (std::vector<path>::const_iterator it (files.begin()); it != files.end(); ++it)
  {
    if (i==0)
    {
      if ( it->stem().string().compare(req.reference_pose) == 0)
        ROS_INFO ("[Registration_Node] Found requested reference pose in %s.", it->c_str());
      else
      {
        ROS_ERROR ("[Registration_Node] Cannot find reference pose %s in %s. Aborting...", req.reference_pose.c_str(), req.directory.c_str() );
        return -1;
      }
    }
    if (is_regular_file (*it) && extension (*it) == ".pcd")
    {
      pose tmp;
      pcl::io::loadPCDFile (it->c_str(), tmp.second);
      tmp.first = *it;
      original_poses.push_back(tmp);
    }
    ++i;
  }
  pcl::registration::CorrespondenceEstimation<PT,PT> corr;
  i=0;
  for (std::vector<pose>::const_iterator it(original_poses.begin()); it !=original_poses.end(); ++it, ++i)
  {   
    lum.addPointCloud(it->second.makeShared());
    if (it == original_poses.begin())
    {//add first cloud the reference
      corr.setInputTarget(it->second.makeShared());
      continue;
    }
    else
    { 
      corr.setInputSource(it->second.makeShared());
      pcl::CorrespondencesPtr i_to_zero (new pcl::Correspondences);
      corr.determineCorrespondences(*i_to_zero, 0.1);
      lum.setCorrespondences(i, 0, i_to_zero);
    }
  }
  ROS_INFO("[Registration_Node] %d poses loaded from %s.", (int)original_poses.size(), req.directory.c_str());
}

bool register_poses::execute(registration_node::execute::Request& req, registration_node::execute::Response& res)
{
}

int main (int argc, char *argv[])
{
  ros::init (argc, argv, "registration_node");
  register_poses registrationNode;
  ros::spin();
  return 0;
}
