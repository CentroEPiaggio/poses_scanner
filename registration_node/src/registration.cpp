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
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
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
#include <boost/algorithm/string/split.hpp>
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

    PC::Ptr concatenated_original;
    PC::Ptr concatenated_registered;

    pcl::registration::LUM<PT> lum;
    bool initialized;
};

register_poses::register_poses()
{
  PC a,b;
  concatenated_original= a.makeShared();
  concatenated_registered = b.makeShared();
  nh = ros::NodeHandle("registration_node");
  srv_set = nh.advertiseService("set_poses", &register_poses::setPoses, this);
  srv_execute = nh.advertiseService("exec_registration", &register_poses::execute, this);
  initialized = false;
}

bool register_poses::setPoses (registration_node::set_poses::Request& req, registration_node::set_poses::Response& res)
{
  original_poses.clear();
  path directory (req.directory);
  if ( !exists(directory) && !is_directory(directory) )
  {
    ROS_ERROR ("[Registration_Node] %s does not exists or it is not a directory! Aborting...\n", directory.string().c_str());
    return false;
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
          std::vector<std::string> va,vb;
          boost::split(va, a.stem().string(), boost::is_any_of("_"), boost::token_compress_on);
          boost::split(vb, b.stem().string(), boost::is_any_of("_"), boost::token_compress_on);
          int lat_a, lat_b, lon_a, lon_b;
          lat_a = std::stoi(va.at(va.size()-2));
          lat_b = std::stoi(vb.at(vb.size()-2));
          lon_a = std::stoi(va.at(va.size()-1));
          lon_b = std::stoi(vb.at(vb.size()-1));
          if (lat_a < lat_b)
            return true;
          else if (lat_a > lat_b)
            return false;
          else
          {
            if (lon_a < lon_b)
              return true;
            else
              return false;
          }
        }
      });
  int i(0);
  concatenated_original->clear();
  for (std::vector<path>::const_iterator it (files.begin()); it != files.end(); ++it)
  {
    if (i==0)
    {
      if ( it->stem().string().compare(req.reference_pose) == 0)
        ROS_INFO ("[Registration_Node] Found requested reference pose in %s.", it->c_str());
      else
      {
        ROS_ERROR ("[Registration_Node] Cannot find reference pose %s in %s. Aborting...", req.reference_pose.c_str(), req.directory.c_str() );
        return false;
      }
    }
    if (is_regular_file (*it) && extension (*it) == ".pcd")
    {
      pose tmp;
      pcl::io::loadPCDFile (it->c_str(), tmp.second);
      tmp.first = *it;
      original_poses.push_back(tmp);
      *concatenated_original += tmp.second;
    }
    ++i;
  }
  pcl::registration::CorrespondenceEstimation<PT,PT> corr;
  i=0;
  for (std::vector<pose>::const_iterator it(original_poses.begin()); it !=original_poses.end(); ++it, ++i)
  {   
    lum.addPointCloud(it->second.makeShared());
    if (it == original_poses.begin())
    {//skip first cloud
      continue;
    }
    //save correspondences from i-th cloud to first one
    corr.setInputTarget(original_poses.begin()->second.makeShared());
    corr.setInputSource(it->second.makeShared());
    pcl::CorrespondencesPtr i_to_zero (new pcl::Correspondences);
    corr.determineCorrespondences(*i_to_zero, 0.05);
    lum.setCorrespondences(i, 0, i_to_zero);
    //save correspondences from frist cloud to i-th cloud
    corr.setInputTarget(it->second.makeShared());
    corr.setInputSource(original_poses.begin()->second.makeShared());
    pcl::CorrespondencesPtr zero_to_i (new pcl::Correspondences);
    corr.determineCorrespondences(*zero_to_i, 0.05);
    lum.setCorrespondences(0, i, zero_to_i);
    //save correspondences to and from i-th cloud and (i-1)-th cloud
    corr.setInputTarget(it->second.makeShared());
    corr.setInputSource((it-1)->second.makeShared());
    pcl::CorrespondencesPtr precedent_to_actual (new pcl::Correspondences);
    corr.determineCorrespondences(*precedent_to_actual, 0.05);
    lum.setCorrespondences(i-1, i, precedent_to_actual);
    
    corr.setInputTarget((it-1)->second.makeShared());
    corr.setInputSource(it->second.makeShared());
    pcl::CorrespondencesPtr actual_to_precedent (new pcl::Correspondences);
    corr.determineCorrespondences(*actual_to_precedent, 0.05);
    lum.setCorrespondences(i, i-1, actual_to_precedent);
  }
  lum.setMaxIterations (500);
  lum.setConvergenceThreshold (0.0);
  ROS_INFO("[Registration_Node] %d poses loaded from %s.", (int)original_poses.size(), req.directory.c_str());
  initialized = true;
  return true;
}

bool register_poses::execute(registration_node::execute::Request& req, registration_node::execute::Response& res)
{
  if (!initialized)
  {
    ROS_ERROR("[Registration_Node] No poses initialized for registration, call service `set_poses` first!!");
    return false;
  }
  ROS_WARN("[Registration_Node] Starting Registration process, PLEASE NOTE THAT IT COULD TAKE A LONG TIME...");
  lum.compute(); //perform registration
  pcl::VoxelGrid<PT> vg;
  *concatenated_registered = *lum.getConcatenatedCloud();
  //ICP
  pcl::IterativeClosestPoint<PT,PT> icp;
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-5);
  icp.setMaximumIterations(1000);
  icp.setUseReciprocalCorrespondences(true);
  icp.setInputTarget(original_poses[0].second.makeShared());
  registered_poses.push_back(original_poses[0]);
  int i=1;
  for (std::vector<pose>::iterator it(original_poses.begin()+1); it!=original_poses.end(); ++it,++i)
  {
    if (i>1)
    {
      icp.setInputTarget(registered_poses[i-1].second.makeShared());
    }
    icp.setInputSource(it->second.makeShared());
    pose reg;
    reg.first = it->first;
    std::cout<<"Registering "<<it->first.c_str()<<" ...\t ["<<i<<"/"<<original_poses.size()<<"]\r" ;
    icp.align(reg.second);
    registered_poses.push_back(reg);
  }
  std::cout<<std::endl<<std::endl;
  PC concatenated_reg_icp;
  for (int j=0; j<registered_poses.size(); ++j)
    concatenated_reg_icp += registered_poses[j].second;

  ROS_INFO("[Registration_Node] Success!!");
  std::string home (std::getenv("HOME")); 
  vg.setInputCloud(concatenated_original);
  PC tmp;
  vg.setLeafSize(0.003, 0.003, 0.003);
  vg.filter(tmp);
  pcl::copyPointCloud(tmp, *concatenated_original);
  vg.setInputCloud(concatenated_registered);
  vg.filter(tmp);
  pcl::copyPointCloud(tmp, *concatenated_registered);
  vg.setInputCloud(concatenated_reg_icp.makeShared());
  vg.filter(tmp);
  pcl::copyPointCloud(tmp, concatenated_reg_icp);
  pcl::io::savePCDFile ( (home + "/original.pcd").c_str(), *concatenated_original );
  pcl::io::savePCDFile ( (home + "/registered.pcd").c_str(), *concatenated_registered );
  pcl::io::savePCDFile ( (home + "/regicp.pcd").c_str(), concatenated_reg_icp );

  return true;
}

int main (int argc, char *argv[])
{
  ros::init (argc, argv, "registration_node");
  register_poses registrationNode;
  ros::spin();
  return 0;
}
