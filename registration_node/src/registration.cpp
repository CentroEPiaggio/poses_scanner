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
//#include <pcl/search/kdtree.h>
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/registration_visualizer.h>

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
    std::vector<pose> registered_poses_lum;

    PC::Ptr concatenated_original;
    PC::Ptr concatenated_registered;
    PC::Ptr concatenated_registered_lum;

    pcl::IterativeClosestPoint<PT,PT> icp;
  //  pcl::RegistrationVisualizer<PT,PT> viewer;
    pcl::registration::LUM<PT> lum;
    bool initialized;
    int i_30, i_70;
};

register_poses::register_poses()
{
  PC a,b,c;
  concatenated_original= a.makeShared();
  concatenated_registered = b.makeShared();
  concatenated_registered_lum = c.makeShared();
  nh = ros::NodeHandle("registration_node");
  srv_set = nh.advertiseService("set_poses", &register_poses::setPoses, this);
  srv_execute = nh.advertiseService("exec_registration", &register_poses::execute, this);
  initialized = false;
  //viewer.setRegistration(icp);
}

bool register_poses::setPoses (registration_node::set_poses::Request& req, registration_node::set_poses::Response& res)
{
  original_poses.clear();
  path directory (req.pose_dir);
  if ( !exists(directory) && !is_directory(directory) )
  {
    ROS_ERROR ("[Registration_Node] %s does not exists or it is not a directory! Aborting...\n", directory.string().c_str());
    return false;
  }
  std::vector<path> files;
  copy (directory_iterator(directory), directory_iterator(), back_inserter(files));
  sort (files.begin(), files.end(),
      [](path const &a, path const &b)
      {
        std::vector<std::string> va,vb;
        boost::split(va, a.stem().string(), boost::is_any_of("_"), boost::token_compress_on);
        boost::split(vb, b.stem().string(), boost::is_any_of("_"), boost::token_compress_on);
        int lat_a, lat_b, lon_a, lon_b;
        lat_a = std::stoi(va.at(va.size()-2));
        lat_b = std::stoi(vb.at(vb.size()-2));
        lon_a = std::stoi(va.at(va.size()-1));
        lon_b = std::stoi(vb.at(vb.size()-1));
        if (lat_a == 50 && lat_b != 50)
          return true;
        else if (lat_b == 50 && lat_a != 50)
          return false;
        else if (lat_a < lat_b)
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
      });
  concatenated_original->clear();
  int i(0);
  for (std::vector<path>::const_iterator it (files.begin()); it != files.end(); ++it, ++i)
  {
    std::vector<std::string> vst;
    boost::split(vst, it->stem().string(), boost::is_any_of("_"), boost::token_compress_on);
    if (std::stoi(vst.at(vst.size()-2)) == 30 && std::stoi(vst.at(vst.size()-1)) == 0)
      i_30 = i;
    if (std::stoi(vst.at(vst.size()-2)) == 70 && std::stoi(vst.at(vst.size()-1)) == 0)
      i_70 = i;

    if (is_regular_file (*it) && extension (*it) == ".pcd")
    {
      pose tmp;
      pcl::io::loadPCDFile (it->c_str(), tmp.second);
      tmp.first = *it;
      original_poses.push_back(tmp);
      *concatenated_original += tmp.second;
    }
  }
  //LUM graph
  //let's suppose poses are acquired with poses_scanner, so they have three latitudes (30,50,70) we want 50_0 to be the reference pose
  pcl::registration::CorrespondenceEstimation<PT,PT> corr;
  int j(0); //track vertices ids
  //cycle poses
  for (i=0; i < original_poses.size(); ++i)
  {   
    lum.addPointCloud(original_poses[i].second.makeShared());
    if (i==0)
      continue;
    if (i==i_30)
    {
      //save correspondences from 30_0 cloud to reference (50_0)
      corr.setInputTarget(original_poses[0].second.makeShared());
      corr.setInputSource(original_poses[i_30].second.makeShared());
      pcl::CorrespondencesPtr to_50 (new pcl::Correspondences);
      corr.determineCorrespondences(*to_50, 0.05);
      lum.setCorrespondences(i_30, 0, to_50);
      continue;
    }
    if (i==i_70)
    {
      //save correspondences from 70_0 cloud to reference (50_0)
      corr.setInputTarget(original_poses[0].second.makeShared());
      corr.setInputSource(original_poses[i_70].second.makeShared());
      pcl::CorrespondencesPtr to_50 (new pcl::Correspondences);
      corr.determineCorrespondences(*to_50, 0.05);
      lum.setCorrespondences(i_70, 0, to_50);
      continue;
    }
    //save correspondences from actual cloud to precedent
    corr.setInputTarget(original_poses[i-1].second.makeShared());
    corr.setInputSource(original_poses[i].second.makeShared());
    pcl::CorrespondencesPtr actual_to_precedent (new pcl::Correspondences);
    corr.determineCorrespondences(*actual_to_precedent, 0.05);
    lum.setCorrespondences(i, i-1, actual_to_precedent);
    if (i>i_30 && i<i_70)
    {
      //save correspondences from actual cloud to correspondent in 50
      corr.setInputTarget(original_poses[i-i_30].second.makeShared());
      corr.setInputSource(original_poses[i].second.makeShared());
      pcl::CorrespondencesPtr actual_to_corr (new pcl::Correspondences);
      corr.determineCorrespondences(*actual_to_corr, 0.05);
      lum.setCorrespondences(i, i-i_30, actual_to_corr);
    }
    if (i>i_70)
    {
      //save correspondences from actual cloud to correspondent in 50
      corr.setInputTarget(original_poses[i-i_70].second.makeShared());
      corr.setInputSource(original_poses[i].second.makeShared());
      pcl::CorrespondencesPtr actual_to_corr (new pcl::Correspondences);
      corr.determineCorrespondences(*actual_to_corr, 0.05);
      lum.setCorrespondences(i, i-i_70, actual_to_corr);
    }
    if (i==i_30-1)
    {
      //last element of 50 branch
      //save correspondences from first to last
      corr.setInputTarget(original_poses[i].second.makeShared());
      corr.setInputSource(original_poses[0].second.makeShared());
      pcl::CorrespondencesPtr first_to_last (new pcl::Correspondences);
      corr.determineCorrespondences(*first_to_last, 0.05);
      lum.setCorrespondences(0, i, first_to_last);
    }
    if (i==i_70-1)
    {
      //last element of 30 branch
      //save correspondences from first to last
      corr.setInputTarget(original_poses[i].second.makeShared());
      corr.setInputSource(original_poses[i_30].second.makeShared());
      pcl::CorrespondencesPtr first_to_last (new pcl::Correspondences);
      corr.determineCorrespondences(*first_to_last, 0.05);
      lum.setCorrespondences(i_30, i, first_to_last);
    }
    if (i==original_poses.size()-1)
    {
      //last element of 70 branch
      //save correspondences from first to last
      corr.setInputTarget(original_poses[i].second.makeShared());
      corr.setInputSource(original_poses[i_70].second.makeShared());
      pcl::CorrespondencesPtr first_to_last (new pcl::Correspondences);
      corr.determineCorrespondences(*first_to_last, 0.05);
      lum.setCorrespondences(i_70, i, first_to_last);
    }
  }
  lum.setMaxIterations (1000);
  lum.setConvergenceThreshold (0.0);
  ROS_INFO("[Registration_Node] %d poses loaded from %s.", (int)original_poses.size(), req.pose_dir.c_str());
  initialized = true;
  res.success = true;
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
  concatenated_registered->clear();
  concatenated_registered_lum->clear();
  lum.compute(); //perform registration
  *concatenated_registered_lum = *lum.getConcatenatedCloud();
  for (int n=0; n<lum.getNumVertices(); ++n)
  {
    pose tmp;
    tmp.first = original_poses[n].first;
    tmp.second = *lum.getTransformedCloud(n);
    //TODO adjust transformation stored in sensor origin
    registered_poses_lum.push_back(tmp);
  }
  //ICP
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-5);
  icp.setMaximumIterations(1000);
  icp.setUseReciprocalCorrespondences(true);
  icp.setInputTarget(original_poses[0].second.makeShared());
  registered_poses.push_back(original_poses[0]);
  // viewer.startDisplay();
  int i=1;
  for (std::vector<pose>::iterator it(original_poses.begin()+1); it!=original_poses.end(); ++it,++i)
  {
    if (i == i_30 || i == i_70)
    {
      icp.setInputTarget(registered_poses[0].second.makeShared());
    }
    else if (i>1)
    {
      icp.setInputTarget(registered_poses[i-1].second.makeShared());
    }
    icp.setInputSource(it->second.makeShared());
    pose reg;
    reg.first = it->first;
    std::cout<<"Registering "<<it->first.c_str()<<" ...\t ["<<i<<"/"<<original_poses.size()<<"]\r" ;
    icp.align(reg.second);
    //TODO save new transformation in cloud sensor origin

    registered_poses.push_back(reg);
  }
//  viewer.stopDisplay();
  std::cout<<std::endl<<std::endl;
  for (int j=0; j<registered_poses.size(); ++j)
    *concatenated_registered += registered_poses[j].second;

  ROS_INFO("[Registration_Node] Success!!");
  std::string home (std::getenv("HOME")); 
  pcl::VoxelGrid<PT> vg;
  vg.setInputCloud(concatenated_original);
  PC tmp;
  vg.setLeafSize(0.003, 0.003, 0.003);
  vg.filter(tmp);
  pcl::copyPointCloud(tmp, *concatenated_original);
  vg.setInputCloud(concatenated_registered);
  vg.filter(tmp);
  pcl::copyPointCloud(tmp, *concatenated_registered);
  vg.setInputCloud(concatenated_registered_lum);
  vg.filter(tmp);
  pcl::copyPointCloud(tmp, *concatenated_registered_lum);
  pcl::io::savePCDFile ( (home + "/original.pcd").c_str(), *concatenated_original );
  pcl::io::savePCDFile ( (home + "/icp.pcd").c_str(), *concatenated_registered );
  pcl::io::savePCDFile ( (home + "/lum.pcd").c_str(), *concatenated_registered_lum );
  res.success = true;
  return true;
}

int main (int argc, char *argv[])
{
  ros::init (argc, argv, "registration_node");
  register_poses registrationNode;
  ros::spin();
  return 0;
}
