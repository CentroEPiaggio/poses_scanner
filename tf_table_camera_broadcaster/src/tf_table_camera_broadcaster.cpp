// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

//general utilities
#include <string>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

using namespace tf;

class broadcaster
{
  public:
    broadcaster();
    Transform T_7_c; //transformation from lwr_7_link to camera
    Transform T_w_t; //transformation from world to turn_table
    TransformBroadcaster br_7c, br_wt; //broadcasters
    //Node handle
    ros::NodeHandle nh;
    bool calibrate;
    boost::filesystem::path calibration_file;
};

//Constructor
broadcaster::broadcaster()
{
  nh = ros::NodeHandle("tf_table_camera_broadcaster");
  T_w_t.setOrigin ( Vector3(-0.6, -0.2, 0.13) ); //TODO tweaks
  T_w_t.setRotation ( Quaternion(0, 0, 0.707106781, 0.707106781) ); //rotation of pi/2 around z
  //check if we have already calibrated
  std::string home ( std::getenv("HOME") );
  boost::filesystem::path base_dir (home + "/PoseScanner");
  calibration_file = (base_dir.string() + "/T_7_c.transform");
  T_7_c.setIdentity();
  calibrate = false;
  if (boost::filesystem::exists(calibration_file) && boost::filesystem::is_regular_file(calibration_file) )
  {
    std::ifstream t_file (calibration_file.string().c_str());
    std::string line;
    if(t_file.is_open())
    {
      calibrate = true;
      while (getline (t_file, line))
      {
        if (line.compare(0,1,"#") == 0)
        {
          //do nothing, comment line...
          continue;
        }
        else
        {
          std::vector<std::string> vst;
          boost::trim (line); //remove white spaces from start and end
          boost::split (vst, line, boost::is_any_of(" "), boost::token_compress_on); 
          if (vst.size() == 4)
          {
            tf::Quaternion rot ( std::stof(vst[0]), std::stof(vst[1]), std::stof(vst[2]), std::stof(vst[3]) );
            T_7_c.setRotation(rot);
          }
          else if (vst.size() == 3)
          {
            tf::Vector3 trasl ( std::stof(vst[0]), std::stof(vst[1]), std::stof(vst[2]) );
            T_7_c.setOrigin(trasl);
          }
          else
          {
            ROS_ERROR("[tf_broadcaster] Incorrect calibration file, try rerunning calibration...");
            calibrate = false;
            break;
          }
        }
      }
      ROS_INFO("[tf_broadcaster] Found calibration saved on disk");
    }
    else
    {
      ROS_ERROR("[tf_broadcaster] Error reading transformation from calibration file...");
      calibrate = false;
    }
  }
  else
  {
    ROS_WARN("[tf_broadcaster] Camera is not calibrated, run calibration service, before trying to acquire poses!");
    calibrate = false;
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scene_acquirer_node");
    broadcaster b;
    ros::Rate rate(10.0);
    ROS_INFO("[tf_broadcaster] Broadcasting transforms");
    while (b.nh.ok())
    {
      if (!b.calibrate)
      {
        if (boost::filesystem::exists(b.calibration_file) && boost::filesystem::is_regular_file(b.calibration_file) )
        {
          std::ifstream t_file (b.calibration_file.string().c_str());
          std::string line;
          if(t_file.is_open())
          {
            b.calibrate = true;
            while (getline (t_file, line))
            {
              if (line.compare(0,1,"#") == 0)
              {
                //do nothing, comment line...
                continue;
              }
              else
              {
                std::vector<std::string> vst;
                boost::trim (line); //remove white spaces from start and end
                boost::split (vst, line, boost::is_any_of(" "), boost::token_compress_on); 
                if (vst.size() == 4)
                {
                  tf::Quaternion rot ( std::stof(vst[0]), std::stof(vst[1]), std::stof(vst[2]), std::stof(vst[3]) );
                  b.T_7_c.setRotation(rot);
                }
                else if (vst.size() == 3)
                {
                  tf::Vector3 trasl ( std::stof(vst[0]), std::stof(vst[1]), std::stof(vst[2]) );
                  b.T_7_c.setOrigin(trasl);
                }
                else
                {
                  ROS_ERROR("[tf_broadcaster] Incorrect calibration file, try rerunning calibration...");
                  b.calibrate = false;
                  break;
                }
              }
            }
            ROS_INFO("[tf_broadcaster] Found calibration saved on disk");
          }
          else
          {
            ROS_ERROR("[tf_broadcaster] Error reading transformation from calibration file...");
            b.calibrate = false;
          }
        }
        else
        {
          ROS_WARN("[tf_broadcaster] Camera is not calibrated, run calibration service in poses_scanner, before trying to acquire poses!");
          b.calibrate = false;
        }
      }
      b.br_7c.sendTransform(StampedTransform( b.T_7_c, ros::Time::now(), "lwr_7_link", "camera_link") );
      b.br_wt.sendTransform(StampedTransform( b.T_w_t, ros::Time::now(), "world", "rot_table" ));
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
