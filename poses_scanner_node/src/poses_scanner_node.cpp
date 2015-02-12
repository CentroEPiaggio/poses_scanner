// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL headers
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// ROS generated headers
#include "poses_scanner_node/acquire.h" 
#include "poses_scanner_node/table.h" 
#include "turn_table_interface_node/setPos.h"
#include "turn_table_interface_node/getPos.h"
#include "scene_acquirer_node/acquire_scene.h"
#include "lwr_controllers/PoseRPY.h"

//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

#define D2R 0.017453293 //degrees to radians conversion

//global stuff used by viewer and class
boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer_ (new pcl::visualization::PCLVisualizer());
Eigen::Vector3f _picked_;
bool _proceed_ (false);

//Point picking event shift+click to activate
void pickEvent (const pcl::visualization::PointPickingEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);  
  viewer->removeShape("pick"); 
  event.getPoint (_picked_[0], _picked_[1], _picked_[2]);
  pcl::PointXYZ centre;
  centre.x = _picked_[0];
  centre.y = _picked_[1];
  centre.z = _picked_[2];
  viewer->addSphere ( centre, 0.005, 0, 0.9, 0.3,"pick");  
}

//keyboard key press callback for viewer
void keyboardEvent (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "t" && event.keyDown ())
  {
  	viewer->removeShape("pick");
  	viewer->removeShape("pick_text");
    viewer->removePointCloud("pick_cloud");
  	viewer->removeShape("final_cloud");
  	viewer->removeShape("final_text");
    viewer->removeCoordinateSystem();
    viewer->removeAllShapes();
    viewer->addText("DO NOT CLOSE THE VIEWER!!\nNODE WILL NOT FUNCTION PROPERLY WITHOUT THE VIEWER", 200,200,18,250,150,150,"text");
    _proceed_ = true;
  }
}

class poseGrabber
{
  public:
    poseGrabber();
    ros::NodeHandle nh;
  private:
    ros::ServiceServer srv_acquire_, srv_table_;
    ros::Publisher pub_poses_;
    ros::Publisher pub_lwr_;
    //service callback
    bool acquirePoses(poses_scanner_node::acquire::Request& req, poses_scanner_node::acquire::Response& res);
    bool calibrate(poses_scanner_node::table::Request& req, poses_scanner_node::table::Response& res);

    //method to move turn table
    bool set_turnTable_pos(float pos);
    //method to read turn table position
    float get_turnTable_pos();
    //method to move lwr
    bool set_lwr_pose(float radius, float latitude);  
  
    //method to acquire scene from openni2
    bool acquire_scene (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired);

    //method to acquire and save table transformation
    bool acquire_table_transform (int latitude);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_;
    bool calibration_;
    boost::posix_time::ptime timestamp_;
    boost::filesystem::path work_dir_;
    boost::filesystem::path current_session_;

    Eigen::Matrix4d T_70, T_50, T_30; //transforms from camera to table
};

//Class Constructor
poseGrabber::poseGrabber()
{
  nh = ros::NodeHandle("poses_scanner_node");
  //service callbacks
  srv_acquire_ = nh.advertiseService("acquire_poses", &poseGrabber::acquirePoses, this);
  srv_table_ = nh.advertiseService("calibrate", &poseGrabber::calibrate, this);
  //advertise acquired poses
  pub_poses_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> > ("acquired_poses",1);
  //publish to lwr controller
  pub_lwr_ = nh.advertise<lwr_controllers::PoseRPY> ("/lwr/OneTaskInverseKinematics/command_configuration",1);
  pcl::PointCloud<pcl::PointXYZRGBA> a,b;
  cloud_ = a.makeShared();
  scene_ = b.makeShared();
  timestamp_ = boost::posix_time::second_clock::local_time();
  calibration_ = false;
  //check if we have already calibrated
  std::string home ( std::getenv("HOME") );
  work_dir_ = (home + "/PoseScanner");
  current_session_ = (work_dir_.string() + "/Session_" + to_simple_string(timestamp_) ); 
  boost::filesystem::path cal_file  (work_dir_.string() + "/table.transform");
  if (boost::filesystem::exists(cal_file) && boost::filesystem::is_regular_file(cal_file) )
  { 
    std::ifstream t_file (cal_file.string().c_str());
    std::string line;
    if(t_file.is_open())
    {
      ROS_INFO("[poses_scanner] Found calibration saved on disk");
      calibration_ = true;
      int tr_type (0), row(0);
      while (getline (t_file, line))
      {
        if (line.compare(0,1,"#") == 0)
        {
          //do nothing, comment line...
          continue;
        }
        else
        {
          boost::trim (line); //remove white spaces from start and end
          if (line.compare("T70:") == 0)
          {
            tr_type = 1;
            row = 0;
            continue;
          }
          else if (line.compare("T50:") == 0)
          {
            tr_type = 2;
            row = 0;
            continue;
          }
          else if (line.compare("T30:") == 0)
          {
            tr_type = 3;
            row = 0;
            continue;
          }
          std::vector<std::string> vst;
          if (tr_type != 0)
          {
            boost::split (vst, line, boost::is_any_of(" "), boost::token_compress_on); 
            if (vst.size() == 4)
            {
              if (tr_type == 1)
              {
                T_70 (row,0) = std::stof(vst[0]);
                T_70 (row,1) = std::stof(vst[1]);
                T_70 (row,2) = std::stof(vst[2]);
                T_70 (row,3) = std::stof(vst[3]);
                ++row;
              }
              if (tr_type == 2)
              {
                T_50 (row,0) = std::stof(vst[0]);
                T_50 (row,1) = std::stof(vst[1]);
                T_50 (row,2) = std::stof(vst[2]);
                T_50 (row,3) = std::stof(vst[3]);
                ++row;
              }
              if (tr_type == 3)
              {
                T_30 (row,0) = std::stof(vst[0]);
                T_30 (row,1) = std::stof(vst[1]);
                T_30 (row,2) = std::stof(vst[2]);
                T_30 (row,3) = std::stof(vst[3]);
                ++row;
              }
            }
            else
            {
              ROS_ERROR("[poses_scanner] Incorrect calibration file, try renunning calibration...");
              calibration_ = false;
              break;
            }
          }
        }
      }
    }
    else
    {
      ROS_ERROR("[poses_scanner] Cant open calibration file, try rerunning calibration...");
      calibration_ = false;
    }
  }
  else
    ROS_WARN("[poses_scanner] Calibration is not done yet, run calibration service before trying to acquire poses!");
}

//wrapper function to grab a cloud
bool poseGrabber::acquire_scene (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired)
{ 
  std::string acquire_scene_srv_name = nh.resolveName("/scene_acquirer_node/acquire_scene");
  scene_acquirer_node::acquire_scene acquire_srv;
  acquire_srv.request.save = "false";
  boost::this_thread::sleep (boost::posix_time::microseconds (300000));
  if ( !ros::service::call<scene_acquirer_node::acquire_scene>(acquire_scene_srv_name, acquire_srv))
  {
    ROS_ERROR("[posesScanner] Acquire scene service failed!");
    return false;
  }
  pcl::fromROSMsg (acquire_srv.response.cloud, *acquired);
  return true;
}

//wrapper function to move turn table
bool poseGrabber::set_turnTable_pos(float pos)
{
  std::string setPos_srv_name = nh.resolveName("/turn_table_interface_node/set_table_pos");
  turn_table_interface_node::setPos set_srv;
  set_srv.request.position = pos;
  if ( !ros::service::call<turn_table_interface_node::setPos>(setPos_srv_name, set_srv) )
  {
    ROS_ERROR("[posesScanner] setPos from turn table failed!");
    return false;
  }
  boost::this_thread::sleep (boost::posix_time::microseconds (10000));
  return true;
}

//wrapper function to move lwr
bool poseGrabber::set_lwr_pose(float radius, float latitude)
{
  lwr_controllers::PoseRPY task;
  
  //measure centre of the table in world robot frame
  task.id = 0;
  task.position.x = -0.62;
  task.position.y =  0 + (radius * cos(latitude*D2R));
  task.position.z = 0.137 + (radius * sin(latitude*D2R)); 
  task.orientation.roll = 1.57079 + (latitude*D2R); //this roll is in world frame! (it acts as a pitch for EE... 90° parallell to the ground)
  task.orientation.pitch = 0; //this pitch is in world frame! (it acts as a roll for EE... keeping it a zero)
  task.orientation.yaw = 0; //this is yaw is in world frame! (zero is looking at the window for right arm)
  
  //send the task to the robot
  pub_lwr_.publish(task); 
  return true;
}

//wrapper function to read table position
float poseGrabber::get_turnTable_pos()
{
  std::string getPos_srv_name = nh.resolveName("/turn_table_interface_node/get_table_pos");
  turn_table_interface_node::getPos get_srv;
  if ( !ros::service::call<turn_table_interface_node::getPos>(getPos_srv_name, get_srv) )
  {
    ROS_ERROR("[posesScanner] getPos from turn table failed!");
    return -1;
  }
  return get_srv.response.current_pos;
}

bool poseGrabber::acquire_table_transform (int latitude)
{
  //wait for a cloud from sensor
  if (! acquire_scene(cloud_) )
  {
    ROS_ERROR("[posesScanner] Cannot acquire a scene!");
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::copyPointCloud(*cloud_, *tmp);
  
  //find first plane
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (2000);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud(tmp);
  seg.segment(*inliers, *coeff);
  //extract plane and whats on top
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  extract.setInputCloud(tmp);
  extract.setNegative(true);
  extract.setIndices(inliers);
  extract.filter(*tmp);

  _viewer_->removeShape("text");
  _viewer_->addText("Pick the centre of the table (shift click), then press 't' when satisfied", 50,50,18,250,150,150,"pick_text");
  _viewer_->addPointCloud(tmp,"pick_cloud");
  while (!_proceed_)
  {
    _viewer_->spinOnce(100);
  }
  _viewer_->spinOnce(100);
  _proceed_ = false;
  
  pcl::SACSegmentation<pcl::PointXYZRGBA> segc;
  pcl::PointIndices::Ptr in (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coe (new pcl::ModelCoefficients);
  segc.setOptimizeCoefficients (true);
  segc.setModelType (pcl::SACMODEL_PLANE);
  segc.setMethodType (pcl::SAC_RANSAC);
  segc.setMaxIterations (2000);
  segc.setDistanceThreshold (0.02);
  segc.setInputCloud(tmp);
  segc.segment(*in, *coe);
  pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setModelCoefficients (coe);
  proj.setInputCloud(tmp);
  proj.filter(*tmp);
  //downsample
  /*
  pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
  vg.setInputCloud (tmp);
  vg.setLeafSize(0.008, 0.008, 0.008);
  vg.setDownsampleAllData(true);
  vg.filter (*tmp);
  */
  
  /*
  pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGBA>::Ptr table_model(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGBA> (tmp));
  pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (table_model); //Ransac algorithm
  ransac.setDistanceThreshold (0.08);
  ransac.setMaxIterations(2000);
  ransac.computeModel();
  Eigen::VectorXf coefficients;
  ransac.getModelCoefficients(coefficients);  
  */
  //compute another plane model to read its normal this time is the table we seek
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr table_model (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (tmp));
  pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (table_model); //Ransac algorithm
  ransac.setDistanceThreshold (0.01);
  ransac.setMaxIterations(2000);
  ransac.computeModel();
  Eigen::VectorXf coefficients;
  ransac.getModelCoefficients(coefficients);  
  /* Coefficients of model are:
   * [0] x coordinate of normal direction
   * [1] y coordinate of normal direction
   * [2] z coordinate of normal direction
   * [3] d  of equation ax+by+cz+d=0
   */
  double nx,ny,nz, cx,cy,cz; //table normal and its centre picked by user
  nx = (double)coeff->values[0];
  ny = (double)coeff->values[1];
  nz = (double)coeff->values[2];
  cx = (double)_picked_[0];
  cy = (double)_picked_[1];
  cz = (double)_picked_[2];
  
  Eigen::Vector3d new_z (nx,ny,nz); //the table normal
  Eigen::Vector3d vp (0 - cx, 0 - cy, 0 - cz); //viewpoint vector 0 (sensor origin) - table centre. It points "up" the table
  vp.normalize();
  new_z.normalize();
  if (vp.dot (new_z) < 0 ) //reorient the normal if needed
  {
    new_z *= -1; 
    ROS_INFO("Flipped Table Normal");
  }
  //check if new_z^t dot x is zero, if it is x lays on the table
  double precision = 1e-4;
  double result = (new_z.transpose() * Eigen::Vector3d::UnitX()); 
  Eigen::Matrix3d R_kt;
  if ( result > -precision && result < precision )
  {
    //we can use the actual x axis, cause it lays on the table
    Eigen::Vector3d new_y = new_z.cross(Eigen::Vector3d::UnitX());
    new_y.normalize();
    //compose the rotation matrix
    R_kt << 1, new_y[0], new_z[0],
            0, new_y[1], new_z[1],
            0, new_y[2], new_z[2];
    ROS_WARN("using old x axis, result %g",result);
  }
  else
  {
    //calculate a new x that lays in null space of new_z transpose
    Eigen::Vector3d new_x;
    //TODO
    new_x = Eigen::Vector3d::UnitX(); //tmpTODO still using oldx
    Eigen::Vector3d new_y = new_z.cross(new_x);
    new_y.normalize();
    //compose the rotation matrix
    R_kt << new_x[0], new_y[0], new_z[0],
            new_x[1], new_y[1], new_z[1],
            new_x[2], new_y[2], new_z[2];
    ROS_WARN("using new x axis, result %g",result);
  }
  Eigen::Vector3d trasl (cx, cy, cz);
  Eigen::Matrix4d T_kt;
  T_kt << R_kt(0,0), R_kt(0,1), R_kt(0,2), trasl(0),
          R_kt(1,0), R_kt(1,1), R_kt(1,2), trasl(1), 
          R_kt(2,0), R_kt(2,1), R_kt(2,2), trasl(2),
          0,         0,         0,         1;
  //create directory to store transformation in "HOME/PoseScanner"
  if (!boost::filesystem::exists(work_dir_) || !boost::filesystem::is_directory(work_dir_) )
  {
    boost::filesystem::create_directory(work_dir_);
  }
  if (latitude == 70)
  {
    T_70 = T_kt;
    //Save transform to disk
    ofstream trans_file;
    trans_file.open( (work_dir_.string() +  "/table.transform").c_str() );
    trans_file << "## Table Transforms (camera to table) saved on " << to_simple_string(timestamp_).c_str() <<std::endl;
    trans_file << "## <Matrix4d> "<<std::endl;
    trans_file <<"T70:\n"<< T_70 <<std::endl;
    trans_file.close();
  }
  else if (latitude == 50)
  {
    T_50 = T_kt; 
    //Save transform to disk
    fstream trans_file;
    trans_file.open( (work_dir_.string() +  "/table.transform").c_str(), std::fstream::out | std::fstream::app );
    trans_file <<"T50:\n"<< T_50 <<std::endl; 
    trans_file.close();
  }
  else if (latitude == 30)
  {
    T_30 = T_kt; 
    //Save transform to disk
    fstream trans_file;
    trans_file.open( (work_dir_.string() +  "/table.transform").c_str(), std::fstream::out | std::fstream::app );
    trans_file <<"T30:\n"<< T_30; 
    trans_file.close();
  }
  else
  {
    ROS_ERROR("wrong latitude passed");
    return false;
  }
  //view if transform is correct
  Eigen::Matrix4d T_inv = T_kt.inverse();
  pcl::transformPointCloud(*cloud_, *tmp, T_inv);
  _viewer_->addPointCloud(tmp, "final_cloud");
  _viewer_->addCoordinateSystem(0.2);
  _viewer_->addText("Final table transformation, press 't' to proceed.",50,50,18,250,150,150,"final_text");
  while (!_proceed_)
  {
    _viewer_->spinOnce (100);
  }
  _viewer_->spinOnce(100);
  _proceed_ = false;
  return true;
}

//service callback for calibration
bool poseGrabber::calibrate(poses_scanner_node::table::Request &req, poses_scanner_node::table::Response &res)
{
  //put lwr at first stop
  set_lwr_pose(0.85, 70);  

  // move table to 0 position, if not there already
  float c_pos = get_turnTable_pos();
  if (c_pos != 0)
  {
    float step = -c_pos/360;
    for (int i=1; i<=360; i++)
    {
      if(!set_turnTable_pos(c_pos + step*i) )
      {
        ROS_ERROR("[posesScanner] turnTable communication failed!");
        return false;
      }
    }
  }
  boost::this_thread::sleep (boost::posix_time::microseconds (18000000)); //wait for lwr  TODO add a topic to monitor if lwr has reached position
  bool cal70(false), cal50(false), cal30(false);
  cal70 = acquire_table_transform(70);
  
  //put lwr at second stop
  set_lwr_pose(0.85, 50);  
  boost::this_thread::sleep (boost::posix_time::microseconds (15000000)); //wait for lwr  TODO add a topic to monitor if lwr has reached position
  cal50 = acquire_table_transform(50);
  //last stop
  set_lwr_pose(0.85, 30);  
  boost::this_thread::sleep (boost::posix_time::microseconds (15000000)); //wait for lwr  TODO add a topic to monitor if lwr has reached position
  cal30 = acquire_table_transform(30);
  if (cal50 && cal70 && cal30)
  {
    calibration_= true;
    ROS_INFO("Calibration complete!!");
    ROS_INFO("You don't need to run this again, unless you move the table, or the camera with respect to lwr_7_link");  
    return true;
  }
  else
  {
    ROS_ERROR("Error calibrating");
    return false;
  }
}

//service callback to acquire poses
bool poseGrabber::acquirePoses(poses_scanner_node::acquire::Request &req, poses_scanner_node::acquire::Response &res)
{
  if (!calibration_)
  {
    ROS_ERROR("[poses_scanner] Calibration is not done yet! Run calibration service before trying to acquire poses! Exiting...");
    return false;
  }
  int lon_pass = req.lon_pass;
  std::string name;
  if (req.objname.compare(0,1,"-") == 0) //check if we wanted flipped object
  {
    name = req.objname.substr(1); //get all name except first character
    //TODO still need to implement this... if it is doable
  }
  else
    name = req.objname; 
  //create directory to store poses writes into "~/PoseScanner"
  if (!boost::filesystem::exists(work_dir_) || !boost::filesystem::is_directory(work_dir_) )
  {
    boost::filesystem::create_directory(work_dir_);
  }
  if (!boost::filesystem::exists(current_session_) || !boost::filesystem::is_directory(current_session_))
  {
    boost::filesystem::create_directory(current_session_);
  }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr c (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PCDWriter writer;
  _viewer_->addPointCloud(c,"pose");
  _viewer_->addCoordinateSystem(0.2);
  _viewer_->removeShape("text");
  //acquisition loops
  //for cycle in latitude 
  for (int lat = 70;  lat > 20; lat-=20) //fixed latitude pass (goes to 70, 50 and 30) 
  {
    //move lwr in position  
    set_lwr_pose(0.85,lat);
    //and table
    float c_pos = get_turnTable_pos();
    if (c_pos != 0)
    {
      float step = -c_pos/360;
      for (int i=1; i<=360; i++)
      {
        if(!set_turnTable_pos(c_pos + step*i) )
        {
          ROS_ERROR("[posesScanner] turnTable communication failed!");
          return false;
        }
      }
    }
    boost::this_thread::sleep (boost::posix_time::microseconds (20000000)); //wait for it    TODO remove and add topic
    for (int lon=0; lon<360; lon+=lon_pass)
    {//for cycle in longitude
      cloud_->clear();
      while (!(get_turnTable_pos() >= lon-1 && get_turnTable_pos() <= lon+1))
      {
        if(!set_turnTable_pos(lon) )
        {
          ROS_ERROR("[posesScanner] turnTable communication failed!");
          return false;
        }
        boost::this_thread::sleep (boost::posix_time::microseconds (50000)); //wait for table to be in position
      }
      if (! acquire_scene (cloud_) )
      {
        ROS_ERROR("[posesScanner] Cannot acquire a scene!");
        return false;
      }

      pcl::copyPointCloud(*cloud_, *scene_);
      //save scene on disk
      std::string scenename (current_session_.string() + "/SCENE_" + name + "_" + std::to_string(lat) + "_" + std::to_string(lon) + ".pcd" );
      writer.writeBinaryCompressed (scenename.c_str(), *scene_);
      
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGBA>);
      
      Eigen::Matrix4d T;
      if (lat==70)
        T = T_70.inverse();
      if (lat==50)
        T = T_50.inverse();
      if (lat==30)
        T = T_30.inverse();
        //temporary transform to local frame for easier cropping
      pcl::transformPointCloud(*cloud_, *temp, T); 

      //Cropping z
      pcl::PassThrough<pcl::PointXYZRGBA> pt;
      pt.setInputCloud (temp);
      pt.setFilterFieldName ("z");
      pt.setFilterLimits (-0.01, 0.7);
      pt.filter (*temp);
      //x
      pt.setInputCloud (temp);
      pt.setFilterFieldName ("x");
      pt.setFilterLimits (-0.25,  0.25);
      pt.filter (*temp);
      //y
      pt.setInputCloud (temp);
      pt.setFilterFieldName ("y");
      pt.setFilterLimits (-0.25, 0.25);
      pt.filter (*temp);

      // plane segmentation
      pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.015);
      seg.setMaxIterations(2000);
      seg.setInputCloud(temp);
      seg.segment (*table_inliers, *coefficients);
      pcl::ExtractIndices<pcl::PointXYZRGBA> exi;
      exi.setInputCloud(temp);
      exi.setNegative(true);
      exi.setIndices(table_inliers);
      exi.filter(*temp);
      //Radius outlier removal (if a point has not at least 2 neighbors in 5mm radius it is considered as an outlier, thus removed)
      pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> radf;
      radf.setInputCloud(temp);
      radf.setRadiusSearch(0.01);
      radf.setMinNeighborsInRadius(4);
      radf.filter(*temp);

      //now transform back to camera frame
      Eigen::Matrix4d T_inverse;
      if (lat==70)
        T_inverse = T_70;
      if (lat==50)
        T_inverse = T_50;
      if (lat==30)
        T_inverse = T_30;

      pcl::transformPointCloud(*temp, *cloud_, T_inverse ); 
    /*  //clustering 
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
      ec.setClusterTolerance (0.008);    
      ec.setMinClusterSize (200);
      ec.setMaxClusterSize (cloud_->points.size());
      ec.setInputCloud (tmp);
      ec.extract (cluster_indices);
      exi.setNegative(false);
      exi.setInputCloud(tmp);
      exi.setIndices(boost::make_shared<pcl::PointIndices>(cluster_indices.at(0)));
      exi.filter(*cloud_);
      if (cluster_indices.size() > 1)
      { 
      std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > clusters;
      ROS_WARN("[posesScanner] More than 1 cluster found... Adding them all together.");
      clusters.resize(cluster_indices.size());
      for (int i=1; i< cluster_indices.size(); ++i)
      {
      exi.setNegative(false);
      exi.setInputCloud(tmp);
      exi.setIndices(boost::make_shared<pcl::PointIndices>(cluster_indices.at(i)));
      exi.filter(clusters[i]);
      }
      for (int i=1; i< clusters.size(); ++i)
      for (int j=0; j< clusters[i].points.size(); ++j)
      cloud_->push_back(clusters[i].points[j]);
      } 
    */
      //publish pose 
      pub_poses_.publish(*cloud_); //automatic conversion to rosmsg
      //save poses on disk
      std::string filename (current_session_.string() + "/" + name + "_" + std::to_string(lat) + "_" + std::to_string(lon) + ".pcd" );
      writer.writeBinaryCompressed (filename.c_str(), *cloud_);
      //also let user view the pose 
      _viewer_->updatePointCloud(cloud_,"pose");
      _viewer_->spinOnce(500);
      //move table
      for (int t=1; t<=lon_pass; ++t)
      {//for table steps: 1 degree
        if (lon+t >= 360)
          break; //skip last step
        if(!set_turnTable_pos(lon+t) )
        {
          ROS_ERROR("[posesScanner] turnTable communication failed!");
          return false;
        }
      }
    }
  }//end of acquisitions
  _viewer_->removePointCloud("pose");
  _viewer_->removeCoordinateSystem();
  _viewer_->addText("DO NOT CLOSE THE VIEWER!!\nNODE WILL NOT FUNCTION PROPERLY WITHOUT THE VIEWER", 200,200,18,250,150,150,"text");
  _viewer_->spinOnce(200);
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "poses_scanner_node");
    poseGrabber poses_scanner_node;
    _viewer_->registerPointPickingCallback ( pickEvent , (void*)&_viewer_);
    _viewer_->registerKeyboardCallback ( keyboardEvent, (void*)&_viewer_);
    _viewer_->addText("DO NOT CLOSE THE VIEWER!!\nNODE WILL NOT FUNCTION PROPERLY WITHOUT THE VIEWER", 200,200,18,250,150,150,"text");
    _viewer_->spinOnce(100);
    ROS_INFO("[posesScanner] Started Poses Scanner Node\n");
    ros::spin();
    return 0;
}
