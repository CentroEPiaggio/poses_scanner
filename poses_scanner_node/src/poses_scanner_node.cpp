// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <ros/spinner.h>

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
#include "scene_filter_node/acquire_scene.h"
#include "lwr_controllers/PoseRPY.h"
#include "poses_scanner_node/poses_scannerConfig.h"

//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
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
    //dynamic reconfigure
    void reconfigure(poses_scanner_node::poses_scannerConfig &config, uint32_t level);
    //dynamic reconfigure server
    dynamic_reconfigure::Server<poses_scanner_node::poses_scannerConfig> srv_dyn;
    dynamic_reconfigure::Server<poses_scanner_node::poses_scannerConfig>::CallbackType callback;

    //method to move turn table
    bool set_turnTable_pos(float pos);
    //method to read turn table position
    float get_turnTable_pos();
    //method to move lwr
    bool set_lwr_pose(double radius, float latitude);  
    
    //method to center table
    void center_table();  
    //method to center object on table
    void center_object();  
    //method to try segmentation and adjust parameters
    void try_segmentation(int lat);
    //method to call both and adjust object
    void adjust_object(std::string name);

    //method to extract object from table
    void extract_object(int lat, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object);

    //method to acquire scene from openni2
    bool acquire_scene (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired, bool keep_organized);

    //method to acquire and save table transformation
    bool acquire_table_transform (int latitude);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_;
    bool calibration_;     
    boost::posix_time::ptime timestamp_;
    boost::filesystem::path work_dir_;
    boost::filesystem::path current_session_local;
    boost::filesystem::path current_session_kinect;
    boost::filesystem::path current_session_scene;
    
    Eigen::Matrix4f T_70, T_50, T_30; //transforms from camera to table
    
    //Variables to store parameters
    bool segment_70, segment_50, segment_30, clustering_70, clustering_50, clustering_30, outlier_70, outlier_50, outlier_30;
    double zmin_70, zmin_50, zmin_30, seg_tol_70, seg_tol_50, seg_tol_30;
    double clus_tol_70, clus_tol_50, clus_tol_30, out_rad_70, out_rad_50, out_rad_30;
    double lwr_x, lwr_y, lwr_z, lwr_roll, lwr_pitch, lwr_yaw, lwr_rad;
    int out_neigh_70, out_neigh_50, out_neigh_30, clus_min_70, clus_min_50, clus_min_30;
};

//Class Constructor
poseGrabber::poseGrabber()
{
  nh = ros::NodeHandle("poses_scanner_node");
  //service callbacks
  srv_acquire_ = nh.advertiseService("acquire_poses", &poseGrabber::acquirePoses, this);
  srv_table_ = nh.advertiseService("calibrate", &poseGrabber::calibrate, this);
  //bind callback for dynamic reconfigure
  callback = boost::bind(&poseGrabber::reconfigure, this, _1, _2);
  srv_dyn.setCallback(callback);
  //advertise acquired poses
  pub_poses_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> > ("acquired_poses",1);
  //publish to lwr controller
  pub_lwr_ = nh.advertise<lwr_controllers::PoseRPY> ("/lwr/OneTaskInverseKinematics/command_configuration",1);
  pcl::PointCloud<pcl::PointXYZRGBA> a,b;
  cloud_ = a.makeShared();
  scene_ = b.makeShared();
  timestamp_ = boost::posix_time::second_clock::local_time();
  calibration_ = false;
  //check if we have already found table transformations
  std::string home ( std::getenv("HOME") );
  work_dir_ = (home + "/PoseScanner");
  current_session_local = (work_dir_.string() + "/Session_" + to_simple_string(timestamp_) + "/Local" ); 
  current_session_kinect = (work_dir_.string() + "/Session_" + to_simple_string(timestamp_) + "/Kinect" ); 
  current_session_scene = (work_dir_.string() + "/Session_" + to_simple_string(timestamp_) + "/Scene" ); 
  boost::filesystem::path cal_file  (work_dir_.string() + "/transforms.h5");
  if (boost::filesystem::exists(cal_file) && boost::filesystem::is_regular_file(cal_file) )
  { 
    flann::Matrix<float> transf;
    flann::load_from_file (transf, cal_file.string(), "Table Transformations");
    for (int i=0; i<transf.rows; ++i)
      for (int j=0; j<transf.cols; ++j)
      {
        if (i>=0 && i<4)
          T_70(i,j) = transf[i][j];
        if (i>=4 && i<8)
          T_50(i-4,j) = transf[i][j];
        if (i>=8 && i<12)
          T_30(i-8,j) = transf[i][j];
      }
    ROS_INFO("[poses_scanner] Found transformations saved on disk");
    calibration_ = true;
  }
  else
    ROS_WARN("[poses_scanner] Calibration is not done yet, run calibration service before trying to acquire poses!");
  
  //Load default values of parameters
  nh.param<double>("/poses_scanner/zmin_70", zmin_70, -0.003);
  nh.param<double>("/poses_scanner/zmin_50", zmin_50, -0.003);
  nh.param<double>("/poses_scanner/zmin_30", zmin_30, -0.003);
  nh.param<bool>("/poses_scanner/segment_70", segment_70, "true");
  nh.param<bool>("/poses_scanner/segment_50", segment_50, "true");
  nh.param<bool>("/poses_scanner/segment_30", segment_30, "true");
  nh.param<double>("/poses_scanner/seg_tol_70", seg_tol_70, 0.01);
  nh.param<double>("/poses_scanner/seg_tol_50", seg_tol_50, 0.01);
  nh.param<double>("/poses_scanner/seg_tol_30", seg_tol_30, 0.01);
  nh.param<bool>("/poses_scanner/outlier_removal_70", outlier_70, "true");
  nh.param<bool>("/poses_scanner/outlier_removal_50", outlier_50, "true");
  nh.param<bool>("/poses_scanner/outlier_removal_30", outlier_30, "true");
  nh.param<double>("/poses_scanner/radius_search_70", out_rad_70, 0.01);
  nh.param<double>("/poses_scanner/radius_search_50", out_rad_50, 0.01);
  nh.param<double>("/poses_scanner/radius_search_30", out_rad_30, 0.01);
  nh.param<int>("/poses_scanner/neighbors_70", out_neigh_70, 5);
  nh.param<int>("/poses_scanner/neighbors_50", out_neigh_50, 5);
  nh.param<int>("/poses_scanner/neighbors_30", out_neigh_30, 5);
  nh.param<bool>("/poses_scanner/clustering_70", clustering_70, "false");
  nh.param<bool>("/poses_scanner/clustering_50", clustering_50, "false");
  nh.param<bool>("/poses_scanner/clustering_30", clustering_30, "false");
  nh.param<double>("/poses_scanner/clus_tol_70", clus_tol_70, 0.01);
  nh.param<double>("/poses_scanner/clus_tol_50", clus_tol_50, 0.01);
  nh.param<double>("/poses_scanner/clus_tol_30", clus_tol_30, 0.01);
  nh.param<int>("/poses_scanner/clus_min_70", clus_min_70, 5);
  nh.param<int>("/poses_scanner/clus_min_50", clus_min_50, 5);
  nh.param<int>("/poses_scanner/clus_min_30", clus_min_30, 5);
  nh.param<double>("/poses_scanner/lwr_x", lwr_x, -0.65);
  nh.param<double>("/poses_scanner/lwr_y", lwr_y, 0);
  nh.param<double>("/poses_scanner/lwr_z", lwr_z, 0.13);
  nh.param<double>("/poses_scanner/lwr_Roll", lwr_roll, 1.57079);
  nh.param<double>("/poses_scanner/lwr_Pitch", lwr_pitch, 0);
  nh.param<double>("/poses_scanner/lwr_Yaw", lwr_yaw, 0);
  nh.param<double>("/poses_scanner/lwr_rad", lwr_rad, 0.8);
}
//dynamic reconfigure callback  
void poseGrabber::reconfigure(poses_scanner_node::poses_scannerConfig &config, uint32_t level)
{
  ROS_INFO("[poses_scanner] Dynamic reconfigure updates");
  zmin_70 = config.zmin_70;
  zmin_50 = config.zmin_50;
  zmin_30 = config.zmin_30;
  segment_70 = config.segment_70;
  segment_50 = config.segment_50;
  segment_30 = config.segment_30;
  seg_tol_70 = config.tolerance_70;
  seg_tol_50 = config.tolerance_50;
  seg_tol_30 = config.tolerance_30;
  outlier_70 = config.outlier_removal_70;
  outlier_50 = config.outlier_removal_50;
  outlier_30 = config.outlier_removal_30;
  out_rad_70 = config.radius_search_70;
  out_rad_50 = config.radius_search_50;
  out_rad_30 = config.radius_search_30;
  out_neigh_70 = config.neighbors_70;
  out_neigh_50 = config.neighbors_50;
  out_neigh_30 = config.neighbors_30;
  clustering_70 = config.clustering_70;
  clustering_50 = config.clustering_50;
  clustering_30 = config.clustering_30;
  clus_tol_70 = config.clus_tolerance_70;
  clus_tol_50 = config.clus_tolerance_50;
  clus_tol_30 = config.clus_tolerance_30;
  clus_min_70 = config.min_points_70;
  clus_min_50 = config.min_points_50;
  clus_min_30 = config.min_points_30;

  //update parameters server also
  nh.setParam("/poses_scanner/zmin_70", zmin_70);
  nh.setParam("/poses_scanner/zmin_50", zmin_50);
  nh.setParam("/poses_scanner/zmin_30", zmin_30);
  nh.setParam("/poses_scanner/segment_70", segment_70);
  nh.setParam("/poses_scanner/segment_50", segment_50);
  nh.setParam("/poses_scanner/segment_30", segment_30);
  nh.setParam("/poses_scanner/seg_tol_70", seg_tol_70);
  nh.setParam("/poses_scanner/seg_tol_50", seg_tol_50);
  nh.setParam("/poses_scanner/seg_tol_30", seg_tol_30);
  nh.setParam("/poses_scanner/outlier_removal_70", outlier_70);
  nh.setParam("/poses_scanner/outlier_removal_50", outlier_50);
  nh.setParam("/poses_scanner/outlier_removal_30", outlier_30);
  nh.setParam("/poses_scanner/radius_search_70", out_rad_70);
  nh.setParam("/poses_scanner/radius_search_50", out_rad_50);
  nh.setParam("/poses_scanner/radius_search_30", out_rad_30);
  nh.setParam("/poses_scanner/neighbors_70", out_neigh_70);
  nh.setParam("/poses_scanner/neighbors_50", out_neigh_50);
  nh.setParam("/poses_scanner/neighbors_30", out_neigh_30);
  nh.setParam("/poses_scanner/clustering_70", clustering_70);
  nh.setParam("/poses_scanner/clustering_50", clustering_50);
  nh.setParam("/poses_scanner/clustering_30", clustering_30);
  nh.setParam("/poses_scanner/clus_tol_70", clus_tol_70);
  nh.setParam("/poses_scanner/clus_tol_50", clus_tol_50);
  nh.setParam("/poses_scanner/clus_tol_30", clus_tol_30);
  nh.setParam("/poses_scanner/clus_min_70", clus_min_70);
  nh.setParam("/poses_scanner/clus_min_50", clus_min_50);
  nh.setParam("/poses_scanner/clus_min_30", clus_min_30);
}

//wrapper function to grab a cloud
bool poseGrabber::acquire_scene (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired, bool keep_organized)
{ 
  if (keep_organized)
    nh.setParam("/scene_filter/keep_organized", true);
  else
    nh.setParam("/scene_filter/keep_organized", false);
  std::string acquire_scene_srv_name = nh.resolveName("/scene_filter_node/acquire_scene");
  scene_filter_node::acquire_scene acquire_srv;
  acquire_srv.request.save = "false";
  boost::this_thread::sleep (boost::posix_time::microseconds (300000));
  if ( !ros::service::call<scene_filter_node::acquire_scene>(acquire_scene_srv_name, acquire_srv))
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
bool poseGrabber::set_lwr_pose(double radius, float latitude)
{
  lwr_controllers::PoseRPY task;
  nh.getParam("/poses_scanner/lwr_x", lwr_x);
  nh.getParam("/poses_scanner/lwr_y", lwr_y);
  nh.getParam("/poses_scanner/lwr_z", lwr_z);
  nh.getParam("/poses_scanner/lwr_Roll", lwr_roll);
  nh.getParam("/poses_scanner/lwr_Pitch", lwr_pitch);
  nh.getParam("/poses_scanner/lwr_Yaw", lwr_yaw);
  
  //measure centre of the table in world robot frame
  task.id = 0;
  task.position.x = lwr_x;
  task.position.y =  lwr_y + (radius * cos(latitude*D2R));
  task.position.z = lwr_z + (radius * sin(latitude*D2R)); 
  task.orientation.roll = lwr_roll + (latitude*D2R); //this roll is in world frame! (it acts as a pitch for EE... 90° parallell to the ground, 0° points at ceiling)
  task.orientation.pitch = lwr_pitch; //this pitch is in world frame! (it acts as a roll for EE... )
  task.orientation.yaw = lwr_yaw; //this is yaw is in world frame! (zero is looking at the window for right arm)
  
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

void poseGrabber::center_table()
{
  nh.getParam("/poses_scanner/lwr_rad", lwr_rad);
  set_lwr_pose(lwr_rad,50);
  _proceed_ = false;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
  _viewer_->removeShape("text");
  _viewer_->addText("Center the table on the blue line, then press 't' when satisfied", 50,50,18,250,150,150,"center_t");
  _viewer_->addPointCloud(tmp,"table_cloud");
  pcl::PointXYZ a,b;
  a.x = a.y = b.x = b.y = a.z =0;
  b.z = 2;
  _viewer_->addLine(a,b,0,0.2,0.95,"line");
  while (!_proceed_)
  { 
    acquire_scene(tmp, false);
    _viewer_->updatePointCloud(tmp,"table_cloud");
    _viewer_->spinOnce(300);
  }
  _proceed_ = false;
  _viewer_->removeShape("line"); 
  _viewer_->removeShape("center_t");
  _viewer_->removePointCloud("table_cloud");
  _viewer_->spinOnce(300);
}

void poseGrabber::center_object()
{
  nh.getParam("/poses_scanner/lwr_rad", lwr_rad);
  _proceed_ = false;
  set_lwr_pose(lwr_rad,50);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acq (new pcl::PointCloud<pcl::PointXYZRGBA>);
  _viewer_->removeShape("text");
  _viewer_->addText("Center the object on the table, then press 't' when satisfied", 50,50,18,250,150,150,"center_ob");
  _viewer_->addPointCloud(tmp,"cloud_ob");
  pcl::ModelCoefficients x_plane, y_plane;
  x_plane.values.resize(4);
  x_plane.values[0] = 0;
  x_plane.values[1] = 1;
  x_plane.values[2] = 0;
  x_plane.values[3] = 0;
  y_plane.values.resize(4);
  y_plane.values[0] = 1;
  y_plane.values[1] = 0;
  y_plane.values[2] = 0;
  y_plane.values[3] = 0;
  Eigen::Matrix4f T_inv = T_50.inverse();
  acquire_scene(acq, false);
  pcl::transformPointCloud(*acq, *tmp, T_inv);
  _viewer_->addPlane(x_plane,"plane_x");
  _viewer_->addPlane(y_plane,"plane_y");
  _viewer_->addCoordinateSystem(0.2);
  _viewer_->spinOnce(300);
  while (!_proceed_)
  { 
    acquire_scene(acq, false);
    pcl::transformPointCloud(*acq, *tmp, T_inv);
    _viewer_->updatePointCloud(tmp,"cloud_ob");
    _viewer_->spinOnce(300);
  }
  _proceed_ = false;
  _viewer_->removeShape("center_ob"); 
  _viewer_->removeShape("plane_x");
  _viewer_->removeShape("plane_y");
  _viewer_->removePointCloud("cloud_ob");
  _viewer_->spinOnce(300);
}

void poseGrabber::try_segmentation(int lat)
{
  _proceed_ = false;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acq (new pcl::PointCloud<pcl::PointXYZRGBA>);
  _viewer_->removeShape("text");
  _viewer_->addText("Check segmentation and adjust parameters of corresponding latitude if needed, then press 't' when satisfied", 50,50,18,250,150,150,"seg_t");
  Eigen::Matrix4f T_inv;
  if (lat == 70)
    T_inv = T_70.inverse();
  if (lat == 50)
    T_inv = T_50.inverse();
  if (lat == 30)
    T_inv = T_30.inverse();
  acquire_scene(acq, false);
  pcl::transformPointCloud(*acq, *tmp, T_inv);
  _viewer_->addCoordinateSystem(0.2);
  _viewer_->addPointCloud(tmp,"seg_cl");
  _viewer_->spinOnce(300);
  while (!_proceed_)
  { 
    acquire_scene(acq, false);
    pcl::transformPointCloud(*acq, *tmp, T_inv);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj (new pcl::PointCloud<pcl::PointXYZRGBA>);
    extract_object(lat, tmp, obj);
    _viewer_->updatePointCloud(obj,"seg_cl");
    _viewer_->spinOnce(300);
  }
  _proceed_ = false;
  _viewer_->removeShape("seg_t");
  _viewer_->removePointCloud("seg_cl");
  _viewer_->spinOnce(300);
}

void poseGrabber::adjust_object(std::string name)
{
  float c_pos = get_turnTable_pos();
  if (c_pos != 0)
  {
    float step = -c_pos/360;
    for (int i=1; i<=360; i++)
    {
      if(!set_turnTable_pos(c_pos + step*i) )
      {
        ROS_ERROR("[posesScanner] turnTable communication failed!");
      }
    }
  }
  nh.getParam("/poses_scanner/lwr_rad", lwr_rad);
  set_lwr_pose(lwr_rad,70);
  try_segmentation(70);
  set_lwr_pose(lwr_rad,50);
  try_segmentation(50);
  set_lwr_pose(lwr_rad,30);
  try_segmentation(30);
}

void poseGrabber::extract_object(int lat, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object)
{
  bool segment,clustering, filter;
  double zmin,seg_tol,clus_tol, fil_rad;
  int fil_neigh, clus_min;
  nh.getParam("/poses_scanner/zmin_70", zmin_70);
  nh.getParam("/poses_scanner/zmin_50", zmin_50);
  nh.getParam("/poses_scanner/zmin_30", zmin_30);
  nh.getParam("/poses_scanner/segment_70", segment_70);
  nh.getParam("/poses_scanner/segment_50", segment_50);
  nh.getParam("/poses_scanner/segment_30", segment_30);
  nh.getParam("/poses_scanner/seg_tol_70", seg_tol_70);
  nh.getParam("/poses_scanner/seg_tol_50", seg_tol_50);
  nh.getParam("/poses_scanner/seg_tol_30", seg_tol_30);
  nh.getParam("/poses_scanner/outlier_removal_70", outlier_70);
  nh.getParam("/poses_scanner/outlier_removal_50", outlier_50);
  nh.getParam("/poses_scanner/outlier_removal_30", outlier_30);
  nh.getParam("/poses_scanner/radius_search_70", out_rad_70);
  nh.getParam("/poses_scanner/radius_search_50", out_rad_50);
  nh.getParam("/poses_scanner/radius_search_30", out_rad_30);
  nh.getParam("/poses_scanner/neighbors_70", out_neigh_70);
  nh.getParam("/poses_scanner/neighbors_50", out_neigh_50);
  nh.getParam("/poses_scanner/neighbors_30", out_neigh_30);
  nh.getParam("/poses_scanner/clustering_70", clustering_70);
  nh.getParam("/poses_scanner/clustering_50", clustering_50);
  nh.getParam("/poses_scanner/clustering_30", clustering_30);
  nh.getParam("/poses_scanner/clus_tol_70", clus_tol_70);
  nh.getParam("/poses_scanner/clus_tol_50", clus_tol_50);
  nh.getParam("/poses_scanner/clus_tol_30", clus_tol_30);
  nh.getParam("/poses_scanner/clus_min_70", clus_min_70);
  nh.getParam("/poses_scanner/clus_min_50", clus_min_50);
  nh.getParam("/poses_scanner/clus_min_30", clus_min_30);
  if (lat==70)
  {
    segment = segment_70;
    clustering = clustering_70;
    zmin = zmin_70;
    seg_tol = seg_tol_70;
    fil_neigh = out_neigh_70;
    filter = outlier_70;
    fil_rad = out_rad_70;
    clus_tol = clus_tol_70;
    clus_min = clus_min_70;
  }
  if (lat==50)
  {
    segment = segment_50;
    clustering = clustering_50;
    zmin = zmin_50;
    seg_tol = seg_tol_50;
    fil_neigh = out_neigh_50;
    filter = outlier_50;
    fil_rad = out_rad_50;
    clus_tol = clus_tol_50;
    clus_min = clus_min_50;
  }
  if (lat==30)
  {
    segment = segment_30;
    clustering = clustering_30;
    zmin = zmin_30;
    seg_tol = seg_tol_30;
    fil_neigh = out_neigh_30;
    filter = outlier_30;
    fil_rad = out_rad_30;
    clus_tol = clus_tol_30;
    clus_min = clus_min_30;
  }
  //Cropping z
  pcl::PassThrough<pcl::PointXYZRGBA> pt;
  pt.setInputCloud (scene);
  pt.setFilterFieldName ("z");
  pt.setFilterLimits (zmin, 0.7);
  pt.filter (*scene);
  //x
  pt.setInputCloud (scene);
  pt.setFilterFieldName ("x");
  pt.setFilterLimits (-0.25,  0.25);
  pt.filter (*scene);
  //y
  pt.setInputCloud (scene);
  pt.setFilterFieldName ("y");
  pt.setFilterLimits (-0.25, 0.25);
  pt.filter (*scene);
  pcl::copyPointCloud(*scene, *object);

  pcl::ExtractIndices<pcl::PointXYZRGBA> exi;
  // plane segmentation
  if (segment)
  {
    pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (seg_tol);
    seg.setMaxIterations(2000);
    seg.setInputCloud(scene);
    seg.segment (*table_inliers, *coefficients);
    exi.setInputCloud(scene);
    exi.setNegative(true);
    exi.setIndices(table_inliers);
    exi.filter(*object);
    pcl::copyPointCloud(*object, *scene);
  }
  if (filter)
  {
    //Radius outlier removal (if a point has not at least 4 neighbors in 1cm radius it is considered as an outlier, thus removed)
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> radf;
    radf.setInputCloud(scene);
    radf.setRadiusSearch(fil_rad);
    radf.setMinNeighborsInRadius(fil_neigh);
    radf.filter(*object);
    pcl::copyPointCloud(*object, *scene);
  }
  //clustering 
  if (clustering)
  {
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance (clus_tol);    
    ec.setMinClusterSize (clus_min);
    ec.setMaxClusterSize (scene->points.size());
    ec.setInputCloud (scene);
    ec.extract (cluster_indices);
    exi.setNegative(false);
    exi.setInputCloud(scene);
    exi.setIndices(boost::make_shared<pcl::PointIndices>(cluster_indices.at(0)));
    exi.filter(*object);
    if (cluster_indices.size() > 1)
    { 
      std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > clusters;
      ROS_WARN("[posesScanner] More than 1 cluster found... Adding them all together.");
      clusters.resize(cluster_indices.size());
      for (int i=1; i< cluster_indices.size(); ++i)
      {
        exi.setNegative(false);
        exi.setInputCloud(scene);
        exi.setIndices(boost::make_shared<pcl::PointIndices>(cluster_indices.at(i)));
        exi.filter(clusters[i]);
      }
      for (int i=1; i< clusters.size(); ++i)
        for (int j=0; j< clusters[i].points.size(); ++j)
          object->push_back(clusters[i].points[j]);
    }
  }
}

bool poseGrabber::acquire_table_transform (int latitude)
{
  //wait for a cloud from sensor
  if (! acquire_scene(cloud_, false) )
  {
    ROS_ERROR("[posesScanner] Cannot acquire a scene!");
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
  //find first plane
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (2000);
  seg.setDistanceThreshold (0.035);
  seg.setInputCloud(cloud_);
  seg.segment(*inliers, *coeff);
  //extract plane and whats on top
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  extract.setInputCloud(cloud_);
  extract.setNegative(true);
  extract.setIndices(inliers);
  extract.filter(*tmp);
  
  _viewer_->removeShape("text");
  _viewer_->addText("Pick the centre of the table (shift click), then press 't' when satisfied", 50,50,18,250,150,150,"pick_text");
  _viewer_->addPointCloud(tmp,"pick_cloud");
  while (!_proceed_)
  {
    _viewer_->spinOnce(300);
  }
  _viewer_->spinOnce(300);
  _proceed_ = false;
  
  pcl::SACSegmentation<pcl::PointXYZRGBA> segc;
  pcl::PointIndices::Ptr in (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coe (new pcl::ModelCoefficients);
  segc.setOptimizeCoefficients (true);
  segc.setModelType (pcl::SACMODEL_PLANE);
  segc.setMethodType (pcl::SAC_RANSAC);
  segc.setMaxIterations (2000);
  segc.setDistanceThreshold (0.015);
  segc.setInputCloud(tmp);
  segc.segment(*in, *coe);
  pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setModelCoefficients (coe);
  proj.setInputCloud(tmp);
  proj.filter(*tmp);
  
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
  
  Eigen::Vector3f new_z (nx,ny,nz); //the table normal
  Eigen::Vector3f vp (0 - cx, 0 - cy, 0 - cz); //viewpoint vector 0 (sensor origin) - table centre. It points "up" the table
  vp.normalize();
  new_z.normalize();
  if (vp.dot (new_z) < 0 ) //reorient the normal if needed
  {
    new_z *= -1; 
    ROS_INFO("Flipped Table Normal");
  }
  //check if new_z^t dot x is zero, if it is x lays on the table
  double precision = 1e-4;
  double result = (new_z.transpose() * Eigen::Vector3f::UnitX()); 
  Eigen::Matrix3f R_kt;
  if ( result > -precision && result < precision )
  {
    //we can use the actual x axis, cause it lays on the table
    Eigen::Vector3f new_y = new_z.cross(Eigen::Vector3f::UnitX());
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
    Eigen::Vector3f new_x;
    //TODO
    new_x = Eigen::Vector3f::UnitX(); //tmpTODO still using oldx
    Eigen::Vector3f new_y = new_z.cross(new_x);
    new_y.normalize();
    //compose the rotation matrix
    R_kt << new_x[0], new_y[0], new_z[0],
            new_x[1], new_y[1], new_z[1],
            new_x[2], new_y[2], new_z[2];
    ROS_WARN("using new x axis, result %g",result);
  }
  Eigen::Vector3f trasl (cx, cy, cz);
  Eigen::Matrix4f T_kt;
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
  }
  else if (latitude == 50)
  {
    T_50 = T_kt; 
  }
  else if (latitude == 30)
  {
    T_30 = T_kt; 
  }
  else
  {
    ROS_ERROR("wrong latitude passed");
    return false;
  }
  //view if transform is correct
  Eigen::Matrix4f T_inv = T_kt.inverse();
  pcl::transformPointCloud(*cloud_, *tmp, T_inv);
  _viewer_->addPointCloud(tmp, "final_cloud");
  _viewer_->addCoordinateSystem(0.2);
  _viewer_->removeShape("text");
  _viewer_->addText("Final table transformation, press 't' to proceed.",50,50,18,250,150,150,"final_text");
  while (!_proceed_)
  {
    _viewer_->spinOnce (300);
  }
  _viewer_->spinOnce(300);
  _proceed_ = false;
  return true;
}

//service callback for calibration
bool poseGrabber::calibrate(poses_scanner_node::table::Request &req, poses_scanner_node::table::Response &res)
{
  center_table();
  nh.getParam("/poses_scanner/lwr_rad", lwr_rad);
  //put lwr at first stop
  set_lwr_pose(lwr_rad, 70);  

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
  boost::this_thread::sleep (boost::posix_time::microseconds (20000000)); //wait for lwr  TODO add a topic to monitor if lwr has reached position
  bool cal70(false), cal50(false), cal30(false);
  cal70 = acquire_table_transform(70);
  
  //put lwr at second stop
  set_lwr_pose(lwr_rad, 50);  
  boost::this_thread::sleep (boost::posix_time::microseconds (20000000)); //wait for lwr  TODO add a topic to monitor if lwr has reached position
  cal50 = acquire_table_transform(50);
  //last stop
  set_lwr_pose(lwr_rad, 30);  
  boost::this_thread::sleep (boost::posix_time::microseconds (20000000)); //wait for lwr  TODO add a topic to monitor if lwr has reached position
  cal30 = acquire_table_transform(30);
  if (cal50 && cal70 && cal30)
  {
    flann::Matrix<double> Transforms (new double[12*4],12,4);
    for (int i=0; i<Transforms.rows; ++i)
      for (int j=0; j<Transforms.cols; ++j)
      {
        if (i>=0 && i<4)
          Transforms[i][j] = T_70(i,j);
        if (i>=4 && i<8)
          Transforms[i][j] = T_50(i-4,j);
        if (i>=8 && i<12)
          Transforms[i][j] = T_30(i-8,j);
      }
    flann::save_to_file (Transforms, work_dir_.string() +  "/transforms.h5", "Table Transformations");
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
  center_object();
  adjust_object(name);
  //create directory to store poses writes into "~/PoseScanner/objname/"
  boost::filesystem::path localpath (current_session_local);//addsubdirs scenes, kinect, local
  boost::filesystem::path kinectpath (current_session_kinect);//addsubdirs scenes, kinect, local
  boost::filesystem::path scenepath (current_session_scene);//addsubdirs scenes, kinect, local
  localpath /= name;
  kinectpath /= name;
  scenepath /= name;
  if (!boost::filesystem::exists(work_dir_) || !boost::filesystem::is_directory(work_dir_) )
  {
    boost::filesystem::create_directory(work_dir_);
  }
  if (!boost::filesystem::exists(localpath) || !boost::filesystem::is_directory(localpath))
  {
    boost::filesystem::create_directories(localpath);
  }
  if (!boost::filesystem::exists(kinectpath) || !boost::filesystem::is_directory(kinectpath))
  {
    boost::filesystem::create_directories(kinectpath);
  }
  if (!boost::filesystem::exists(scenepath) || !boost::filesystem::is_directory(scenepath))
  {
    boost::filesystem::create_directories(scenepath);
  }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr c (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PCDWriter writer;
  _viewer_->addPointCloud(c,"pose");
  _viewer_->addCoordinateSystem(0.15);
  _viewer_->removeShape("text");
  //acquisition loops
  //for cycle in latitude 
  nh.getParam("/poses_scanner/lwr_rad", lwr_rad);
  for (int lat = 70;  lat > 20; lat-=20) //fixed latitude pass (goes to 70, 50 and 30) 
  {
    //move lwr in position  
    set_lwr_pose(lwr_rad,lat);
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
    boost::this_thread::sleep (boost::posix_time::microseconds (18000000)); //wait for it    TODO remove and add topic
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
      if (! acquire_scene (scene_, true) )
      {
        ROS_ERROR("[posesScanner] Cannot acquire a scene!");
        return false;
      }

      //save scene on disk
      std::string scenename (scenepath.string() + "/" + name + "_" + std::to_string(lat) + "_" + std::to_string(lon) + ".pcd" );
      writer.writeBinaryCompressed (scenename.c_str(), *scene_);
      
      if (! acquire_scene (cloud_, false) )
      {
        ROS_ERROR("[posesScanner] Cannot acquire a scene!");
        return false;
      }
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGBA>);
      
      //Transforms used
      Eigen::Matrix4f T_l0k, T_kl0, T_l0li, T_lil0;

      if (lat==70)
      {
        T_kl0 = T_70;
        T_l0k = T_70.inverse();
      }
      if (lat==50)
      {
        T_kl0 = T_50;
        T_l0k = T_50.inverse();
      }
      if (lat==30)
      {
        T_kl0 = T_30;
        T_l0k = T_30.inverse();
      }
        //temporary transform to local frame (li) for easier cropping
      pcl::transformPointCloud(*cloud_, *temp, T_l0k); 
      extract_object(lat, temp, object); //temp is in l0
      
      //save the cloud in local frame (li)
      Eigen::AngleAxisf rot (lon*D2R, Eigen::Vector3f::UnitZ() );
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_local (new pcl::PointCloud<pcl::PointXYZRGBA>);
      //init matrix as simple rotation around z
      T_lil0 << rot.matrix()(0,0), rot.matrix()(0,1), rot.matrix()(0,2), 0,
                rot.matrix()(1,0), rot.matrix()(1,1), rot.matrix()(1,2), 0,
                rot.matrix()(2,0), rot.matrix()(2,1), rot.matrix()(2,2), 0,
                0,                 0,                 0,                 1;
      T_l0li = T_lil0.inverse();
      pcl::transformPointCloud(*object, *cloud_local, T_lil0); //now cloud local is in li
      
      //also let user view the local pose 
      _viewer_->updatePointCloud(cloud_local,"pose");
      _viewer_->spinOnce(300);
      
      //transform cloud back in sensor frame
      pcl::transformPointCloud(*cloud_local, *temp, T_l0li ); 
      pcl::transformPointCloud(*temp, *cloud_, T_kl0 );
      
      //get sensor information
      Eigen::Matrix4f T_sensor; //create one matrix for convinience
      T_sensor = T_kl0 * T_l0li;
      //extract quaternion of orientation from it
      Eigen::Matrix3f R_sensor;
      R_sensor = T_sensor.topLeftCorner(3,3);
      Eigen::Quaternionf Q_sensor (R_sensor); //init quaternion from rotation matrix
      Q_sensor.normalize();
      Eigen::Vector4f trasl_sensor (T_sensor(0,3), T_sensor(1,3), T_sensor(2,3), 1);
      //save sensor information in cloud local
      cloud_local->sensor_origin_ = trasl_sensor;
      cloud_local->sensor_orientation_ = Q_sensor;
      //now save local cloud
      std::string filename (localpath.string() + "/" + name + "_" + std::to_string(lat) + "_" + std::to_string(lon) + ".pcd" );
      writer.writeBinaryCompressed (filename.c_str(), *cloud_local);
      //publish local pose
      pub_poses_.publish(*cloud_local); //automatic conversion to rosmsg
      
      //save pose in sensor(kinect) on disk
      std::string kinectname (kinectpath.string() + "/" + name + "_" + std::to_string(lat) + "_" + std::to_string(lon) + ".pcd" );
      writer.writeBinaryCompressed (kinectname.c_str(), *cloud_);
      
      //move turntable
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
  _viewer_->spinOnce(300);
  float cur_pos = get_turnTable_pos();
  if (cur_pos != 0)
  {
    float step = -cur_pos/360;
    for (int i=1; i<=360; i++)
    {
      if(!set_turnTable_pos(cur_pos + step*i) )
      {
        ROS_ERROR("[posesScanner] turnTable communication failed!");
        return false;
      }
    }
  }
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "poses_scanner_node");
    poseGrabber node;
    _viewer_->registerPointPickingCallback ( pickEvent , (void*)&_viewer_);
    _viewer_->registerKeyboardCallback ( keyboardEvent, (void*)&_viewer_);
    _viewer_->addText("DO NOT CLOSE THE VIEWER!!\nNODE WILL NOT FUNCTION PROPERLY WITHOUT THE VIEWER", 200,200,18,250,150,150,"text");
    _viewer_->spinOnce(300);
    ros::AsyncSpinner spinner(0); //auto thread allocation
    ROS_INFO("[posesScanner] Started Poses Scanner Node\n");
    while (node.nh.ok())
    {
      spinner.start(); 
    }
    spinner.stop();
    return 0;
}
