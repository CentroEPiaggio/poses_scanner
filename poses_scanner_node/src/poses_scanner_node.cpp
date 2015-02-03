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
    bool calibration_, flipped_;
    boost::posix_time::ptime timestamp_;
    boost::filesystem::path work_dir_;
    boost::filesystem::path current_session_;

    Eigen::Matrix4f T_60, T_40, T_20; //transforms from camera to table
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
  flipped_ = false;
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
          if (line.compare("T60:") == 0)
          {
            tr_type = 1;
            row = 0;
            continue;
          }
          else if (line.compare("T40:") == 0)
          {
            tr_type = 2;
            row = 0;
            continue;
          }
          else if (line.compare("T20:") == 0)
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
                T_60 (row,0) = std::stof(vst[0]);
                T_60 (row,1) = std::stof(vst[1]);
                T_60 (row,2) = std::stof(vst[2]);
                T_60 (row,3) = std::stof(vst[3]);
                ++row;
              }
              if (tr_type == 2)
              {
                T_40 (row,0) = std::stof(vst[0]);
                T_40 (row,1) = std::stof(vst[1]);
                T_40 (row,2) = std::stof(vst[2]);
                T_40 (row,3) = std::stof(vst[3]);
                ++row;
              }
              if (tr_type == 3)
              {
                T_20 (row,0) = std::stof(vst[0]);
                T_20 (row,1) = std::stof(vst[1]);
                T_20 (row,2) = std::stof(vst[2]);
                T_20 (row,3) = std::stof(vst[3]);
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

//global stuff used by viewer callbacks
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_crop_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_original_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer () );
bool cropped(false);
bool revision(false);
bool proceed(false);
Eigen::Vector3f centre;
struct selection_cube
{
  float xmin, xmax;
  float ymin, ymax;
  float zmin, zmax;
} selection;
std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > poses;
int id(0);
bool keep_acquiring(true);
std::vector<int> lati;
std::vector<int> longi;

//viewer callbacks (keyboard)
void keyboardEvent (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);  
  if (event.getKeySym () == "c" && event.keyDown () && selection.zmin != 0 && cropped == false && revision == false)
  {
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (cloud_crop_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (selection.zmin, selection.zmax);
    pass.filter (*cloud_crop_);
    
    pass.setInputCloud (cloud_crop_);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (selection.ymin, selection.ymax);
    pass.filter (*cloud_crop_);

    pass.setInputCloud (cloud_crop_);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (selection.xmin, selection.xmax);
    pass.filter (*cloud_crop_);
    viewer->removeShape("Scene");
    viewer->addPointCloud(cloud_crop_, "Cropped");
    cropped = true;
    viewer->updateText("If unsatisfied press 'r' to reset, otherwise press 't' to proceed.", 25,25,18,0,200,0,"info1");
  }
  if (event.getKeySym () == "t" && event.keyDown () )
  {
    proceed=true;
//    viewer->removeShape("viewpoint");
    viewer->removePointCloud("pose");
    viewer->removeShape("info1");
    viewer->removeShape("pose_t");
    viewer->removeShape("cube");
    viewer->removeShape("info");
    viewer->removePointCloud("final");
    viewer->removePointCloud("Cropped");
    viewer->removePointCloud("Scene");
    viewer->removeShape("circle");
    viewer->removeCoordinateSystem();
    viewer->addText("DO NOT CLOSE THE VIEWER!!\nNODE WILL NOT FUNCTION PROPERLY WITHOUT THE VIEWER", 200,200,18,250,150,150,"end");
  }
  if (event.getKeySym () == "r" && event.keyDown () && cropped == true && revision ==false)
  {
    cropped = false;
    viewer->removeShape("Cropped");
    viewer->updateText("Hold 'Shift' and 'Leftclick' the centre of the table.\n When satisfied with the region selection, press 'c'", 25,25,18,0,200,0,"info1");
    viewer->addPointCloud(cloud_original_, "Scene");
    selection.xmin= selection.xmax= selection.ymin= selection.ymax= selection.zmax= selection.zmin = 0;
  }
  if (event.getKeySym () == "n" && event.keyDown () && revision==true )
  {
    if (++id >= poses.size() )
      id=poses.size()-1;
    viewer->updatePointCloud(poses[id].makeShared(), "pose");
    viewer->addCoordinateSystem(0.2);
    std::string la(std::to_string(lati[id]));
    std::string lo(std::to_string(longi[id]));
    std::string pose_t ("latitude: " + la + "  longitude: " + lo);
    viewer->updateText(pose_t.c_str(), 25,95,18,0,50,200,"pose_t");

 /*   viewer->removeShape("viewpoint");
    pcl::PointXYZ a,b;
    a.x=a.y=a.z=0;
    b.y= 0.5 * sin(lati[id]*D2R);
    b.z= 0.5 * cos(lati[id]*D2R)*cos(longi[id]*D2R);
    b.x= 0.5 * cos(lati[id]*D2R)*sin(longi[id]*D2R);
    viewer->addLine(a,b,0,1,1,"viewpoint");
    */
  }
  if (event.getKeySym () == "p" && event.keyDown () && revision ==true)
  {
    if (--id <= 0)
      id = 0;
    viewer->updatePointCloud(poses[id].makeShared(), "pose");
    viewer->addCoordinateSystem(0.2);
    std::string la(std::to_string(lati[id]));
    std::string lo(std::to_string(longi[id]));
    std::string pose_t ("latitude: " + la + "  longitude: " + lo);
    viewer->updateText(pose_t.c_str(), 25,95,18,0,50,200,"pose_t");
/*    viewer->removeShape("viewpoint");
    pcl::PointXYZ a,b;
    a.x=a.y=a.z=0;
    b.y= 0.5 * sin(lati[id]*D2R);
    b.z= 0.5 * cos(lati[id]*D2R)*cos(longi[id]*D2R);
    b.x= 0.5 * cos(lati[id]*D2R)*sin(longi[id]*D2R);
    viewer->addLine(a,b,0,1,1,"viewpoint");
    */
  }
  if (event.getKeySym () == "r" && event.keyDown () && revision ==true)
  {
    keep_acquiring = true;
  //  viewer->removeShape("viewpoint");
    viewer->removeShape("pose_t");
    viewer->removePointCloud("pose");
    viewer->removeShape("circle");
    viewer->removeCoordinateSystem();
    viewer->addText("DO NOT CLOSE THE VIEWER!!\nNODE WILL NOT FUNCTION PROPERLY WITHOUT THE VIEWER", 200,200,18,250,150,150,"end");
    proceed = true;
  }
}

//pick (shift+clik to activate)
void pickEvent (const pcl::visualization::PointPickingEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);  
  if (cropped || revision)
    return;
  viewer->removeShape("cube"); 
  event.getPoint (centre[0], centre[1], centre[2]);
  selection.xmin = centre[0] - 0.2;
  selection.xmax = centre[0] + 0.2;
  selection.ymin = centre[1] - 0.2;
  selection.ymax = centre[1] + 0.2;
  selection.zmin = centre[2] - 0.2;
  selection.zmax = centre[2] + 0.2;
  viewer->addCube ( selection.xmin, selection.xmax, selection.ymin, selection.ymax, selection.zmin, selection.zmax, 0, 0.9,0.2,"cube");  
}

//Area picking (x to activate)
void AreaSelectEvent (const  pcl::visualization::AreaPickingEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  boost::shared_ptr<std::vector<int> > selected (new std::vector<int>);
  if (event.getPointsIndices	(	*selected ) && revision == true)	
  {
    //viewer->removePointCloud("cloud");
    pcl::PointCloud<pcl::PointXYZRGBA> temp;
    pcl::ExtractIndices<pcl::PointXYZRGBA> fil (true);
    fil.setInputCloud (poses[id].makeShared());
    fil.setIndices (selected);
    fil.setNegative (true);
    fil.filter (temp);
    copyPointCloud(temp, poses[id]);
    viewer->updatePointCloud(poses[id].makeShared(), "pose");
  }
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
  task.position.x = -0.61;
  task.position.y =  -0.14 + (radius * cos(latitude*D2R));
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
  
  cropped = false;
  centre.setZero();
  selection.xmin= selection.xmax= selection.ymin= selection.ymax= selection.zmax= selection.zmin = 0;

  //First transofrmation, rotation by Pi around x (camera link frame) express it in eigen for pcl 
  Eigen::Affine3f RxPi;
  RxPi = Eigen::AngleAxisf(3.1415962, Eigen::Vector3f::UnitX() );

  //wait for a cloud from sensor
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired (new pcl::PointCloud<pcl::PointXYZRGBA>);

  if (! acquire_scene(acquired) )
  {
    ROS_ERROR("[posesScanner] Cannot acquire a scene!");
    return false;
  }
  pcl::transformPointCloud (*acquired, *cloud_, RxPi);
  pcl::copyPointCloud(*cloud_,*cloud_original_); //save a copy before cropping
  pcl::copyPointCloud (*cloud_, *cloud_crop_); //save cloud for callbacks so it can get cropped
  
  viewer->removeShape("end");
  viewer->addPointCloud(cloud_, "Scene");
  viewer->setWindowName("Table Scene");
  viewer->addCoordinateSystem(0.2);
  viewer->addText("Hold 'Shift' and 'Leftclick' the centre of the table.\n When satisfied with the region selection, press 'c'", 25,25,18,0,200,0,"info1");
  //wait user to toy with viewer
  while (!proceed)
  {
    viewer->spinOnce (100);
  }
  viewer->spinOnce(100); //one last spin to update viewer
  proceed = false;
  //plane segmentation
  pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.015);
  seg.setInputCloud(cloud_crop_);
  seg.segment (*table_inliers, *coeffs);
  //project points
  pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (table_inliers);
  proj.setInputCloud (cloud_crop_);
  proj.setCopyAllData(true);
  proj.setModelCoefficients (coeffs);
  proj.filter (*cloud_crop_);
  //find circle
  pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGBA>::Ptr table_model(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGBA> (cloud_crop_));
  pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (table_model); //Ransac algorithm
  ransac.setDistanceThreshold (0.02);
  ransac.setMaxIterations(2000);
  ransac.computeModel();
  Eigen::VectorXf coefficients;
  ransac.getModelCoefficients(coefficients);  
  /* Coefficients of model are:
   * [0] x coordinate of centre
   * [1] y coordinate of centre
   * [2] z coordinate of centre
   * [3] circle's radius
   * [4] x coordinate of normal direction
   * [5] y coordinate of normal direction
   * [6] z coordinate of normal direction
   */

  Eigen::Affine3f trasl, RaA, RzPi2;
  trasl = Eigen::Translation3f(-centre);
  // Second Transformation, transalte into table center
  pcl::transformPointCloud (*cloud_crop_, *cloud_, trasl);
  ransac.computeModel(); //recompute model and coefficients
  ransac.getModelCoefficients(coefficients);
  Eigen::Vector3f normal (coefficients[4], coefficients[5], coefficients[6]);
  if (normal[2] < 0)
  {
    normal *= -1; //sometimes the normal gets under the table (unpredicatably)
    std::cout<<"Flipped Table Normal\n";
  }
  //get rotation from old z axis to new z (table normal) 
  Eigen::Vector3f zaxis (0,0,1);
  Eigen::Vector3f rot_axis;
  rot_axis = normal.cross(zaxis);
  rot_axis.normalize();
  double angle = acos (normal.dot(zaxis)); //in radians
  //get the rotation matrix from axis angle
  RaA = Eigen::AngleAxisf(angle,rot_axis);
  //Third Transformation, rotate to align zaxis on table normal
  pcl::transformPointCloud (*cloud_, *cloud_, RaA);
  //Fourth Transformation, rotate around new z by +pi/2, so that x "points" to the camera
  RzPi2 = Eigen::AngleAxisf(3.1415962/2, Eigen::Vector3f::UnitZ());
  pcl::transformPointCloud (*cloud_, *cloud_, RzPi2);
  Eigen::Affine3f f = RzPi2 * RaA * trasl * RxPi;
  //create directory to store transformation in "HOME/PoseScanner"
  if (!boost::filesystem::exists(work_dir_) || !boost::filesystem::is_directory(work_dir_) )
  {
    boost::filesystem::create_directory(work_dir_);
  }
  if (latitude == 60)
  {
    T_60 = f.matrix(); 
    //Save transform to disk
    ofstream trans_file;
    trans_file.open( (work_dir_.string() +  "/table.transform").c_str() );
    trans_file << "## Table Transforms (camera to table) saved on " << to_simple_string(timestamp_).c_str() <<std::endl;
    trans_file << "## <Matrix4f> "<<std::endl;
    trans_file <<"T60:\n"<< T_60 <<std::endl; 
    trans_file.close();
  }
  else if (latitude == 40)
  {
    T_40 = f.matrix(); 
    //Save transform to disk
    fstream trans_file;
    trans_file.open( (work_dir_.string() +  "/table.transform").c_str(), std::fstream::out | std::fstream::app );
    trans_file <<"T40:\n"<< T_40 <<std::endl; 
    trans_file.close();
  }
  else if (latitude == 20)
  {
    T_20 = f.matrix(); 
    //Save transform to disk
    fstream trans_file;
    trans_file.open( (work_dir_.string() +  "/table.transform").c_str(), std::fstream::out | std::fstream::app );
    trans_file <<"T20:\n"<< T_20; 
    trans_file.close();
  }
  else
  {
    ROS_ERROR("wrong latitude passed");
    return false;
  }
  viewer->removeShape("end");
  viewer->setWindowName("Table Model");
  viewer->addPointCloud(cloud_, "final");
  viewer->addText("Final table model, press 't' to proceed.", 25,25,18,0,200,0,"info");
  viewer->addCoordinateSystem(0.2);
  while (!proceed)
  {
    viewer->spinOnce (100);
  }
  viewer->spinOnce(100); //one last spin to update viewer
  proceed = false;
  return true;
}

//service callback for calibration
bool poseGrabber::calibrate(poses_scanner_node::table::Request &req, poses_scanner_node::table::Response &res)
{
  //put lwr at first stop
  set_lwr_pose(0.9, 60);  

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
  boost::this_thread::sleep (boost::posix_time::microseconds (12000000)); //wait for lwr  TODO add a topic to monitor if lwr has reached position
  bool cal60(false), cal40(false), cal20(false);
  cal60 = acquire_table_transform(60);
  //put lwr at second stop
  set_lwr_pose(0.9, 40);  
  boost::this_thread::sleep (boost::posix_time::microseconds (12000000)); //wait for lwr  TODO add a topic to monitor if lwr has reached position
  cal40 = acquire_table_transform(40);
  //last stop
  set_lwr_pose(0.9, 20);  
  boost::this_thread::sleep (boost::posix_time::microseconds (20000000)); //wait for lwr  TODO add a topic to monitor if lwr has reached position
  cal20 = acquire_table_transform(20);
  if (cal60 && cal40 && cal20)
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
  keep_acquiring = true;
  revision = false;
  int lon_pass = req.lon_pass;
  std::string name;
  if (req.objname.compare(0,1,"-") == 0) //check if we wanted flipped object
  {
    flipped_ = true;
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
  viewer->removeShape("end");
  viewer->addText("prova", 25,95,18,0,50,200,"pose_t");
  viewer->addPointCloud(cloud_, "pose");
  pcl::ModelCoefficients circ;
  circ.values.resize(3);
  circ.values[0]=0;
  circ.values[1]=0;
  circ.values[2]=0.15;
  viewer->addCircle(circ,"circle");
  pcl::PCDWriter writer;
  //acquisition loops
  while (keep_acquiring)
  {
    poses.clear();
    lati.clear();
    longi.clear();
    id=0;
    //for cycle in latitude 
    for (int lat = 60;  lat > 10; lat-=20) //fixed latitude pass (goes to 60, 40 and 20)
    {
      //move lwr in position  
      set_lwr_pose(0.9,lat);
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
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired (new pcl::PointCloud<pcl::PointXYZRGBA>);
        if (! acquire_scene (acquired) )
        {
          ROS_ERROR("[posesScanner] Cannot acquire a scene!");
          return false;
        }
        if (lat == 60)
        {
        //transform cloud into turn_table reference system
          pcl::transformPointCloud (*acquired, *cloud_, T_60);
        }
        else if (lat == 40)
        {
          pcl::transformPointCloud (*acquired, *cloud_, T_40);
        }
        else if (lat == 20)
        {
          pcl::transformPointCloud (*acquired, *cloud_, T_20);
        }

        //rotate back of how the table has rotated
        Eigen::Affine3f lon_tran; 
        lon_tran = Eigen::AngleAxisf((lon*D2R), Eigen::Vector3f::UnitZ());  
        pcl::transformPointCloud(*cloud_, *scene_, lon_tran); //also saves a copy of scene in memory
        
        //save scene on disk
        std::string scenename (current_session_.string() + "/SCENE_" + name + "_" + std::to_string(lat) + "_" + std::to_string(lon) + ".pcd" );
        writer.writeBinaryCompressed (scenename.c_str(), *scene_);
        
        //Cropping z
        pcl::PassThrough<pcl::PointXYZRGBA> pt;
        pt.setInputCloud (scene_);
        pt.setFilterFieldName ("z");
        pt.setFilterLimits (+0.003, 0.5);
        pt.filter (*acquired);
        //x
        pt.setInputCloud (acquired);
        pt.setFilterFieldName ("x");
        pt.setFilterLimits (-0.25,  0.25);
        pt.filter (*cloud_);
        //y
        pt.setInputCloud (cloud_);
        pt.setFilterFieldName ("y");
        pt.setFilterLimits (-0.25, 0.25);
        pt.filter (*acquired);
        
        //Radius outlier removal (if a point has not at least 2 neighbors in 5mm radius it is considered as an outlier, thus removed)
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> radf;
        radf.setInputCloud(acquired);
        radf.setRadiusSearch(0.005);
        radf.setMinNeighborsInRadius(5);
        radf.filter(*tmp);

/* no plane segmentation
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZRGBA> seg(true);
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.005);
        seg.setMaxIterations(2000);
        seg.setInputCloud(cloud_);
        seg.setAxis(Eigen::Vector3f::UnitZ());
        seg.setEpsAngle(5*D2R);
        seg.segment (*table_inliers, *coefficients);
*/
        pcl::ExtractIndices<pcl::PointXYZRGBA> exi;
        //clustering 
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

        lati.push_back(lat);
        longi.push_back(lon);
        //publish pose 
        pub_poses_.publish(*cloud_); //automatic conversion to rosmsg
        //save poses in memory
        poses.push_back(*cloud_);
        //and let the user view it
        std::string la(std::to_string(lat));
        std::string lo(std::to_string(lon));
        std::string pose_t ("latitude: " + la + "  longitude: " + lo);
        viewer->updateText(pose_t.c_str(), 25,95,18,0,50,200,"pose_t");
        viewer->updatePointCloud(cloud_, "pose");
        viewer->setWindowName("Acquired Poses");
        viewer->addCoordinateSystem(0.2);
        viewer->spinOnce(100);
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
    //prompt user to review acquired poses, if not satisfied, start all over again 
    viewer->updatePointCloud(poses[0].makeShared(), "pose");
    viewer->setWindowName("Acquired Poses");
    /*
    pcl::PointXYZ a,b;
    a.x = a.y = a.z = b.x = 0;
    b.y = 0.5 * sin(lati[0]*D2R);
    b.z = 0.5 * cos(lati[0]*D2R);
    viewer->addLine(a,b,0,1,1,"viewpoint");
    */
    keep_acquiring = false;
    revision = true;
    viewer->addText("Press 'n-p' to view next/previous pose.\nTo restart the whole acquisition process press 'r'. Otherwise press 't' to proceed.\nQuick cropping with mouse is activable by pressing 'x', changes will not be saved to disk unless 't' is pressed.", 25,25,18,0,200,0,"info");
    viewer->addCoordinateSystem(0.2);
    std::string la(std::to_string(lati[0]));
    std::string lo(std::to_string(longi[0]));
    std::string pose_t ("latitude: " + la + "  longitude: " + lo);
    viewer->updateText(pose_t.c_str(), 25,95,18,0,50,200,"pose_t");
    while (!proceed)
    {
      viewer->spinOnce(100);
    }
    viewer->spinOnce(100); //one last spin to update viewer
    revision = false;
    proceed = false;
    if (!keep_acquiring)
    {
      for (int i=0; i<poses.size(); ++i)
      {
        //save poses on disk
        std::string filename (current_session_.string() + "/" + name + "_" + std::to_string(lati[i]) + "_" + std::to_string(longi[i]) + ".pcd" );
        writer.writeBinaryCompressed (filename.c_str(), poses[i]);
      }
    }
  }
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "poses_scanner_node");
    poseGrabber poses_scanner_node;
    viewer->registerPointPickingCallback ( pickEvent , (void*)&viewer);
    viewer->registerKeyboardCallback ( keyboardEvent, (void*)&viewer);
    viewer->registerAreaPickingCallback (AreaSelectEvent, (void*)&viewer);
    viewer->addText("DO NOT CLOSE THE VIEWER!!\nNODE WILL NOT FUNCTION PROPERLY WITHOUT THE VIEWER", 200,200,18,250,150,150,"end");
    viewer->spinOnce(100);
    ROS_INFO("[posesScanner] Started Poses Scanner Node\n");
    ros::spin();
    return 0;
}
