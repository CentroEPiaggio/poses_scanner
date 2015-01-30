// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

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
    tf::TransformListener listen_;
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

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_;
    bool calibration_;
    boost::posix_time::ptime timestamp_;
    boost::filesystem::path work_dir_;
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
  boost::filesystem::path cal_file  (work_dir_.string() + "/table.transform");
  if (boost::filesystem::exists(cal_file) && boost::filesystem::is_regular_file(cal_file) )
  {
    calibration_ = true;
    //TODO review
  }
  else
  {
    ROS_WARN("[posesScanner] Camera is not calibrated, run calibration service, before trying to acquire poses!");
  }
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
    viewer->removeShape("cube");
    viewer->removeShape("info");
    viewer->removePointCloud("final");
    viewer->removePointCloud("Cropped");
    viewer->removePointCloud("Scene");
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
    std::string la(std::to_string(lati[id]);
    std::string lo(std::to_string(longi[id]);
    std::string pose_t ("latitude: " + la + "  longitude: " + lo);
    viewer->updateText(pose_t.c_str(), 25,25,18,0,200,0,"pose_t");

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
    std::string la(std::to_string(lati[id]);
    std::string lo(std::to_string(longi[id]);
    std::string pose_t ("latitude: " + la + "  longitude: " + lo);
    viewer->updateText(pose_t.c_str(), 25,25,18,0,200,0,"pose_t");
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
  
  //assume centre of table into (-0.6364, -0.21, 0.137) in world robot frame
  task.id = 0;
  task.position.x = -0.6364;
  task.position.y =  -0.21 + (radius * cos(latitude*D2R));
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

//service callback for calibration
bool poseGrabber::calibrate(poses_scanner_node::table::Request &req, poses_scanner_node::table::Response &res)
{
  //put lwr as high as possible to better see table
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
  boost::this_thread::sleep (boost::posix_time::microseconds (5000000)); //wait for lwr  TODO add a topic to monitor if lwr has reached position
  
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
  pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.015);
  seg.setInputCloud(cloud_crop_);
  seg.segment (*table_inliers, *coeffs);

  pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (table_inliers);
  proj.setInputCloud (cloud_crop_);
  proj.setCopyAllData(true);
  proj.setModelCoefficients (coeffs);
  proj.filter (*cloud_crop_);

  pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGBA>::Ptr table_model(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGBA> (cloud_crop_));
  pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (table_model); //Ransac algorithm
  ransac.setDistanceThreshold (0.01);
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
  centre[0]=coefficients[0];
  centre[1]=coefficients[1];
  centre[2]=coefficients[2];
  if (normal[1] < 0)
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
  Eigen::Matrix4f t = f.matrix(); //save final transformation from camera to table
  tf::Transform T_c_t (tf::Matrix3x3( t(0,0), t(0,1), t(0,2), 
                                    t(1,0), t(1,1), t(1,2),
                                    t(2,0), t(2,1), t(2,2) ),
                     tf::Vector3 (  t(0,3), t(1,3), t(2,3)));
  
  tf::StampedTransform T_7_t; //from lwr_7_link to rot_table
  tf::Transform T_7_c; //from lwr_7_link to camera (our calibration)
  listen_.lookupTransform("/turn_table", "/lwr_7_link", ros::Time(0), T_7_t); //fill it
  T_7_c = T_c_t.inverseTimes(T_7_t); //return T_c_t inverted and multiplied by T_7_t      T_7_c = (Tct^-1 * T_7_t)
  
  //Save transform to disk
  //create directory to store transformation in "HOME/PoseScanner"
  if (!boost::filesystem::exists(work_dir_) || !boost::filesystem::is_directory(work_dir_) )
  {
    boost::filesystem::create_directory(work_dir_);
  }
  ofstream trans_file;
  trans_file.open( (work_dir_.string() +  "/T_7_c.transform").c_str() );
  trans_file << "## Camera Calibration (lwr_7_link to camera) saved on " << to_simple_string(timestamp_).c_str() <<std::endl;
  trans_file << "## <Rotation quaternion xyzw> <new line> <translation> "<<std::endl;
  trans_file << T_7_c.getRotation().getX() <<" "<< T_7_c.getRotation().getY()<<" "<< T_7_c.getRotation().getZ()<<" "<<  T_7_c.getRotation().getW()<<std::endl; 
  trans_file << T_7_c.getOrigin()[0] <<" "<< T_7_c.getOrigin()[1] <<" "<< T_7_c.getOrigin()[2] << std::endl; 
  trans_file.close();

  
  viewer->removeShape("end");
  viewer->setWindowName("Final Table Model");
  viewer->addPointCloud(cloud_, "final");
  viewer->addText("Final table model, press 't' to proceed.", 25,25,18,0,200,0,"info");
  viewer->addCoordinateSystem(0.2);
  while (!proceed)
  {
    viewer->spinOnce (100);
  }
  viewer->spinOnce(100); //one last spin to update viewer
  proceed = false;
  calibration_=true;
  ROS_INFO("Calibration complete!!");
  
  return true;
}

//service callback to acquire poses
bool poseGrabber::acquirePoses(poses_scanner_node::acquire::Request &req, poses_scanner_node::acquire::Response &res)
{
  if (!calibration_)
  {
    ROS_ERROR("Camera is not calibrated!! Run calibration service before trying to acquire poses!");
    return false;
  }
  keep_acquiring = true;
  revision = false;
  int lon_pass = req.lon_pass;
  std::string name = req.objname;
  //create directory to store poses writes into "~/PoseScanner"
  if (!boost::filesystem::exists(work_dir_) || !boost::filesystem::is_directory(work_dir_) )
  {
    boost::filesystem::create_directory(work_dir_);
  }
  boost::filesystem::path current_session (work_dir_.string() + "/Session_" + to_simple_string(timestamp_) ); 
  if (!boost::filesystem::exists(current_session) || !boost::filesystem::is_directory(current_session))
  {
    boost::filesystem::create_directory(current_session);
  }
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
      boost::this_thread::sleep (boost::posix_time::microseconds (5000000)); //wait for it    TODO remove and add topic
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
        tf::StampedTransform T_c_t; //from camera_link to rot_table
        listen_.lookupTransform("/turn_table", "/camera_link", ros::Time(0) , T_c_t); //search and calculate it
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired (new pcl::PointCloud<pcl::PointXYZRGBA>);
        if (! acquire_scene (acquired) )
        {
          ROS_ERROR("[posesScanner] Cannot acquire a scene!");
          return false;
        }
        Eigen::Quaternionf cal_rot( T_c_t.getRotation().getW(), T_c_t.getRotation().getX(), T_c_t.getRotation().getY(), T_c_t.getRotation().getZ() );
        Eigen::Matrix<float,3,1> cal_trasl;
        cal_trasl << T_c_t.getOrigin()[0], T_c_t.getOrigin()[1], T_c_t.getOrigin()[2];

        //transform cloud into rot_table reference system
        pcl::transformPointCloud (*acquired, *cloud_, cal_trasl,cal_rot);

        //Cropping z
        pcl::PassThrough<pcl::PointXYZRGBA> pt;
        pt.setInputCloud (cloud_);
        pt.setFilterFieldName ("z");
        pt.setFilterLimits (-0.003, 0.4);
        pt.filter (*acquired);
        //x
        pt.setInputCloud (acquired);
        pt.setFilterFieldName ("x");
        pt.setFilterLimits (-0.3,  0.3);
        pt.filter (*cloud_);
        //y
        pt.setInputCloud (cloud_);
        pt.setFilterFieldName ("y");
        pt.setFilterLimits (-0.3, 0.3);
        pt.filter (*acquired);
        
        //rotate back of how the table has rotated
        Eigen::Affine3f lon_tran; 
        lon_tran = Eigen::AngleAxisf((lon*D2R), Eigen::Vector3f::UnitZ());  
        pcl::transformPointCloud(*acquired, *cloud_, lon_tran);

        pcl::copyPointCloud(*cloud_, *scene_); //save a copy of acquired scene

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

        pcl::ExtractIndices<pcl::PointXYZRGBA> exi;
        exi.setInputCloud(cloud_);
        exi.setIndices(table_inliers);
        exi.setNegative(true);
        exi.filter(*tmp);

        //clustering 
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
        ec.setClusterTolerance (0.01);    
        ec.setMinClusterSize (150);
        ec.setMaxClusterSize (tmp->points.size());
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
    viewer->addPointCloud(poses[0].makeShared(), "pose");
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
    viewer->removeShape("end");
    viewer->addText("Press 'n-p' to view next/previous pose.\nTo restart the whole acquisition process press 'r'. Otherwise press 't' to proceed.\nQuick cropping with mouse is activable by pressing 'x', changes will not be saved to disk unless 't' is pressed.", 25,25,18,0,200,0,"info");
    viewer->addCoordinateSystem(0.2);
    while (!proceed)
    {
      viewer->spinOnce(100);
    }
    viewer->spinOnce(100); //one last spin to update viewer
    revision = false;
    proceed = false;
    pcl::PCDWriter writer;
    if (!keep_acquiring)
    {
      for (int i=0; i<poses.size(); ++i)
      {
        //save poses on disk
        std::string filename (current_session.string() + "/" + name + "_" + std::to_string(lati[i]) + "_" + std::to_string(longi[i]) + ".pcd" );
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
