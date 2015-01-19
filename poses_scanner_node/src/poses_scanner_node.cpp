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
  private:
    ros::NodeHandle nh_;
    ros::ServiceServer srv_acquire_, srv_table_;
    ros::Publisher pub_poses_;
    //service callback
    bool acquirePoses(poses_scanner_node::acquire::Request& req, poses_scanner_node::acquire::Response& res);
    bool acquireTable(poses_scanner_node::table::Request& req, poses_scanner_node::table::Response& res);

    //method to move turn table
    bool set_turnTable_pos(float pos);
    //methos to read turn table position
    float get_turnTable_pos();

    //method to acquire scene from openni2
    bool acquire_scene (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_;
    Eigen::Matrix4f table_transform_;
    float table_radius_;
    bool table_set_;
    boost::posix_time::ptime timestamp_;
};

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

/*void keybReview (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);  
}
*/

//viewer callbacks
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
    viewer->removeShape("viewpoint");
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
    viewer->removeShape("viewpoint");
    pcl::PointXYZ a,b;
    a.x=a.y=a.z=0;
    b.y= 0.5 * sin(lati[id]*D2R);
    b.z= 0.5 * cos(lati[id]*D2R)*cos(longi[id]*D2R);
    b.x= 0.5 * cos(lati[id]*D2R)*sin(longi[id]*D2R);
    viewer->addLine(a,b,0,1,1,"viewpoint");
  }
  if (event.getKeySym () == "p" && event.keyDown () && revision ==true)
  {
    if (--id <= 0)
      id = 0;
    viewer->updatePointCloud(poses[id].makeShared(), "pose");
    viewer->removeShape("viewpoint");
    pcl::PointXYZ a,b;
    a.x=a.y=a.z=0;
    b.y= 0.5 * sin(lati[id]*D2R);
    b.z= 0.5 * cos(lati[id]*D2R)*cos(longi[id]*D2R);
    b.x= 0.5 * cos(lati[id]*D2R)*sin(longi[id]*D2R);
    viewer->addLine(a,b,0,1,1,"viewpoint");
  }
  if (event.getKeySym () == "r" && event.keyDown () && revision ==true)
  {
    keep_acquiring = true;
    viewer->removeShape("viewpoint");
    viewer->removePointCloud("pose");
    viewer->removeCoordinateSystem();
    viewer->addText("DO NOT CLOSE THE VIEWER!!\nNODE WILL NOT FUNCTION PROPERLY WITHOUT THE VIEWER", 200,200,18,250,150,150,"end");
    proceed = true;
  }
}

void pickEvent (const pcl::visualization::PointPickingEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);  
  if (cropped || revision)
    return;
  viewer->removeShape("cube"); //TODO adjust cube based on kuka position (should be ~90° lat when acquiring a table)
  event.getPoint (centre[0], centre[1], centre[2]);
  selection.xmin = centre[0] - 0.17;
  selection.xmax = centre[0] + 0.17;
  selection.ymin = centre[1] - 0.07;
  selection.ymax = centre[1] + 0.07;
  selection.zmin = centre[2] - 0.17;
  selection.zmax = centre[2] + 0.17;
  viewer->addCube ( selection.xmin, selection.xmax, selection.ymin, selection.ymax, selection.zmin, selection.zmax, 0, 0.9,0.2,"cube");  
}
//Area picking callback for viewer (x to activate)
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

//Constructor
poseGrabber::poseGrabber()
{
  nh_ = ros::NodeHandle("poses_scanner_node");
  //service callbacks
  srv_acquire_ = nh_.advertiseService("acquire_poses", &poseGrabber::acquirePoses, this);
  srv_table_ = nh_.advertiseService("acquire_table_model", &poseGrabber::acquireTable, this);
  table_set_=false;
  table_transform_.setZero();
  //advertise acquired poses
  pub_poses_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBA> > ("acquired_poses",3);
  pcl::PointCloud<pcl::PointXYZRGBA> a,b;
  cloud_ = a.makeShared();
  scene_ = b.makeShared();
  timestamp_ = boost::posix_time::second_clock::local_time();
}

bool poseGrabber::acquire_scene (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired)
{ 
  std::string acquire_scene_srv_name = nh_.resolveName("/scene_acquirer_node/acquire_scene");
  scene_acquirer_node::acquire_scene acquire_srv;
  acquire_srv.request.save = "false";
  boost::this_thread::sleep (boost::posix_time::microseconds (300000));
  if ( !ros::service::call<scene_acquirer_node::acquire_scene>(acquire_scene_srv_name, acquire_srv))
  {
    ROS_ERROR("[posesScanner] Acquire scene service failed!");
    return false;
  }
//  boost::this_thread::sleep (boost::posix_time::microseconds (1500000));
  //sensor_msgs::PointCloud2 acq ;
  //acq = acquire_srv.response.cloud;
  pcl::fromROSMsg (acquire_srv.response.cloud, *acquired);
  return true;
}

bool poseGrabber::set_turnTable_pos(float pos)
{
  std::string setPos_srv_name = nh_.resolveName("/turn_table_interface_node/set_table_pos");
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
float poseGrabber::get_turnTable_pos()
{
  std::string getPos_srv_name = nh_.resolveName("/turn_table_interface_node/get_table_pos");
  turn_table_interface_node::getPos get_srv;
  if ( !ros::service::call<turn_table_interface_node::getPos>(getPos_srv_name, get_srv) )
  {
    ROS_ERROR("[posesScanner] getPos from turn table failed!");
    return -1;
  }
  return get_srv.response.current_pos;
}

bool poseGrabber::acquireTable(poses_scanner_node::table::Request &req, poses_scanner_node::table::Response &res)
{
  // Requested a table model
  // TODO move kuka over the table (almost 90 degrees latitude) so that table is better acquired
  
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
  cropped = false;
  centre.setZero();
  selection.xmin= selection.xmax= selection.ymin= selection.ymax= selection.zmax= selection.zmin = 0;

  //First transofrmation, invert Y and Z
  Eigen::Affine3f RxPi;
  RxPi = Eigen::AngleAxisf(3.14159265359, Eigen::Vector3f::UnitX());  
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
  //
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
  
  /* keep centre as clicked point and not centre found with ransac
  centre[0]=coefficients[0];
  centre[1]=coefficients[1];
  centre[2]=coefficients[2];
  */

  Eigen::Affine3f trasl, RaA;
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
  //get rotation from old y axis to new y (table normal)
  Eigen::Vector3f yaxis (0,1,0);
  Eigen::Vector3f rot_axis;
  rot_axis = normal.cross(yaxis);
  rot_axis.normalize();
  double angle = acos (normal.dot(yaxis)); //in radians
  //get the rotation matrix from axis angle
  RaA = Eigen::AngleAxisf(angle,rot_axis);
  //Third Transformation, rotate to align yaxis on table normal
  pcl::transformPointCloud (*cloud_, *cloud_, RaA);
  Eigen::Affine3f f = RaA * trasl * RxPi;
  table_transform_ = f.matrix();
  table_radius_ = coefficients[3];
  if (req.save_to_disk)
  {//requested transform saved to disk
    //create directory to store poses writes into "~/PosesScanner/"
    std::string home ( std::getenv("HOME") );
    boost::filesystem::path base_dir (home + "/PosesScanner");
    if (!boost::filesystem::exists(base_dir) || !boost::filesystem::is_directory(base_dir) )
    {
      boost::filesystem::create_directory(base_dir);
    }
    ofstream trans_file;
    trans_file.open( (base_dir.string() +  "/table_transform.actual").c_str() );
    trans_file << "## Table transformation saved on " << to_simple_string(timestamp_).c_str() <<std::endl;
    trans_file << "## <transformation_matrix 4x4> <newline> <table_radius>" <<std::endl;
    trans_file << table_transform_ <<std::endl<<table_radius_;
    trans_file.close();
  }
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
  table_set_=true;
  return true;
}
// this function is called when service is called
bool poseGrabber::acquirePoses(poses_scanner_node::acquire::Request &req, poses_scanner_node::acquire::Response &res)
{
  std::string home ( std::getenv("HOME") );
  boost::filesystem::path base_dir (home + "/PosesScanner");
  boost::filesystem::path table_trans_file (base_dir.string() + "/table_transform.actual");
  if (!table_set_)
  {
    if (boost::filesystem::exists(table_trans_file) && boost::filesystem::is_regular_file(table_trans_file) )
    {
      ifstream t_file (table_trans_file.string().c_str());
      std::string line;
      if(t_file.is_open())
      {
        int i(0);
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
              table_transform_(i,0) = std::stof(vst[0]);
              table_transform_(i,1) = std::stof(vst[1]);
              table_transform_(i,2) = std::stof(vst[2]);
              table_transform_(i,3) = std::stof(vst[3]);
              ++i;
            }
            else if (vst.size() == 1)
            {
              table_radius_ = std::stof(vst[0]);
            }
            else
            {
              ROS_ERROR("[posesScanner] Incorrect table transformation file...");
              return false;
            }
          }
        }
        table_set_ = true;
        ROS_INFO("[posesScanner] Found table transformation saved on disk:");
        std::cout<<table_transform_;
        ROS_INFO("[posesScanner] With table radius of %g", table_radius_);
      }
      else
      {
        ROS_ERROR("[posesScanner] Error reading table transformation from file... (can not open file)");
        return false;
      }
    }
    else
    {
      ROS_ERROR("[posesScanner] Set up a table model before trying to acquire poses!");
      return false;
    }
  }
  /*  TODO - Set Kuka arm in position  */
  
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
  keep_acquiring = true;
  revision = false;
  int lat_pass = req.lat_pass;
  int lon_pass = req.lon_pass;
  std::string name = req.objname;
 // std::string topic = nh_.resolveName("/camera/depth_registered/points");
  //create directory to store poses writes into "~/PosesScanner/"
  if (!boost::filesystem::exists(base_dir) || !boost::filesystem::is_directory(base_dir) )
  {
    boost::filesystem::create_directory(base_dir);
  }
  boost::filesystem::path current_session (base_dir.string() + "/Session_" + to_simple_string(timestamp_) ); 
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
    //for cycle in latitude TODO
    int lat = req.lat_pass; //tmp lat workaround (set fixed latitute from service instead of pass)  TODO remove
    for (int lon=0; lon<360; lon+=lon_pass)
    {//for cycle in longitude
      //transform into table refernce system
      cloud_->clear();
      while (!(get_turnTable_pos() >= lon-1 && get_turnTable_pos() <= lon+1))
      {
        boost::this_thread::sleep (boost::posix_time::microseconds (50000)); //wait for table to be in position
      }
      Eigen::Affine3f t_tran (table_transform_);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired (new pcl::PointCloud<pcl::PointXYZRGBA>);
      if (! acquire_scene (acquired) )
      {
        ROS_ERROR("[posesScanner] Cannot acquire a scene!");
        return false;
      }
     
      pcl::transformPointCloud (*acquired, *cloud_, t_tran);

      //Cropping z
      pcl::PassThrough<pcl::PointXYZRGBA> pt;
      pt.setInputCloud (cloud_);
      pt.setFilterFieldName ("z");
      pt.setFilterLimits (-table_radius_*2, table_radius_*2);
      pt.filter (*acquired);
      //x
      pt.setInputCloud (acquired);
      pt.setFilterFieldName ("x");
      pt.setFilterLimits (-table_radius_*2,  table_radius_*2);
      pt.filter (*cloud_);
      //y
      pt.setInputCloud (cloud_);
      pt.setFilterFieldName ("y");
      pt.setFilterLimits (-0.003, table_radius_*3);
      pt.filter (*acquired);
      //rotate back
      Eigen::Affine3f lon_tran; 
      lon_tran = Eigen::AngleAxisf((lon*D2R), Eigen::Vector3f::UnitY());  
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
      seg.setAxis(Eigen::Vector3f::UnitY());
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
    }//end of acquisitions
    //put table back at zero
    for (int t=358; t>=0; --t)
    {//for table steps: 1 degree
      if(!set_turnTable_pos(t) )
      {
        ROS_ERROR("[posesScanner] turnTable communication failed!");
        return false;
      }
    }
    //prompt user to review acquired poses, if not satisfied, start all over again 
    viewer->addPointCloud(poses[0].makeShared(), "pose");
    viewer->setWindowName("Acquired Poses");
    //viewer->addCoordinateSystem(0.2);
    pcl::PointXYZ a,b;
    a.x = a.y = a.z = b.x = 0;
    b.y = 0.5 * sin(lati[0]*D2R);
    b.z = 0.5 * cos(lati[0]*D2R);
    viewer->addLine(a,b,0,1,1,"viewpoint");
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
    poseGrabber pose_scanner_node;
    viewer->registerPointPickingCallback ( pickEvent , (void*)&viewer);
    viewer->registerKeyboardCallback ( keyboardEvent, (void*)&viewer);
    viewer->registerAreaPickingCallback (AreaSelectEvent, (void*)&viewer);
    viewer->addText("DO NOT CLOSE THE VIEWER!!\nNODE WILL NOT FUNCTION PROPERLY WITHOUT THE VIEWER", 200,200,18,250,150,150,"end");
    viewer->spinOnce(100);
    ROS_INFO("[posesScanner] Starting Poses Scanner Node, waiting for service calls...\n");
    ros::spin();
    return 0;
}
