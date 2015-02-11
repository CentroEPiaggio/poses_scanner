// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
// ROS generated headers
#include "scene_acquirer_node/acquire_scene.h" 

//general utilities
#include <string>
#include <stdlib.h>

class sceneAcquirer
{
  public:
    sceneAcquirer();
  private:
    //Node handle
    ros::NodeHandle nh_;

    //Service Server
    ros::ServiceServer srv_acquire_;
    
    //Message Subscriber
    ros::Subscriber sub_stream_;
    
    //Message Publisher
    ros::Publisher pub_stream_;
    
    //Service callback, gets executed when service is called
    bool acquireScene(scene_acquirer_node::acquire_scene::Request& req, scene_acquirer_node::acquire_scene::Response& res);

    //Message callback, gets executed when a new message is available on topic
    void new_cloud_in_stream(const sensor_msgs::PointCloud2::ConstPtr& message);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_stream_;

    //parameters
    bool filter_, downsample_;
    double xmin,xmax,ymin,ymax,zmin,zmax,leaf_;
};

//Constructor
sceneAcquirer::sceneAcquirer()
{
  nh_ = ros::NodeHandle("scene_acquirer_node");
  pcl::PointCloud<pcl::PointXYZRGBA> a;
  scene_stream_ = a.makeShared();

  //service callbacks
  srv_acquire_ = nh_.advertiseService("acquire_scene", &sceneAcquirer::acquireScene, this);
  
  //subscribe to depth_registered pointclouds topic
  std::string topic = nh_.resolveName("/camera/depth_registered/points");
  sub_stream_ = nh_.subscribe(topic, 1, &sceneAcquirer::new_cloud_in_stream, this);

  pub_stream_ = nh_.advertise<sensor_msgs::PointCloud2> ("/scene_acquirer/scene",1);

  //load parameters
  nh_.param<bool>("/filter", filter_, "false");
  nh_.param<bool>("/downsample", downsample_, "false");
  nh_.param<double>("/xmin", xmin, -100);
  nh_.param<double>("/xmax", xmax, 100);
  nh_.param<double>("/ymin", ymin, -100);
  nh_.param<double>("/ymax", ymax, 100);
  nh_.param<double>("/zmin", zmin, -100);
  nh_.param<double>("/zmax", zmax, 100);
  nh_.param<double>("/leaf_s", leaf_, 0.005);
}

void sceneAcquirer::new_cloud_in_stream(const sensor_msgs::PointCloud2::ConstPtr& message)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
  //constantly copy cloud from stream into class scene_stream_ to be accessible for service callback
  pcl::fromROSMsg (*message, *tmp);

  //check if we need to filter stream
  nh_.getParam("/filter", filter_);
  if (filter_)
  {
    nh_.getParam("/xmin", xmin);
    nh_.getParam("/xmax", xmax);
    nh_.getParam("/ymin", ymin);
    nh_.getParam("/zmin", zmin);
    nh_.getParam("/ymax", ymax);
    nh_.getParam("/zmax", zmax);
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (tmp);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (zmin, zmax);
    pass.filter (*tmp);
    pass.setInputCloud (tmp);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ymin, ymax);
    pass.filter (*tmp);
    pass.setInputCloud (tmp);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (xmin, xmax);
    pass.filter (*scene_stream_);
    pcl::copyPointCloud(*scene_stream_ , *tmp);
  }
  //check if we need to downsample stream
  nh_.getParam("/downsample", downsample_);
  if (downsample_)
  {
    nh_.getParam("/leaf_s", leaf_);
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud (tmp);
    vg.setLeafSize(leaf_, leaf_, leaf_);
    vg.filter (*scene_stream_);
    pcl::copyPointCloud(*scene_stream_ , *tmp);
  }
  if (filter_ || downsample_)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg (*tmp, msg);
    pub_stream_.publish( msg ); //republish the modified scene
  }
  else
  {
    pub_stream_.publish( *message ); //or republish the same scene
  }
}

bool sceneAcquirer::acquireScene (scene_acquirer_node::acquire_scene::Request& req, scene_acquirer_node::acquire_scene::Response& res)
{
  if (req.save.compare("false") != 0)
  {
    //user requested the scene to be saved on disk, lets comply him
    std::string home = std::getenv( "HOME" );
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed ((home + "/" + req.save + ".pcd").c_str(), *scene_stream_);
    ROS_INFO("[sceneAcquirer] Scene saved to %s", (home + "/" + req.save + ".pcd").c_str() );
  }
  //also send it to service response
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scene_stream_, msg);
  res.cloud = msg; 
  ROS_INFO("[sceneAcquirer] Sent scene to service response");
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scene_acquirer_node");
    sceneAcquirer scene_acquirer_node;
    ROS_INFO("[sceneAcquirer] Node is ready");
    ros::spin(); //go as fast as you can !
    return 0;
}
