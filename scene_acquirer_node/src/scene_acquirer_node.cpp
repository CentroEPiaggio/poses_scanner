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
    
    //Service callback, gets executed when service is called
    bool acquireScene(scene_acquirer_node::acquire_scene::Request& req, scene_acquirer_node::acquire_scene::Response& res);

    //Message callback, gets executed when a new message is available on topic
    void new_cloud_in_stream(const sensor_msgs::PointCloud2::ConstPtr& message);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_stream_;
};

//Constructor
sceneAcquirer::sceneAcquirer()
{
  nh_ = ros::NodeHandle("scene_acquirer_node");
  pcl::PointCloud<pcl::PointXYZRGBA> a;
  cloud_stream_ = a.makeShared();

  //service callbacks
  srv_acquire_ = nh_.advertiseService("acquire_scene", &sceneAcquirer::acquireScene, this);
  
  //subscribe to depth_registered pointclouds topic
  std::string topic = nh_.resolveName("/camera/depth_registered/points");
  sub_stream_ = nh_.subscribe(topic, 1, &sceneAcquirer::new_cloud_in_stream, this);
}

void sceneAcquirer::new_cloud_in_stream(const sensor_msgs::PointCloud2::ConstPtr& message)
{
  //constantly copy cloud from stream into class cloud_stream_ to be accessible for service callback
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr stream (new pcl::PointCloud<pcl::PointXYZRGBA> );
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pcl::fromROSMsg (*message, *stream);
  pass.setInputCloud (stream);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.7, 1.8);
  pass.filter (*cloud_stream_);
  pass.setInputCloud (cloud_stream_);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-2, 0.3);
  pass.filter (*cloud_stream_);
}

bool sceneAcquirer::acquireScene (scene_acquirer_node::acquire_scene::Request& req, scene_acquirer_node::acquire_scene::Response& res)
{
  if (req.save.compare("false") != 0)
  {
    std::string home = std::getenv( "HOME" );
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed ((home + "/" + req.save + ".pcd").c_str(), *cloud_stream_);
  }
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud_stream_, msg);
  res.cloud = msg; 
  ROS_INFO("[sceneAcquirer] Sent cloud to service response");
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scene_acquirer_node");
    sceneAcquirer scene_acquirer_node;
    ROS_INFO("[sceneAcquirer] Syncing to /camera/depth_registered/points...");
    ROS_INFO("[sceneAcquirer] Waiting service call to grab a scene...");
    ros::spin();
    return 0;
}
