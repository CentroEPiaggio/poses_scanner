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

    sensor_msgs::PointCloud2 cloud_stream_;
};

//Constructor
sceneAcquirer::sceneAcquirer()
{
  nh_ = ros::NodeHandle("scene_acquirer_node");
  
  //service callbacks
  srv_acquire_ = nh_.advertiseService("acquire_scene", &sceneAcquirer::acquireScene, this);
  
  //subscribe to depth_registered pointclouds topic
  std::string topic = nh_.resolveName("/camera/depth_registered/points");
  sub_stream_ = nh_.subscribe(topic, 1, &sceneAcquirer::new_cloud_in_stream, this, ros::TransportHints().unreliable().tcpNoDelay());
}

void sceneAcquirer::new_cloud_in_stream(const sensor_msgs::PointCloud2::ConstPtr& message)
{
  //constantly copy cloud from stream into class cloud_stream_ to be accessible for service callback
  cloud_stream_ = *message;
}

bool sceneAcquirer::acquireScene (scene_acquirer_node::acquire_scene::Request& req, scene_acquirer_node::acquire_scene::Response& res)
{
  if (req.save.compare("false") != 0)
  {
    std::string home = std::getenv( "HOME" );
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg (cloud_stream_, cloud);
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed ((home + "/" + req.save + ".pcd").c_str(), cloud);
  }
  res.cloud = cloud_stream_;
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
