#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
//#include <sensor_msgs/JointState.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/duration.h>

#include <time.h>
#include <boost/thread/mutex.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <string.h>

#include "qb_cube_lib.h"
#include "turn_table_interface_node/setPos.h"
#include "turn_table_interface_node/getPos.h"

class turnTable
{
public:
  turnTable();
  virtual ~turnTable();

private:
  ros::NodeHandle nh_;
  ros::ServiceServer srv_table_pos_, srv_read_pos_;
  //service callback
  bool setTablePos(turn_table_interface_node::setPos::Request  &req,
                        turn_table_interface_node::setPos::Response &res );
  bool getTablePos(turn_table_interface_node::getPos::Request  &req,
                        turn_table_interface_node::getPos::Response &res );

  std::string port_;
  double encoderRate_;
  int target_encoder_value_;
  boost::mutex cube_mutex_;
  comm_settings cube_comm_;
  void connectToCube();
};

turnTable::turnTable()
{
  ROS_INFO("[turnTable] Starting turn table interface node");
  nh_ = ros::NodeHandle("turn_table_interface_node");
  nh_.param<std::string>("port", port_,"/dev/ttyUSB0");
  nh_.param<double>("encoderRate", encoderRate_, DEG_TICK_MULTIPLIER);
  
  ROS_INFO_STREAM("[turnTable] Connecting to table at " << port_ );
  
  cube_mutex_.lock();
  this->connectToCube();
  commActivate(&cube_comm_, 1, true); //cube_id is 1 for table
  cube_mutex_.unlock();
  
  srv_table_pos_ = nh_.advertiseService("set_table_pos", &turnTable::setTablePos, this);
  srv_read_pos_ = nh_.advertiseService("get_table_pos", &turnTable::getTablePos, this);
  
}

turnTable::~turnTable()
{
  cube_mutex_.lock();
  closeRS485(&cube_comm_);
  cube_mutex_.unlock();
  ROS_INFO_STREAM("[turnTable] Communication closed");
}

void turnTable::connectToCube() 
{
  openRS485(&cube_comm_, port_.c_str());
  if(cube_comm_.file_handle <= 0) 
  {
    ROS_ERROR("[turnTable] Panic, cube file handle was invalid");
    return;
  }
  else
  {
    ROS_INFO_STREAM("[turnTable] Opened communication with file handle " << cube_comm_.file_handle);
  }
}

bool turnTable::setTablePos(turn_table_interface_node::setPos::Request  &req,
             turn_table_interface_node::setPos::Response &res )
{
  double position = req.position; // pos in degrees

  ROS_INFO_STREAM("[turnTable] Sending Turn Table to position: " << position);

  short int inputs;
  inputs = encoderRate_*position; 
  short int curr_ref[NUM_OF_MOTORS];
  curr_ref[0] = inputs;
  curr_ref[1] = inputs;

  cube_mutex_.lock();
  commSetInputs(&cube_comm_, 1, curr_ref); //actual communication
  cube_mutex_.unlock();
  return true;
}

bool turnTable::getTablePos(turn_table_interface_node::getPos::Request  &req,
             turn_table_interface_node::getPos::Response &res )
{
  short int measurements[3];
  double position;
  cube_mutex_.lock();
  commGetMeasurements(&cube_comm_, 1, measurements);
  cube_mutex_.unlock();
  position = (double)(measurements[0]) /encoderRate_;
  ROS_INFO_STREAM("[turnTable] Table position reads: " << position );
  res.current_pos = (short int)position;
  return true;
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "turn_table_interface_node");
    turnTable turn_table_node;
    ros::spin();
    return 0;
}
