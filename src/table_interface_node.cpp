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
#include "turn_table_interface/setPos.h"
#include "turn_table_interface/getPos.h"

class TurnTable
{
public:
  TurnTable();
  virtual ~TurnTable();

private:
  ros::NodeHandle nh_;
  ros::ServiceServer srv_table_pos_, srv_read_pos_;
  //service callback
  bool setTablePos(turn_table_interface::setPos::Request  &req,
                        turn_table_interface::setPos::Response &res );
  bool getTablePos(turn_table_interface::getPos::Request  &req,
                        turn_table_interface::getPos::Response &res );

  std::string port_;
  double encoderRate_;
  int target_encoder_value_;
  boost::mutex cube_mutex_;
  comm_settings cube_comm_;
  void connectToCube();
};

TurnTable::TurnTable()
{
  ROS_INFO("[TurnTable] Starting turn table interface node");
  nh_ = ros::NodeHandle("turn_table_interface");
  nh_.param<std::string>("port", port_,"/dev/ttyUSB0");
  nh_.param<double>("encoderRate", encoderRate_, DEG_TICK_MULTIPLIER);

  ROS_INFO_STREAM("[TurnTable] Connecting to table at " << port_ );

  cube_mutex_.lock();
  this->connectToCube();
  commActivate(&cube_comm_, 1, true); //cube_id is 1 for table
  cube_mutex_.unlock();

  srv_table_pos_ = nh_.advertiseService("set_table_pos", &TurnTable::setTablePos, this);
  srv_read_pos_ = nh_.advertiseService("get_table_pos", &TurnTable::getTablePos, this);

}

TurnTable::~TurnTable()
{
  cube_mutex_.lock();
  closeRS485(&cube_comm_);
  cube_mutex_.unlock();
  ROS_INFO_STREAM("[TurnTable] Communication closed");
}

void TurnTable::connectToCube()
{
  openRS485(&cube_comm_, port_.c_str());
  if(cube_comm_.file_handle <= 0)
  {
    ROS_ERROR("[TurnTable] Panic, cube file handle was invalid");
    return;
  }
  else
  {
    ROS_INFO_STREAM("[TurnTable] Opened communication with file handle " << cube_comm_.file_handle);
  }
}

bool TurnTable::setTablePos(turn_table_interface::setPos::Request  &req,
             turn_table_interface::setPos::Response &res )
{
  double position = req.position; // pos in degrees

  ROS_INFO_STREAM("[TurnTable] Sending Turn Table to position: " << position);

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

bool TurnTable::getTablePos(turn_table_interface::getPos::Request  &req,
             turn_table_interface::getPos::Response &res )
{
  short int measurements[3];
  double position;
  cube_mutex_.lock();
  commGetMeasurements(&cube_comm_, 1, measurements);
  cube_mutex_.unlock();
  position = (double)(measurements[0]) /encoderRate_;
  ROS_INFO_STREAM("[TurnTable] Table position reads: " << position );
  res.current_pos = (short int)position;
  return true;
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "turn_table_interface");
    TurnTable turn_table_node;
    ros::spin();
    return 0;
}
