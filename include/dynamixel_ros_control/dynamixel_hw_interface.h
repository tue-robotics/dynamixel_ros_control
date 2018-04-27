#pragma once

#include <ros_control_boilerplate/generic_hw_interface.h>

namespace dynamixel_ros_control
{

class DynamixelHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  DynamixelHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  virtual void read(ros::Duration &elapsed_time);

  virtual void write(ros::Duration &elapsed_time);

  virtual void enforceLimits(ros::Duration &period);

}; 

}