#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <dynamixel_ros_control/dynamixel_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_hw_interface");
  ros::NodeHandle nh;

  if (ros::console::set_logger_level("ros.ros_control_boilerplate", ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<dynamixel_ros_control::DynamixelHWInterface> dynamixel_hw_interface(
      new dynamixel_ros_control::DynamixelHWInterface(nh));
  dynamixel_hw_interface->init();

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, dynamixel_hw_interface);

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
