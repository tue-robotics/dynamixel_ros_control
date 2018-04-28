#include <dynamixel_ros_control/dynamixel_hw_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace dynamixel_ros_control
{
DynamixelHWInterface::DynamixelHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("dynamixel_hw_interface", "DynamixelHWInterface Starting...");

  // TODO: load from param server
  dynamixel_ids_ = { 2 };

  std::string device_name = nh.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate = nh.param<int>("baud_rate", 57600);

  uint8_t scan_range = nh.param<int>("scan_range", 20);

  uint32_t profile_velocity = nh.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = nh.param<int>("profile_acceleration", 50);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);

  uint8_t dxl_id_[16];
  uint8_t dxl_cnt_;
  if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true)
  {
    ROS_ERROR("Not found Motors, Please check scan range and baud rate");
    ros::shutdown();
    return;
  }

  for (const auto id : dynamixel_ids_)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(id));
    printf("ID      : %d\n", id);
    printf("\n");
  }
  for (const auto id : dynamixel_ids_)
  {
    dxl_wb_->jointMode(id, profile_velocity, profile_acceleration);
  }

  // for (int index = 0; index < dxl_cnt_; index++)
  // {
  //   printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
  //   printf("ID      : %d\n", dxl_id_[index]);
  //   printf("\n");
  // }
  printf("-----------------------------------------------------------------------\n");

  ROS_INFO_NAMED("dynamixel_hw_interface", "DynamixelHWInterface Ready.");
}

void DynamixelHWInterface::read(ros::Duration& elapsed_time)
{
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    joint_position_[joint_id] = dxl_wb_->convertValue2Radian(
        dynamixel_ids_[joint_id], dxl_wb_->itemRead(dynamixel_ids_[joint_id], "Present_Position"));
    //  joint_velocity_[joint_id] = dxl_wb_->convertValue2Velocity(dynamixel_ids_[joint_id],
    //  dxl_wb_->itemRead(dynamixel_ids_[joint_id], "Present_Velocity")); joint_effort_[joint_id] =
    //  dxl_wb_->convertValue2Torque(dynamixel_ids_[joint_id], dxl_wb_->itemRead(dynamixel_ids_[joint_id],
    //  "Present_Load"));
  }
}

void DynamixelHWInterface::write(ros::Duration& elapsed_time)
{
  printState();

  // Safety
  enforceLimits(elapsed_time);

  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    int32_t goal_position = 0;
    goal_position = dxl_wb_->convertRadian2Value(dynamixel_ids_[joint_id], joint_position_command_[joint_id]);
    bool ret = dxl_wb_->goalPosition(dynamixel_ids_[joint_id], goal_position);
  }
}

void DynamixelHWInterface::enforceLimits(ros::Duration& period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace
