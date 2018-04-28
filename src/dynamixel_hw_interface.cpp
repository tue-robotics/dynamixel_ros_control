#include <dynamixel_ros_control/dynamixel_hw_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#define ROS_ERROR_NAMED_AND_SHUTDOWN_AND_RETURN(...)                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    ROS_ERROR_NAMED(__VA_ARGS__);                                                                                      \
    ros::shutdown();                                                                                                   \
    return;                                                                                                            \
  } while (0)

namespace dynamixel_ros_control
{
template <typename T, typename U>
bool CanValueFitType(const U value)
{
  return value >= std::numeric_limits<T>::min() && value <= std::numeric_limits<T>::max();
}

DynamixelHWInterface::DynamixelHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("dynamixel_hw_interface", "DynamixelHWInterface Starting...");

  // read the ids from the parameter server
  std::vector<int> ids;
  if (!nh.getParam("dynamixel_ids", ids))
  {
    ROS_ERROR_NAMED_AND_SHUTDOWN_AND_RETURN("dynamixel_hw_interface", "Parameter dynamixel_ids should be an array of "
                                                                      "dynamixel ids");
  }

  if (joint_names_.size() != ids.size())
  {
    ROS_ERROR_NAMED_AND_SHUTDOWN_AND_RETURN("dynamixel_hw_interface", "Wrong number of dynamixel_ids, should be the "
                                                                      "same as joint_names");
  }

  // try to convert to uint8_t
  dynamixel_ids_.reserve(ids.size());
  for (const auto id : ids)
  {
    if (!CanValueFitType<uint8_t>(id))
    {
      ROS_ERROR_NAMED_AND_SHUTDOWN_AND_RETURN("dynamixel_hw_interface", "Dynamixel id '%i' is not uint8_t", id);
    }
    dynamixel_ids_.push_back(id);
  }

  // open the dynamixel workbench
  std::string device_name = nh.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate = nh.param<int>("baud_rate", 57600);

  uint8_t scan_range = nh.param<int>("scan_range", 10);

  uint32_t profile_velocity = nh.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = nh.param<int>("profile_acceleration", 50);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);

  // do a scan for all available dynamixels

  std::vector<uint8_t> dxl_ids;
  {
    dxl_ids.resize(scan_range);
    uint8_t dxl_cnt;
    if (dxl_wb_->scan(dxl_ids.data(), &dxl_cnt, scan_range) != true)
    {
      ROS_ERROR_NAMED_AND_SHUTDOWN_AND_RETURN("dynamixel_hw_interface", "Not found Motors, Please check scan range and "
                                                                        "baud rate");
    }
    dxl_ids.resize(dxl_cnt);
  }

  for (const auto id : dxl_ids)
  {
    ROS_INFO_NAMED("dynamixel_hw_interface", "Found dynamixel id=%u, model=%s", id, dxl_wb_->getModelName(id));
  }

  for (const uint8_t id : dynamixel_ids_)
  {
    if (std::find(dxl_ids.begin(), dxl_ids.end(), id) != dxl_ids.end())
    {
      ROS_INFO_NAMED("dynamixel_hw_interface", "Setting dynamixel id=%u to jointMode", id);
      dxl_wb_->jointMode(id, profile_velocity, profile_acceleration);
    }
    else
    {
      ROS_ERROR_NAMED_AND_SHUTDOWN_AND_RETURN("dynamixel_hw_interface", "Dynamixel id=%u not found", id);
    }
  }

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
  // printState();

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
