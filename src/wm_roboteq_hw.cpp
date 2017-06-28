//
// Created by gortium on 03/05/17.
//

#include "wm_roboteq_hardware_interface/wm_roboteq_hw.h"


namespace wm_roboteq_hardware_interface
{
// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>

  bool WMRoboteqHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
  {
    using namespace hardware_interface;

    // Get parameters
    std::vector<std::string> Joints;
    if (!robot_hw_nh.getParam("joints", Joints)) { return false; }
    name_ = Joints[0];
    std::string cmd_topic;
    std::string feedback_topic;
    if (!robot_hw_nh.getParam("cmd_topic", cmd_topic)) { return false; }
    if (!robot_hw_nh.getParam("feedback_topic", feedback_topic)) { return false; }

    // Initialise interface variables
    cmd_ = 0;
    pos_ = 0;
    vel_ = 0;
    eff_ = 0;

    // Register interfaces
    joint_state_interface_.registerHandle(JointStateHandle(name_, &pos_, &vel_, &eff_));
    joint_velocity_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(name_), &cmd_));
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_velocity_interface_);

    // advertise publisher
    cmdPub_ = robot_hw_nh.advertise<roboteq_msgs::Command>( cmd_topic, 1 );
    feedbackSub_ = robot_hw_nh.subscribe( feedback_topic, 1, &WMRoboteqHardwareInterface::feedbackCB, this );

    return true;
  }

  void WMRoboteqHardwareInterface::read(const ros::Time &time, const ros::Duration &period)
  {
    vel_ = last_msg_.measured_velocity;
    pos_ = last_msg_.measured_position;
  }

  void WMRoboteqHardwareInterface::write(const ros::Time &time, const ros::Duration &period)
  {
    roboteq_msgs::Command msg;
    msg.mode = msg.MODE_VELOCITY;
    msg.setpoint = cmd_;
    cmdPub_.publish( msg );
  }

  void WMRoboteqHardwareInterface::feedbackCB( roboteq_msgs::Feedback msg )
  {
    last_msg_ = msg;
  }
}

PLUGINLIB_EXPORT_CLASS( wm_roboteq_hardware_interface::WMRoboteqHardwareInterface, hardware_interface::RobotHW)