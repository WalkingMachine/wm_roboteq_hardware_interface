//
// Created by gortium on 03/05/17.
//

#ifndef PROJECT_WMRoboteqHardwareInterface_H
#define PROJECT_WMRoboteqHardwareInterface_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <ros/ros.h>
#include <roboteq_msgs/Feedback.h>
#include <roboteq_msgs/Command.h>
#include <pluginlib/class_list_macros.h>

namespace wm_roboteq_hardware_interface
{
  class WMRoboteqHardwareInterface : public hardware_interface::RobotHW {
    public:
      // << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
      // Functions
      bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
      void read(const ros::Time &time, const ros::Duration &period);
      void write(const ros::Time &time, const ros::Duration &period);

      // Interface variables
      std::string name_;
      double cmd_;
      double pos_;
      double vel_;
      double eff_;

    private:
      // Variables
      hardware_interface::VelocityJointInterface joint_velocity_interface_;
      hardware_interface::JointStateInterface joint_state_interface_;
      ros::Publisher cmdPub_;
      ros::Subscriber feedbackSub_;
      roboteq_msgs::Feedback last_msg_;

      // Functions
      void feedbackCB( roboteq_msgs::Feedback msg );

  };
}
#endif //PROJECT_WMRoboteqHardwareInterface_H
