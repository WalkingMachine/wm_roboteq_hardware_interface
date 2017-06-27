#ifndef WM_ROBOTEQ_HARDWARE_INTERFACE_H
#define WM_ROBOTEQ_HARDWARE_INTERFACE_H

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include "sensor_msgs/JointState.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "serial/serial.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <unistd.h>
#include <iostream>
#include <sstream>

#define PI  3.141592653
#define MODE_STOPPED -1
#define MODE_VELOCITY 0
#define MODE_POSITION 1
#define ENCODER_TICKS 500
#define MAX_RPM 1000
#define CHANNEL 1
#define NUM_JOINTS 1

namespace WMRoboteq
{

  class WMRoboteqHardwareInterface : public hardware_interface::RobotHW
  {
    public:

      WMRoboteqHardwareInterface();

      ~WMRoboteqHardwareInterface(){delete serial_;};

      bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);

      void read();

      void write();

    private:

      const char *port_;
      bool connected_;
      int baud_;
      int device_id_;
      int param_id_;
      bool idSet_;
      bool receiving_script_messages_;
      serial::Serial *serial_;
      std::stringstream tx_buffer_;
      const std::string eol = "\r";
      const size_t max_line_length = 128;
      // These track our progress in attempting to initialize the controller.
      uint8_t start_script_attempts_;

      hardware_interface::VelocityJointInterface joint_velocity_interface_;
      hardware_interface::JointStateInterface joint_state_interface_;

      double joint_position_[NUM_JOINTS];
      double joint_effort_[NUM_JOINTS];
      double joint_velocity_[NUM_JOINTS];
      double joint_velocity_command_[NUM_JOINTS];
      std::string joint_name_[NUM_JOINTS];

      ros::NodeHandle nh_;
      ros::Timer timeout_timer_;

      void connect();
      void getId();
      void timeoutCallback(const ros::TimerEvent &);
      void flush();
      void sendMessage(std::string);
      void processStatus(std::string msg);
      void processFeedback(std::string msg);
      void processId(std::string msg);
      void setID(std::string str);

      // Send commands to motor driver.
      void setEstop() { command_ << "EX" << send_; }
      void resetEstop() { command_ << "MG" << send_; }
      void resetDIOx(int i) { command_ << "D0" << i << send_; }
      void setDIOx(int i) { command_ << "D1" << i << send_; }
      void startScript() { command_ << "R" << send_; }
      void stopScript() { command_ << "R" << 0 << send_; }
      void setUserVariable(int var, int val) { command_ << "VAR" << var << val << send_; }
      void setUserBool(int var, bool val) { command_ << "B" << var << (val ? 1 : 0) << send_; }
      void setSerialEcho(bool serial_echo) { param_ << "ECHOF" << (serial_echo ? 0 : 1) << sendVerify_; }

      class EOMSend
      {
      };

      class MessageSender
      {
        public:
        MessageSender(std::string init, WMRoboteqHardwareInterface *interface)
            : init_(init), interface_(interface)
        {}

        template<typename T>
        MessageSender &operator<<(const T val)
        {
          if (ss.tellp() == 0)
          {
            ss << init_ << val;
          } else
          {
            ss << ' ' << val;
          }
          return *this;
        }

        void operator<<(EOMSend)
        {
          interface_->sendMessage(ss.str());
          ss.str("");
        }

        private:
        std::string init_;
        WMRoboteqHardwareInterface* interface_;
        std::stringstream ss;
      };

      MessageSender command_;
      MessageSender query_;
      MessageSender param_;
      EOMSend send_, sendVerify_;

      /**
      * @param x Angular velocity in radians/s.
      * @return Angular velocity in RPM.
      */
      static double to_rpm(double x) { return x * 60 / (2 * M_PI); }

      /**
       * @param x Angular velocity in RPM.
       * @return Angular velocity in rad/s.
       */
      static double from_rpm(double x) { return x * (2 * M_PI) / 60; }

      /**
       * Conversion of radians to encoder ticks. Note that this assumes a
       * 1024-line quadrature encoder (hence 4096).
       *
       * @param x Angular position in radians.
       * @return Angular position in encoder ticks.
       */
      static double to_encoder_ticks(double x) { return x * ENCODER_TICKS / (2 * M_PI); }

      /**
       * Conversion of encoder ticks to radians. Note that this assumes a
       * 1024-line quadrature encoder (hence 4096).
       *
       * @param x Angular position in encoder ticks.
       * @return Angular position in radians.
       */
      static double from_encoder_ticks(double x) { return x * (2 * M_PI) / ENCODER_TICKS; }
  };

}
#endif
