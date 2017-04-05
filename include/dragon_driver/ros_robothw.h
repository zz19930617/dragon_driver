/*
 * ros_robothw.h
 *
 *  Created on: Jan 16, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_ROS_ROBOTHW_H_
#define INCLUDE_MIDDLEWARE_ROS_ROBOTHW_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>

#include "middleware.h"

namespace middleware {

class RosRobotHW: public hardware_interface::RobotHW {
protected:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;
  // Robot API
  Middleware* robot_;
  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  // Effort 接口实际并未实现
  hardware_interface::EffortJointInterface effort_joint_interface_;

  bool velocity_interface_running_;
  bool position_interface_running_;
  bool effort_interface_running_;

  const std::vector<std::string>& joint_names_;
  std::size_t num_joints_;

  // Shared memory
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  // 速度控制时， 使用到的辅助变量
  std::vector<double> prev_joint_velocity_command_;
public:
  virtual ~RosRobotHW();
  RosRobotHW(ros::NodeHandle&, Middleware*);

  /// \brief Initialize the hardware interface
  virtual void init();
  /// \brief Read the state from the robot hardware.
  virtual void read();
  /// \brief write the command to the robot hardware.
  virtual void write();

  bool canSwitch(
        const std::list<hardware_interface::ControllerInfo> &start_list,
        const std::list<hardware_interface::ControllerInfo> &stop_list) const override;

  void doSwitch(const std::list<hardware_interface::ControllerInfo>&start_list,
      const std::list<hardware_interface::ControllerInfo>&stop_list) override;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_ROS_ROBOTHW_H_ */
