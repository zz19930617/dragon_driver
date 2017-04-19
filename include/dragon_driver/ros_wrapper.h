/*
 * qr_ros_wrapper.h
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#ifndef INCLUDE_QR_ROS_WRAPPER_H_
#define INCLUDE_QR_ROS_WRAPPER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_manager/controller_manager.h>

#include "middleware.h"
#include "util/parser.h"

#define DEBUG_TOPIC
#ifdef DEBUG_TOPIC
#include "hardware/encoder.h"
#include "hardware/motor.h"
#endif

namespace middleware {

class RosRobotHW;

class RosWrapper {

public:
  ~RosWrapper();
  // 获取QuadrupedRobotDriver对象实例
  static RosWrapper* getInstance();

  bool start();
  void halt();

  void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction>);
  void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction>);

private:
  RosWrapper();

  // 发布实时消息， 例如"/joint_states"
  void publishRTMsg();
  void rosControlLoop();
  void publishEleCurMsg();

  /**
   * FollowJointTrajectory实际执行线程
   */
  void trajThread(std::vector<double> timestamps,
      std::vector<std::vector<double> > positions,
      std::vector<std::vector<double> > velocities);
  /**
   * ActionServer执行的辅助函数
   */
  bool validateJointNames();
  bool has_velocities();
  bool has_positions();
  bool traj_is_finite();
  void reorder_traj_joints(trajectory_msgs::JointTrajectory&);
  bool start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps);

  // 测试消息回调函数
#ifdef DEBUG_TOPIC
  void cbForDebug(const std_msgs::Float64MultiArrayConstPtr&);
  ros::Subscriber cmd_sub_;
#endif

private:
  static RosWrapper* instance_;
  bool alive_;

  ros::NodeHandle nh_;
  // FollowJointTrjectoryAction服务器相关变量。 实现FollowJointTrajectory功能
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle_;
  control_msgs::FollowJointTrajectoryResult result_;
  bool has_goal_;

  Middleware* robot_;
  std::chrono::milliseconds rt_duration_; // 实时消息发布频率， 默认是50Hz(使用周期表示, 即20ms）
  std::chrono::milliseconds ros_ctrl_duration_; // ros_control_thread_循环频率， 默认是100Hz(使用周期表示, 即10ms）
  std::thread* rt_publish_thread_; // 该线程发布实时消息
  std::thread* cur_publish_thread_;//该线程发布电流信息
  std::thread* ros_control_thread_; // 若启动ros_control机制， 该线程维护ros_control的正常流程
  bool use_ros_control_;
  boost::shared_ptr<RosRobotHW> hardware_interface_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

} /* namespace qr_driver */

#endif /* INCLUDE_QR_ROS_WRAPPER_H_ */
