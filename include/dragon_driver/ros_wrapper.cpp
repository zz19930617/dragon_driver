/*
 * qr_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#include "ros_robothw.h"
#include "ros_wrapper.h"

namespace middleware {

RosWrapper* RosWrapper::instance_ = nullptr;

RosWrapper* RosWrapper::getInstance() {
  if (nullptr == instance_) {
    instance_ = new RosWrapper;
    LOG_INFO << "Create the singleton instance: RosWrapper";
  }

  LOG_INFO << "Return the singleton instance: RosWrapper";
  return instance_;
}

RosWrapper::RosWrapper()
  : alive_(true),
    as_(nh_, "follow_joint_trajectory",
      boost::bind(&RosWrapper::goalCB, this, _1),
      boost::bind(&RosWrapper::cancelCB, this, _1),
      false),
    has_goal_(false),
    robot_(nullptr),
    rt_duration_(1000/50),
    ros_ctrl_duration_(1000/100),
    rt_publish_thread_(nullptr),
    ros_control_thread_(nullptr),
    use_ros_control_(true) {
  google::InitGoogleLogging("dragon_ros_wrapper");
  // google::SetLogDestination(google::GLOG_INFO, "/path/to/log/INFO_");
  // google::LogMessage::Init();
  FLAGS_colorlogtostderr = true;
  google::FlushLogFiles(google::GLOG_INFO);
  ; // Nothing to do here, all of variables initialize in the method @start()
}

bool RosWrapper::start() {
  bool debug = false;
  ros::param::get("~debug", debug);
  if (debug) {
    google::SetStderrLogging(google::GLOG_INFO);
  } else {
    google::SetStderrLogging(google::GLOG_WARNING);
  }
  
  robot_ = Middleware::getInstance();
  if (!robot_->init(nh_)) {
    LOG_ERROR << "Launch the robot fail from ros::NodeHandle";
    return false;
  }

  if (!robot_->isInit()) {
    LOG_ERROR << "Start RosWrapper FAIL";
    return false;
  }

  ros::param::get("~use_ros_control", use_ros_control_);
  if (use_ros_control_) {
    hardware_interface_.reset(
              new middleware::RosRobotHW(nh_, robot_));
    controller_manager_.reset(
        new controller_manager::ControllerManager(
            hardware_interface_.get(), nh_));
  }

  if (robot_->start()) {
    if (use_ros_control_) {
      double frequency = 100.0;
      ros::param::get("~ctrl_loop_frequency", frequency);
      if (frequency > 0)
        ros_ctrl_duration_ = std::chrono::milliseconds((int)(1000.0 / frequency));

      ros_control_thread_ = new std::thread(
          boost::bind(&RosWrapper::rosControlLoop, this));
      LOG_INFO << "The control thread for this driver has been started";
    } else {
      // start actionserver
      has_goal_ = false;
      as_.start();

      double frequency = 50.0;
      ros::param::get("~rt_frequency", frequency);
      if (frequency > 0)
        rt_duration_ = std::chrono::milliseconds((int)(1000.0 / frequency));

      // 若启动ros_control, 则使用joint_state_controller来发布该/joint_states
      rt_publish_thread_ = new std::thread(
          boost::bind(&RosWrapper::publishRTMsg, this));
      LOG_INFO << "The action server for this driver has been started";
    }
  } else {
    LOG_ERROR << "The robot thread which for the real-time message has failed";
    return false;
  }

  // For debug
#ifdef DEBUG_TOPIC
  cmd_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("debug", 100,
      &RosWrapper::cbForDebug, this);
#endif
  return true;
}

void RosWrapper::halt() {
  alive_ = false;
  if (nullptr != rt_publish_thread_) {
    rt_publish_thread_->join();
    delete rt_publish_thread_;
    rt_publish_thread_ = nullptr;
  }
  if (nullptr != ros_control_thread_) {
    ros_control_thread_->join();
    delete ros_control_thread_;
    ros_control_thread_ = nullptr;
  }
  controller_manager_.reset();
  hardware_interface_.reset();
  if (nullptr != robot_) {
    robot_->halt();
    delete robot_;
    robot_ = nullptr;
  }
}

void RosWrapper::goalCB(
    actionlib::ServerGoalHandle<
        control_msgs::FollowJointTrajectoryAction> gh) {
  std::string buf;
  LOG_INFO << "on_goal";
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal
    = *gh.getGoal(); //make a copy that we can modify
  if (has_goal_) {
    LOG_WARNING << "Received new goal while still executing previous trajectory. "
        << "Canceling previous trajectory";
    has_goal_ = false;
    robot_->stopTraj();
    result_.error_code = -100; // nothing is defined for this...?
    result_.error_string = "Received another trajectory";
    goal_handle_.setAborted(result_, result_.error_string);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  goal_handle_ = gh;
  if (!validateJointNames()) {
    std::string outp_joint_names = "";
    for (unsigned int i = 0; i < goal.trajectory.joint_names.size();
        i++) {
      outp_joint_names += goal.trajectory.joint_names[i] + " ";
    }
    result_.error_code = result_.INVALID_JOINTS;
    result_.error_string =
        "Received a goal with incorrect joint names: "
            + outp_joint_names;
    gh.setRejected(result_, result_.error_string);
    LOG_ERROR << result_.error_string;
    return;
  }
  if (!has_positions()) {
    result_.error_code = result_.INVALID_GOAL;
    result_.error_string = "Received a goal without positions";
    gh.setRejected(result_, result_.error_string);
    LOG_ERROR << result_.error_string;
    return;
  }

  if (!has_velocities()) {
    result_.error_code = result_.INVALID_GOAL;
    result_.error_string = "Received a goal without velocities";
    gh.setRejected(result_, result_.error_string);
    LOG_ERROR << result_.error_string;
    return;
  }

  if (!traj_is_finite()) {
    result_.error_string = "Received a goal with infinities or NaNs";
    result_.error_code = result_.INVALID_GOAL;
    gh.setRejected(result_, result_.error_string);
    LOG_ERROR << result_.error_string;
    return;
  }

  /* TODO 未实现
  if (!has_limited_velocities()) {
    result_.error_code = result_.INVALID_GOAL;
    result_.error_string =
        "Received a goal with velocities that are higher than "
            + std::to_string(max_velocity_);
    gh.setRejected(result_, result_.error_string);
    LOG_ERROR << result_.error_string;
    return;
  }
  */

  reorder_traj_joints(goal.trajectory);

  if (!start_positions_match(goal.trajectory, 0.01)) {
    result_.error_code = result_.INVALID_GOAL;
    result_.error_string = "Goal start doesn't match current pose";
    gh.setRejected(result_, result_.error_string);
    LOG_ERROR << result_.error_string;
    return;
  }

  std::vector<double> timestamps;
  std::vector<std::vector<double>> positions, velocities;
  if (goal.trajectory.points[0].time_from_start.toSec() != 0.) {
    LOG_WARNING << "Trajectory's first point should be the current position, "
        << "with time_from_start set to 0.0 - Inserting point in malformed trajectory";
    timestamps.push_back(0.0);
    robot_->getJointPositions(positions[0]);
    robot_->getJointVelocities(velocities[0]);
  }
  for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
    timestamps.push_back(
        goal.trajectory.points[i].time_from_start.toSec());
    positions.push_back(goal.trajectory.points[i].positions);
    velocities.push_back(goal.trajectory.points[i].velocities);
  }

  goal_handle_.setAccepted();
  has_goal_ = true;
  std::thread(&RosWrapper::trajThread, this, timestamps, positions,
      velocities).detach();
}

void RosWrapper::trajThread(std::vector<double> timestamps,
      std::vector<std::vector<double>> positions,
      std::vector<std::vector<double>> velocities) {
  robot_->doTraj(timestamps, positions, velocities);
  if (has_goal_) {
    result_.error_code = result_.SUCCESSFUL;
    goal_handle_.setSucceeded(result_);
    has_goal_ = false;
  }
}

void RosWrapper::cancelCB(
    actionlib::ServerGoalHandle<
        control_msgs::FollowJointTrajectoryAction> gh) {
  // set the action state to preempted
  LOG_INFO << "on_cancel";
  if (has_goal_) {
    if (gh == goal_handle_) {
      robot_->stopTraj();
      has_goal_ = false;
    }
  }
  result_.error_code = -100; //nothing is defined for this...?
  result_.error_string = "Goal cancelled by client";
  gh.setCanceled(result_);

  LOG_INFO << result_.error_string;
}

void RosWrapper::publishRTMsg() {
  ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  std::chrono::high_resolution_clock::time_point t0;
  std::chrono::milliseconds sleep_time;

  t0 = std::chrono::high_resolution_clock::now();
  while (alive_ && ros::ok()) {
    // TODO 待实现
    sensor_msgs::JointState joint_msg;
    robot_->getJointStates(joint_msg);
    joint_msg.header.stamp = ros::Time::now();
    joint_pub.publish(joint_msg);

    // 控制发布Message的频率
    sleep_time = rt_duration_ - std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - t0);
    if (sleep_time.count() > 0) {
      // keep real-time message frequency
      std::this_thread::sleep_for(sleep_time);
    }
    t0 = std::chrono::high_resolution_clock::now();
  }
}

void RosWrapper::rosControlLoop() {
  ros::Duration elapsed_time;
  struct timespec last_time, current_time;
  static const double BILLION = 1000000000.0;
  clock_gettime(CLOCK_MONOTONIC, &last_time);

  std::chrono::high_resolution_clock::time_point t0;
  std::chrono::milliseconds sleep_time;

  t0 = std::chrono::high_resolution_clock::now();
  while (alive_ && ros::ok()) {
    // Input
    hardware_interface_->read();

    // Control
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec)/ BILLION);
    controller_manager_->update(ros::Time::now(), elapsed_time);
    last_time = current_time;

    // Output
    hardware_interface_->write();

    // 控制发布循环的频率
    sleep_time = ros_ctrl_duration_ - std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - t0);
    // LOG_INFO << "sleep_time: " << sleep_time.count();
    if (sleep_time.count() > 0) {
      // keep real-time message frequency
      std::this_thread::sleep_for(sleep_time);
    }
    t0 = std::chrono::high_resolution_clock::now();
  }
}

RosWrapper::~RosWrapper() {
  halt();
  if (nullptr != robot_) {
    delete robot_;
    robot_ = nullptr;
  }
  /*if (nullptr != instance_) {
    delete instance_;
  }*/

  google::ShutdownGoogleLogging();
}

#ifdef DEBUG_TOPIC
void RosWrapper::cbForDebug(const std_msgs::Float64MultiArrayConstPtr& msg) {
  // 实现方式0
  /*LOG_INFO << "test write style 0";
  for (auto& jnt : robot_->jnt_names_) {
    Motor::CmdType cmd(msg->data, Motor::CmdType::MODE_POS_);
    robot_->addCommand(jnt, cmd);
  }*/
  // 实现方式1
  /*LOG_INFO << "test write style 1";
  for (auto& jnt : robot_->jnt_names_) {
    Motor::CmdTypeSp cmd(new Motor::CmdType(msg->data, Motor::CmdType::MODE_POS_));
    robot_->addCommand(jnt, cmd);
  }*/
  // 实现方式2
  LOG_INFO << "test write style 2";
  std::vector<HwCmdSp> cmd_vec;
  std::vector<std::string> cmd_name;
  int index = 0;
  for (auto& jnt : robot_->jnt_names_) {
    Motor::CmdTypeSp cmd(new Motor::CmdType(msg->data[index], Motor::CmdType::MODE_POS_));
    ++index;
    cmd_vec.push_back(cmd);
    cmd_name.push_back(jnt);
  }
  robot_->addCommand(cmd_name, cmd_vec);

  LOG_INFO << "Add Command Successful";
}
#endif

bool RosWrapper::validateJointNames() {
  std::vector<std::string> actual_joint_names;
  robot_->getJointNames(actual_joint_names);
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
      *goal_handle_.getGoal();
  if (goal.trajectory.joint_names.size() != actual_joint_names.size())
    return false;

  for (unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
    unsigned int j;
    for (j = 0; j < actual_joint_names.size(); j++) {
      if (goal.trajectory.joint_names[i] == actual_joint_names[j])
        break;
    }
    if (goal.trajectory.joint_names[i] == actual_joint_names[j]) {
      actual_joint_names.erase(actual_joint_names.begin() + j);
    } else {
      return false;
    }
  }

  return true;
}

bool RosWrapper::has_velocities() {
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
      *goal_handle_.getGoal();
  for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
    if (goal.trajectory.points[i].positions.size()
        != goal.trajectory.points[i].velocities.size())
      return false;
  }
  return true;
}

bool RosWrapper::has_positions() {
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
      *goal_handle_.getGoal();
  if (goal.trajectory.points.size() == 0)
    return false;
  for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
    if (goal.trajectory.points[i].positions.size()
        != goal.trajectory.joint_names.size())
      return false;
  }
  return true;
}

bool RosWrapper::traj_is_finite() {
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
      *goal_handle_.getGoal();
  for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
    for (unsigned int j = 0;
        j < goal.trajectory.points[i].velocities.size(); j++) {
      if (!std::isfinite(goal.trajectory.points[i].positions[j]))
        return false;
      if (!std::isfinite(goal.trajectory.points[i].velocities[j]))
        return false;
    }
  }
  return true;
}

void RosWrapper::reorder_traj_joints(trajectory_msgs::JointTrajectory& traj) {
  /* Reorders trajectory - destructive */
  std::vector<std::string> actual_joint_names;
  robot_->getJointNames(actual_joint_names);
  std::vector<unsigned int> mapping;
  mapping.resize(actual_joint_names.size(), actual_joint_names.size());
  for (unsigned int i = 0; i < traj.joint_names.size(); i++) {
    for (unsigned int j = 0; j < actual_joint_names.size(); j++) {
      if (traj.joint_names[i] == actual_joint_names[j])
        mapping[j] = i;
    }
  }
  traj.joint_names = actual_joint_names;
  std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
  for (unsigned int i = 0; i < traj.points.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint new_point;
    for (unsigned int j = 0; j < traj.points[i].positions.size(); j++) {
      new_point.positions.push_back(
          traj.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(
          traj.points[i].velocities[mapping[j]]);
      if (traj.points[i].accelerations.size() != 0)
        new_point.accelerations.push_back(
            traj.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = traj.points[i].time_from_start;
    new_traj.push_back(new_point);
  }
  traj.points = new_traj;
}

bool RosWrapper::start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps) {
  for (size_t i = 0; i < traj.points[0].positions.size(); i++) {
    std::vector<double> qActual;
    robot_->getJointPositions(qActual);
    if( fabs(traj.points[0].positions[i] - qActual[i]) > eps ) {
      return false;
    }
  }
  return true;
}

} /* namespace qr_driver */
