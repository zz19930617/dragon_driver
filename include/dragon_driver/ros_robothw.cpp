/*
 * ros_robothw.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: silence
 */

#include "ros_robothw.h"
#include "util/log.h"

namespace middleware {

RosRobotHW::~RosRobotHW()
{ }

RosRobotHW::RosRobotHW(ros::NodeHandle& nh, Middleware* robot)
  : nh_(nh), robot_(robot),
    joint_names_(robot->jnt_names_),
    num_joints_(joint_names_.size()) {
  init();
  LOG_INFO << "Loaded QrHardwareInterface";
}

/// \brief Initialize the hardware interface
void RosRobotHW::init() {
  if (joint_names_.empty()) {
    LOG_FATAL <<
        "No joints found on parameter server for controller, "
        << "did you load the proper yaml file?";
    exit(-1);
  }
  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);
  prev_joint_velocity_command_.resize(num_joints_);

  // Initialize controller
  for (std::size_t i = 0; i < num_joints_; ++i) {
    std::string joint_name = joint_names_[i];
    LOG(INFO) << "QrHardwareInterface Loading joint name: "
        << joint_name;

    // Create joint state interface
    joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(joint_name,
            &joint_position_[i], &joint_velocity_[i],
            &joint_effort_[i]));

    // Create position joint interface
    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(
            joint_state_interface_.getHandle(joint_name),
            &joint_position_command_[i]));

    // Create velocity joint interface
    velocity_joint_interface_.registerHandle(
        hardware_interface::JointHandle(
            joint_state_interface_.getHandle(joint_name),
            &joint_velocity_command_[i]));

    // Create effort joint interface
    effort_joint_interface_.registerHandle(
        hardware_interface::JointHandle(
            joint_state_interface_.getHandle(joint_name),
            &joint_effort_command_[i]));
    prev_joint_velocity_command_[i] = 0.;
  }

  registerInterface(&joint_state_interface_); // From RobotHW base class.
  registerInterface(&position_joint_interface_); // From RobotHW base class.
  registerInterface(&velocity_joint_interface_); // From RobotHW base class.
  registerInterface(&effort_joint_interface_); // From RobotHW base class.
  velocity_interface_running_ = false;
  position_interface_running_ = false;
  effort_interface_running_ = false;
}

/// \brief Read the state from the robot hardware.
void RosRobotHW::read() {
  sensor_msgs::JointState msg;
  robot_->getJointStates(msg);
  for (std::size_t idx_msg = 0; idx_msg < msg.name.size(); ++idx_msg) {
    for (std::size_t idx_jnt = 0; idx_jnt < num_joints_; ++idx_jnt) {
      if (0 == msg.name[idx_msg].compare(joint_names_[idx_jnt])) {
        joint_position_[idx_jnt] = msg.position[idx_msg];
        joint_velocity_[idx_jnt] = msg.velocity[idx_msg];
        joint_effort_[idx_jnt] = msg.effort[idx_msg];
        break;
      }
    }
  }
}

/// \brief write the command to the robot hardware.
void RosRobotHW::write() {
  // TODO
  if (velocity_interface_running_) {
    robot_->executeJointPositions(joint_names_, joint_velocity_command_);
  } else if (position_interface_running_) {
    robot_->executeJointPositions(joint_names_, joint_position_command_);
  } else if (effort_interface_running_) {
    LOG(WARNING) << "NO IMPLEMENTES"; // Nothing to do here
  } else {
    ; // Nothing to do here
  }
}

bool RosRobotHW::canSwitch(
      const std::list<hardware_interface::ControllerInfo> &start_list,
      const std::list<hardware_interface::ControllerInfo> &stop_list) const {
  for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
      start_list.begin(); controller_it != start_list.end();
      ++controller_it) {
    if (0 == controller_it->hardware_interface.compare(
        "hardware_interface::VelocityJointInterface")) {
      if (velocity_interface_running_) {
        LOG(ERROR) << controller_it->name.c_str()
            << ": An interface of that type ("
            << controller_it->hardware_interface.c_str()
            << ") is already running";
        return false;
      }
      if (position_interface_running_) {
        bool error = true;
        for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
            stop_list.begin();
            stop_controller_it != stop_list.end();
            ++stop_controller_it) {
          if (0 == stop_controller_it->hardware_interface.compare(
              "hardware_interface::PositionJointInterface")) {
            error = false;
            break;
          }
        }
        if (error) {
          LOG(ERROR) << controller_it->name.c_str()
              << " (type " << controller_it->hardware_interface.c_str()
              << ") can not be run simultaneously with a PositionJointInterface";
          return false;
        }
      }
      if (effort_interface_running_) {
        bool error = true;
        for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
            stop_list.begin();
            stop_controller_it != stop_list.end();
            ++stop_controller_it) {
          if (0 == stop_controller_it->hardware_interface.compare(
              "hardware_interface::EffortJointInterface")) {
            error = false;
            break;
          }
        }
        if (error) {
          LOG(ERROR) << controller_it->name.c_str()
              << " (type " << controller_it->hardware_interface.c_str()
              << ") can not be run simultaneously with a EffortJointInterface";
          return false;
        }
      }
    } else if (0 == controller_it->hardware_interface.compare(
        "hardware_interface::PositionJointInterface")) {
      if (position_interface_running_) {
        LOG(ERROR) << "%s: An interface of that type (%s) is already running"
            << controller_it->name.c_str()
            << controller_it->hardware_interface.c_str();
        return false;
      }
      if (velocity_interface_running_) {
        bool error = true;
        for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
            stop_list.begin();
            stop_controller_it != stop_list.end();
            ++stop_controller_it) {
          if (0 == stop_controller_it->hardware_interface.compare(
              "hardware_interface::VelocityJointInterface")) {
            error = false;
            break;
          }
        }
        if (error) {
          LOG(ERROR) << "%s (type %s) can not be run simultaneously with a VelocityJointInterface"
              << controller_it->name.c_str()
              << controller_it->hardware_interface.c_str();
          return false;
        }
      }
      if (effort_interface_running_) {
          bool error = true;
          for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
              stop_list.begin();
              stop_controller_it != stop_list.end();
              ++stop_controller_it) {
            if (0 == stop_controller_it->hardware_interface.compare(
                "hardware_interface::EffortJointInterface")) {
              error = false;
              break;
            }
          }
          if (error) {
            LOG(ERROR) << controller_it->name.c_str() << " (type "
                << controller_it->hardware_interface.c_str()
                << ") can not be run simultaneously with a VelocityJointInterface";
            return false;
          }
        }
    } else if (0 == controller_it->hardware_interface.compare(
        "hardware_interface::EffortJointInterface")) {
      if (effort_interface_running_) {
        LOG(ERROR) << controller_it->name.c_str()
            << ": An interface of that type ("
            << controller_it->hardware_interface.c_str()
            << ") is already running";
        return false;
      }
      if (velocity_interface_running_) {
        bool error = true;
        for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
            stop_list.begin();
            stop_controller_it != stop_list.end();
            ++stop_controller_it) {
          if (0 == stop_controller_it->hardware_interface.compare(
              "hardware_interface::VelocityJointInterface")) {
            error = false;
            break;
          }
        }
        if (error) {
          LOG(ERROR) << controller_it->name.c_str()
                    << " (type " << controller_it->hardware_interface.c_str()
                    << ") can not be run simultaneously with a VelocityJointInterface";
          return false;
        }
      }
      if (position_interface_running_) {
          bool error = true;
          for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
              stop_list.begin();
              stop_controller_it != stop_list.end();
              ++stop_controller_it) {
            if (0 == stop_controller_it->hardware_interface.compare(
                "hardware_interface::PositionJointInterface")) {
              error = false;
              break;
            }
          }
          if (error) {
            LOG(ERROR) << controller_it->name.c_str()
                << " (type " << controller_it->hardware_interface.c_str()
                << ") can not be run simultaneously with a PositionJointInterface";
            return false;
          }
        }
    }
  }

  // we can always stop a controller
  return true;
}

void RosRobotHW::doSwitch(const std::list<hardware_interface::ControllerInfo>&start_list,
    const std::list<hardware_interface::ControllerInfo>&stop_list) {
  for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
      stop_list.begin(); controller_it != stop_list.end();
      ++controller_it) {
    if (0 == controller_it->hardware_interface.compare(
        "hardware_interface::VelocityJointInterface")) {
      velocity_interface_running_ = false;
      LOG(INFO) << ("Stopping velocity interface");
    }
    if (0 == controller_it->hardware_interface.compare(
        "hardware_interface::PositionJointInterface")) {
      position_interface_running_ = false;
      // std::vector<double> tmp;
      // robot_->closeServo(tmp);
      LOG(INFO) << ("Stopping position interface");
    }
    if (0 == controller_it->hardware_interface.compare(
        "hardware_interface::EffortJointInterface")) {
      effort_interface_running_ = false;
      // std::vector<double> tmp;
      // robot_->closeServo(tmp);
      LOG(INFO) << ("Stopping position interface");
    }
  }
  for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
      start_list.begin(); controller_it != start_list.end();
      ++controller_it) {
    if (0 == controller_it->hardware_interface.compare(
        "hardware_interface::VelocityJointInterface")) {
      velocity_interface_running_ = true;
      LOG(INFO) << ("Starting velocity interface");
    }
    if (0 == controller_it->hardware_interface.compare(
        "hardware_interface::PositionJointInterface")) {
      position_interface_running_ = true;
      // robot_->uploadProg();
      LOG(INFO) << ("Starting position interface");
    }
    if (0 == controller_it->hardware_interface.compare(
        "hardware_interface::EffortJointInterface")) {
      effort_interface_running_ = true;
      // robot_->uploadProg();
      LOG(INFO) << ("Starting effort interface");
    }
  }
}

} /* namespace middleware */
