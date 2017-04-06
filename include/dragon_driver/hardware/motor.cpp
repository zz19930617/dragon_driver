/*
 * actuator.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include "motor.h"

namespace middleware {

MotorState::MotorState(double pos, double vel, double tor)
    : pos_(pos), vel_(vel), tor_(tor)
{ }

MotorState::~MotorState()
{ }

MotorCmd::MotorCmd(double cmd, MODE_ mode)
    : command_(cmd), mode_(mode)
{ }

MotorCmd::~MotorCmd()
{ }


Motor::Motor(const std::string&  name, CmdType::MODE_ mode)
    : HwUnit(name),
      motor_state_(new StateType()),
      motor_cmd_(new CmdType(0, mode))
{ }

Motor::~Motor()
{ }

bool Motor::init(TiXmlElement* para) {
  if (nullptr == para->Attribute("name")) {
    LOG_ERROR << "Can't found the 'name' TAG in the 'parameter' TAG";
    return false;
  }
  name_ = para->Attribute("name");
  std::string mode_str = "";
  CmdType::MODE_ mode = CmdType::MODE_::MODE_POS_;
  if (nullptr != para->Attribute("mode")) {
    mode_str = para->Attribute("mode");
  }
  if (0 == mode_str.compare("velocity")) {
    mode = CmdType::MODE_::MODE_VEL_;
  } else if (0 == mode_str.compare("torque")) {
    mode = CmdType::MODE_::MODE_TOR_;
  } else {
    ;
  }
  motor_cmd_->mode_ = mode;

  double val = 0;
  if (nullptr != para->Attribute("value")) {
    std::stringstream ss;
    ss << para->Attribute("value");
    ss >> val;
  }
  motor_cmd_->command_ = val;

  return true;
}

HwStateSp Motor::getStataHandle() {
  return motor_state_;
}

HwCmdSp Motor::getCommandHandle() {
  return motor_cmd_;
}

HwStateSp Motor::getState(const std::string& name) {
  if (0 != name_.compare(name)) {
    LOG_WARNING << "Requset the ERROR name state (actual vs request): ("
        << name_ << " vs " << name << ")";
    return HwStateSp(nullptr);
  } else {
    return HwStateSp(new StateType(
        motor_state_->pos_, motor_state_->vel_, motor_state_->tor_));
  }
}

HwCmdSp Motor::getCommand(const std::string& name) {
  if (0 != name_.compare(name)) {
    LOG_WARNING << "Requset the ERROR name command (actual vs request): ("
        << name_ << " vs " << name << ")";
    return HwCmdSp(nullptr);
  } else {
    return HwCmdSp(new CmdType(motor_cmd_->command_, motor_cmd_->mode_));
  }
}

void Motor::setState(const std::string& name, const HwState& state) {
  if (0 != name_.compare(name)) return;

  const StateType& motor_state = dynamic_cast<const StateType&>(state);
  double val = motor_state.pos_;
  motor_state_->pos_ = val;
  val = motor_state.vel_;
  motor_state_->vel_ = val;
  val = motor_state.tor_;
  motor_state_->tor_ = val;
}

void Motor::setCommand(const std::string& name, const HwCommand& cmd) {
  if (0 != name_.compare(name)) return;

  const CmdType& motor_cmd = dynamic_cast<const CmdType&>(cmd);
  CmdType::MODE_ mode = motor_cmd.mode_;
  motor_cmd_->mode_ = mode;
  double val = motor_cmd.command_;
  motor_cmd_->command_ = val;
}

void Motor::check() {
  LOG_WARNING << "=============CHECK=============";
  LOG_INFO << "NAME: " << name_;
  // LOG_WARNING << "\t-------------------------------";
  LOG_INFO << "TYPE\tADDR\tCOUNT";
  LOG_INFO << "STATE\t" << motor_state_.get() << "\t" << motor_state_.use_count();
  LOG_INFO << "COMMAND\t" << motor_cmd_.get() << "\t" << motor_cmd_.use_count();
  LOG_WARNING << "===============================";
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::Motor, middleware::HwUnit)
