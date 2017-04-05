/*
 * propagate_imp_pcan.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#include "com.h"

namespace middleware {

ComChannel::ComChannel(const std::string& name)
  :Propagate(name)
{ }

ComChannel::~ComChannel() { }

// 完成PCAN的初始化
// 以及act_state_map_, act_cmd_map_, enc_state_map_三个MAP的从cmd_map_和state_map_中初始化
bool ComChannel::init() {
  return true;
}

void ComChannel::stop() {
  return;
}

// 完成数据的读写. (下面全是测试代码)
bool ComChannel::write(const std::vector<std::string>& names) {
  if (names.empty()) return true;

  // LOG_INFO << "PCAN write: ";
  for (const std::string& name : names) {
    auto itr = cmd_composite_.find(name);
    if (cmd_composite_.end() != itr) {
      Motor::CmdTypeSp cmd = boost::dynamic_pointer_cast<Motor::CmdType>(itr->second);
      // Motor::CmdTypeSp cmd = boost::dynamic_pointer_cast<Motor::CmdType>(itr->second);
      LOG_INFO << "command: " << cmd->command_
          << " mode: " << cmd->mode_;

      if (0 == name.compare("hip_motor")) {
        std::string enc_name = "hip_encoder";
        auto itr_state = state_composite_.find(enc_name);
        Encoder::StateTypeSp act_state
          = boost::dynamic_pointer_cast<Encoder::StateType>(itr_state->second);

        double current_pos = cmd->command_;
        auto current_time = std::chrono::high_resolution_clock::now();
        act_state->vel_ = (current_pos - act_state->pos_)
            / std::chrono::duration_cast<std::chrono::duration<double>>(
                current_time - act_state->previous_time_).count();
        act_state->pos_ = current_pos;
        act_state->previous_time_ = current_time;

      } else if (0 == name.compare("knee_motor")) {
        std::string enc_name = "knee_encoder";
        auto itr_state = state_composite_.find(enc_name);
        Encoder::StateTypeSp act_state
          = boost::dynamic_pointer_cast<Encoder::StateType>(itr_state->second);

        double current_pos = cmd->command_;
        auto current_time = std::chrono::high_resolution_clock::now();
        act_state->vel_ = (current_pos - act_state->pos_)
            / std::chrono::duration_cast<std::chrono::duration<double>>(
                current_time - act_state->previous_time_).count();
        act_state->pos_ = current_pos;
        act_state->previous_time_ = current_time;
      } else {
        ; // Nothing to de here
      }
    } else {
      LOG_WARNING << "Could not found the " << name << " command handle";
    }
  }
  return true;
}

// (下面全是测试代码)
bool ComChannel::read() {
  // 从PCAN中获取到的数据对应到具体的状态name
  // 也可以从PCAN的数据中， 明确到底是什么类型的State
  // 转化为对应类型的State, 在进行赋值
  std::string name = "knee_encoder";
  auto itr = state_composite_.find(name);
  if (state_composite_.end() == itr) {
    LOG_WARNING << "Could not found the " << name << " state handle: ";
  } else {
    Encoder::StateTypeSp act_state
      = boost::dynamic_pointer_cast<Encoder::StateType>(itr->second);
    double current_pos = act_state->pos_ + 0.00001;
    auto current_time = std::chrono::high_resolution_clock::now();
    act_state->vel_ = (current_pos - act_state->pos_)
        / std::chrono::duration_cast<std::chrono::duration<double>>(
            current_time - act_state->previous_time_).count();
    act_state->pos_ = current_pos;
    act_state->previous_time_ = current_time;
  }

  name = "hip_encoder";
  itr = state_composite_.find(name);
  if (state_composite_.end() == itr) {
    LOG_WARNING << "Could not found the " << name << " state handle: ";
  } else {
    Encoder::StateTypeSp act_state
      = boost::dynamic_pointer_cast<Encoder::StateType>(itr->second);
    double current_pos = act_state->pos_ + 0.0000001;
    auto current_time = std::chrono::high_resolution_clock::now();
    act_state->vel_ = (current_pos - act_state->pos_)
        / std::chrono::duration_cast<std::chrono::duration<double>>(
            current_time - act_state->previous_time_).count();
    act_state->pos_ = current_pos;
    act_state->previous_time_ = current_time;
  }

  return true;
}

} /* namespace qr_driver */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::ComChannel, middleware::Propagate)
