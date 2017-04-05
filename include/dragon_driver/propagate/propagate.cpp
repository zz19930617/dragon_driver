/*
 * propagate.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include "propagate.h"

namespace middleware {

Propagate::Propagate(const std::string& name)
  : name_(name), connected_(false)
{ }

Propagate::~Propagate()
{ }

const std::string& Propagate::getName() {
  return name_;
}

void Propagate::setName(const std::string& name) {
  name_ = name;
}

/**
 * 每一个注册的通讯通道都初始化成功， 才返回true
 */
bool Propagate::init() {
  connected_ = true;
  for (auto& channel : composite_) {
    connected_ = (connected_ && channel.second->init());
  }
  return connected_;
}

/**
 * names是将要写入的HwUnit名称向量
 * names可能包含多种通信通道
 * 当且仅当names中存在CMD名称是属于本imp子类所管理， 并且写入失败时，
 * 返回false, 其他情况均返回true
 */
bool Propagate::write(const std::vector<std::string>& names) {
  connected_ = false;
  for (auto& channel : composite_) {
    connected_ = (connected_ || channel.second->write(names));
  }
  return connected_;
}

/**
 * 所有通信通道均read正常， 方才返回true
 */
bool Propagate::read() {
  for (auto& imp : composite_) {
    connected_ = connected_ && imp.second->read();
  }
  return connected_;
}

void Propagate::stop() {
  for (auto& imp : composite_) {
    imp.second->stop();
  }
}

/**
 * 注册HwUnit名称为name的命令句柄, 强制使用智能指针
 */
void Propagate::registerHandle(const std::string& name, HwCmdSp cmd, const std::string& channel) {
  if (channel.empty()) {
    LOG_INFO << "Register the COMMAND handle of " << name << " into cmd_composite successful!";
    cmd_composite_.add(name, cmd);
  } else {
    if (composite_.end() == composite_.find(channel)) {
      LOG_ERROR << "The channel '" << channel << "' does exist "
          << "in the " << name_ << ", We will register into " << name_;
      cmd_composite_.add(name, cmd);
    } else {
      composite_[channel]->registerHandle(name, cmd);
    }
  }
}

/**
 * 注册HwUnit名称为name的状态句柄, 强制使用智能指针
 */
void Propagate::registerHandle(const std::string& name, HwStateSp state, const std::string& channel) {
  if (channel.empty()) {
    LOG_INFO << "Register the STATE handle of " << name << " into state_composite successful!";
    state_composite_.add(name, state);
  } else {
    if (composite_.end() == composite_.find(channel)) {
      LOG_ERROR << "The channel '" << channel << "' does exist "
          << "in the " << name_ << ", We will register into " << name_;
      state_composite_.add(name, state);
    } else {
      composite_[channel]->registerHandle(name, state);
    }
  }
}

void Propagate::check() {
  for (auto& imp : composite_) {
    imp.second->check();
  }
}

} /* namespace middleware */
