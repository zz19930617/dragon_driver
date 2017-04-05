/*
 * hw_unit.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: silence
 */

#include "hw_unit.h"

namespace middleware {
/**
 * 默认实现返回nullptr
 */
HwStateSp HwUnit::getStataHandle() {
  return nullptr;
}

/**
 * 默认实现返回nullptr
 */
HwCmdSp HwUnit::getCommandHandle() {
  return nullptr;
}

/**
 * 获取名称为name的状态
 */
HwStateSp HwUnit::getState(const std::string& name) {
  HwStateSp ret(nullptr);
  auto hw = composite_.find(name);
  if (composite_.end() == hw) {
    for (auto& itr : composite_) {
      ret = itr.second->getState(name);
      if (nullptr != ret.get())
        break;
    }
    if (nullptr == ret.get())
      LOG_WARNING << "Could not found the haredware interface: " << name;
  } else {
    ret = hw->second->getState(name);
  }

  return ret;
}

/**
 * 获取名称为name的命令
 */
HwCmdSp HwUnit::getCommand(const std::string& name) {
  HwCmdSp ret(nullptr);
  auto hw = composite_.find(name);
  if (composite_.end() == hw) {
    for (auto& itr : composite_) {
      ret = itr.second->getCommand(name);
      if (nullptr != ret.get())
        break;
    }
    if (nullptr == ret.get())
      LOG_WARNING << "Could not found the haredware interface: " << name;
  } else {
    ret = hw->second->getCommand(name);
  }

  return ret;
}

/**
 * 设定名称为name的状态数据
 */
void HwUnit::setState(const std::string& name, const HwState& state) {
  auto hw = composite_.find(name);
  if (composite_.end() == hw) {
    for (auto& itr : composite_) {
      itr.second->setState(name, state);
    }
    //if (nullptr == ret.get())
    //  LOG_WARNING << "Could not found the haredware interface: " << name;
  } else {
    hw->second->setState(name, state);
  }
}

/**
 * 设定名称为name的命令数据
 */
void HwUnit::setCommand(const std::string& name, const HwCommand& cmd) {
  auto hw = composite_.find(name);
  if (composite_.end() == hw) {
    for (auto& itr : composite_) {
      itr.second->setCommand(name, cmd);
    }
    // LOG_WARNING << "Could not found the haredware interface: " << name;
  } else
    hw->second->setCommand(name, cmd);
}

/**************************************************
 * 若无特殊需要, 下列函数可以保持默认实现
 * 仅仅用作接口转发, 实质仍是调用的上述4个函数
 **************************************************/

/**
 * 设定名称为name的状态数据
 */
void HwUnit::setState(const std::string& name, const HwStateSp& state) {
  setState(name, *state);
}

/**
 * 设定名称为name的命令数据
 */
void HwUnit::setCommand(const std::string& name, const HwCmdSp& cmd) {
  setCommand(name, *cmd);
}

/**
 * 设定对应名称的对应的状态数据
 */
void HwUnit::setState(const std::vector<std::string>& names,
                        const std::vector<HwStateSp>& states) {
  assert(names.size() == states.size());

  for (size_t i = 0; i < names.size(); ++i) {
    setState(names[i], states[i]);
  }
}

/**
 * 设定对应名称的对应的命令数据
 */
void HwUnit::setCommand(const std::vector<std::string>& names,
                        const std::vector<HwCmdSp>& cmds) {
  assert(names.size() == cmds.size());

  for (size_t i = 0; i < names.size(); ++i) {
    setCommand(names[i], cmds[i]);
  }
}

/**
 * 设定对应名称的对应的状态数据
 */
void HwUnit::setState(const std::vector<std::string>& names,
                        const std::vector<HwState>& states) {
  assert(names.size() == states.size());

  for (size_t i = 0; i < names.size(); ++i) {
    setState(names[i], states[i]);
  }
}

/**
 * 设定对应名称的对应的命令数据
 */
void HwUnit::setCommand(const std::vector<std::string>& names,
                        const std::vector<HwCommand>& cmds) {
  assert(names.size() == cmds.size());

  for (size_t i = 0; i < names.size(); ++i) {
    setCommand(names[i], cmds[i]);
  }
}

HwUnit::HwUnit(const std::string& name)
    : name_(name) { };
HwUnit::~HwUnit() { };

bool HwUnit::init(TiXmlElement*) { return true; };

const std::string& HwUnit::getName() {
  return name_;
}

void HwUnit::setName(const std::string& name) {
  name_ = name;
}

void HwUnit::check() {
  //LOG_INFO << "NAME: " << name_;
  //LOG_INFO << "list: TYPE\tADDR\tCOUNT";
  for (auto& itr : composite_) {
    itr.second->check();
  }
}

} /* namespace quadruped_robot_driver */


