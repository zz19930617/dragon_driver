/*
 * joint.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#include "joint.h"

namespace middleware {

Joint::Joint(const std::string& name): HwUnit(name) { }

Joint::~Joint() { }

HwStateSp Joint::getStataHandle() {
  return composite_[encoders_[0]]->getStataHandle();
}

HwCmdSp Joint::getCommandHandle() {
  return composite_[actuators_[0]]->getCommandHandle();
}

HwStateSp Joint::getState(const std::string& name) {
  if (0 == name_.compare(name))
    return composite_[encoders_[0]]->getState(encoders_[0]);

  if (composite_.end() == composite_.find(name)) {
    LOG_ERROR << "Can't found the handle ('" << name <<"' in the " << name_;
    return HwStateSp(nullptr);
  } else {
    return composite_[name]->getState(name);
  }
}

HwCmdSp Joint::getCommand(const std::string& name) {
  if (0 == name_.compare(name))
    return composite_[actuators_[0]]->getCommand(actuators_[0]);

  if (composite_.end() == composite_.find(name)) {
    LOG_ERROR << "Can't found the handle ('" << name <<"' in the " << name_;
    return nullptr;
  } else {
    return composite_[name]->getCommand(name);
  }
}

void Joint::setCommand(const std::string& name, const HwCommand& c) {
  if (0 == name_.compare(name)) {
    composite_[actuators_[0]]->setCommand(actuators_[0], c);
    return;
  }

  if (composite_.end() == composite_.find(name)) {
    LOG_ERROR << "Can't found the handle ('" << name <<"' in the " << name_;
  } else {
    composite_[name]->setCommand(name, c);
  }
}

void Joint::setState(const std::string&, const HwState&) {
  ;
}

bool Joint::init(TiXmlElement* para_root) {
  if (nullptr == para_root) {
    LOG_ERROR << "Can't found the 'parameter' TAG in the 'Joint' TAG";
    return false;
  }
  if (nullptr == para_root->Attribute("name"))
    LOG_WARNING << "Can't found the 'name' TAG in the 'parameter' TAG";
  else
    name_ = para_root->Attribute("name");

  std::stringstream ss;
  std::string compoent;
  if (nullptr == para_root->Attribute("actuators")) {
    LOG_ERROR << "Could not found the TAG('actuators') in the 'joint/parameters', "
        << "Did you forget define the file?";
  } else {
    ss << para_root->Attribute("actuators");
    while (ss >> compoent) {
      actuators_.push_back(compoent);
    }
  }

  if (nullptr == para_root->Attribute("encoders")) {
    LOG_ERROR << "Could not found the TAG('encoders') in the 'joint/parameters', "
        << "Did you forget define the file?";
  } else {
    ss.clear();
    ss << para_root->Attribute("encoders");
    while (ss >> compoent) {
      encoders_.push_back(compoent);
    }
  }

  return true;
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::Joint, middleware::HwUnit)
