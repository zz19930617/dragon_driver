/*
 * encoder.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include "encoder.h"

namespace middleware {

EncoderState::EncoderState(double pos, double vel, double ele_cur)
  : pos_(pos), vel_(vel),ele_current_(ele_cur),
    previous_time_(std::chrono::high_resolution_clock::now())
{ };

EncoderState::~EncoderState()
{ };

Encoder::Encoder(const std::string& name)
  : HwUnit(name), state_(new StateType)
{ };

Encoder::~Encoder()
{ };

bool Encoder::init(TiXmlElement* para) {
  if (nullptr == para->Attribute("name")) {
    LOG_ERROR << "Can't found the 'name' TAG in the 'parameter' TAG";
    return false;
  }
  name_ = para->Attribute("name");
  return true;
}


HwStateSp Encoder::getStataHandle() {
  return state_;
}

HwStateSp Encoder::getState(const std::string& name) {
  if (0 == name.compare(name_)) {
    return HwStateSp(new StateType(state_->pos_, state_->vel_, state_->ele_current_));
  } else {
    LOG_WARNING << "Requset the ERROR name state (actual vs request): ("
        << name_ << " vs " << name << ")";
    return HwStateSp(nullptr);
  }
}

void Encoder::check() {
  LOG_WARNING << "=============CHECK=============";
  LOG_INFO << "NAME: " << name_;
  LOG_INFO << "TYPE\tADDR\tCOUNT";
  LOG_INFO << "STATE\t" << state_.get() << "\t" << state_.use_count();
  LOG_WARNING << "===============================";
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::Encoder, middleware::HwUnit)
