/*
 * joint.h
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_

#include "hw_unit.h"
#include "encoder.h"
#include "motor.h"

namespace middleware {

class Joint: public HwUnit {
public:
  typedef Motor::CmdType                CmdType;
  typedef Motor::CmdTypeSp              CmdTypeSp;
  typedef Encoder::StateType            StateType;
  typedef Encoder::StateTypeSp          StateTypeSp;

public:
  Joint(const std::string& name = "joint");
  virtual ~Joint();

  virtual bool init(TiXmlElement*) override;

  virtual HwStateSp getStataHandle() override;
  virtual HwCmdSp getCommandHandle() override;

  virtual HwStateSp getState(const std::string&) override;
  virtual HwCmdSp getCommand(const std::string&) override;
  virtual void setState(const std::string&, const HwState&) override;
  virtual void setCommand(const std::string&, const HwCommand&) override;

protected:
  std::vector<std::string> actuators_;
  std::vector<std::string> encoders_;
  // boost::shared_ptr<Encoder> encoder_;
  // boost::shared_ptr<Motor>   motor_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_ */
