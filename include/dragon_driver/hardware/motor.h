/*
 * actuator.h
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_MOTOR_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_MOTOR_H_

#include <atomic>

#include "hw_unit.h"

namespace middleware {

/*
 * The actual motor state
 */
struct MotorState : public HwState {
  std::atomic<double> pos_;
  std::atomic<double> vel_;
  std::atomic<double> tor_;

  MotorState(double pos = 0, double vel = 0, double tor = 0);
  virtual ~MotorState();
};

/*
 * The motor command
 */
struct MotorCmd : public HwCommand {
  typedef enum {MODE_POS_ = 0, MODE_VEL_, MODE_TOR_} MODE_;

  std::atomic<double> command_;
  std::atomic<MODE_>  mode_;

  MotorCmd(double cmd = 0, MODE_ = MODE_TOR_);
  virtual ~MotorCmd();
};

/*
 * The actuator state and command handle
 */
class Motor : public HwUnit {
public:
  typedef MotorCmd    CmdType;
  typedef MotorState  StateType;
  typedef boost::shared_ptr<MotorCmd>   CmdTypeSp;
  typedef boost::shared_ptr<MotorState> StateTypeSp;

  Motor(const std::string& name = "motor", CmdType::MODE_ mode = CmdType::MODE_TOR_);
  virtual ~Motor();

  virtual bool init(TiXmlElement*) override;
  // for Debug
  virtual void check() override;

  virtual HwStateSp getStataHandle() override;
  virtual HwCmdSp getCommandHandle() override;

  // 该函数子类选择性进行实现, 在函数内部, 需要完成数据的读写.
  virtual HwStateSp getState(const std::string& name) override;
  virtual HwCmdSp getCommand(const std::string& name) override;
  virtual void setState(const std::string& name, const HwState& state) override;
  virtual void setCommand(const std::string& name, const HwCommand& state) override;

private:
  StateTypeSp motor_state_;
  CmdTypeSp   motor_cmd_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_MOTOR_H_ */
