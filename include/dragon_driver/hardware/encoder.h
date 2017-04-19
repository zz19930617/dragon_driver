/*
 * encoder.h
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#ifndef INCLUDE_JOINT_ENCODER_H_
#define INCLUDE_JOINT_ENCODER_H_

#include <atomic>
#include <chrono>

#include "hw_unit.h"

namespace middleware {

struct EncoderState : public HwState {
  // 实际获取的数据
  std::atomic<double> pos_;

  // 需要propagate实例中， 通过软件代码计算出速度填入数据
  std::atomic<double> vel_;
  // 计算速度的辅助变量, 保存前一次更新的时间
  std::chrono::high_resolution_clock::time_point previous_time_;
  //保存电源的电流值
  std::atomic<double> ele_current_;
  EncoderState(double pos = 0, double vel = 0 , double ele_current_ = 0);
  ~EncoderState();
};

class Encoder : public HwUnit {
public:
  typedef EncoderState StateType;
  typedef boost::shared_ptr<EncoderState> StateTypeSp;

  Encoder(const std::string& name = "encoder");
  virtual ~Encoder();

  virtual bool init(TiXmlElement*) override;
  // for debug
  virtual void check() override;

  virtual HwStateSp getStataHandle() override;
  virtual HwStateSp getState(const std::string& name) override;

private:
  StateTypeSp state_;
};

} /* namespace qr_driver */

#endif /* INCLUDE_JOINT_ENCODER_H_ */
