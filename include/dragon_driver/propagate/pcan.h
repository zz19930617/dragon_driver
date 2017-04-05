/*
 * propagate_imp_pcan.h
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#ifndef INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_
#define INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_

#include "../hardware/encoder.h"
#include "../hardware/motor.h"
#include "../util/propagate_parser.h"
#include "propagate.h"
#include <map>
#include <stdio.h>
#include <unistd.h>
#include <asm/types.h>


#define linux

#include <PCANBasic.h>

namespace middleware {

class PcanChannel: public Propagate {
public:
  PcanChannel(const std::string& name = "pcan");
  virtual ~PcanChannel();

  virtual bool init() override;
  virtual bool write(const std::vector<std::string>&) override;
  virtual bool read() override;
  virtual void stop() override;

  virtual void check() override;

private:
  TPCANMsg msg_;
  TPCANMsg rec_msg_;
  TPCANStatus pcan_status_;
  short position_; //记录pcan收到的位置信息
  PropagateParser* propagate_parser;
};

} /* namespace qr_driver */

#endif /* INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_ */
