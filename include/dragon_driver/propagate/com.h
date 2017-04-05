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
#include "propagate.h"

#include <map>

namespace middleware {

class ComChannel: public Propagate {
public:
  ComChannel(const std::string& name = "com");
  virtual ~ComChannel();

  virtual bool init();
  virtual void stop();

  // 完成数据的读写.
  virtual bool write(const std::vector<std::string>&);
  virtual bool read();
};

} /* namespace qr_driver */

#endif /* INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_ */
