/*
 * propagate_parser.h
 *
 *  Created on: 2017年1月17日
 *      Author: zhangzhi
 */

#ifndef INCLUDE_DRAGON_DRIVER_UTIL_PROPAGATE_PARSER_H_
#define INCLUDE_DRAGON_DRIVER_UTIL_PROPAGATE_PARSER_H_

#define linux
#include <PCANBasic.h>
#include "../propagate/propagate.h"
#include "../hardware/motor.h"
#include "../hardware/encoder.h"

#define KNEE "knee"
#define HIP "hip"
#define YAW "yaw"
#define LEFT_FRONT "left_front"
#define LEFT_BACK "left_back"
#define RIGHT_FRONT "right_front"
#define RIGHT_BACK "right_back"

namespace middleware {

class PropagateParser {
public:
  PropagateParser();
  virtual ~PropagateParser();
  bool parsePcan(TPCANMsg& ,  Component<HwState>&);
  TPCANMsg packagePCAN(const std::string& , Component<HwCommand>&);

private:
  TPCANMsg msg_;
  short position_;
  short velocity_;
  short ele_current_;
  double last_position_;
  double last_velocity_;
  double last_ele_current_;
  std::vector <std::string> names_;
  std::string leg_name_;
  std::string joint_name_;
  std::string name;
  std::string dataType_ ;
};

} /* namespace middleware */

#endif /* INCLUDE_DRAGON_DRIVER_UTIL_PROPAGATE_PARSER_H_ */
