/*
 * propagate.h
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_
#define INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "../hardware/hw_unit.h"
#include "../util/component.h"

namespace middleware {

class Propagate : public Component<Propagate> {
public:
  Propagate(const std::string&);
  virtual ~Propagate();

  virtual bool init();
  virtual bool write(const std::vector<std::string>&);
  virtual bool read();
  virtual void stop();
  // for Debug
  virtual void check();

public:
  const std::string& getName();
  void setName(const std::string& name);
  /**************************************************
   * 下述两个函数注册机构单元的状态或命令句柄
   * 注册句柄, 是为了效率考虑, 注册后, 解析更新后的数据
   * 直接可以在本类中进行更新. 第一个参数为机构单元的名称
   * 应小心使用下述四个函数, 必须传入对应机构单元的状态或命令实际句柄
   * 否则将会发生不能更新数据或断错误
   * 参数1: 指定注册句柄的关节名称
   * 参数2: 指定注册句柄
   * 参数3: 可选, 指定注册的通信通道
   **************************************************/
  void registerHandle(const std::string&, HwCmdSp, const std::string& channel = "");
  void registerHandle(const std::string&, HwStateSp, const std::string& channel = "");

protected:
  Component<HwCommand>  cmd_composite_;
  Component<HwState>    state_composite_;

  std::string name_;
  bool        connected_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_ */
