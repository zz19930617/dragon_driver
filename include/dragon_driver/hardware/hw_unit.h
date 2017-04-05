/*
 * robot_state_base.h
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#ifndef ROBOT_STATE_BASE_H_
#define ROBOT_STATE_BASE_H_

#include <map>
#include <string>
#include <tinyxml.h>
#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>

#include "../util/component.h"
#include "../util/log.h"

namespace middleware {
// This is a parameter entry. Note that the arguments should match the enum.
typedef boost::variant<bool, char, unsigned char, int, double,
    std::string, std::vector<std::string>> __Value;
// This is the options map.
typedef std::map<std::string, __Value> _OptionMap;

struct HwState {
  HwState() { };
  virtual ~HwState() { };
};
struct HwCommand {
  HwCommand() { };
  virtual ~HwCommand() { };
};

// 前向申明
class HwUnit;
typedef boost::shared_ptr<HwState>    HwStateSp;
typedef boost::shared_ptr<HwCommand>  HwCmdSp;
typedef boost::shared_ptr<HwUnit>     HwUnitSp;

class HwUnit : public Component<HwUnit> {
protected:
  // 关于本类的一些附加信息, 可以在init()中初始化.
  _OptionMap  opts_;
  std::string name_;
  // std::string propagate_channel_;
public:
  /**************************************************
   * 任何该类的子类, 若包含State or Cmd
   * 都必须重定义下述四个typedef
   **************************************************/
  typedef HwCommand                     CmdType;
  typedef HwState                       StateType;
  typedef boost::shared_ptr<HwState>    StateTypeSp;
  typedef boost::shared_ptr<HwCommand>  CmdTypeSp;

  HwUnit(const std::string& name);
  virtual ~HwUnit();

  const std::string& getName();
  void setName(const std::string& name);

  /**
   * 初始化本类对象, 使用xml文件中内容
   * 基类默认实现为空
   */
  virtual bool init(TiXmlElement*);
  // for Debug
  virtual void check();
  /**
   * 子类必须实现下述虚函数
   * 返回所保存状态/命令数据的地址
   * 用以在初始化时注册到Propagate中
   */
  virtual HwStateSp getStataHandle();
  virtual HwCmdSp getCommandHandle();
/**************************************************
 * 下述四个函数选择性进行实现, 在函数内部, 需要完成数据的读写.
 * 若HwUnit子类具备State or Command
 * 则必须重写对应的get/set函数实现
 * get函数, 以名称为标识符, get对应名称的对象,
 * 为了防止使用后不手动释放内存, 强制使用智能指针
 * set函数, 也以名称为标识符, 设定对应的数据
 **************************************************/
  virtual HwStateSp getState(const std::string&);
  virtual HwCmdSp getCommand(const std::string&);
  virtual void setState(const std::string&, const HwState&);
  virtual void setCommand(const std::string&, const HwCommand&);

public:
  /**************************************************
   * 若无特殊需要, 下列函数可以保持默认实现
   * 仅仅用作接口转发, 实质仍是调用的上述4个函数
   **************************************************/
  void setState(const std::string&, const HwStateSp&);
  void setState(const std::vector<std::string>&, const std::vector<HwStateSp>&);
  void setState(const std::vector<std::string>&, const std::vector<HwState>&);
  void setCommand(const std::string&, const HwCmdSp&);
  void setCommand(const std::vector<std::string>&, const std::vector<HwCmdSp>&);
  void setCommand(const std::vector<std::string>&, const std::vector<HwCommand>&);
};

} /* namespace quadruped_robot_driver */

#endif /* ROBOT_STATE_BASE_H_ */
