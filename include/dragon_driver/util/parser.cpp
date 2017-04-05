/*
 * robot_parser.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: silence
 */


#include "parser.h"
#include "log.h"

namespace middleware {

TiXmlElement*               Parser::xml_root_      = nullptr;
class_loader::ClassLoader*  Parser::propa_loader_  = nullptr;
class_loader::ClassLoader*  Parser::unit_loader_   = nullptr;

bool Parser::parser(const std::string& filename, Middleware* robot) {
  if (!Parser::init(filename)) {
    LOG_ERROR << "RobotParser has initialized fail";
    return false;
  }

  return (parserPropagates(robot) && parserJointStates(robot));
}

bool Parser::parser(Middleware* robot) {
  if (!Parser::init()) {
    LOG_ERROR << "RobotParser has initialized fail";
    return false;
  }

  return (parserPropagates(robot) && parserJointStates(robot));
}

/**
 * 完成解析函数MAP的初始化， 以及别的初始化工作
 */
bool Parser::init(const std::string& filename) {
  // 初始化XML文档相关内容
  TiXmlDocument* xml_doc = new TiXmlDocument();
  if (!xml_doc->LoadFile(filename)) {
    LOG_ERROR << "Could not found the "<< filename << ", did you forget define the file?";
    return false;
  }

  return initVariants(xml_doc);
}

bool Parser::init() {
  std::string configure;
  // if (!nh.getParam("configure", configure)) {
  if (!ros::param::get("~configure", configure)) {
    LOG(FATAL) << "No 'configure' parameter in ROS Parameter Server, "
        << "Did you forget define this parameter, in namespace ";
        //<< nh.getNamespace();
    return false;
  }
  // LOG_INFO << configure;
  // 初始化XML文档相关内容
  TiXmlDocument* xml_doc = new TiXmlDocument();
  xml_doc->Parse(configure.c_str());

  return initVariants(xml_doc);
}

bool Parser::initVariants(TiXmlDocument* xml_doc) {
  xml_root_ = xml_doc->RootElement();  // Robot
  // Initialize the class_loader
  TiXmlElement* lib_root = xml_root_->FirstChildElement("lib_paths");
  if (nullptr == lib_root) {
    LOG_ERROR << "No define the 'lib_paths' child element in configure";
    return false;
  }
  TiXmlElement* propa_lib = lib_root->FirstChildElement("propagate");
  TiXmlElement* unit_lib  = lib_root->FirstChildElement("hw_unit");
  if ((nullptr == propa_lib) || (nullptr == propa_lib->Attribute("path"))) {
    LOG_ERROR << "No define the lib paths of propagate";
    return false;
  } else {

    propa_loader_ = new class_loader::ClassLoader(propa_lib->Attribute("path"));
  }
  if ((nullptr == unit_lib) || (nullptr == unit_lib->Attribute("path"))) {
    LOG_ERROR << "No define the lib paths of unit";
    return false;
  } else {
    unit_loader_ = new class_loader::ClassLoader(unit_lib->Attribute("path"));
  }

  return true;
}

bool Parser::parserPropagates(Middleware* robot) {
  TiXmlElement* propagates = xml_root_->FirstChildElement("propagates");
  if (nullptr == propagates) {
    LOG(FATAL) << "No 'propagates' parameter in configure content, "
            << "Did you forget define this parameter";
    return false;
  }
  if (nullptr == propagates->Attribute("name")) {
    LOG_WARNING << "Could not found the 'name' attribute of the 'propagates' block, "
        << "using the default name 'propagates'";
    propagates->SetAttribute("name", "propagates");
  }

  LOG_INFO << "Assemble propagates: '" << propagates->Attribute("name") << "'";
  // Propagate* propa = new Propagate(propagates->Attribute("name"));
  robot->propagate_.reset(new Propagate(propagates->Attribute("name")));
  int counter = 0;
  for (auto c_root = propagates->FirstChildElement("channel");
      c_root != nullptr; c_root = c_root->NextSiblingElement("channel")) {
    if (nullptr == c_root->Attribute("name")) {
      std::stringstream ss;
      ss << "propagate_" << counter;
      LOG_WARNING << "Could not found the 'name' attribute of the 'propagates/channel' block, "
          << "using the default name '" << ss.str() << "'";
      c_root->SetAttribute("name", ss.str());
    }
    const char* type = c_root->Attribute("type");
    if (nullptr == type) {
      LOG_ERROR << "Could not found the 'type' attribute of "
          << c_root->Attribute("name")
          << " in the 'propagates/channel' block";
      continue;
    }
    boost::shared_ptr<Propagate> channel
        = propa_loader_->createInstance<Propagate>(type);
    channel->setName(c_root->Attribute("name"));

    LOG_INFO << "Push the " << counter + 1 << "st propagates('" << channel->getName()
        << "') into '" << robot->propagate_->getName() << "'";
    robot->propagate_->add(channel->getName(), channel);
    ++counter;
  }

  // robot->propagate_.reset(propa);
  return true;
}

/**
 * Parser Helper Method
 * parserJointStates(), parserJoint() and parserPropagates()
 */
bool Parser::parserJointStates(Middleware* robot) {
  if (nullptr == unit_loader_) {
    LOG_ERROR << "unit_loader is nullptr!";
    return false;
  }
  if (nullptr == robot->propagate_.get()) {
    LOG_ERROR << "The instance of Middleware is nullptr"
        << ", Is you call the parserJoints method before parserPropagates method?";
    return false;
  }

  // 寻找joint_states块， 并初始化HwUnit实例
  TiXmlElement* jnts_root = xml_root_->FirstChildElement("joint_states");
  if (nullptr == jnts_root) {
    LOG_ERROR << "Could not found the 'joint_states' block.";
    return false;
  }
  if (nullptr == jnts_root->Attribute("name")) {
    LOG_WARNING << "The 'joint_states' tag has no 'name' attribute, "
        << "we will use the default name: 'joint_states'";
    jnts_root->SetAttribute("name", "joint_states");
  }
  // fill the hw_unit, if parser fail, need to free.
  robot->hw_unit_.reset(new HwUnit(jnts_root->Attribute("name")));
  LOG_INFO << "Assemble hardware unit: '" << robot->hw_unit_->getName() << "'";
  for (auto jnt_tag = jnts_root->FirstChildElement("joint");
        nullptr != jnt_tag; jnt_tag = jnt_tag->NextSiblingElement()) {
    parserJoint(jnt_tag, robot);
  }

  int counter = 0;
  for (auto part = jnts_root->FirstChildElement("part");
        nullptr != part; part = part->NextSiblingElement("part")) {
    if (nullptr == part->Attribute("name")) {
      std::stringstream ss;
      ss << "part_" << counter;
      LOG_WARNING << "The 'joint_states/part' tag has no 'name' attribute, "
          << "we will use the default name: ''" << ss.str() << "'";
      jnts_root->SetAttribute("name", ss.str());
    }
    HwUnitSp part_unit(new HwUnit(jnts_root->Attribute("name")));
    for (auto jnt_tag = part->FirstChildElement("joint");
          nullptr != jnt_tag; jnt_tag = jnt_tag->NextSiblingElement()) {
      parserJoint(jnt_tag, robot, part_unit.get());
    }
    robot->hw_unit_->add(part_unit->getName(), part_unit);
    counter++;
  }

  return true;
}

bool Parser::parserJoint(TiXmlElement* jnt_root, Middleware* robot, HwUnit* parent) {
  // 解析出来的所有Actuator and Encoder都将要注册到通讯通道中
  PropaSp& robot_propa = robot->propagate_;
/*
  if (nullptr == jnt_root->Attribute("name")) {
    std::stringstream ss;
    ss << "joint_" << robot->jnt_names_.size();
    LOG_WARNING << "The joint tag has no 'name' attribute, "
        << "we will use the default name: \"" << ss.str() << "\"";
    jnt_root->SetAttribute("name", ss.str());
  }*/
  HwUnitSp joint_handle;
  if (nullptr == jnt_root->Attribute("type")) {
    joint_handle.reset(new HwUnit(jnt_root->Attribute("name")));
  } else {
    joint_handle = unit_loader_->createInstance<HwUnit>(jnt_root->Attribute("type"));
    joint_handle->init(jnt_root->FirstChildElement("parameter"));
  }

  LOG_INFO << "Assemble joint: \"" << joint_handle->getName() << "\"";
  // TODO 对type进行判断, 是否存在对应的类型
  int counter = 0;
  // 解析并构建Actuator
  std::string channel_act = "";
  for (auto xml_act = jnt_root->FirstChildElement("actuator");
      nullptr != xml_act; xml_act = xml_act->NextSiblingElement("actuator")) {
    if (nullptr != xml_act->Attribute("type")) {
      LOG_INFO << "Push the " << counter + 1
                << "st actuator of the \"" << joint_handle->getName() << "\" joint";

      boost::shared_ptr<HwUnit> act
        = unit_loader_->createInstance<HwUnit>(xml_act->Attribute("type"));
      act->init(xml_act->FirstChildElement("parameter"));

      // Register the handle into propagate and HwUint
      // std::string c = ""; // TODO using "" point no channel.
      if (nullptr != xml_act->Attribute("channel")) {
        channel_act = xml_act->Attribute("channel");
      }
      // 注册句柄到通信通道中
      // robot_propa->registerHandle(act->getName(), act->getStataHandle(), c);
      // robot_propa->registerHandle(act->getName(), act->getCommandHandle(), c);
      // 增加Actuaor到joint
      joint_handle->add(act->getName(), act);
    } else {
      LOG_ERROR << "Could not found the 'type' attribute of the "
          << counter << "st actuator of the \"" << joint_handle->getName() << "\" joint"
          << " in the 'joint_states/joint' block";
    }
    ++counter;
  }

  counter = 0;
  std::string channel_enc = "";
  for (auto xml_enc = jnt_root->FirstChildElement("encoder");
      nullptr != xml_enc; xml_enc = xml_enc->NextSiblingElement("encoder")) {
    if (nullptr != xml_enc->Attribute("type")) {
      LOG_INFO << "Push the " << counter + 1
                << "st encoder of the \"" << joint_handle->getName() << "\" joint";

      boost::shared_ptr<HwUnit> enc
        = unit_loader_->createInstance<HwUnit>(xml_enc->Attribute("type"));
      enc->init(xml_enc->FirstChildElement("parameter"));
      // Register the handle into propagate and HwUint
      // std::string c = ""; // TODO using "" point no channel.
      if (nullptr != xml_enc->Attribute("channel")) {
        channel_enc = xml_enc->Attribute("channel");
      }

      // robot_propa->registerHandle(enc->getName(), enc->getStataHandle(), c);
      // robot_propa->registerHandle(enc->getName(), enc->getCommandHandle(), c);

      joint_handle->add(enc->getName(), enc);
    } else {
      LOG_ERROR << "Could not found the 'type' attribute of the "
          << counter << "st actuator of the \"" << joint_handle->getName() << "\" joint"
          << " in the 'joint_states/joint' block";
    }
    ++counter;
  }
  //if (0 != channel_enc.compare(channel_act)) {
  //  LOG_WARNING << "The current version don't support the fact "
  //      << "that is different between actuator and encoder, "
  //      << "using the channel of actuator.";
  //}
  robot_propa->registerHandle(joint_handle->getName(),
      joint_handle->getCommandHandle(), channel_act);
  robot_propa->registerHandle(joint_handle->getName(),
      joint_handle->getStataHandle(), channel_enc);
  robot->jnt_names_.push_back(joint_handle->getName());
  if (nullptr == parent)
    robot->hw_unit_->add(joint_handle->getName(), joint_handle);
  else
    parent->add(joint_handle->getName(), joint_handle);

  return true;
}

} /* namespace qr_driver */
