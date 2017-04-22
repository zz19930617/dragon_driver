/*
 * propagate_imp_pcan.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#include "pcan.h"

namespace middleware {

PcanChannel::PcanChannel(const std::string& name)
  :Propagate(name),
    pcan_status_(0x00),
    position_(0),
    propagate_parser(new PropagateParser())
{ }

PcanChannel::~PcanChannel() { }

// 完成PCAN的初始化
// 以及act_state_map_, act_cmd_map_, enc_state_map_三个MAP的从cmd_map_和state_map_中初始化
bool PcanChannel::init() {
  pcan_status_ = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_500K, 0, 0, 0);
  //propagate_parser = new PropagateParser();
  if (PCAN_ERROR_OK != pcan_status_){
    LOG(ERROR) << "Initialize CAN FAIL: "<< pcan_status_;
  }
  return true;
}

void PcanChannel::stop() {
  return;
}

// 完成数据的读写. (下面全是测试代码)
bool PcanChannel::write(const std::vector<std::string>& names) {
  if (names.empty()) return true;

  for(const std::string& name : names) {
      msg_ = propagate_parser->packagePCAN(name, cmd_composite_);
      pcan_status_=CAN_Write(PCAN_USBBUS1, &msg_);
      printf("leg is 0x%02x ", msg_.ID);
      printf("write_msg is: 0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",
       msg_.DATA[0],msg_.DATA[1],msg_.DATA[2],msg_.DATA[3],msg_.DATA[4],
       msg_.DATA[5],msg_.DATA[6]);
      if (PCAN_ERROR_OK != pcan_status_) {
        LOG(ERROR) << "PCAN_WRITE IS FALSE !";
      }
    }
  return true;
}

// (下面全是测试代码)
bool PcanChannel::read() {
  // 从PCAN中获取到的数据对应到具体的状态name
  // 也可以从PCAN的数据中， 明确到底是什么类型的State
  // 转化为对应类型的State, 在进行赋值
  // LOG_INFO << "PCAN read: ";
  while ((pcan_status_=CAN_Read(PCAN_USBBUS1, &rec_msg_, NULL)) == PCAN_ERROR_QRCVEMPTY){
    usleep(1000);
  }
  //LOG_INFO<<"PCAN READ OK";
  //if (rec_msg_.DATA[0] == 0x62){
    //printf("msg_ID is : 0x%02x ", rec_msg_.ID);
    //printf("read_msg is: 0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",
   //rec_msg_.DATA[0],rec_msg_.DATA[1],rec_msg_.DATA[2],rec_msg_.DATA[3],
   //rec_msg_.DATA[4],rec_msg_.DATA[5],rec_msg_.DATA[6]);
  //}
  return propagate_parser->parsePcan(rec_msg_ , state_composite_);
}

void PcanChannel::check() {
  LOG_WARNING << "=============CHECK=============";
  LOG_INFO << "NAME: " << name_;
  LOG_WARNING << "-------------------------------";
  LOG_INFO << "STATE:";
  LOG_INFO << "NAME\tADDR\tCOUNT";
  for (auto& s : state_composite_) {
    LOG_INFO << s.first << "\t" << s.second.get()
        << "\t" << s.second.use_count();
  }
  LOG_WARNING << "-------------------------------";
  LOG_INFO << "COMMAND:";
  LOG_INFO << "NAME\tADDR\tCOUNT";
  for (auto& c : cmd_composite_) {
    LOG_INFO << c.first << "\t" << c.second.get() << "\t" << c.second.use_count();
  }
  LOG_WARNING << "===============================";
}

} /* namespace qr_driver */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::PcanChannel, middleware::Propagate)
