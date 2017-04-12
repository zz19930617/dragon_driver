/*
 * propagate_parser.cpp
 *
 *  Created on: 2017年1月17日
 *      Author: zhangzhi
 */

#include <dragon_driver/util/propagate_parser.h>

namespace middleware {

PropagateParser::PropagateParser() {
  // TODO Auto-generated constructor stub
  position_ = 0;
  last_position_ = 0;
  tmp_name_ = "left_front_hip_encoder";
}

PropagateParser::~PropagateParser() {
  // TODO Auto-generated destructor stub
}

bool PropagateParser::parsePcan(TPCANMsg& msg ,  Component<HwState>& state_composite){
  //DATA[0]确定四条腿
   switch(msg.DATA[0]) {
   case 0x62:
   case 0x42:{
     tmp_name_ = LEFT_FRONT;
     break;
   }
   case 0x63:
   case 0x43:{
     tmp_name_ = LEFT_BACK;
     break ;
   }
   case 0x64:
   case 0x44:{
     tmp_name_ = RIGHT_FRONT;
     break;
   }
   case 0x65:
   case 0x45:{
     tmp_name_ = RIGHT_BACK;
     break;
   }
   default:break;
   }

   //0x42~0x45表示位置信息， 0x64~0x67表示速度信息
   if(msg.DATA[0] <= 0x45 && msg.DATA[0] >= 0x42 ){
     dataType_ = "position";
   }
   else if(msg.DATA[0] >=0x62 && msg.DATA[0] <= 0x65){
     dataType_ = "velocity";
   }
   else{
     //LOG_WARNING << "msg type is wrong!";
   }
   //names_.push_back(tmp_name_ + "_knee_encoder");
   //names_.push_back(tmp_name_ + "_hip_encoder");
  // for(const std::string& name : names_){
     name = tmp_name_ + "_knee";
     auto itr = state_composite.find(name);
       if (state_composite.end() != itr){
       Encoder::StateTypeSp act_state
        = boost::dynamic_pointer_cast<Encoder::StateType>(itr->second);
       auto current_time = std::chrono::high_resolution_clock::now();
       if (std::string::npos != name.find(KNEE)){
         if ("position" == dataType_){
           memcpy(&position_ , msg.DATA+3, 2 * sizeof(position_));
         }else if ("velocity" == dataType_){
           memcpy(&velocity_ , msg.DATA+3, 2 * sizeof(velocity_));
         }
      } else {
        if ("position" == dataType_){
          memcpy(&position_, msg.DATA+5, 2*sizeof(position_));
        }else if ("velocity" == dataType_){
          memcpy(&velocity_, msg.DATA+5, 2*sizeof(velocity_));
        }
      }
       last_position_ = act_state->pos_;
       act_state->pos_ = (double) (position_)/10000.0;
       //act_state->vel_ = (act_state->pos_ - last_position_) * 1000 /std::chrono::duration_cast<std::chrono::duration<double>>
           //(current_time - act_state->previous_time_).count();
       act_state->vel_ = (double)(velocity_);
       act_state->previous_time_ = current_time;
     }


      name = tmp_name_ + "_hip";
      itr = state_composite.find(name);
       if (state_composite.end() != itr){
       Encoder::StateTypeSp act_state
        = boost::dynamic_pointer_cast<Encoder::StateType>(itr->second);
       auto current_time = std::chrono::high_resolution_clock::now();
       if (std::string::npos != name.find(KNEE)){
         if ("position" == dataType_){
           memcpy(&position_ , msg.DATA+3, 2 * sizeof(position_));
         }else if ("velocity" == dataType_){
           memcpy(&velocity_ , msg.DATA+3, 2 * sizeof(velocity_));
         }
      } else {
        if ("position" == dataType_){
          memcpy(&position_, msg.DATA+5, 2*sizeof(position_));
        }else if ("velocity" == dataType_){
          memcpy(&velocity_, msg.DATA+5, 2*sizeof(velocity_));
        }
      }
       last_position_ = act_state->pos_;
       act_state->pos_ = (double) (position_)/10000.0;
       //act_state->vel_ = (act_state->pos_ - last_position_) * 1000 /std::chrono::duration_cast<std::chrono::duration<double>>
           //(current_time - act_state->previous_time_).count();
       act_state->vel_ = (double)(velocity_)/10000.0;
       act_state->previous_time_ = current_time;
     }
 //} for
   return true;
}

TPCANMsg PropagateParser::packagePCAN(const std::string& name, Component<HwCommand>& cmd_composite){

  if (std::string::npos != name.find(LEFT_FRONT)) {
    msg_.ID = 0x02; //ID确定四条腿，0x02~0x05分别为左前、左后、右前、右后
  } else if (std::string::npos != name.find(LEFT_BACK)) {
    msg_.ID = 0x03;
  } else if (std::string::npos != name.find(RIGHT_FRONT)) {
    msg_.ID = 0x04;
  } else if (std::string::npos != name.find(RIGHT_BACK)) {
    msg_.ID = 0x05;
  } else{
    ;
  }

  msg_.LEN = 5; //LEN 确定数据长度
  msg_.MSGTYPE = PCAN_MESSAGE_STANDARD;  // MSGTYPE 确定数据类型
  msg_.DATA[1] = 0x11; //DATA[1]确定数据帧总数和帧位置
  msg_.DATA[2] = 0X88; // DATA[2]为预留位

  auto itr = cmd_composite.find(name);
  if (cmd_composite.end() != itr) {
    Motor::CmdTypeSp cmd = boost::dynamic_pointer_cast<Motor::CmdType>(itr->second);
    LOG_WARNING << "mode is " << cmd->mode_;
    if (std::string::npos != name.find(KNEE)){
      if (Motor::CmdType::MODE_POS_ == cmd->mode_){
        msg_.DATA[0] = 0x11; // DATA[0] 用于确定膝关节和髋关节
        position_ = 10000 * abs(cmd->command_);
        memcpy(msg_.DATA + 3 , &position_ , 2 * sizeof(msg_.DATA));
      } else if (Motor::CmdType::MODE_VEL_ == cmd->mode_){
        msg_.DATA[0] = 0x21; // DATA[0] 用于确定膝关节和髋关节
        velocity_ = 10000 * abs(cmd->command_);
        memcpy(msg_.DATA + 3 , &velocity_ , 2 * sizeof(msg_.DATA));
      }
    } else if (std::string::npos != name.find(HIP)) {
      if (Motor::CmdType::MODE_POS_ == cmd->mode_){
        msg_.DATA[0] = 0x12; // DATA[0] 用于确定膝关节和髋关节
        position_ = 10000 * abs(cmd->command_);
        memcpy(msg_.DATA + 3 , &position_ , 2 * sizeof(msg_.DATA));
      } else if (Motor::CmdType::MODE_VEL_ == cmd->mode_) {
        msg_.DATA[0] = 0x22; // DATA[0] 用于确定膝关节和髋关节
        velocity_ = 10000 * abs(cmd->command_);
        memcpy(msg_.DATA + 3 , &velocity_ , 2 * sizeof(msg_.DATA));
      }
    }
  }
  return msg_;
}

} /* namespace middleware */
