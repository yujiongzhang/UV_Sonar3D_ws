/** 
 *****************************Copyright (c) 2023  ZJU****************************
 * @file      : lpms.cpp
 * @brief     : 阿鲁比IMU设备驱动
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2023-08-12         WPJ        1. <note>
 *******************************************************************************
 * @verbatim :
 *==============================================================================
 *                                                                              
 *                                                                              
 *==============================================================================
 * @endverbatim :
 *****************************Copyright (c) 2023  ZJU****************************
 */

#include "imu/lpms/lpms.hpp"
#include <assert.h>

#define g 9.8f
#define Pi 3.1415926f

LPMS::LPMS(serial::Serial *serialPtr)
{
  assert(serialPtr != nullptr);
  SerialPtr_ = serialPtr;
  memset(imu_tx_buf, 0, sizeof(imu_tx_buf));
}


/**
* @brief     发送内容打包
* @param[in] cmd_type:  命令内容ID
* @retval			返回要发送的数据大小
*/
uint16_t LPMS::send_pack(uint8_t cmd_type, uint8_t *p_data, uint16_t len)
{

  memset(imu_tx_buf, 0, sizeof(imu_tx_buf));
  uint16_t send_len = 9 + len;
  imu_tx_buf[0] = IMU_HEAD_SOF;
  imu_tx_buf[1] = IMU_ID & 0xFF;
  imu_tx_buf[2] = IMU_ID >> 8;
  imu_tx_buf[3] = cmd_type & 0xFF;
  imu_tx_buf[4] = cmd_type;
  imu_tx_buf[5] = len & 0xFF;
  imu_tx_buf[6] = len >> 8;
  if(len == 0)
  {
    imu_tx_buf[7] = get_LRC_sum(imu_tx_buf, send_len-4) & 0xFF;
    imu_tx_buf[8] = get_LRC_sum(imu_tx_buf, send_len-4) >> 8;
    send_len = send_len + 2;
  }
  else
  {
    memcpy(&imu_tx_buf[7], p_data, len);
  }
  imu_tx_buf[send_len-4] = get_LRC_sum(imu_tx_buf, send_len-4) & 0xFF;
  imu_tx_buf[send_len-3] = get_LRC_sum(imu_tx_buf, send_len-4) >> 8;
  imu_tx_buf[send_len-2] = IMU_END_SOF & 0xFF;
  imu_tx_buf[send_len-1] = IMU_END_SOF >> 8;
  SerialPtr_->write(imu_tx_buf, send_len);
  return send_len;
}

/**
 * @brief 进入IMU命令模式
 * @retval none
*/
void LPMS::GOTO_Command_Mode(void)
{ 
  send_pack(Goto_Command_Mode_CMD, NULL, 0);
}

/**
 * @brief 进入IMU数据流发送
 * @retval none
*/
void LPMS::GOTO_Streaming_Mode(void)
{ 
  send_pack(Goto_Streaming_Mode_CMD, NULL, 0);
}

/**
 * @brief IMU参数保存
 * @retval none
*/
void LPMS::GOTO_Save_Setting(void)
{ 
  send_pack(Save_Setting_CMD, NULL, 0);
}



/**
 * @brief          阿鲁比 IMU 协议解析
 * @param[in]      imu_frame: 原生数据指针
 * @return  	   none
*/
void LPMS::imu_data_solve(volatile const uint8_t *imu_frame)
{
  uint8_t* imu_frame_temp = (uint8_t*)imu_frame;
  /*数据校验*/
  if (imu_frame_temp == NULL)
  {
      return;
  }
  //TODO:进行IMU数据解析，校验和不包括包头3A
  if (imu_frame_temp[0] == 0x3A && imu_frame_temp[1] == 0x01 && imu_frame_temp[2] == 0x00 ) //&& LRC_check(imu_frame_temp+1, IMU_DATA_LENGTH)
  {
    //转换到m/s²
    this->imu_data.Acc_x = Byte_to_Float(&imu_frame_temp[23]) * g;
    this->imu_data.Acc_y = Byte_to_Float(&imu_frame_temp[27]) * g;
    this->imu_data.Acc_z = Byte_to_Float(&imu_frame_temp[31]) * g;
    //转换dps到rad/s
    float scale = Pi / 180.0f;
    this->imu_data.Gyrol_x = Byte_to_Float(&imu_frame_temp[83]) * scale;
    this->imu_data.Gyrol_y = Byte_to_Float(&imu_frame_temp[87]) * scale;
    this->imu_data.Gyrol_z = Byte_to_Float(&imu_frame_temp[91]) * scale;

    //四元数
    this->imu_data.quaternion[0] = Byte_to_Float(&imu_frame_temp[131]);
    this->imu_data.quaternion[1] = Byte_to_Float(&imu_frame_temp[135]);
    this->imu_data.quaternion[2] = Byte_to_Float(&imu_frame_temp[139]);
    this->imu_data.quaternion[3] = Byte_to_Float(&imu_frame_temp[143]);

    this->imu_data.Angle_x = Byte_to_Float(&imu_frame_temp[147]) * scale;
    this->imu_data.Angle_y = Byte_to_Float(&imu_frame_temp[151]) * scale;
    this->imu_data.Angle_z = Byte_to_Float(&imu_frame_temp[155]) * scale;
  }
}