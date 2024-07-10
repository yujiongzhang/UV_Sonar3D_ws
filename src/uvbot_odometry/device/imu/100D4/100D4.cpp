/** 
 *****************************Copyright (c) 2023  ZJU****************************
 * @file      : 100D4.cpp
 * @brief     : 北京三驰100D4设备驱动
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2023-10-12       Hao Lion      1. <note>
 *******************************************************************************
 * @verbatim :
 *==============================================================================
 *                                                                              
 *                                                                              
 *==============================================================================
 * @endverbatim :
 *****************************Copyright (c) 2023  ZJU****************************
 */

#include "imu/100D4/100D4.hpp"
#include <assert.h>
#include <tf2/LinearMath/Quaternion.h>

#define g 9.8f
#define Pi 3.1415926f

SC100D4::SC100D4(serial::Serial *serialPtr)
{
  assert(serialPtr != nullptr);
  SerialPtr_ = serialPtr;
}


/**
 * @brief          SC100D4 IMU 协议解析
 * @param[in]      imu_frame: 原生数据指针
 * @return  	   none
*/
void SC100D4::imu_data_solve(volatile const uint8_t *imu_frame)
{
  uint8_t* imu_frame_temp = (uint8_t*)imu_frame;

  /*数据校验*/
  if (imu_frame_temp == NULL)
  {
      return;
  }
  if(imu_frame_temp[0] == 0xA5 && imu_frame_temp[1] == 0x5A && imu_frame_temp[39] == 0xAA)
  {
        /*数据解析*/
        float scale = Pi / 180.0f;
        imu_data.Angle_z = (int16_t)(((imu_frame_temp[3]<<8) + imu_frame_temp[4]))*0.1*scale;						//俯仰角，单位为rad
        imu_data.Angle_y = (int16_t)(((imu_frame_temp[5]<<8) + imu_frame_temp[6]))*0.1*scale;						//偏航角
        imu_data.Angle_x = (int16_t)(((imu_frame_temp[7]<<8) + imu_frame_temp[8]))*0.1*scale;						//横滚角
        imu_data.Acc_x = (int16_t)(((imu_frame_temp[9]<<8) + imu_frame_temp[10]))/16384 * g;						//X方向加速度，单位为g
        imu_data.Acc_y = (int16_t)(((imu_frame_temp[11]<<8) + imu_frame_temp[12]))/16384 * g;					//y方向加速度
        imu_data.Acc_z = (int16_t)(((imu_frame_temp[13]<<8) + imu_frame_temp[14]))/16384 * g;					//z方向加速度
        imu_data.Gyrol_x = (int16_t)(((imu_frame_temp[15]<<8) + imu_frame_temp[16]))/32.8;						//X方向角速度，单位为°/s
        imu_data.Gyrol_y = (int16_t)(((imu_frame_temp[17]<<8) + imu_frame_temp[18]))/32.8;						//y方向角速度
        imu_data.Gyrol_z = (int16_t)(((imu_frame_temp[19]<<8) + imu_frame_temp[20]))/32.8;						//z方向角速度

        tf2::Quaternion orientation;
        orientation.setRPY(imu_data.Angle_x, imu_data.Angle_y, imu_data.Angle_z);

        this->imu_data.quaternion[0] = orientation.w();
        this->imu_data.quaternion[1] = orientation.x();
        this->imu_data.quaternion[2] = orientation.y();
        this->imu_data.quaternion[3] = orientation.z();
  }
}


bool SC100D4::imu_check(uint8_t *data_message)
{
    (void)data_message;
    return true;
}

/**
 * @brief          IMU属性设置
 * @param[in]      imu_cmd_id 命令标识
*/
void SC100D4::set_imu_cmd(uint16_t imu_cmd_id)
{
    uint8_t cmd[6];
    cmd[0] = IMU_HEAD_SOF>>8;
    cmd[1] = IMU_HEAD_SOF&0xff;
    cmd[2]	= 0x04;
    cmd[3] = imu_cmd_id;
    cmd[4] = imu_cmd_id+0x04;
    cmd[5] = 0xAA;
    SerialPtr_->write(cmd, sizeof(cmd));
}

/**
 * @brief          设置磁修正
 * @param[in]      isMagnetic 是否开启磁修正
 * @retval         none
*/
void SC100D4::set_magnetic(bool isMagnetic)
{
    if(isMagnetic)
        set_imu_cmd(OPEN_MAGNETIC_CORRECT_CMD_ID);
    else
        set_imu_cmd(CLOSE_MAGNETIC_CORRECT_CMD_ID);
}

/**
 * @brief          设置IMU状态
 * @param[in]      status 是否启动命令
 * @retval         none
*/
void SC100D4::set_status(bool status)
{
    if(status)
    {
        set_imu_cmd(START_CMD_ID);
    }
    else
    {
        set_imu_cmd(STOP_CMD_ID);
    }
}