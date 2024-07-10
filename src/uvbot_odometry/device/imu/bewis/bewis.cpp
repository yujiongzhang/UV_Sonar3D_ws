/** 
 *****************************Copyright (c) 2023  ZJU****************************
 * @file      : bewis.cpp
 * @brief     : 北微IMU设备驱动
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2023-10-12       Yujio Zhang     1. <note>
 *******************************************************************************
 * @verbatim :
 *==============================================================================
 *                                                                              
 *                                                                              
 *==============================================================================
 * @endverbatim :
 *****************************Copyright (c) 2023  ZJU****************************
 */

#include "imu/bewis/bewis.hpp"
#include <assert.h>
#include <tf2/LinearMath/Quaternion.h>

#define g 9.8f
#define Pi 3.1415926f

BEWIS::BEWIS(serial::Serial *serialPtr)
{
    assert(serialPtr != nullptr);
    SerialPtr_ = serialPtr;
}


/**
 * @brief          北微传感 IMU 协议解析。需要根据接收命令字来判断是哪种数据。
 * @param[in]      imu_frame: 原生数据指针
 * @return  	   none
*/
void BEWIS::imu_data_solve(volatile const uint8_t *imu_frame)
{
    uint8_t* imu_frame_temp = (uint8_t*)imu_frame;

    /*数据校验*/
    if (imu_frame_temp == NULL)
    {
        return;
    }
    //TODO:自动输出模式
    //发送 77 05 00 56 05 60 为【角度、加速度、 陀螺仪、 四元数输出（BCD 码）】
    //发送 77 05 00 56 06 61 为【角度、加速度、 陀螺仪（FLOAT 类型输出）】 建议用这个！！！！
    if (imu_frame_temp[0] == 0x77 && imu_frame_temp[1]== 0x38) //&& imu_check(imu_frame_temp)
    {
        float scale = Pi / 180.0f;
        //原始输出为deg
        this->imu_data.Angle_y = BCD_SXXXYY_to_Float(imu_frame_temp+4) * scale;
        this->imu_data.Angle_x = BCD_SXXXYY_to_Float(imu_frame_temp+7) * scale;
        this->imu_data.Angle_z = BCD_SXXXYY_to_Float(imu_frame_temp+10) * scale;

        this->imu_data.Acc_x = BCD_SXYYYY_to_Float(imu_frame_temp+13) * g;
        this->imu_data.Acc_y = BCD_SXYYYY_to_Float(imu_frame_temp+16) * g;
        this->imu_data.Acc_z = BCD_SXYYYY_to_Float(imu_frame_temp+19) * g;

        this->imu_data.Gyrol_x = BCD_SXXXYY_to_Float(imu_frame_temp+22) * scale;
        this->imu_data.Gyrol_y = BCD_SXXXYY_to_Float(imu_frame_temp+25) * scale;
        this->imu_data.Gyrol_z = BCD_SXXXYY_to_Float(imu_frame_temp+28) * scale;

        tf2::Quaternion orientation;
        orientation.setRPY(imu_data.Angle_x, imu_data.Angle_y, imu_data.Angle_z);

        this->imu_data.quaternion[0] = orientation.w();
        this->imu_data.quaternion[1] = orientation.x();
        this->imu_data.quaternion[2] = orientation.y();
        this->imu_data.quaternion[3] = orientation.z();

    }

}

bool BEWIS::imu_check(uint8_t *data_message)
{
    uint8_t temp = 0;
    uint8_t len = data_message[1];//北微的数据长度，不包括第一个字节标示符
    data_message = data_message + 1;//北微的校验和不包括第一个字节标示符
    len = len - 1;//先长度减一：最后一位为校验标志
    while (len)
    {
    temp = temp + *data_message;
    data_message++;
    len--;
    }
    temp = temp & 0xff;
    if(*data_message == temp){
    return 1;
    }
    else{return 0;}
}