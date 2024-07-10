/** 
 *****************************Copyright (c) 2023  ZJU****************************
 * @file      : isa500.cpp
 * @brief     : ISA500高度计驱动文件
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2023-08-14         WPJ        1. <note>
 *******************************************************************************
 * @verbatim :
 *==============================================================================
 * 高度计数据读取通过上位机设置为定时发送
 * 指令ID122
 * 返回数据帧格式为：ddd.ddd\r\n (直接是字符串形式ASCII码)                                        
 *==============================================================================
 * @endverbatim :
 *****************************Copyright (c) 2023  ZJU****************************
 */


#include "altimeter/isa500/isa500.hpp"
#include <assert.h>
/**
 * @brief ISA500构造函数
*/
ISA500::ISA500(serial::Serial *serialPtr)
{
    assert(serialPtr != nullptr);
    this->SerialPtr_ = serialPtr;
}


/**
  * @brief          高度计协议解析
  * @param[in]      am_frame: 原生数据指针
  * @param[in]     am_data: 高度计数据指针
  * @retval         none
  */
void ISA500::altimeter_data_solve(volatile const uint8_t *am_frame)
{
	/*数据校验*/
    if (am_frame == NULL)
    {
        return;
    }
    //TODO:进行高度计数据解析
    this->altimeter_data = atof((char *)am_frame);

}