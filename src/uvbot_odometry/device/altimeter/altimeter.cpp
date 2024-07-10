/** 
 *****************************Copyright (c) 2023  ZJU****************************
 * @file      : altimeter.cpp
 * @brief     : 高度计设备基类
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2023-08-14         WPJ        1. <note>
 *******************************************************************************
 * @verbatim :
 *==============================================================================
 * 
 * 
 *                                        
 *==============================================================================
 * @endverbatim :
 *****************************Copyright (c) 2023  ZJU****************************
 */


#include "altimeter/altimeter.hpp"

/**
 * @brief Altimeter基类的构造函数，初始化Altimeter_data
 * @retval none
*/
Altimeter::Altimeter(){
    memset(&altimeter_data,0,sizeof(altimeter_data));
}


