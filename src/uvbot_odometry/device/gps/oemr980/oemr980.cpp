/** 
 *****************************Copyright (c) 2023  ZJU****************************
 * @file      : oemr980.cpp
 * @brief     : oemr980北斗GPS设备驱动
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2023-08-12        Hao Lion        1. <note>
 *******************************************************************************
 * @verbatim :
 *==============================================================================
 *                                                                              
 *                                                                              
 *==============================================================================
 * @endverbatim :
 *****************************Copyright (c) 2023  ZJU****************************
 */

#include "gps/oemr980/oemr980.hpp"
#include <cstring>
#include <math.h>
#include <assert.h>


OEMR980::OEMR980(serial::Serial *serialPtr)
{
    assert(serialPtr != nullptr);
    SerialPtr_ = serialPtr;
}

OEMR980::~OEMR980()
{
}

/**
 * $GNGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,M,<10>,M,<11>,<12>*xx<CR><LF>
 * $GNGGA：起始引导符及语句格式说明(本句为GPS定位数据)；
 * <1> UTC 时间，格式为hhmmss.sss；
 * <2> 纬度，格式为ddmm.mmmm(第一位是零也将传送)；
 * <3> 纬度半球，N 或S(北纬或南纬)
 * <4> 经度，格式为dddmm.mmmm(第一位零也将传送)；
 * <5> 经度半球，E 或W(东经或西经)
 * <6> 定位质量指示，0=定位无效，1=定位有效；
 * <7>使用卫星数量，从00到12(第一个零也将传送)
 * <8>水平精确度，0.5到99.9
 * <9>天线离海平面的高度，-9999.9到9999.9米M指单位米
 * <10>大地水准面高度，-9999.9到9999.9米M指单位米
 * <11>差分GPS数据期限(RTCMSC-104)，最后设立RTCM传送的秒数量
 * <12>差分参考基站标号，从0000到1023(首位0也将传送)。
*/
void OEMR980::gps_data_solve(volatile const uint8_t *gps_frame)
{
    char* gps_frame_temp = (char*)gps_frame;

    char* start;
    char  array[20][14]; // 存储信息
    char *field;
    int index = 0;

    start = strstr(gps_frame_temp,"GNGGA");
 
    if (start)
    {
        field = strtok(start,",");
        while (field)
        {
            strcpy(array[index],field);
            field = strtok(NULL,",");
            index++;
        }
    }
    else{
        //结束
    }

    int gps_status = atoi(array[6]);//是否可信位

    if (gps_status)
    {
        int8_t longitude_sign,latitude_sign;
        double longtitude_d = atof(array[4]);
        double latitude_d = atof(array[2]);
        longitude_sign = array[3][0] == 'E' ? 1 : -1;
        latitude_sign = array[5][0] == 'N' ? 1 : -1;
        gps_data.longitude = (int)((longtitude_d / 100) + fmod(longtitude_d, 100) / 60.0) * longitude_sign;
        gps_data.latitude = (int)((latitude_d / 100) + fmod(latitude_d, 100) / 60.0) * latitude_sign;
    }
    else{
        //结束
    }
    
}