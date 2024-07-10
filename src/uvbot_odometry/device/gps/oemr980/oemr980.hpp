#ifndef __OEMR980_HPP_
#define __OEMR980_HPP_

#include "gps/gps.hpp"
#include <serial/serial.h>


/**
 * @brief   北斗定位模块 原本为TTL，组装成485通讯
*/
class OEMR980 : public GPS
{

public:
	OEMR980(serial::Serial *serialPtr);
	~OEMR980();

	
	void gps_data_solve(volatile const uint8_t *gps_frame);

private:
	serial::Serial *SerialPtr_;	//GPS串口指针
	
};

#endif /* __OEMR980_HPP_ */
