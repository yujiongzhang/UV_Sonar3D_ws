#ifndef __BEWIS_HPP_
#define __BEWIS_HPP_

#include "imu/imu.hpp"
#include <serial/serial.h>


/**
 * @brief   北微传感 IMU 具体型号：
*/
class BEWIS : public IMU
{
public:
	BEWIS(serial::Serial *serialPtr);
	~BEWIS() = default;
	/**
	 * @brief          IMU协议解析
	 * @param[in]      imu_frame: 原生数据指针
	 * @return  	   none
	*/
	void imu_data_solve(volatile const uint8_t *imu_frame);
	/**
	 * @brief          发送内容打包
	 * @param[in]      cmd_type:  命令内容ID
	 * @param[in]      p_data:  数据指针
	 * @param[in]      len:  数据长度
	 * @return  	   返回要发送的数据大小
	*/
	uint16_t send_pack(uint8_t cmd_type, uint8_t *p_data, uint16_t len);
	/**
	 * @brief          IMU数据校验
	 * @param[in]      imu_frame: 原生数据指针
	 * @return  	   bool
	*/
	bool imu_check(uint8_t *data_message);

public:
	static constexpr auto IMU_HEAD_SOF = 0x77; //帧头
	static constexpr auto IMU_DATA_LENGTH = 57; //IMU回传数据大小

private:
	serial::Serial *SerialPtr_;	//IMU串口指针
};

#endif /* __BEWIS_HPP_ */