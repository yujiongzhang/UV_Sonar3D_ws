#ifndef _IMU_HPP_
#define _IMU_HPP_
#include "common.hpp"

/* ----------------------- Data Struct ------------------------------------- */
#pragma pack(push, 1)	

typedef  struct
{
	//TODO: 添加IMU数据结构体
	//加速度 单位：g(9.8m/s)
	float Acc_x;
	float Acc_y;
	float Acc_z;
	//角度 单位：rad
	float Angle_x;
	float Angle_y;
	float Angle_z;
	//角速度 单位：rad/s
	float Gyrol_x;
	float Gyrol_y;
	float Gyrol_z;
	//四元数
	float quaternion[4];
} imu_data_t;

#pragma pack(pop)
/* ----------------------- extern Function ----------------------------------- */


/**
 * @brief   IMU 基类
*/
class IMU
{
public:
	IMU();
	virtual ~IMU() {}
	/**
	 * @brief          返回IMU数据指针
	*/
	imu_data_t get_imu_data(void) const
	{
		return imu_data;
	}
	/**
	 * @brief          IMU协议解析
	 * @param[in]      imu_frame: 原生数据指针
	 * @return  	   none
	*/
	virtual void imu_data_solve(volatile const uint8_t *imu_frame) = 0;

	
protected:
	imu_data_t imu_data;
};

#endif /* _IMU_HPP_ */
