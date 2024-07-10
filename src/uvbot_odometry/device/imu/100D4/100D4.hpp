#ifndef __100D4_HPP__
#define __100D4_HPP__

#include "imu/imu.hpp"
#include <serial/serial.h>


/**
 * @brief   北京三驰 IMU 具体型号：
*/
class SC100D4 : public IMU
{
public:
	SC100D4(serial::Serial *serialPtr);
	~SC100D4() = default;
	/**
	 * @brief          IMU协议解析
	 * @param[in]      imu_frame: 原生数据指针
	 * @return  	   none
	*/
	void imu_data_solve(volatile const uint8_t *imu_frame);
	/**
     * @brief          IMU属性设置
     * @param[in]      imu_cmd_id 命令标识
     * @retval         none
     */
    void set_imu_cmd(uint16_t imu_cmd_id);
	/**
	 * @brief          IMU数据校验
	 * @param[in]      imu_frame: 原生数据指针
	 * @return  	   bool
	*/
	bool imu_check(uint8_t *data_message);
    /**
     * @brief          设置磁修正
     * @param[in]      isMagnetic 是否开启磁修正
     * @retval         none
    */
    void set_magnetic(bool isMagnetic);
    /**
     * @brief          设置IMU状态
     * @param[in]      status 是否启动命令
     * @retval         none
    */
    void set_status(bool status);
public:
    static constexpr auto IMU_HEAD_SOF = 0xA55A; //帧头
	static constexpr auto IMU_DATA_LENGTH = 40; //IMU回传数据大小
    static constexpr auto START_CMD_ID                    = 0x01;
    static constexpr auto STOP_CMD_ID                	    = 0x02;
    static constexpr auto CALIBRATION_CMD_ID              = 0xE3;
    static constexpr auto SAVE_CALIBRATION_CMD_ID         = 0xE1;
    static constexpr auto OPEN_MAGNETIC_CORRECT_CMD_ID   	= 0xE2;
    static constexpr auto CLOSE_MAGNETIC_CORRECT_CMD_ID  	= 0xE4;

private:
	serial::Serial *SerialPtr_;	//IMU串口指针
};

#endif /* __100D4_HPP__ */
