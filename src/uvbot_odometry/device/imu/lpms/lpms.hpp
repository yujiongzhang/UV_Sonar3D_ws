#ifndef __LPMS_HPP__
#define __LPMS_HPP__

#include "imu/imu.hpp"
#include <serial/serial.h>

/**
 * @brief   阿鲁比 IMU 具体型号：LPMS-IG1 mini
*/
class LPMS : public IMU
{
public:
	LPMS(serial::Serial *serialPtr);
	~LPMS() = default;

	/**
	 * @brief          初始化硬件
	 * @param[in]      none
	 * @return  	   none
	*/
	void Init()
	{
		if(!SerialPtr_->isOpen())
		{
			SerialPtr_->open();	
		}
	}

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
	 * @brief          计算前n字节累加校验和
	 * @param[in]      LRC_message: 数据
	 * @param[in]      LRC_length: 数据和校验的长度
	 * @retval         计算完的校验和
	 */
	static uint8_t get_LRC_sum(uint8_t *LRC_message, uint16_t LRC_length){
		uint16_t check_sum = 0;
		uint16_t len = LRC_length;
		if (LRC_message == nullptr)
		{
			return 0XFF;
		}
		while(--len)
		{
			check_sum += *LRC_message++;
		}
		return check_sum;
	};

	/**
	 * @brief          LRC校验
	 * @param[in]      LRC_message: 数据
	 * @param[in]      LRC_length: 数据的长度
	 * @retval         计算完的校验和
	 */
	static bool LRC_check(uint8_t *data_message, uint32_t LRC_length){
		uint16_t temp = 0;
		uint16_t len = LRC_length;
		temp = get_LRC_sum(data_message, len-4);
		if(data_message[len-4] == (temp & 0xFF) && data_message[len-3] == (temp >> 8))
		{
			return 1;
		}
		else
		{
			return 0;
		}
	};

	/**
	 * @brief 进入IMU命令模式
	 * @retval none
	*/
	void GOTO_Command_Mode(void);

	/**
	 * @brief 进入IMU数据流发送
	 * @retval none
	*/
	void GOTO_Streaming_Mode(void);

	/**
	 * @brief IMU参数保存
	 * @retval none
	*/
	void GOTO_Save_Setting(void);

public:
	static constexpr auto IMU_HEAD_SOF = 0x3A; //帧头
	static constexpr auto IMU_ID = 0x0001; //IMU设备ID
	static constexpr auto IMU_END_SOF = 0x0A0D; //帧尾
	static constexpr auto IMU_CMD_LENGTH = 11; //IMU指令长度
	static constexpr auto IMU_DATA_LENGTH = 167; //IMU回传数据大小
	static enum
	{
		Goto_Command_Mode_CMD   = 0x0006,  //进入命令模式
		Goto_Streaming_Mode_CMD = 0x0007, //进入数据发送模式
		Save_Setting_CMD        = 0x0004, //保存IMU参数设置

	}imu_cmd_e;

private:
	
	serial::Serial *SerialPtr_;	//IMU串口指针

	
	uint8_t imu_tx_buf[IMU_CMD_LENGTH]={0};
	
};

#endif /* __LPMS_HPP__ */
