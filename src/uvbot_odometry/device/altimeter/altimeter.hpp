#ifndef _ALTIMETER_HPP_
#define _ALTIMETER_HPP_
#include "common.hpp"
/* ----------------------- Data Struct ------------------------------------- */
#pragma pack(push, 1)


#pragma pack(pop)
/* ----------------------- extern Function ----------------------------------- */

/**
 * @brief   高度计基类
*/
class Altimeter
{
public:
	Altimeter();
	virtual ~Altimeter()
	{
		
	}
	/**
	 * @brief          返回高度计数据指针
	*/
	float get_altimeter_data(void) const
	{
		return altimeter_data;
	}
	/**
	 * @brief          高度计协议解析
	 * @param[in]      depth_frame: 原生数据指针
	 * @return  	   none
	*/
	virtual void altimeter_data_solve(volatile const uint8_t *depth_frame) = 0;

	

	float altimeter_data;		//高度,单位为m
};

#endif /* _ALTIMETER_HPP_ */
