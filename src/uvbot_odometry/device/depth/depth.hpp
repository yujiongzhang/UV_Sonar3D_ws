#ifndef _DEPTH_HPP_
#define _DEPTH_HPP_
#include "common.hpp"
/* ----------------------- Data Struct ------------------------------------- */
#pragma pack(push, 1)


#pragma pack(pop)
/* ----------------------- extern Function ----------------------------------- */

/**
 * @brief   Depth 基类
*/
class Depth
{
public:
	Depth();
	virtual ~Depth(){}
	/**
	 * @brief          返回深度计数据指针
	*/
	float get_depth_data(void) const
	{
		return depth_data;
	}
	/**
	 * @brief          深度计协议解析
	 * @param[in]      depth_frame: 原生数据指针
	 * @return  	   none
	*/
	virtual void depth_data_solve(volatile const uint8_t *depth_frame) = 0;

	float depth_data;
};

#endif /* _DEPTH_HPP_ */
