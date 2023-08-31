/*********************************************************************************
*@函数功能简介：
*@	M2006_Set_I：		M2006电流控制
*@	M2006_Set_Speed：	M2006速度闭环，通过电流作为输出
*@	M2006_Set_Pos：		M2006位置闭环，通过速度作为输出，串级PID
*@
*@	M2006_Get_Feedback：M2006数据反馈获取，CAN中断中使用
*@	M_2006_Get_Torque：	M2006转矩信息获取
*@	M2006_Get_Speed：	速度信息获取
*@	M2006_Get_Pos：		位置信息获取
*@	pos_rec：			位置信息更新，CAN中断中使用，以足够高的采样率解决边界问题
*@
*@	Ang2Cnt：			角度制数据转化为编码器的Cnt值
*@	Cnt2Ang：			编码器Cnt值转化为实际转过的角度
*@
*@	M2006和3508互通的，区别只是电机减速比不同以及3508多了一个电机温度反馈
*@	因此该文件修改电机的减速比后可以直接用于3508的控制
*@  
*@ @file	:	rm2006.c
*@ @author	:	SHJ
*@ @version	:	v2.1，修复了同时控制多个电机，数据被覆写的Bug；每个电机给了一套独立的PID
*@				调参时可以对每个电机针对性的调整；将位置闭环的入口参数改为cnt值
*@ @date	:	2019-1-30
*********************************************************************************/


#ifndef __RM2006_H
#define __RM2006_H	 
#include "can.h"
#include "string.h"


#define CNT_PER_ROUND			(8192)								//编码器线程
#define CNT_PER_ROUND_OUT(x)	(CNT_PER_ROUND*RM_3508Reduction_Ratio[(x-1)])
#define CLAMP(x, lower, upper)	(x >= upper ? upper : (x <= lower ? lower : x))

typedef struct
{
	int Err;
	int Err_1;
	int Err_sum;
	
	float Kp;
	float Ki;
	float Kd;
	
	int Max;
	int Min;

	float P_Out;
	float I_Out;
	float D_Out;
	float PID_Out;
}M2006_PID;

//extern CAN_HandleTypeDef   CAN1_Handler;   //CAN1句柄
extern uint8_t M2006_Feedback_Buf[8][6];      //电机反馈报文
extern int M2006_Pos[8];
extern uint8_t can_Sendbuf_3508[8];

extern int d;

uint8_t		M2006_Set_I(int target_i,uint8_t motor_id);
void	    M2006_Set_Speed(int goal_speed,int ID);
void	    M2006_Set_Pos(int goal_cnt,int ID);

void	M2006_Get_Feedback(uint32_t std_id,uint8_t* data_p);
int		M_2006_Get_Torque(uint8_t motor_id);
int		M2006_Get_Speed(uint8_t motor_id);
int		M2006_Get_Pos(uint8_t motor_id);
void	pos_rec(uint8_t motor_id);									//CAN中断中实时更新位置信息
//void CAN_SendData(CAN_HandleTypeDef *hcant, uint8_t *pData);
int		DJI_Ang2Cnt(float angle,int ID);
double	DJI_Cnt2Ang(int32_t cnt,int ID);
uint8_t DJI_CAN_Send_Data(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID, uint16_t Len);
//uint8_t CAN1_Send_Msg(CAN_HandleTypeDef *CAN_Handle,uint8_t* msg,uint8_t len,int id);


#endif

