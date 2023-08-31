#include "rm2006.h"
//采样时间在5~10ms
const uint8_t RM_3508Reduction_Ratio[8] = {36,36,36,36,36,36,36,36};//电机减速比数组
/*
********************************************************************************
M2006 电调反馈报文格式：
DATA[0]:转子机械角度高8位
DATA[1]:转子机械角度低8位
DATA[2]:转子转速高8位
DATA[3]:转子转速低8位
DATA[4]:实际输出转矩高8位
DATA[5]:实际输出转矩低8位
DATA[6]:Null
DATA[6]:Null

发送频率：1KHz(CAN中断频率)
标识符：0x200+电调ID
********************************************************************************
*/

//用于存储电机反馈的全局变量
uint8_t M2006_Feedback_Buf[8][6];		//电机反馈值(全局变量)
int M2006_Pos[8];					//每一个元素对应一个ID的电机的信息

/*
********************************************************************************
M2006 电调接收报文格式：
TxMessage.StdId = 0x200
电调ID:1
DATA[0]:控制电流高8位
DATA[1]:控制电流低8位
电调ID:2
DATA[2]:控制电流高8位
DATA[3]:控制电流低8位
电调ID:3
DATA[4]:控制电流高8位
DATA[5]:控制电流低8位
电调ID:4
DATA[6]:控制电流高8位
DATA[7]:控制电流低8位

TxMessage.StdId = 0x1FF
电调ID:5
DATA[0]:控制电流高8位
DATA[1]:控制电流低8位
电调ID:6
DATA[2]:控制电流高8位
DATA[3]:控制电流低8位
电调ID:7
DATA[4]:控制电流高8位
DATA[5]:控制电流低8位
电调ID:8
DATA[6]:控制电流高8位
DATA[7]:控制电流低8位

控制转矩电流值范围：-10000~10000，对应电调输出电流-10~10A
********************************************************************************
*/

//PID参数初始化	(每个电机一套参数)
M2006_PID M2006_Speed_Pid[8] =
{
	{.Kp = 8,.Ki = 0.1,.Kd = 1.5,.Max = 15000,.Min = -15000},//ID = 1
	{.Kp = 8,.Ki = 0.1,.Kd = 1.5,.Max = 15000,.Min = -15000},//ID = 2
	{.Kp = 10.5,.Ki = 0.05,.Kd = 0.5,.Max = 15000,.Min = -15000},//ID = 3
	{.Kp = 8,.Ki = 0.1,.Kd = 1.5,.Max = 15000,.Min = -15000},//ID = 4
	{.Kp = 15,.Ki = 0.6275,.Kd = 2.5,.Max = 10000,.Min = -10000},//ID = 5
	{.Kp = 30,.Ki = 0.0,.Kd = 10,.Max = 10000,.Min = -10000},//ID = 6
	{.Kp = 6.5,.Ki = 0,.Kd = 1.5,.Max = 10000,.Min = -10000},//ID = 7
	{.Kp = 6.5,.Ki = 0,.Kd = 1.5,.Max = 10000,.Min = -10000},//ID = 8
};
//1-4是3508的参数，5-8是2006的参数
 M2006_PID M2006_Pos_Pid[8] = 
{
	{.Kp = 0.07,.Ki = 0.00001,.Kd = 0.2,.Max = 260,.Min = -14000},	//ID = 1
	{.Kp = 0.05,.Ki = 0.001,.Kd = 0.0,.Max = 14000,.Min = -14000},	//ID = 2
	{.Kp = 0.0075,.Ki = 0.00001,.Kd = 0.005,.Max = 260,.Min = -260},	//ID = 3
	{.Kp = 0.0075,.Ki = 0.00001,.Kd = 0.005,.Max = 260,.Min = -260},	//ID = 4
	{.Kp = 0.0075,.Ki = 0.00001,.Kd = 0.005,.Max = 260,.Min = -260},	//ID = 5
	{.Kp = 0.05,.Ki = 0.001,.Kd = 0.0,.Max = 14000,.Min = -14000},	//ID = 6
	{.Kp = 0.07,.Ki = 0,.Kd = 0.2,.Max = 14000,.Min = -14000},	//ID = 7
	{.Kp = 0.07,.Ki = 0,.Kd = 0.2,.Max = 14000,.Min = -14000},	//ID = 8
};


/*********************************************************************************
  *@  name      : M2006_Set_I
  *@  function  : M2006电机电流设置
  *@  input     : 目标电流，电机id
  *@  output    : 成功返回0，失败返回1
*********************************************************************************/
uint8_t can_Sendbuf_2006[8];
uint8_t M2006_Set_I(int target_i,uint8_t motor_id)
{
	if( motor_id>=1 && motor_id<=8 ) 
	{
		int send_id = 0;
		send_id = send_id;
		
		if( target_i<=-15000 )
			target_i=-15000;
		else if( target_i>=15000 )
			target_i=15000;

		if(motor_id<=4)
			send_id=0x200;  
		else 
		{
			send_id=0x1ff;
			motor_id -=4;
		}

		can_Sendbuf_2006[2*motor_id-2]=target_i>>8;                  //电流值高8位
		can_Sendbuf_2006[2*motor_id-1]=target_i & 0x00ff;            //电流值低8位

		//CAN_SendData(&hcan,can_Sendbuf_3508);
		DJI_CAN_Send_Data(&hcan1, can_Sendbuf_2006, send_id,8);
		return 0;
	}
	else 
	return 1;
}


/********************************************************************************
  *@  name      : M2006_Set_Speed
  *@  function  : rm3508速度设置
  *@  input     : 目标速度（-15000~15000都可接受），电机id
  *@  output    : 无
********************************************************************************/
void M2006_Set_Speed(int goal_speed,int ID)
{
	uint8_t id = ID-1;
	M2006_Speed_Pid[id].Err_1 = M2006_Speed_Pid[id].Err;
	M2006_Speed_Pid[id].Err 	= goal_speed - M2006_Get_Speed(ID);
	M2006_Speed_Pid[id].Err_sum += M2006_Speed_Pid[id].Err; 
	M2006_Speed_Pid[id].Err_sum = CLAMP(M2006_Speed_Pid[id].Err_sum,-1000,1000);
	
	M2006_Speed_Pid[id].P_Out = M2006_Speed_Pid[id].Kp * M2006_Speed_Pid[id].Err;
	M2006_Speed_Pid[id].I_Out = M2006_Speed_Pid[id].Ki * M2006_Speed_Pid[id].Err_sum;
	M2006_Speed_Pid[id].D_Out = M2006_Speed_Pid[id].Kd * (M2006_Speed_Pid[id].Err - M2006_Speed_Pid[id].Err_1); 
	
	M2006_Speed_Pid[id].PID_Out = M2006_Speed_Pid[id].P_Out + M2006_Speed_Pid[id].I_Out + M2006_Speed_Pid[id].D_Out;
	M2006_Speed_Pid[id].PID_Out = CLAMP(M2006_Speed_Pid[id].PID_Out,M2006_Speed_Pid[id].Min,M2006_Speed_Pid[id].Max);

	M2006_Set_I(M2006_Speed_Pid[id].PID_Out,ID);	
}


/********************************************************************************
  *@  name      : M2006_Set_Pos
  *@  function  : rm3508位置设置
  *@  input     : 目标角度（任意角度），电机id
  *@  output    : 无
********************************************************************************/
void M2006_Set_Pos(int angle,int ID)
{
	int goal_cnt=DJI_Ang2Cnt(angle,ID);
	uint8_t id = ID-1;
	M2006_Pos_Pid[id].Err_1 	= M2006_Pos_Pid[id].Err;
	M2006_Pos_Pid[id].Err 	= goal_cnt - M2006_Get_Pos(ID);
	
	M2006_Pos_Pid[id].Err_sum += M2006_Pos_Pid[id].Err; 
	M2006_Pos_Pid[id].Err_sum = CLAMP(M2006_Pos_Pid[id].Err_sum,-20000,20000);
	
	M2006_Pos_Pid[id].P_Out = M2006_Pos_Pid[id].Kp * M2006_Pos_Pid[id].Err;
	M2006_Pos_Pid[id].I_Out = M2006_Pos_Pid[id].Ki * M2006_Pos_Pid[id].Err_sum;
	M2006_Pos_Pid[id].D_Out = M2006_Pos_Pid[id].Kd * (M2006_Pos_Pid[id].Err - M2006_Pos_Pid[id].Err_1); 
	
	M2006_Pos_Pid[id].PID_Out = M2006_Pos_Pid[id].P_Out + M2006_Pos_Pid[id].I_Out + M2006_Pos_Pid[id].D_Out;
	M2006_Pos_Pid[id].PID_Out = CLAMP(M2006_Pos_Pid[id].PID_Out,M2006_Pos_Pid[id].Min,M2006_Pos_Pid[id].Max );

	if(M2006_Pos_Pid[id].PID_Out<1&&M2006_Pos_Pid[id].PID_Out>-1)
		M2006_Pos_Pid[id].PID_Out = 0;
	
//	if(Pos_Pid[id].PID_Out == 0)
//		Pos_Pid[id].Kp = 1;
//	else 
//		Pos_Pid[id].Kp = 0.0075;
	
	M2006_Set_Speed(M2006_Pos_Pid[id].PID_Out,ID);	

}
/*********************************************************************************
  *@  name      : M2006_Get_Feedback
  *@  function  : 获取M2006电机的反馈并存入全局变量M2006_Feedback_Buf[8][6];
  *@  input     : message_id,message数组指针
  *@  output    : 
*********************************************************************************/
void M2006_Get_Feedback(uint32_t std_id,uint8_t* data_p)
{
	int i;
	for(i=1;i<9;i++)        //前四电机匹配
	{
		if(std_id==0x200+i)
		{
			memcpy(M2006_Feedback_Buf[i-1],data_p,6);
			pos_rec(i);
			return;
		}
	}
}
/*********************************************************************************
  *@  name      : M_2006_Get_Real_I
  *@  function  : 获取M2006电机的实际转矩信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的转矩,读取失败返回0
*********************************************************************************/
int M_2006_Get_Torque(uint8_t motor_id)
{
	int torque = 0;
	if(M2006_Feedback_Buf[motor_id-1][2]>>7==1)
		torque = -( 0xffff-(  (M2006_Feedback_Buf[motor_id-1][4]<<8)+M2006_Feedback_Buf[motor_id-1][5])  ) ;
	else 
		torque = (M2006_Feedback_Buf[motor_id-1][4]<<8)+M2006_Feedback_Buf[motor_id-1][5];
	return torque;
}
/*********************************************************************************
  *@  name      : M2006_Get_Speed
  *@  function  : 获取M2006电机的反馈的速度信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的速度,读取失败返回0
*********************************************************************************/
int M2006_Get_Speed(uint8_t motor_id)
{
	int speed = 0;
	if(M2006_Feedback_Buf[motor_id-1][2]>>7==1)
		speed = -( 0xffff-(  (M2006_Feedback_Buf[motor_id-1][2]<<8)+M2006_Feedback_Buf[motor_id-1][3])  ) ;
	else 
		speed = (M2006_Feedback_Buf[motor_id-1][2]<<8)+M2006_Feedback_Buf[motor_id-1][3];
	return speed;
}
/*********************************************************************************
  *@  name      : M2006_Get_Pos
  *@  function  : 获取M2006电机当前的位置信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的位置，编码器的CNT值
*********************************************************************************/
int M2006_Get_Pos(uint8_t motor_id)
{
	return M2006_Pos[motor_id - 1];
}
/*********************************************************************************
  *@  name      : pos_rec
  *@  function  : 获取2006电机的反馈的位置信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的位置信息,读取失败返回-1
*********************************************************************************/
void pos_rec(uint8_t motor_id)
{
	int id=motor_id-1;
	int32_t	m2006_tmp[8];
	static int32_t	m2006_base[8] = {0};	//用来标记已经转过的圈数，一圈8192
	static int32_t m2006tmp_pre[8] = {0};

	m2006_tmp[id]=(M2006_Feedback_Buf[id][0]<<8)+M2006_Feedback_Buf[id][1];
	if ( m2006_tmp[id] - m2006tmp_pre[id] > 4095 )  //转过8191到0时记录圈数
		m2006_base[id] -= 8191;
	else if ( m2006_tmp[id] - m2006tmp_pre[id] < -4095 )
		m2006_base[id] += 8191;
	
	m2006tmp_pre[id] = m2006_tmp[id];
	M2006_Pos[id] = m2006_base[id] + m2006_tmp[id];	

}
/********************************************************************************
  *@  name      : DJI_Ang2Cnt
  *@  function  : 角度转换为实际电机应该转动的位置数 //未经减速箱的转轴转一圈数值为8192
  *@  input     : 目标角度（任意角度），电机id  //id不同，减速比不同
  *@  output    : 电机位置
********************************************************************************/
int DJI_Ang2Cnt(float angle,int ID)  
{

	int cnt;
	cnt = (int)(CNT_PER_ROUND_OUT(ID) * angle/360);
	return cnt;
}
/********************************************************************************
  *@  name      : DJI_Cnt2Ang
  *@  function  : 电机位置转换为角度 //未经减速箱的转轴转一圈数值为8192
  *@  input     : 电机位置，电机id  //id不同，减速比不同
  *@  output    : 电机转过的角度
********************************************************************************/
double DJI_Cnt2Ang(int32_t cnt,int ID)
{
	 
	int angled;
	angled = (double)((cnt * 360.0)/CNT_PER_ROUND_OUT(ID));
	return angled;
}

//void CAN_SendData(CAN_HandleTypeDef *hcant, uint8_t *pData){
//	
//	CAN_TxHeaderTypeDef TxMessage;
//	TxMessage.StdId = 0x200;
//	TxMessage.DLC = 8; /*默认一帧传输长度为8*/
//	TxMessage.IDE = CAN_ID_STD;
//	TxMessage.RTR = CAN_RTR_DATA;
//	//等待空邮箱
//	HAL_CAN_GetTxMailboxesFreeLevel(hcant);

//	while (!HAL_CAN_GetTxMailboxesFreeLevel(hcant)); //等待空邮箱
//	HAL_CAN_AddTxMessage(hcant, &TxMessage, pData, (uint32_t *)CAN_TX_MAILBOX1);
//}

uint8_t DJI_CAN_Send_Data(CAN_HandleTypeDef* hcant, uint8_t *pData, uint16_t ID, uint16_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR;
	uint8_t FreeTxNum = 0;
	CAN_TxHeaderTypeDef TxMessage;
	
	TxMessage.IDE = CAN_ID_STD;  //标准帧,CAN_ID_EXT扩展帧;
	TxMessage.RTR = CAN_RTR_DATA;  //数据帧,CAN_RTR_REMOTE遥控帧
	TxMessage.StdId = ID;
	TxMessage.DLC = Len;
	
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcant);	
		
	while(FreeTxNum==0)  //等待空邮箱
	{
		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcant);
	}
	
	HAL_Delay(1); //没有延时很有可能会发送失败
	
	HAL_RetVal = HAL_CAN_AddTxMessage(hcant,&TxMessage,pData,(uint32_t*)CAN_TX_MAILBOX0);
	
	if(HAL_RetVal!=HAL_OK)
	{
		return 2;
	}
	
	return 0;
}




