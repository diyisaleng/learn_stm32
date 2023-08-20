
#include "RM3508.h"
#include "string.h"

/*********************************************************************************
  绝对值函数
*********************************************************************************/
double FabsD(double x){
	if(x>0)x=x;
else if(x<=0)x=-x;
return x;
}


/*********************************************************************************
  速度环PID设置：Kp Ki Kd
  云台俯仰：     8  0  0
  云台旋转：     8  0  0
  发射电机：     8  0  0
*********************************************************************************/
M3508_PID M3508_Speed_Pid[8] =//8 0 0    8 0 0   
{
	{.Kp = 8,.Ki = 0.f,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	//ID = 1
	{.Kp = 8,.Ki = 0,.Kd = 12,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	//ID = 2
	{.Kp = 8,.Ki = 0,.Kd = 0.8,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	//ID = 3
	{.Kp = 8,.Ki = 1,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	 	//ID = 4
	/**/{.Kp = 8,.Ki = 0.f,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	 	//ID = 5
	{.Kp = 8,.Ki = 0,.Kd = 12,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	 	//ID = 6
	{.Kp = 8,.Ki = 0,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	 	//ID = 7
	{.Kp = 8,.Ki = 0,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	 	//ID = 8
};
/*********************************************************************************
位置PID设置： Kp    Ki     Kd
云台俯仰：   0.2  0.009   0.01
云台旋转：   0.2    0       0
发射电机：
*********************************************************************************/

M3508_PID M3508_Pos_Pid[8] = //云台0.2, 0.009 0.01  底盘0.2 0 0
{
	{.Kp = 0.2,.Ki = 0.001,.Kd = 0.01,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 100},	//ID = 10
	{.Kp = 0.01,.Ki = 0,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 100},	//ID = 2
	{.Kp = 0.01,.Ki = 0,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 0},	//ID = 3
	{.Kp = 3.2,.Ki = 0.005,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 0},	//ID = 4
	/**/{.Kp = 0.1,.Ki = 0,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 100},	//ID = 5// 
	//{.Kp = 0.7,.Ki = 0.05,.Kd = 5,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 0},	//ID = 5
	//{.Kp = 0.7,.Ki = 0,.Kd = 10,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 0},	//ID = 5
	{.Kp = 0.16,.Ki = 0.001,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 0},	//ID = 6
	{.Kp = 0.5,.Ki = 0.005,.Kd = 10,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 200},	//ID = 7
	{.Kp = 3.2,.Ki = 0.005,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 2000}	//ID = 8   8 0.001 20
};

//采样时间在5~10ms
const uint8_t RM3508_Reduction_Ratio[8] = {19,19,19,19,19,19,19,19};//电机减速比数组





//用于存储电机反馈的全局变量
uint8_t RM3508_Feedback_Buf[8][7];		//电机反馈值(全局变量)
int RM3508_Pos[8];					//每一个元素对应一个ID的电机的信息
uint8_t RM3508_Sendbuf1[8] = {0};  //CAN发送数据
uint8_t RM3508_Sendbuf2[8] = {0};  //CAN发送数据



/*********************************************************************************
  *@  name      : RM3508_Set_I
  *@  function  : RM3508电机电流设置
  *@  input     : 目标电流，电机id
  *@  output    : 成功返回0，失败返回1
*********************************************************************************/
uint8_t RM3508_Set_I(int target_i,uint8_t motor_id)
{
	if( motor_id>=1 && motor_id<=8 ) 
	{
		int send_id = 0;
		send_id = send_id;
		
		if( target_i<=-10000 )
			target_i=-10000;
		else if( target_i>=10000 )
			target_i=10000;

        if(motor_id <=4 )   //前四个ID对应的sendID标识符是0x200
        {
            send_id=0x200;  

            RM3508_Sendbuf2[2*motor_id-2]=target_i>>8;                  //电流值高8位
            RM3508_Sendbuf2[2*motor_id-1]=target_i & 0x00ff;            //电流值低8位

        }
        else 							//后四个ID对应的sendID标识符是0x1ff
        {
            send_id=0x1ff;
            motor_id-=4;

        RM3508_Sendbuf1[2*motor_id-2]=target_i>>8;                  //电流值高8位
        RM3508_Sendbuf1[2*motor_id-1]=target_i & 0x00ff;            //电流值低8位

        }

    if(send_id==0x200)//ID1-4由can2发
		 RM3508_CAN_Send_Data(&hcan1, RM3508_Sendbuf2, send_id ,8);
    else
		 RM3508_CAN_Send_Data(&hcan1, RM3508_Sendbuf1, send_id ,8);
      
		return 0;
	}
	else 
	return 1;
}
/********************************************************************************
  *@  name      : RM3508_CAN_Send_Data
  *@  function  : CAN发送数据
  *@  input     : 哪条CAN线，要发送的数据，发送的仲裁段ID，数据长度
  *@  output    : 成功返回0，失败返回2
*********************************************************************************/
uint8_t RM3508_CAN_Send_Data(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID, uint16_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR;
	uint8_t FreeTxNum = 0;
	CAN_TxHeaderTypeDef TxMessage;
	
	TxMessage.IDE = CAN_ID_STD;  //标准帧,CAN_ID_EXT扩展帧;
	TxMessage.RTR = CAN_RTR_DATA;  //数据帧,CAN_RTR_REMOTE遥控帧
	TxMessage.StdId = ID;
	TxMessage.DLC = Len;
	
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);	
		
	while(FreeTxNum==0)  //等待空邮箱
	{
		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	}
	
//	HAL_Delay(1); //没有延时很有可能会发送失败
	
	HAL_RetVal = HAL_CAN_AddTxMessage(hcan,&TxMessage,pData,(uint32_t*)CAN_TX_MAILBOX0);
	
	if(HAL_RetVal!=HAL_OK)
	{
		return 2;
	}
	
	return 0;
}
/********************************************************************************
  *@  name      : RM3508_Set_Speed
  *@  function  : RM3508速度设置
  *@  input     : 目标速度（-15000~15000都可接受），电机id
  *@  output    : 无
********************************************************************************/
void RM3508_Set_Speed(int goal_speed,int ID)
{
	uint8_t id = ID-1;
	M3508_Speed_Pid[id].Target = goal_speed;
	M3508_Speed_Pid[id].NowIS = RM3508_Get_Speed(ID);
	M3508_Speed_Pid[id].Err_last = M3508_Speed_Pid[id].Err;
	M3508_Speed_Pid[id].Err 	= M3508_Speed_Pid[id].Target -  M3508_Speed_Pid[id].NowIS;
	
	if(FabsD(M3508_Speed_Pid[id].Err) < M3508_Speed_Pid[id].DeadBand) M3508_Speed_Pid[id].Err = 0;
  M3508_Speed_Pid[id].Err_sum += M3508_Speed_Pid[id].Err; 
	M3508_Speed_Pid[id].Err_sum = CLAMP(M3508_Speed_Pid[id].Err_sum,-M3508_Speed_Pid[id].IntegralLimit,M3508_Speed_Pid[id].IntegralLimit);
	M3508_Speed_Pid[id].P_Out = M3508_Speed_Pid[id].Kp * M3508_Speed_Pid[id].Err;
	M3508_Speed_Pid[id].I_Out = M3508_Speed_Pid[id].Ki * M3508_Speed_Pid[id].Err_sum;
	M3508_Speed_Pid[id].D_Out = M3508_Speed_Pid[id].Kd * (M3508_Speed_Pid[id].Err - M3508_Speed_Pid[id].Err_last); 
	
	M3508_Speed_Pid[id].PID_Out = M3508_Speed_Pid[id].P_Out +M3508_Speed_Pid[id].I_Out + M3508_Speed_Pid[id].D_Out;
	M3508_Speed_Pid[id].PID_Out = CLAMP(M3508_Speed_Pid[id].PID_Out,M3508_Speed_Pid[id].Min,M3508_Speed_Pid[id].Max);

	
	RM3508_Set_I(M3508_Speed_Pid[id].PID_Out,ID);	
}
/********************************************************************************
  *@  name      : Head_Set_Speed
  *@  function  : rm3508速度设置
  *@  input     : 目标速度（-15000~15000都可接受），电机id
  *@  output    : 无
********************************************************************************/
void Head_Set_Speed(int goal_speed,int ID)
{
	uint8_t id = ID-1;
	M3508_Speed_Pid[id].Err_last = M3508_Speed_Pid[id].Err;
	M3508_Speed_Pid[id].Err 	= goal_speed -  RM3508_Get_Speed(ID);
	M3508_Speed_Pid[id].Err_sum += M3508_Speed_Pid[id].Err; 
	M3508_Speed_Pid[id].Err_sum = CLAMP(M3508_Speed_Pid[id].Err_sum,-1000,1000);
	
	M3508_Speed_Pid[id].P_Out = M3508_Speed_Pid[id].Kp * M3508_Speed_Pid[id].Err;
	M3508_Speed_Pid[id].I_Out = M3508_Speed_Pid[id].Ki * M3508_Speed_Pid[id].Err_sum;
	M3508_Speed_Pid[id].D_Out = M3508_Speed_Pid[id].Kd * (M3508_Speed_Pid[id].Err - M3508_Speed_Pid[id].Err_last); 
	
	M3508_Speed_Pid[id].PID_Out = M3508_Speed_Pid[id].P_Out + M3508_Speed_Pid[id].I_Out + M3508_Speed_Pid[id].D_Out;
	M3508_Speed_Pid[id].PID_Out = CLAMP(M3508_Speed_Pid[id].PID_Out,M3508_Speed_Pid[id].Min,M3508_Speed_Pid[id].Max);

	
	RM3508_Set_I(M3508_Speed_Pid[id].PID_Out,ID);	
}


/********************************************************************************
  *@  name      : RM3508_Set_Pos
  *@  function  : rm3508位置设置
  *@  input     : 目标角度（任意角度），电机id
  *@  output    : 无
********************************************************************************/
void RM3508_Set_Pos(float pos,int ID)
{
	uint8_t id = ID-1;
	float goal_cnt;
	goal_cnt=pos;
//	goal_cnt=RM3508_Ang2Cnt(angle,ID);//该函数为电机角度转cnt
	M3508_Pos_Pid[id].Target = goal_cnt;
	M3508_Pos_Pid[id].NowIS = RM3508_Get_Pos(ID);
	M3508_Pos_Pid[id].Err_last 	= M3508_Pos_Pid[id].Err;
	M3508_Pos_Pid[id].Err 	= goal_cnt - M3508_Pos_Pid[id].NowIS;
	
	if(FabsD(M3508_Pos_Pid[id].Err) < M3508_Pos_Pid[id].DeadBand) M3508_Pos_Pid[id].Err = 0;
	
	M3508_Pos_Pid[id].Err_sum += M3508_Pos_Pid[id].Err; 
	M3508_Pos_Pid[id].Err_sum = CLAMP(M3508_Pos_Pid[id].Err_sum,-M3508_Pos_Pid[id].IntegralLimit,M3508_Pos_Pid[id].IntegralLimit);
	
	M3508_Pos_Pid[id].P_Out = M3508_Pos_Pid[id].Kp * M3508_Pos_Pid[id].Err;
	M3508_Pos_Pid[id].I_Out = M3508_Pos_Pid[id].Ki * M3508_Pos_Pid[id].Err_sum;
	M3508_Pos_Pid[id].D_Out = M3508_Pos_Pid[id].Kd * (M3508_Pos_Pid[id].Err - M3508_Pos_Pid[id].Err_last); 
	
	M3508_Pos_Pid[id].PID_Out = M3508_Pos_Pid[id].P_Out + M3508_Pos_Pid[id].I_Out + M3508_Pos_Pid[id].D_Out;
	M3508_Pos_Pid[id].PID_Out = CLAMP(M3508_Pos_Pid[id].PID_Out,M3508_Pos_Pid[id].Min,M3508_Pos_Pid[id].Max );

	RM3508_Set_Speed(M3508_Pos_Pid[id].PID_Out,ID);	
}
/********************************************************************************
  *@  name      : RM3508_Set_Pos
  *@  function  : rm3508位置设置
  *@  input     : 目标角度（任意角度），电机id
  *@  output    : 无
********************************************************************************/
void RM3508_Set_Ang(float angle,int ID)
{
	uint8_t id = ID-1;
	float goal_cnt;
	goal_cnt=RM3508_Ang2Cnt(angle,ID);//该函数为电机角度转cnt
	M3508_Pos_Pid[id].Target = goal_cnt;
	M3508_Pos_Pid[id].NowIS = RM3508_Get_Pos(ID);
	M3508_Pos_Pid[id].Err_last 	= M3508_Pos_Pid[id].Err;
	M3508_Pos_Pid[id].Err 	= goal_cnt - M3508_Pos_Pid[id].NowIS;
	
	if(FabsD(M3508_Pos_Pid[id].Err) < M3508_Pos_Pid[id].DeadBand) M3508_Pos_Pid[id].Err = 0;
	
	M3508_Pos_Pid[id].Err_sum += M3508_Pos_Pid[id].Err; 
	M3508_Pos_Pid[id].Err_sum = CLAMP(M3508_Pos_Pid[id].Err_sum,-M3508_Pos_Pid[id].IntegralLimit,M3508_Pos_Pid[id].IntegralLimit);
	
	M3508_Pos_Pid[id].P_Out = M3508_Pos_Pid[id].Kp * M3508_Pos_Pid[id].Err;
	M3508_Pos_Pid[id].I_Out = M3508_Pos_Pid[id].Ki * M3508_Pos_Pid[id].Err_sum;
	M3508_Pos_Pid[id].D_Out = M3508_Pos_Pid[id].Kd * (M3508_Pos_Pid[id].Err - M3508_Pos_Pid[id].Err_last); 
	
	M3508_Pos_Pid[id].PID_Out = M3508_Pos_Pid[id].P_Out + M3508_Pos_Pid[id].I_Out + M3508_Pos_Pid[id].D_Out;
	M3508_Pos_Pid[id].PID_Out = CLAMP(M3508_Pos_Pid[id].PID_Out,M3508_Pos_Pid[id].Min,M3508_Pos_Pid[id].Max );

	RM3508_Set_Speed(M3508_Pos_Pid[id].PID_Out,ID);	
}
/*********************************************************************************
  *@  name      : RM3508_Get_Feedback
  *@  function  : 获取RM3508电机的反馈并存入全局变量RM3508_Feedback_Buf[8][7];
  *@  input     : message_id,message数组指针
  *@  output    : 无
*********************************************************************************/
void RM3508_Get_Feedback(uint32_t std_id,uint8_t* data_p)
{
	int i;
	for(i=1;i<9;i++)
	{
		if(std_id==0x200+i)
		{
			memcpy(RM3508_Feedback_Buf [i-1],data_p,7);
			RM3508_Pos_Rec(i);
			return;
		}
	}
}
/*********************************************************************************
  *@  name      : RM3508_Get_Torque
  *@  function  : 获取RM3508电机的实际转矩信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的转矩,读取失败返回0
*********************************************************************************/
int RM3508_Get_Torque(uint8_t motor_id)
{
	int torque = 0;
	if(RM3508_Feedback_Buf[motor_id-1][2]>>7==1)
		torque = -( 0xffff-(  (RM3508_Feedback_Buf[motor_id-1][4]<<8)+RM3508_Feedback_Buf[motor_id-1][5])  ) ;
	else 
		torque = (RM3508_Feedback_Buf[motor_id-1][4]<<8)+RM3508_Feedback_Buf[motor_id-1][5];
	return torque;
}
/*********************************************************************************
  *@  name      : RM3508_Get_Speed
  *@  function  : 获取RM3508电机的反馈的速度信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的速度,读取失败返回0
*********************************************************************************/
int RM3508_Get_Speed(uint8_t motor_id)
{
	int speed = 0;
	if(RM3508_Feedback_Buf[motor_id-1][2]>>7==1)
		speed = -( 0xffff-(  (RM3508_Feedback_Buf[motor_id-1][2]<<8)+RM3508_Feedback_Buf[motor_id-1][3])  ) ;
	else 
		speed = (RM3508_Feedback_Buf[motor_id-1][2]<<8)+RM3508_Feedback_Buf[motor_id-1][3];
	return speed;
}
/*********************************************************************************
  *@  name      : RM3508_Get_Pos
  *@  function  : 获取RM3508电机当前的位置信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的位置，编码器的CNT值
*********************************************************************************/
int RM3508_Get_Pos(uint8_t motor_id)
{
	return RM3508_Pos[motor_id - 1];
}
/*********************************************************************************
  *@  name      : RM3508_Temperature
  *@  function  : 获取RM3508电机当前的温度信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的温度值
*********************************************************************************/
uint8_t RM3508_Temperature(uint8_t id)
{
    uint8_t Tem;
    Tem = RM3508_Feedback_Buf[id-1][6];
    return Tem;
}
/*********************************************************************************
  *@  name      : RM3508_Pos_Rec
  *@  function  : 获取RM3508电机的反馈的位置信息  //累积路程
  *@  input     : 电机id号
  *@  output    : 对应id电机的位置信息,读取失败返回-1
*********************************************************************************/
static int32_t	RM3508_base[8] = {0};	//用来标记已经转过的编码器线数，一圈8192

void RM3508_Pos_Rec(uint8_t motor_id)
{
	int id=motor_id-1;
	int32_t	RM3508_tmp[8];
	
	static int32_t RM3508tmp_pre[8] = {0};

	RM3508_tmp[id]=(RM3508_Feedback_Buf[id][0]<<8)+RM3508_Feedback_Buf[id][1];
	if ( RM3508_tmp[id] - RM3508tmp_pre[id] > 4095 )  //转过8191到0时记录圈数
		RM3508_base[id] -= 8191;
	else if ( RM3508_tmp[id] - RM3508tmp_pre[id] < -4095 )
		RM3508_base[id] += 8191;
	
	RM3508tmp_pre[id] = RM3508_tmp[id];
	RM3508_Pos[id] = RM3508_base[id] + RM3508_tmp[id];	
	
}
/********************************************************************************
  *@  name      : RM3508_Ang2Cnt
  *@  function  : 角度转换为实际电机应该转动的位置数 //未经减速箱的转轴转一圈数值为8192
  *@  input     : 目标角度（任意角度），电机id  //id不同，减速比不同
  *@  output    : 电机位置
********************************************************************************/
int RM3508_Ang2Cnt(float angle,int ID)  
{

	int cnt;
	cnt = (int)(RM3508_CNT_PER_ROUND_OUT(ID) * angle/360);
	return cnt;
}
/********************************************************************************
  *@  name      : RM3508_Cnt2Ang
  *@  function  : 电机位置转换为角度 //未经减速箱的转轴转一圈数值为8192
  *@  input     : 电机位置，电机id  //id不同，减速比不同
  *@  output    : 电机转过的角度
********************************************************************************/
double RM3508_Cnt2Ang(int32_t cnt,int ID)
{
	double angled;
	angled = (double)((cnt * 360.0)/RM3508_CNT_PER_ROUND_OUT(ID));
	return angled;
}

/********************************************************************************
  *@  name      : RM3508_Set_NowPos
  *@  function  : 将M3508电机的当前值设置为任意位置
  *@  input     : 电机id，当前位置设置为多少 //单位是编码器线数
  *@  output    : void
********************************************************************************/
void RM3508_Set_NowPos(uint8_t ID,int32_t Pos_Angle)
{
	uint8_t id;
	id=ID-1;
	RM3508_base[id]=Pos_Angle;
}
