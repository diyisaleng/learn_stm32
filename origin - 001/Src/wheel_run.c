#include "RM3508.h"
#include "string.h"
#include "wheel_run.h"

struct wheel
{
	int Wheel_Spd;//每个轮子的速度
	int Dir_Inv;//正方向,向前为1
};
 
struct wheel Wheel_Info[4]={0,1,0,0,0,0,0,1};//我初始都速度都设置的0


void Wheels_Run(void){///*驱动电机运动*/
for(int8_t ID=1;ID<=4;ID++){
int8_t id = ID-1;
 if(Wheel_Info[id].Dir_Inv)
  Wheel_Info[id].Wheel_Spd = -Wheel_Info[id].Wheel_Spd;///*轮子反向*/
 RM3508_Set_Speed(Wheel_Info[id].Wheel_Spd,ID);
}

}

 void Speed_Robot_Coordinate(float VX, float VY, float VZ){/*四麦轮速度解算，右下角轮子起始，顺时针id增大*/
  Wheel_Info[0].Wheel_Spd = -VY + VX - VZ;
  Wheel_Info[1].Wheel_Spd = -VY - VX + VZ;
  Wheel_Info[2].Wheel_Spd = -VY + VX + VZ;
  Wheel_Info[3].Wheel_Spd = -VY - VX - VZ;
 Wheels_Run();
}
 
