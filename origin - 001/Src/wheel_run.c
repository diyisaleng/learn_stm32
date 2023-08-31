#include "RM3508.h"
#include "string.h"
#include "wheel_run.h"

struct wheel
{
	int Wheel_Spd;//ÿ�����ӵ��ٶ�
	int Dir_Inv;//������,��ǰΪ1
};
 
struct wheel Wheel_Info[4]={0,1,0,0,0,0,0,1};//�ҳ�ʼ���ٶȶ����õ�0


void Wheels_Run(void){///*��������˶�*/
for(int8_t ID=1;ID<=4;ID++){
int8_t id = ID-1;
 if(Wheel_Info[id].Dir_Inv)
  Wheel_Info[id].Wheel_Spd = -Wheel_Info[id].Wheel_Spd;///*���ӷ���*/
 RM3508_Set_Speed(Wheel_Info[id].Wheel_Spd,ID);
}

}

 void Speed_Robot_Coordinate(float VX, float VY, float VZ){/*�������ٶȽ��㣬���½�������ʼ��˳ʱ��id����*/
  Wheel_Info[0].Wheel_Spd = -VY + VX - VZ;
  Wheel_Info[1].Wheel_Spd = -VY - VX + VZ;
  Wheel_Info[2].Wheel_Spd = -VY + VX + VZ;
  Wheel_Info[3].Wheel_Spd = -VY - VX - VZ;
 Wheels_Run();
}
 
