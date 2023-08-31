#include "main.h"
#include "stm32f4xx_it.h"
#include "usart.h"
#include "catch_data.h"

#define BUFFER_SIZE 255
extern float speed,angle;
extern uint8_t receive_buff1[255];
extern int getdata_x,getdata_y,getdata_w;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern int turn;

_Bool rx_flag=0;
_Bool flag_x=0;
	_Bool rw_flag=0;
int	negative=1;

void catch_arm_gm6020(uint8_t pointer)//纯按键版本控制
{
	
		switch (receive_buff1[pointer]) {
		case '>':if(receive_buff1[pointer+2]=='u')turn -= 5;
			if (receive_buff1[pointer + 1] == '>'&&receive_buff1[pointer+3]=='u')turn -= 30;
			break;
		case '<':if(receive_buff1[pointer+2]=='u')turn += 5;
			if (receive_buff1[pointer + 1] == '<'&&receive_buff1[pointer+3]=='u')turn += 30;
			break;
			case 'R':if(receive_buff1[pointer+2]=='S'&&receive_buff1[pointer+6]=='u')turn=100;
			break;
		
			
			
	}
}

void catch_arm_rm2006(uint8_t pointer)//纯按键版本控制
{
	
		switch (receive_buff1[pointer]) {
		case 'U':if(receive_buff1[pointer+3]=='u')angle -= 10;
			break;
		case 'D':if(receive_buff1[pointer+5]=='u')angle += 10;
			break;
		case '+':if(receive_buff1[pointer+2]=='u')angle -= 45;
			break;
		case '-':if(receive_buff1[pointer+2]=='u')angle += 45;
			break;
	}

}



void catch_direction(uint8_t pointer)
{
if(receive_buff1[pointer]=='x'&&rx_flag==0)rx_flag=1,getdata_x=0,getdata_y=0,negative=1;
			else if(rx_flag==1&&receive_buff1[pointer]=='-')negative=-1;
			else if(rx_flag==1&&flag_x==0&&receive_buff1[pointer]<='9'&&receive_buff1[pointer]>='0')
				getdata_x=getdata_x*10+(receive_buff1[pointer]-'0');
			else if(rx_flag==1&&receive_buff1[pointer]==';')
				{
					getdata_x=negative*getdata_x;
					flag_x=1,negative=1;
				}
			else if(rx_flag==1&&flag_x==1&&receive_buff1[pointer]<='9'&&receive_buff1[pointer]>='0')
				getdata_y=getdata_y*10+(receive_buff1[pointer]-'0');
			else if(receive_buff1[pointer]=='x')
				{
					getdata_y=negative*getdata_y;
					rx_flag=0,negative=1;
				}
			else if(receive_buff1[pointer]=='y'&&rw_flag==0)rw_flag=1,getdata_w=0,negative=1;
			else if(rw_flag==1&&receive_buff1[pointer]=='-')negative=-1;
			else if(rw_flag==1&&flag_x==0&&receive_buff1[pointer]<='9'&&receive_buff1[pointer]>='0')
				getdata_w=getdata_w*10+(receive_buff1[pointer]-'0');
			else if(rw_flag==1&&receive_buff1[pointer]==';')
				{
					getdata_w=negative*getdata_w;
					flag_x=1,negative=1;
				}
			else if(receive_buff1[pointer]=='y')
				{
					rw_flag=0,negative=1,flag_x=0;
				}

}

void catch_speed(uint8_t pointer)//切换速度档位
{
	switch (receive_buff1[pointer]) {
	case 'L':
		if (receive_buff1[pointer+2] == 'u')
			speed=15;
		break;
	case 'M':
		if (receive_buff1[pointer+2] == 'u')
			speed=25;
		break;
	case 'H':
		if (receive_buff1[pointer+2] == 'u')
			speed=35;
		break;
	}
}
void catch_arm(uint8_t pointer)//控制机械臂和发射台的四个气缸
{
	switch (receive_buff1[pointer]) {
	case 'a':
		if (receive_buff1[pointer+2] == 'u')
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);//爪子		
		break;
	case 'b':
		if (receive_buff1[pointer+2] == 'u')
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);//小臂
		break;
		case 'c':
 	if (receive_buff1[pointer+2] == 'u')
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);//大臂
	
		break;
	case 'F':
		if (receive_buff1[pointer+2] == 'd')
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);//发射
		else if (receive_buff1[pointer+2] == 'u')
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		break;
	}
}

void catch_data()//获取wifi模块数据
{
rx_flag=0,flag_x=0,rw_flag=0,negative=1;
   uint8_t data_length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);   //计算接收到的数据长度
	
		for(uint8_t pointer=0;pointer<=data_length-1;pointer++)
			{
				catch_direction(pointer);
				catch_speed(pointer);
				catch_arm(pointer);
				catch_arm_gm6020(pointer);
				catch_arm_rm2006(pointer);
			}
}

