#include "main.h"
#include "stm32f4xx_it.h"
#include "usart.h"
#include "catch_data.h"

#define BUFFER_SIZE 255

extern uint8_t receive_buff1[255];
extern int getdata_x,getdata_y,getdata_w;
extern DMA_HandleTypeDef hdma_usart1_rx;

void catch_data()
{
_Bool rx_flag=0;
_Bool flag_x=0;
	_Bool rw_flag=0;
 int negative=1;
   uint8_t data_length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);   //计算接收到的数据长度
	
		for(uint8_t pointer=0;pointer<=data_length-1;pointer++)
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
			
			}}

			