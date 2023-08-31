#ifndef __CATCH_DATA_H__
#define __CATCH_DATA_H__
#include "main.h"
#include "stm32f4xx_it.h"
#include "usart.h"

#define BUFFER_SIZE 255

extern uint8_t receive_buff1[255];
extern int getdata_x,getdata_y,getdata_w;
extern DMA_HandleTypeDef hdma_usart1_rx;

void catch_data(void);

#endif
