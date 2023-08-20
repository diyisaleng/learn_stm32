#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void CAN_Init(CAN_HandleTypeDef *hcan);
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint8_t *pData, uint16_t ID);

#endif
