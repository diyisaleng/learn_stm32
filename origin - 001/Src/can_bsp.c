#include "can.h"
#include "can_bsp.h"
#include "main.h"

/*初始化CAN*/
void CAN_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    HAL_StatusTypeDef HAL_Status;
    /*这里默认CAN1和CAN2使用不同的FIFO*/
    if (hcan->Instance == hcan1.Instance)
    {
        sFilterConfig.FilterBank = 0;                     // 过滤器组
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // CAN_FILTERMODE_IDLIST  CAN_FILTERMODE_IDMASK
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterIdHigh = 0x0000; // filter id
        sFilterConfig.FilterIdLow = 0x0000;
        sFilterConfig.FilterMaskIdHigh = 0x0000;
        sFilterConfig.FilterMaskIdLow = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 用FIFO接收
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 1;

        HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
        HAL_CAN_Start(hcan); // 开启CAN
        HAL_Status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (hcan->Instance == hcan2.Instance)
    {
        sFilterConfig.FilterBank = 15;                    // 过滤器组
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // CAN_FILTERMODE_IDLIST  CAN_FILTERMODE_IDMASK
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterIdHigh = 0x0000; // filter id
        sFilterConfig.FilterIdLow = 0x0000;
        sFilterConfig.FilterMaskIdHigh = 0x0000;
        sFilterConfig.FilterMaskIdLow = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; // 用FIFO接收
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 15;

        HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
        HAL_CAN_Start(hcan); // 开启CAN
        HAL_Status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
    if (HAL_Status != HAL_OK)
    {
        Error_Handler();
    }
}

/*CAN发送函数*/
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint8_t *pData, uint16_t ID)
{
    HAL_StatusTypeDef HAL_RetVal = HAL_ERROR;
    CAN_TxHeaderTypeDef TxMessage;
    uint32_t TxMailboxX = CAN_TX_MAILBOX0; // CAN发送邮箱

    TxMessage.StdId = ID;
    TxMessage.DLC = 8; /*默认一帧传输长度为8*/
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;

    while (!HAL_CAN_GetTxMailboxesFreeLevel(hcan)) // 等待空邮箱
        ;
    if ((hcan->Instance->TSR & CAN_TSR_TME0) != RESET) // 检查发送邮箱状态
        TxMailboxX = CAN_TX_MAILBOX0;
    else if ((hcan->Instance->TSR & CAN_TSR_TME1) != RESET)
        TxMailboxX = CAN_TX_MAILBOX1;
    else if ((hcan->Instance->TSR & CAN_TSR_TME2) != RESET)
        TxMailboxX = CAN_TX_MAILBOX2;

    HAL_RetVal = HAL_CAN_AddTxMessage(hcan, &TxMessage, pData, (uint32_t *)TxMailboxX);

    if (HAL_RetVal != HAL_OK)
    {
        return 2;
    }

    return 0;
}

///*FIFO0的回调函数*/
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	if(hcan->Instance == hcan1.Instance)
//	{
//		CAN_RxHeaderTypeDef RxMessage;
//	uint8_t RxData[8] = {0};
//
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxData);
//	/*自己写解析函数*/
//
//	}
// }

///*FIFO0的回调函数*/
// void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	if(hcan->Instance == hcan2.Instance)
//	{
//		CAN_RxHeaderTypeDef RxMessage;
//	uint8_t RxData[8] = {0};
//
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxData);
//	/*自己写解析函数*/
//
//	}
// }
