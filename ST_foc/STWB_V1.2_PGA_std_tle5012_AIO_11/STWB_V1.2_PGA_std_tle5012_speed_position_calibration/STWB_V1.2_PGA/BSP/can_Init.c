#include "can_Init.h"
#include "can.h"


void CAN_Init(void)
{
		CAN_FilterTypeDef pFilter;
	
		pFilter.FilterIdHigh                 = 0;
    pFilter.FilterIdLow                  = 0;
    pFilter.FilterMaskIdHigh         =    0;
    pFilter.FilterMaskIdLow             =    0;
    pFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    pFilter.FilterBank                     = 0;
    pFilter.FilterMode                     = CAN_FILTERMODE_IDMASK;
    pFilter.FilterScale                     = CAN_FILTERSCALE_32BIT;
    pFilter.FilterActivation         = ENABLE;
    pFilter.SlaveStartFilterBank = 0;
	
		HAL_CAN_ConfigFilter(&hcan,&pFilter);
	
		/*----Æô¶¯CAN ---------------------------------------------------*/
		while(HAL_CAN_Start(&hcan) != HAL_OK )
		{
				HAL_Delay(100);
		}
		

		if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		{
			/* Notification Error */
			Error_Handler();
		}
	
}



CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		/* Get RX message */
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}
	 

	}
