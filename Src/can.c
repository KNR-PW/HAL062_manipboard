
/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
void COM_RunCanAction1();
void COM_RunCanAction2();
void ReadToInstructions();

extern float vx, vy, vz;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, RxData2);
		COM_RunCanAction2();
	}
	if (hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, RxData1);
		COM_RunCanAction1();
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader2, RxData2);

}

void COM_RunCanAction1()
{
	if (RxHeader1.StdId == 128)
	{
		if (RxData1[0] == 1 && RxData1[1] == 1 && RxData1[2] == RxData1[3]
				&& RxData1[4] == RxData1[5])
		{
			switch (RxData1[3])
			{
			case DISABLED:
				if (ManipulatorMode != DISABLED)
				{
					ManipulatorMode = DISABLED;
					newmode = 1;
				}
				break;
			case HOLD:
				if (ManipulatorMode != HOLD)
				{
					ManipulatorMode = HOLD;
					newmode = 1;
				}
				break;
			case ANGLES:
				if (ManipulatorMode != ANGLES)
				{
					ManipulatorMode = ANGLES;
					newmode = 1;
				}
				break;
			case VELOCITY:
				if (ManipulatorMode != VELOCITY)
				{
					ManipulatorMode = VELOCITY;
					newmode = 1;
				}
				break;
			case XYZ_GLOBAL:
				if (ManipulatorMode != XYZ_GLOBAL)
				{
					ManipulatorMode = XYZ_GLOBAL;
					newmode = 1;
				}
				break;
			case VEL_GLOB:
				if (ManipulatorMode != VEL_GLOB)
				{
					ManipulatorMode = VEL_GLOB;
					newmode = 1;
				}
				break;
			case VEL_TOOL:
				if (ManipulatorMode != VEL_TOOL)
				{
					ManipulatorMode = VEL_TOOL;
					newmode = 1;
				}
				break;
			case DELTA_TOOL:
				if (ManipulatorMode != DELTA_TOOL)
				{
					ManipulatorMode = DELTA_TOOL;
					newmode = 1;
				}
				break;
			case TRAJECTORY:
				if (ManipulatorMode != TRAJECTORY)
				{
					ManipulatorMode = TRAJECTORY;
					newmode = 1;
				}
				break;

			}
			if (DisconnectDoF != RxData1[5])
			{
				DisconnectDoF = RxData1[5];
				newmode = 1;
			}
		}
		else
		{
			ManipulatorMode = DISABLED;
			newmode = 1;
		}
	}
	else if (RxHeader1.StdId > 128 && RxHeader1.StdId < 132)
	{
		int z = (RxHeader1.StdId - 129) * 2;
		for (int i = 0; i < 2; i++)
		{
			read_value.i[i + z] = (RxData1[0 + (4 * i)] << 24)
					| (RxData1[1 + (4 * i)] << 16) | (RxData1[2 + (4 * i)] << 8)
					| (RxData1[3 + (4 * i)]);
		}
		ReadToInstructions();
	}
	else if (RxHeader1.StdId == 157)
	{
		GripperState = RxData1[0];
	}
	else if (RxHeader1.StdId == 117)
	{
		if (RxData1[0] == 1)
		{
			HAL_NVIC_SystemReset();
		}
		if (RxData1[1] == 1)
		{
			TxHeader1.StdId = 117;
			TxHeader1.DLC = 2;
			TxData2[0] = RxData1[0];
			TxData2[1] = RxData1[1];
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &CAN_TxMailbox);
		}
	}
}
void COM_RunCanAction2()
{
	if (RxHeader2.StdId >= 158 && RxHeader2.StdId <= 163)
	{
		static int i = 0;
		meas_angle.i[RxHeader2.StdId - 158] = (RxData2[0] << 24)
				| (RxData2[1] << 16) | (RxData2[2] << 8) | (RxData2[3]);
		if(i>=13)
		{
		TxHeader1.StdId = RxHeader2.StdId;
		TxHeader1.ExtId = 0;
		TxHeader1.DLC = 8;

			HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, RxData2, &CAN_TxMailbox1);
			i = 0;
		}
		i++;
	}

}
void SendInstructions()
{
	for (int j = 0; j < 6; j++)
	{

		TxHeader2.StdId = 151 + j;
		TxHeader2.ExtId = 0;
		TxHeader2.DLC = 8;
		TxData2[0] = 1;
		TxData2[1] = 1;
		TxData2[2] = ManipulatorMode;
		TxData2[3] = ManipulatorMode;
		if (ManipulatorMode != 0)
		{
			if (ManipulatorMode == HOLD)
			{
				TxData2[2] = ManipulatorMode;
				TxData2[3] = ManipulatorMode;
				for (int i = 4; i < 8; i++)
				{
					TxData2[i] = (angle.i[j] >> 8 * (7 - i)) & 0xFF;
				}
			}
			else if (ManipulatorMode == VELOCITY)
			{
				TxData2[2] = ManipulatorMode;
				TxData2[3] = ManipulatorMode;
				for (int i = 4; i < 8; i++)
				{
					TxData2[i] = (speed.i[j] >> 8 * (7 - i)) & 0xFF;
				}
			}
			else if(DisconnectDoF != 0 && j>4)
			{
				TxData2[2] = VELOCITY;
				TxData2[3] = VELOCITY;
				for (int i = 4; i < 8; i++)
				{
					TxData2[i] = (speed.i[j] >> 8 * (7 - i)) & 0xFF;
				}
			}
			else
			{
				TxData2[2] = ANGLES;
				TxData2[3] = ANGLES;
				for (int i = 4; i < 8; i++)
				{
					TxData2[i] = (angle.i[j] >> 8 * (7 - i)) & 0xFF;
				}
			}
		}
		while (HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &CAN_TxMailbox)
				!= HAL_OK)
			;

	}
}

void SendAngles()
{
	return;
	TxHeader2.StdId = 110;
	TxHeader2.ExtId = 0;
	TxHeader2.DLC = 8;
	for (int i = 0; i < 4; i++)
	{
		TxData2[i] = (angle.i[0] >> 8 * (3 - i)) & 0xFF;
	}
	for (int i = 4; i < 8; i++)
	{
		TxData2[i] = (angle.i[1] >> 8 * (7 - i)) & 0xFF;
	}
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &CAN_TxMailbox);
	TxHeader2.StdId = 111;
	for (int i = 0; i < 4; i++)
	{
		TxData2[i] = (angle.i[2] >> 8 * (3 - i)) & 0xFF;
	}
	for (int i = 4; i < 8; i++)
	{
		TxData2[i] = (angle.i[3] >> 8 * (7 - i)) & 0xFF;
	}
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &CAN_TxMailbox);
	TxHeader2.StdId = 112;
	for (int i = 0; i < 4; i++)
	{
		TxData2[i] = (angle.i[4] >> 8 * (3 - i)) & 0xFF;
	}
	for (int i = 4; i < 8; i++)
	{
		TxData2[i] = (angle.i[5] >> 8 * (7 - i)) & 0xFF;
	}
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &CAN_TxMailbox);
}
void SendSpeeds()
{
	return;
	TxHeader2.StdId = 113;
	TxHeader2.ExtId = 0;
	TxHeader2.DLC = 8;
	for (int i = 0; i < 4; i++)
	{
		TxData2[i] = (speed.i[0] >> 8 * (3 - i)) & 0xFF;
	}
	for (int i = 4; i < 8; i++)
	{
		TxData2[i] = (speed.i[1] >> 8 * (7 - i)) & 0xFF;
	}
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &CAN_TxMailbox);
	TxHeader2.StdId = 114;
	for (int i = 0; i < 4; i++)
	{
		TxData2[i] = (speed.i[2] >> 8 * (3 - i)) & 0xFF;
	}
	for (int i = 4; i < 8; i++)
	{
		TxData2[i] = (speed.i[3] >> 8 * (7 - i)) & 0xFF;
	}
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &CAN_TxMailbox);
	TxHeader2.StdId = 115;
	for (int i = 0; i < 4; i++)
	{
		TxData2[i] = (speed.i[4] >> 8 * (3 - i)) & 0xFF;
	}
	for (int i = 4; i < 8; i++)
	{
		TxData2[i] = (speed.i[5] >> 8 * (7 - i)) & 0xFF;
	}
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &CAN_TxMailbox);
}

void SendGripper()
{
	TxHeader2.StdId = 157;
	TxHeader2.DLC = 1;
	TxData2[0] = GripperState;
	while (HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &CAN_TxMailbox)
					!= HAL_OK)
				;
}
void ReadToInstructions()
{
	for (int j = 0; j < 6; j++)
	{
		if (DisconnectDoF != NO && j > 4)
		{
			speed.f[j] = read_value.f[j];
		}
		else if (DisconnectDoF == DOF456 && j>2 && j<=4)
		{
			angle.f[j] = read_value.f[j];
		}
		else
		{
			switch (ManipulatorMode)
			{
			case DISABLED:
				/*angle.f[j] = read_value.f[j];*/
				break;
			case HOLD:
				/*angle.f[j] = read_value.f[j];*/
				break;
			case ANGLES:
				angle.f[j] = read_value.f[j];
				break;
			case VELOCITY:
				speed.f[j] = read_value.f[j];
				break;
			case XYZ_GLOBAL:
				kinematics_in.f[j] = read_value.f[j];
				break;
			case VEL_GLOB:
				kinematics_in.f[j] = read_value.f[j];
				break;
			case VEL_TOOL:
				kinematics_in.f[j] = read_value.f[j];
				break;
			case DELTA_TOOL:
				kinematics_in.f[j] = read_value.f[j];
				break;
			case TRAJECTORY:
				break;
			}
		}
	}
}
/* CAN1 init function */
void MX_CAN1_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 5;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	//sFilterConfig.FilterNumber = 0;

	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		//Filter configuration Error
	}

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK)
	{
		Error_Handler();
	}
	/*	  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING)!=HAL_OK)
	 {
	 Error_Handler();
	 }*/
	TxHeader1.StdId = 101;
	TxHeader1.ExtId = 0;
	TxHeader1.DLC = 8;
	TxHeader1.RTR = CAN_RTR_DATA;
	TxHeader1.IDE = 0;

	HAL_CAN_Start(&hcan1);

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 5;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_8TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = DISABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	//sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
	{
		//Filter configuration Error
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK)
	{
		Error_Handler();
	}
	/*	  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING)!=HAL_OK)
	 {
	 Error_Handler();
	 }
	 */
	TxHeader2.StdId = 110;
	TxHeader2.ExtId = 0;
	TxHeader2.DLC = 8;
	TxHeader2.RTR = CAN_RTR_DATA;
	TxHeader2.IDE = 0;
	HAL_CAN_Start(&hcan2);

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED = 0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if (canHandle->Instance == CAN1)
	{
		/* USER CODE BEGIN CAN1_MspInit 0 */

		/* USER CODE END CAN1_MspInit 0 */
		/* CAN1 clock enable */
		HAL_RCC_CAN1_CLK_ENABLED++;
		if (HAL_RCC_CAN1_CLK_ENABLED == 1)
		{
			__HAL_RCC_CAN1_CLK_ENABLE()
			;
		}

		/**CAN1 GPIO Configuration
		 PA11     ------> CAN1_RX
		 PA12     ------> CAN1_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* CAN1 interrupt Init */
		HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
		/* USER CODE BEGIN CAN1_MspInit 1 */

		/* USER CODE END CAN1_MspInit 1 */
	}
	else if (canHandle->Instance == CAN2)
	{
		/* USER CODE BEGIN CAN2_MspInit 0 */

		/* USER CODE END CAN2_MspInit 0 */
		/* CAN2 clock enable */
		__HAL_RCC_CAN2_CLK_ENABLE()
		;
		HAL_RCC_CAN1_CLK_ENABLED++;
		if (HAL_RCC_CAN1_CLK_ENABLED == 1)
		{
			__HAL_RCC_CAN1_CLK_ENABLE()
			;
		}

		/**CAN2 GPIO Configuration
		 PB5     ------> CAN2_RX
		 PB6     ------> CAN2_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* CAN2 interrupt Init */
		HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
		HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
		/* USER CODE BEGIN CAN2_MspInit 1 */

		/* USER CODE END CAN2_MspInit 1 */
	}
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

	if (canHandle->Instance == CAN1)
	{
		/* USER CODE BEGIN CAN1_MspDeInit 0 */

		/* USER CODE END CAN1_MspDeInit 0 */
		/* Peripheral clock disable */
		HAL_RCC_CAN1_CLK_ENABLED--;
		if (HAL_RCC_CAN1_CLK_ENABLED == 0)
		{
			__HAL_RCC_CAN1_CLK_DISABLE();
		}

		/**CAN1 GPIO Configuration
		 PA11     ------> CAN1_RX
		 PA12     ------> CAN1_TX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

		/* CAN1 interrupt Deinit */
		HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
		/* USER CODE BEGIN CAN1_MspDeInit 1 */

		/* USER CODE END CAN1_MspDeInit 1 */
	}
	else if (canHandle->Instance == CAN2)
	{
		/* USER CODE BEGIN CAN2_MspDeInit 0 */

		/* USER CODE END CAN2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_CAN2_CLK_DISABLE();
		HAL_RCC_CAN1_CLK_ENABLED--;
		if (HAL_RCC_CAN1_CLK_ENABLED == 0)
		{
			__HAL_RCC_CAN1_CLK_DISABLE();
		}

		/**CAN2 GPIO Configuration
		 PB5     ------> CAN2_RX
		 PB6     ------> CAN2_TX
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);

		/* CAN2 interrupt Deinit */
		HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
		/* USER CODE BEGIN CAN2_MspDeInit 1 */

		/* USER CODE END CAN2_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
