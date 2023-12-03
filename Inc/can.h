
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAN_H
#define CAN_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "InverseKinematics.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

void SendAngles();
void SendSpeeds();
void SendGripper();
void SendInstructions();
uint32_t CAN_TxMailbox;
CAN_TxHeaderTypeDef TxHeader2;
uint8_t TxData2[8];
CAN_RxHeaderTypeDef RxHeader2;
uint8_t RxData2[8];

uint32_t CAN_TxMailbox1;
CAN_TxHeaderTypeDef TxHeader1;
uint8_t TxData1[8];
CAN_RxHeaderTypeDef RxHeader1;
uint8_t RxData1[8];

uint8_t PredkosciGlobal;



/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
