
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAN_H
#define CAN_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
//#include "main.h"
#include "InverseKinematics.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void COM_RunCanAction1();
void COM_RunCanAction2();
void ReadToInstructions();

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

void SendAngles();
void SendSpeeds();
void SendGripper();
void SendInstructions();
extern uint32_t CAN_TxMailbox;
extern CAN_TxHeaderTypeDef TxHeader2;
extern uint8_t TxData2[8];
extern CAN_RxHeaderTypeDef RxHeader2;
extern uint8_t RxData2[8];

extern uint32_t CAN_TxMailbox1;
extern CAN_TxHeaderTypeDef TxHeader1;
extern uint8_t TxData1[8];
extern CAN_RxHeaderTypeDef RxHeader1;
extern uint8_t RxData1[8];

extern uint8_t PredkosciGlobal;



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
