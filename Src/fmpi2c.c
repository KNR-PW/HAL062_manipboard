/**
  ******************************************************************************
  * File Name          : FMPI2C.c
  * Description        : This file provides code for the configuration
  *                      of the FMPI2C instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fmpi2c.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

FMPI2C_HandleTypeDef hfmpi2c1;
DMA_HandleTypeDef hdma_fmpi2c1_rx;
DMA_HandleTypeDef hdma_fmpi2c1_tx;

/* FMPI2C1 init function */
void MX_FMPI2C1_Init(void)
{

  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 0xC0000E12;
  hfmpi2c1.Init.OwnAddress1 = 0;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_FMPI2C_MspInit(FMPI2C_HandleTypeDef* fmpi2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(fmpi2cHandle->Instance==FMPI2C1)
  {
  /* USER CODE BEGIN FMPI2C1_MspInit 0 */

  /* USER CODE END FMPI2C1_MspInit 0 */
  
    /**FMPI2C1 GPIO Configuration    
    PC6     ------> FMPI2C1_SCL
    PC7     ------> FMPI2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_FMPI2C1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* FMPI2C1 clock enable */
    __HAL_RCC_FMPI2C1_CLK_ENABLE();
  
    /* FMPI2C1 DMA Init */
    /* FMPI2C1_RX Init */
    hdma_fmpi2c1_rx.Instance = DMA1_Stream2;
    hdma_fmpi2c1_rx.Init.Channel = DMA_CHANNEL_2;
    hdma_fmpi2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_fmpi2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_fmpi2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_fmpi2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_fmpi2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_fmpi2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_fmpi2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_fmpi2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_fmpi2c1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(fmpi2cHandle,hdmarx,hdma_fmpi2c1_rx);

    /* FMPI2C1_TX Init */
    hdma_fmpi2c1_tx.Instance = DMA1_Stream5;
    hdma_fmpi2c1_tx.Init.Channel = DMA_CHANNEL_2;
    hdma_fmpi2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_fmpi2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_fmpi2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_fmpi2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_fmpi2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_fmpi2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_fmpi2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_fmpi2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_fmpi2c1_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(fmpi2cHandle,hdmatx,hdma_fmpi2c1_tx);

  /* USER CODE BEGIN FMPI2C1_MspInit 1 */

  /* USER CODE END FMPI2C1_MspInit 1 */
  }
}

void HAL_FMPI2C_MspDeInit(FMPI2C_HandleTypeDef* fmpi2cHandle)
{

  if(fmpi2cHandle->Instance==FMPI2C1)
  {
  /* USER CODE BEGIN FMPI2C1_MspDeInit 0 */

  /* USER CODE END FMPI2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FMPI2C1_CLK_DISABLE();
  
    /**FMPI2C1 GPIO Configuration    
    PC6     ------> FMPI2C1_SCL
    PC7     ------> FMPI2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* FMPI2C1 DMA DeInit */
    HAL_DMA_DeInit(fmpi2cHandle->hdmarx);
    HAL_DMA_DeInit(fmpi2cHandle->hdmatx);
  /* USER CODE BEGIN FMPI2C1_MspDeInit 1 */

  /* USER CODE END FMPI2C1_MspDeInit 1 */
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
