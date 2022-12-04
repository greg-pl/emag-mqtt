/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ON_OFF_3_8V_Pin GPIO_PIN_2
#define ON_OFF_3_8V_GPIO_Port GPIOE
#define GSM_AP_RDY_Pin GPIO_PIN_4
#define GSM_AP_RDY_GPIO_Port GPIOE
#define GSM_STATUS_IN_Pin GPIO_PIN_5
#define GSM_STATUS_IN_GPIO_Port GPIOE
#define CFG0_Pin GPIO_PIN_0
#define CFG0_GPIO_Port GPIOC
#define CFG1_Pin GPIO_PIN_2
#define CFG1_GPIO_Port GPIOC
#define CFG2_Pin GPIO_PIN_3
#define CFG2_GPIO_Port GPIOC
#define CFG3_Pin GPIO_PIN_0
#define CFG3_GPIO_Port GPIOA
#define AC_TEMP_HEAT_Pin GPIO_PIN_4
#define AC_TEMP_HEAT_GPIO_Port GPIOA
#define ETH_PHY_CLKIN_Pin GPIO_PIN_5
#define ETH_PHY_CLKIN_GPIO_Port GPIOA
#define ETH_PHY_RST_Pin GPIO_PIN_6
#define ETH_PHY_RST_GPIO_Port GPIOA
#define HEAT_FLG_Pin GPIO_PIN_0
#define HEAT_FLG_GPIO_Port GPIOB
#define PMS_SET_Pin GPIO_PIN_1
#define PMS_SET_GPIO_Port GPIOB
#define PYL_FLG_Pin GPIO_PIN_8
#define PYL_FLG_GPIO_Port GPIOE
#define HEAT_PWR_EN_Pin GPIO_PIN_9
#define HEAT_PWR_EN_GPIO_Port GPIOE
#define PYL_PWR_EN_Pin GPIO_PIN_10
#define PYL_PWR_EN_GPIO_Port GPIOE
#define LIGHT_EN_Pin GPIO_PIN_13
#define LIGHT_EN_GPIO_Port GPIOE
#define LIGHT_FLG_Pin GPIO_PIN_14
#define LIGHT_FLG_GPIO_Port GPIOE
#define ETH_PWR_EN_Pin GPIO_PIN_15
#define ETH_PWR_EN_GPIO_Port GPIOE
#define USART1_TX_EN_Pin GPIO_PIN_15
#define USART1_TX_EN_GPIO_Port GPIOB
#define RS2_PWR_EN_Pin GPIO_PIN_10
#define RS2_PWR_EN_GPIO_Port GPIOD
#define USART3_TX_EN_Pin GPIO_PIN_12
#define USART3_TX_EN_GPIO_Port GPIOD
#define RS2_FLG_Pin GPIO_PIN_13
#define RS2_FLG_GPIO_Port GPIOD
#define RS1_PWR_EN_Pin GPIO_PIN_14
#define RS1_PWR_EN_GPIO_Port GPIOD
#define RS1_FLG_Pin GPIO_PIN_15
#define RS1_FLG_GPIO_Port GPIOD
#define GSM_UART1_DTR_Pin GPIO_PIN_6
#define GSM_UART1_DTR_GPIO_Port GPIOC
#define GSM_UART1_RI_Pin GPIO_PIN_7
#define GSM_UART1_RI_GPIO_Port GPIOC
#define GSM_UART1_DCD_Pin GPIO_PIN_8
#define GSM_UART1_DCD_GPIO_Port GPIOC
#define GSM_UART2_CTS_Pin GPIO_PIN_3
#define GSM_UART2_CTS_GPIO_Port GPIOD
#define GSM_UART2_RTS_Pin GPIO_PIN_4
#define GSM_UART2_RTS_GPIO_Port GPIOD
#define GSM_UART2_TX_Pin GPIO_PIN_5
#define GSM_UART2_TX_GPIO_Port GPIOD
#define GSM_UART2_RX_Pin GPIO_PIN_6
#define GSM_UART2_RX_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_3
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_5
#define LED_B_GPIO_Port GPIOB
#define GSM_RESET_Pin GPIO_PIN_8
#define GSM_RESET_GPIO_Port GPIOB
#define GSM_ON_OFF_Pin GPIO_PIN_1
#define GSM_ON_OFF_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
