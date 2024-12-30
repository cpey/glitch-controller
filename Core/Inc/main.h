/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f0xx_hal.h"

typedef enum { false, true } bool;

extern bool led_enabled;
extern bool reset_timer;

#define USB_BUFFER_SIZE 256
extern uint8_t usb_msg[USB_BUFFER_SIZE];
extern uint16_t usb_msg_len;
extern bool usb_msg_locked;

#define HEX_TO_BYTE_A_TO_F(val) ((val) % 32 + 9 )
#define HEX_TO_BYTE(val) (((val) % 32 + 9) % 25)
#define IS_VALID_HEX(val) (((val) >= 0x30 && (val) <= 0x39) || \
					(HEX_TO_BYTE_A_TO_F(val) >= 10 && HEX_TO_BYTE_A_TO_F(val) <= 15))
		
extern TIM_HandleTypeDef htim2;

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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
