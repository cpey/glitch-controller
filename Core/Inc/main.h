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
		
void MX_TIM2_Init(uint32_t, uint32_t);

#define GLITCH_DELAY_US_LENGTH  3
#define GLITCH_DELAY_MS_LENGTH  4

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
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define B1_EXTI_IRQn EXTI0_1_IRQn
#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOC
#define LD6_Pin GPIO_PIN_7
#define LD6_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD5_Pin GPIO_PIN_9
#define LD5_GPIO_Port GPIOC
#define TRESET_Pin GPIO_PIN_9
#define TRESET_GPIO_Port GPIOA
#define USBF4_DM_Pin GPIO_PIN_11
#define USBF4_DM_GPIO_Port GPIOA
#define USBF4_DP_Pin GPIO_PIN_12
#define USBF4_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PG0_Pin GPIO_PIN_10
#define PG0_GPIO_Port GPIOC
#define PG1_Pin GPIO_PIN_11
#define PG1_GPIO_Port GPIOC
#define PG2_Pin GPIO_PIN_12
#define PG2_GPIO_Port GPIOC
#define PG3_Pin GPIO_PIN_2
#define PG3_GPIO_Port GPIOD
#define PG4_Pin GPIO_PIN_3
#define PG4_GPIO_Port GPIOB
#define PG5_Pin GPIO_PIN_4
#define PG5_GPIO_Port GPIOB
#define PG6_Pin GPIO_PIN_5
#define PG6_GPIO_Port GPIOB
#define PG7_Pin GPIO_PIN_6
#define PG7_GPIO_Port GPIOB
#define PGCLK_Pin GPIO_PIN_7
#define PGCLK_GPIO_Port GPIOB
#define PGSIG_Pin GPIO_PIN_8
#define PGSIG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define GPIOB_BSRR ((volatile uint32_t *) &GPIOB->BSRR)
#define GPIOD_BSRR ((volatile uint32_t *) &GPIOD->BSRR)
#define GPIOC_BSRR ((volatile uint32_t *) &GPIOC->BSRR)

#define GPIOB_VALUE_CFG(value) ((value & 0xf0) >> 1)
#define GPIOD_VALUE_CFG(value) ((value & 0x08) >> 1)
#define GPIOC_VALUE_CFG(value) ((value & 0x07) << 10)

inline void pg_set_value_fast(uint8_t value) {
    // Clk: PB7
    // Data: PB6, PB5, PB4, PB3, PD2, PC12, PC11, PC10
    *GPIOB_BSRR = GPIOB_VALUE_CFG(value) | (GPIOB_VALUE_CFG(~value) << 16);
    *GPIOD_BSRR = GPIOD_VALUE_CFG(value) | (GPIOD_VALUE_CFG(~value) << 16);
    *GPIOC_BSRR = GPIOC_VALUE_CFG(value) | (GPIOC_VALUE_CFG(~value) << 16);

    // Generate a  pulse on the PGCLK pin
    GPIOB->BSRR = (uint32_t)PGCLK_Pin;
    GPIOB->BRR  = (uint32_t)PGCLK_Pin;
}

inline void pg_sig_set_high() {
    PGSIG_GPIO_Port->BSRR = (uint32_t)PGSIG_Pin;
}

inline void pg_sig_set_low() {
    PGSIG_GPIO_Port->BRR  = (uint32_t)PGSIG_Pin;
}

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
