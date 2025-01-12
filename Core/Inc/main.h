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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

typedef enum { false, true } bool;

extern bool led_enabled;
extern bool reset_TIM2;

#define USB_BUFFER_SIZE 256
extern uint8_t usb_msg[USB_BUFFER_SIZE];
extern uint16_t usb_msg_len;
extern bool usb_msg_locked;

#define HEX_TO_BYTE_A_TO_F(val) ((val) % 32 + 9 )
#define HEX_TO_BYTE(val) (((val) % 32 + 9) % 25)
#define IS_VALID_HEX(val) (((val) >= 0x30 && (val) <= 0x39) || \
					(HEX_TO_BYTE_A_TO_F(val) >= 10 && HEX_TO_BYTE_A_TO_F(val) <= 15))
		
extern TIM_HandleTypeDef htim2;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
