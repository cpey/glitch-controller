#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "stm32f0xx_ll_tim.h"

#include <stdint.h>

void reset_target();
void SystemClock_Config();
void MX_TIM2_Init(uint32_t, uint32_t);
void MX_GPIO_Init();
void Error_Handler();

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

#define GPIOB_BSRR ((volatile uint32_t *) &GPIOB->BSRR)
#define GPIOD_BSRR ((volatile uint32_t *) &GPIOD->BSRR)
#define GPIOC_BSRR ((volatile uint32_t *) &GPIOC->BSRR)

#define GPIOB_VALUE_CFG(value) ((value & 0xf0) >> 1)
#define GPIOD_VALUE_CFG(value) ((value & 0x08) >> 1)
#define GPIOC_VALUE_CFG(value) ((value & 0x07) << 10)

extern TIM_HandleTypeDef htim2;

#endif
