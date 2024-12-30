/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "comm.h"
#include "serial.h"
#include "controller.h"
#include "glitcher.h"
#include "usb_device.h"
#include "stm32f0xx_ll_tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

uint8_t usb_msg[USB_BUFFER_SIZE];
uint16_t usb_msg_len = 0;
bool usb_msg_locked = false;

bool led_enabled = true;
bool reset_timer = false;

void print_timer_value() {
    char display_timer[50];
    uint32_t timer_val;
    timer_val = __HAL_TIM_GET_COUNTER(&htim2);
    sprintf(display_timer, "Timer: %ld", timer_val);
    send_to_usb(display_timer);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init(GLITCH_DELAY_MS, GLITCH_DELAY_US);

  /* USER CODE BEGIN 2 */
  pg_sig_set_high();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        if (led_enabled)
            HAL_GPIO_TogglePin(GPIOC, LD5_Pin);

        if (reset_timer) {
			reset_target();
            send_to_usb("Timer reset");
            reset_timer = false;
        }

        /* USER CODE BEGIN 3 */
        if (usb_msg_len > 0) {
            usb_msg_locked = true;
            process_cmd(usb_msg, usb_msg_len);
            usb_msg_len = 0;
            usb_msg_locked = false;
        }

        HAL_Delay(1);
    }
  /* USER CODE END 3 */
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
