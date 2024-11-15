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
#include "usb_device.h"

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
static void MX_GPIO_Init(void);
void delay(uint32_t);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
uint8_t usb_msg[USB_BUFFER_SIZE];
uint16_t usb_msg_len = 0;
bool usb_msg_locked = false;
uint8_t pg_value = 0;

bool led_enabled = true;

#define PG_MASK_BIT0    0x01
#define PG_MASK_BIT1    0x02
#define PG_MASK_BIT2    0x04
#define PG_MASK_BIT3    0x08
#define PG_MASK_BIT4    0x10
#define PG_MASK_BIT5    0x20
#define PG_MASK_BIT6    0x40
#define PG_MASK_BIT7    0x80

#define RESET_TIME_MS   500


void pg_set_value(uint8_t value) {

    HAL_GPIO_WritePin(PG0_GPIO_Port, PG0_Pin, (GPIO_PinState)(value & PG_MASK_BIT0));
    HAL_GPIO_WritePin(PG1_GPIO_Port, PG1_Pin, (GPIO_PinState)(value & PG_MASK_BIT1));
    HAL_GPIO_WritePin(PG2_GPIO_Port, PG2_Pin, (GPIO_PinState)(value & PG_MASK_BIT2));
    HAL_GPIO_WritePin(PG3_GPIO_Port, PG3_Pin, (GPIO_PinState)(value & PG_MASK_BIT3));
    HAL_GPIO_WritePin(PG4_GPIO_Port, PG4_Pin, (GPIO_PinState)(value & PG_MASK_BIT4));
    HAL_GPIO_WritePin(PG5_GPIO_Port, PG5_Pin, (GPIO_PinState)(value & PG_MASK_BIT5));
    HAL_GPIO_WritePin(PG6_GPIO_Port, PG6_Pin, (GPIO_PinState)(value & PG_MASK_BIT6));
    HAL_GPIO_WritePin(PG7_GPIO_Port, PG7_Pin, (GPIO_PinState)(value & PG_MASK_BIT7));
    HAL_GPIO_TogglePin(PGCLK_GPIO_Port, PGCLK_Pin);
}

void reset_target() {
    HAL_GPIO_WritePin(TRESET_GPIO_Port, TRESET_Pin, GPIO_PIN_SET);
    delay(RESET_TIME_MS);
    HAL_GPIO_WritePin(TRESET_GPIO_Port, TRESET_Pin, GPIO_PIN_RESET);
}

void delay(uint32_t num_ms) {
    uint32_t init = uwTick;
    while (uwTick - init < num_ms);
}

/**
 * cmd: '0x??' -> 4 chars
 * 
 */
void process_cmd_voltage(uint8_t *usb_msg, uint16_t usb_msg_len) {
    if (usb_msg_len < 4)
        return;

    if (usb_msg[0] != '0' || usb_msg[1] != 'x')
        return;

    if (!(IS_VALID_HEX(usb_msg[2]) && IS_VALID_HEX(usb_msg[3])))
        return;

    pg_value = (HEX_TO_BYTE(usb_msg[2]) << 4) + HEX_TO_BYTE(usb_msg[3]);
}

/**
 * cmd: ' v 0x??' -> 8 chars
 * 
 */
void process_cmd_set(uint8_t *usb_msg, uint16_t usb_msg_len) {
    if (usb_msg_len < 8)
        return;

    uint8_t arg = usb_msg[1];
    
    switch (arg) {
        case 'v':
            if (usb_msg[2] != ' ')
                return;
            process_cmd_voltage(usb_msg + 3, usb_msg_len - 3);
            break;
        default:
            break;
    }
}

/**
 * cmd: 's v 0x??' -> 7 chars
 * 
 */
void process_cmd(uint8_t *usb_msg, uint16_t usb_msg_len) {
    if (!usb_msg_len) 
        return;
    uint8_t cmd = usb_msg[0];
    switch (cmd) {
        case 's':
            process_cmd_set(usb_msg + 1, usb_msg_len - 1);
            break;
        default:
            break;
    }
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        if (led_enabled)
            HAL_GPIO_TogglePin(GPIOC, LD5_Pin);

        pg_set_value(pg_value);
        //for (uint32_t i=0; i<500000; i++);
        HAL_GPIO_WritePin(TRESET_GPIO_Port, TRESET_Pin, GPIO_PIN_SET);
        delay(100);
        HAL_GPIO_TogglePin(PGCLK_GPIO_Port, PGCLK_Pin);
        HAL_GPIO_WritePin(TRESET_GPIO_Port, TRESET_Pin, GPIO_PIN_RESET);
        delay(200);
        /* USER CODE BEGIN 3 */
        if (usb_msg_len > 0) {
            usb_msg_locked = true;
            process_cmd(usb_msg, usb_msg_len);
            usb_msg_len = 0;
            usb_msg_locked = false;
        }
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD3_Pin|LD6_Pin|LD4_Pin|LD5_Pin
                          |PG0_Pin|PG1_Pin|PG2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRESET_GPIO_Port, TRESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PG3_GPIO_Port, PG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PG4_Pin|PG5_Pin|PG6_Pin|PG7_Pin
                          |PGCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD6_Pin LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD6_Pin|LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TRESET_Pin */
  GPIO_InitStruct.Pin = TRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0_Pin PG1_Pin PG2_Pin */
  GPIO_InitStruct.Pin = PG0_Pin|PG1_Pin|PG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG3_Pin */
  GPIO_InitStruct.Pin = PG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PG3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG4_Pin PG5_Pin PG6_Pin PG7_Pin
                           PGCLK_Pin */
  GPIO_InitStruct.Pin = PG4_Pin|PG5_Pin|PG6_Pin|PG7_Pin
                          |PGCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB6);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB7);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
