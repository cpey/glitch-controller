#include <math.h>

#include "controller.h"
#include "glitcher.h"
#include "serial.h"

#define RESET_TIME_MS           100

void MX_TIM2_Init(uint32_t, uint32_t);
void MX_TIM15_Init(uint8_t, uint8_t, uint32_t);

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

void reset_target() {
    HAL_GPIO_WritePin(TRESET_GPIO_Port, TRESET_Pin, GPIO_PIN_SET);
    HAL_Delay(RESET_TIME_MS);
    HAL_GPIO_WritePin(TRESET_GPIO_Port, TRESET_Pin, GPIO_PIN_RESET);
}

void ctrl_TIM2_init(uint32_t period_ms, uint32_t period_us) {
    MX_TIM2_Init(period_ms, period_us);
}

void ctrl_TIM15_init(uint32_t period_ms, uint32_t period_us) {
    uint32_t counter = 0;
    if (period_ms <= CTRL_TIM15_MAX_PERIOD_MS) {
        MX_TIM15_Init(period_ms, period_us, counter);
        return;
    }

    uint8_t period_search = CTRL_TIM15_MAX_PERIOD_MS;
    while ((period_ms % period_search != 0)  && period_search) {
        period_search -=1;
    }
    counter = (period_ms / period_search) - 1;
    period_us = period_us / counter;
    MX_TIM15_Init(period_search, period_us, counter);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int level = 0;
    if (htim == &htim2) {
        generate_glitch();
    } else if (htim == &htim15) {
        if (!level) {
            HAL_GPIO_WritePin(TRESET_GPIO_Port, TRESET_Pin, GPIO_PIN_SET);
            level = 1;
        } else {
            HAL_GPIO_WritePin(TRESET_GPIO_Port, TRESET_Pin, GPIO_PIN_RESET);
            level = 0;
        }
        //HAL_TIM_Base_Stop_IT(&htim15);
        //send_to_usb("TIM15");
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM2_Init(uint32_t period_ms, uint32_t period_us)
{

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 48 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = (period_ms * 1e3) + period_us - 1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
    sSlaveConfig.InputTrigger = TIM_TS_ETRF;
    sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
    sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    sSlaveConfig.TriggerFilter = 0;
    if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable the TIM Update Interrupt */
    LL_TIM_ClearFlag_UPDATE(htim2.Instance);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    /* Set One Pulse Mode to avoid automatic reload */
    htim2.Instance->CR1 |= TIM_CR1_OPM;
}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM15_Init(uint8_t period_ms, uint8_t period_us, uint32_t counter)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim15.Instance = TIM15;
    htim15.Init.Prescaler = 48 - 1;
    htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim15.Init.Period = (period_ms * 1e3) + period_us - 1;
    htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.RepetitionCounter = counter;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

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
                            |PGCLK_Pin|PGSIG_Pin, GPIO_PIN_RESET);

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
                             PGCLK_Pin PGSIG_Pin */
    GPIO_InitStruct.Pin = PG4_Pin|PG5_Pin|PG6_Pin|PG7_Pin
                            |PGCLK_Pin|PGSIG_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /**/
    HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB6);

    /**/
    HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB7);

    /**/
    HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB8);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

